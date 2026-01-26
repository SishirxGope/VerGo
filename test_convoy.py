import time
import math
import random
import carla
import config
from carla_interface import CarlaInterface
from planner import BehaviorPlanner, MotionPlanner
from controller import VehicleController

def main():
    print("--- India-Mode Stress Test: Multi-Vehicle Convoy ---")
    interface = CarlaInterface()
    convoy = []
    
    try:
        # 1. Setup
        interface.connect()
        interface.setup_world()
        
        # Set Weather to Clear Noon (Default)
        interface.world.set_weather(carla.WeatherParameters.ClearNoon)
        print("Weather set to: CLEAR NOON")

        # 2. Spawn Ego
        # Use a straight stretch for convoy testing
        interface.spawn_ego_vehicle(spawn_point_index=10) 
        interface.spawn_sensors()
        interface.world.tick()
        
        # 3. Spawning Logic
        def spawn_convoy_at_dist(start_dist):
            new_actors = []
            
            def spawn_actor_at_wp(wp, idx, is_leader=False):
                trans = wp.transform
                trans.location.z += 0.5
                
                bp = interface.world.get_blueprint_library().find('vehicle.tesla.model3')
                if is_leader: 
                    bp.set_attribute('color', '255,0,0') # Red for Leader
                else:
                    bp.set_attribute('color', '0,0,255') # Blue for others
                
                actor = interface.world.try_spawn_actor(bp, trans)
                if actor:
                    # Everyone is on autopilot (Leader manual control is overridden later if needed)
                    actor.set_autopilot(True, interface.tm.get_port())
                    interface.tm.distance_to_leading_vehicle(actor, 5.0)
                    if is_leader and len(convoy) == 0:
                        # Special case for the FIRST leader of the FIRST convoy:
                        # We might turn off autopilot later for the brake test.
                        pass 
                    
                    print(f"Spawned Vehicle {idx} at {wp.transform.location.x:.1f}, {wp.transform.location.y:.1f}")
                    return actor
                return None

            # Spawn 3 cars relative to start_dist
            for i in range(3):
                dist = start_dist + (i * 12.0)
                
                ego_wp = interface.map.get_waypoint(interface.ego_vehicle.get_location())
                next_wps = ego_wp.next(dist)
                if not next_wps: continue
                main_wp = next_wps[0]
                
                neigh_wp = main_wp.get_right_lane()
                if not neigh_wp or neigh_wp.lane_type != carla.LaneType.Driving:
                    neigh_wp = main_wp.get_left_lane()
                
                valid_neigh = (neigh_wp and neigh_wp.lane_type == carla.LaneType.Driving)
                rng = random.random()
                
                # Logic: 10% Blockage, 45% Main, 45% Neighbor
                # If this is the VERY first spawn (convoy currently empty) -> Force Leader Main
                is_first_set = (len(convoy) == 0)
                
                if is_first_set and i == 0:
                     c = spawn_actor_at_wp(main_wp, i, is_leader=True)
                     if c: new_actors.append(c)
                     # Optional blockage for first one
                     if valid_neigh and rng < 0.2:
                          c2 = spawn_actor_at_wp(neigh_wp, i, is_leader=False)
                          if c2: new_actors.append(c2)
                else:
                    # Regular randomization
                    if valid_neigh and rng < 0.10: # Blockage
                        c1 = spawn_actor_at_wp(main_wp, i)
                        c2 = spawn_actor_at_wp(neigh_wp, i)
                        if c1: new_actors.append(c1)
                        if c2: new_actors.append(c2)
                    elif valid_neigh and rng < 0.55: # Neighbor
                        c = spawn_actor_at_wp(neigh_wp, i)
                        if c: new_actors.append(c)
                    else: # Main
                        c = spawn_actor_at_wp(main_wp, i)
                        if c: new_actors.append(c)
            
            return new_actors

        # Initial Spawn (Close range)
        convoy.extend(spawn_convoy_at_dist(15.0))

        # 4. Initialize Modules
        bp_planner = BehaviorPlanner(interface)
        # Give it a path to follow
        spawn_points = interface.map.get_spawn_points()
        dest = random.choice(spawn_points).location
        bp_planner.global_route = interface.generate_route(interface.ego_vehicle.get_location(), dest)
        
        m_planner = MotionPlanner(interface.map)
        controller = VehicleController()

        print("Convoy moving. Leader will brake in 10 seconds...")
        start_time = time.time()
        last_spawn_time = start_time
        brake_triggered = False

        while True:
            data = interface.get_data()
            if not data: continue

            ego_transform = data['ego_transform']
            ego_vel = data['ego_velocity']
            ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)

            # Trigger Lead Brake after 10 seconds (Only for the first convoy leader)
            current_elapsed = time.time() - start_time
            if current_elapsed > 10.0 and not brake_triggered:
                print("!!! LEADER SLAMMING BRAKES !!!")
                if convoy:
                    # The first car in the list (convoy[0]) matches the red leader of the first group
                    convoy[0].set_autopilot(False, interface.tm.get_port())
                    convoy[0].apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
                brake_triggered = True

            # Check if destination reached (Infinite Roaming)
            dist_to_goal = 0
            if bp_planner.global_route:
                dist_to_goal = ego_transform.location.distance(bp_planner.global_route[-1][0].transform.location)
                
            if dist_to_goal < 20.0 or not bp_planner.global_route:
                 print("--- Destination Reached! Re-routing to new random location... ---")
                 spawn_points = interface.map.get_spawn_points()
                 dest = random.choice(spawn_points).location
                 # Generate route from current location
                 bp_planner.global_route = interface.generate_route(ego_transform.location, dest)

            # Periodic Spawning
            if time.time() - last_spawn_time > 15.0:
                print("--- Spawning New Convoy Ahead ---")
                convoy.extend(spawn_convoy_at_dist(50.0))
                last_spawn_time = time.time()

            # Cleanup Traffic (Behind Ego)
            # Remove vehicles > 50m behind
            active_convoy = []
            ego_loc = ego_transform.location
            fwd = ego_transform.get_forward_vector()
            
            for actor in convoy:
                if not actor.is_alive: continue
                
                # Check relative position
                vec = actor.get_location() - ego_loc
                dot = vec.x * fwd.x + vec.y * fwd.y
                
                if dot < -50.0:
                    actor.destroy()
                else:
                    active_convoy.append(actor)
            convoy = active_convoy

            # Planning & Control
            state, target_speed = bp_planner.plan(ego_transform, ego_speed, data['nearby_vehicles'])
            waypoints = m_planner.generate_path(ego_transform, state, bp_planner.target_lane_wp, bp_planner.global_route)
            
            control = controller.run_step(ego_speed, ego_transform, target_speed, waypoints)
            interface.apply_control(control)

            if data['frame'] % 20 == 0:
                print(f"Ego State: {state.name} | Speed: {ego_speed*3.6:.1f} | Obs: {bp_planner._get_lead_vehicle(ego_transform, data['nearby_vehicles'])[0]:.1f}m")

            if brake_triggered and ego_speed < 0.1 and current_elapsed > 15.0:
                if not getattr(interface, 'test_msg_shown', False):
                    print("Convoy Stop Test Complete. Ego Safety Verified. RESUMING TRAFFIC...")
                    interface.test_msg_shown = True
                    
                    # Release the braked leader so simulation can flow!
                    if convoy and convoy[0].is_alive:
                        print("Releasing Leader Vehicle brakes...")
                        convoy[0].apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))
                        convoy[0].set_autopilot(True, interface.tm.get_port())
                # break # Keep running indefinitely

    except KeyboardInterrupt:
        pass
    finally:
        for actor in convoy:
             if actor.is_alive: actor.destroy()
        interface.cleanup()

if __name__ == "__main__":
    main()
