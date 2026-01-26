import time
import math
import random
import carla
import config
from carla_interface import CarlaInterface
from planner import BehaviorPlanner, MotionPlanner
from controller import VehicleController

from visualization import SensorVisualizer

def main():
    print("Starting India-Mode Autonomy Stack (Phase 1)...")
    
    interface = CarlaInterface()
    visualizer = SensorVisualizer()
    lead_vehicle = None
    
    try:
        # 1. Setup
        interface.connect()
        interface.setup_world()
        
        # 2. Spawn Ego
        interface.spawn_ego_vehicle(spawn_point_index=0)
        interface.spawn_sensors()
        interface.world.tick() # Ensure transform updates before lead spawn
        
        # 2b. Generate Global Route (Phase 3)
        spawn_points = interface.map.get_spawn_points()
        # Pick a random destination
        destination = random.choice(spawn_points).location
        
        # Helper to spawn lead
        def spawn_lead_vehicle(dist=20.0):
             ego_wp = interface.map.get_waypoint(interface.ego_vehicle.get_location())
             next_wps = ego_wp.next(dist)
             if not next_wps:
                 print("Warning: Road ends, cannot spawn lead vehicle.")
                 return None
                 
             lead_wp = next_wps[0]
             lead_trans = lead_wp.transform
             lead_trans.location.z += 1.0 # Safety height
             
             bp = interface.world.get_blueprint_library().find('vehicle.nissan.patrol')
             lead = interface.world.try_spawn_actor(bp, lead_trans)
             
             if lead:
                 lead.set_autopilot(True, interface.tm.get_port())
                 interface.tm.vehicle_percentage_speed_difference(lead, 60.0)
                 interface.tm.ignore_lights_percentage(lead, 0.0)
                 loc = lead.get_location()
                 print(f"SUCCESS: Lead Vehicle {lead.id} spawned at {loc}")
             else:
                 print(f"FAILED: Lead Vehicle spawn failed at {lead_trans.location}")
             return lead

        # Initial Spawn
        lead_vehicle = spawn_lead_vehicle(15.0)
        spawn_points = interface.map.get_spawn_points()
        # Pick a random destination
        destination = random.choice(spawn_points).location
        
        print(f"Generating Route to Random Destination: {destination}")
        route = interface.generate_route(interface.ego_vehicle.get_location(), destination)
        print(f"Route generated with {len(route)} waypoints.")
        
        # 3. Setup Modules
        behavior_planner = BehaviorPlanner(interface) # Pass full interface
        behavior_planner.global_route = route
        
        motion_planner = MotionPlanner(interface.map)
        controller = VehicleController()
        
        # 4. Main Loop
        print("Starting Simulation Loop...")
        while True:
            # A. Perception / Data Retrieval
            data = interface.get_data()
            if not data:
                continue
            
            # Update Visualization
            if not visualizer.render(data):
                print("Visualizer closed by user.")
                break
            
            ego_transform = data['ego_transform']
            ego_vel = data['ego_velocity']
            ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)
            
            # Check Goal Reached
            if behavior_planner.global_route:
                 d_goal = ego_transform.location.distance(behavior_planner.global_route[-1][0].transform.location)
                 if d_goal < 5.0:
                     print("DESTINATION REACHED!")
                     break
            
            # B. Planning
            # 1. Behavior (Decision)
            state, target_speed = behavior_planner.plan(
                ego_transform, 
                ego_speed, 
                data['nearby_vehicles']
            )
            
            # 2. Motion (Trajectory)
            waypoints = motion_planner.generate_path(
                ego_transform, 
                state, 
                behavior_planner.target_lane_wp,
                global_route=behavior_planner.global_route
            )
            
            # C. Control
            control_cmd = controller.run_step(
                ego_speed, 
                ego_transform, 
                target_speed, 
                waypoints
            )
            
            # D. Actuation
            interface.apply_control(control_cmd)
            
            # E. Stress Test: Respawn Lead Intermittently
            if data['frame'] % 1200 == 0:
                 print(f"STRESS TEST: Respawning Lead Vehicle...")
                 # Destroy old one to prevent duplicates if it survived
                 if lead_vehicle and lead_vehicle.is_alive:
                      lead_vehicle.destroy()
                 
                 # Spawn new one using robust logic
                 lead_vehicle = spawn_lead_vehicle(20.0)
                 if lead_vehicle:
                      # Reset physics just in case
                      lead_vehicle.set_target_velocity(carla.Vector3D(0,0,0))
                 # Reset speed
                 lead_vehicle.set_target_velocity(carla.Vector3D(0,0,0))
            
            # Debug Output
            if data['frame'] % 20 == 0:
                print(f"Frame: {data['frame']} | State: {state.name} | Speed: {ego_speed*3.6:.1f} km/h | Target: {target_speed*3.6:.1f} km/h | Steer: {control_cmd['steer']:.2f}")

    except KeyboardInterrupt:
        print("\nUser interrupted.")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        visualizer.cleanup()
        if lead_vehicle and lead_vehicle.is_alive:
            lead_vehicle.destroy()
            print("Lead vehicle destroyed.")
        interface.cleanup()

if __name__ == '__main__':
    main()
