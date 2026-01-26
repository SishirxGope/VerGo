import time
import math
import random
import carla
import config
import numpy as np
from carla_interface import CarlaInterface

def main():
    print("--- Aggressive Weather Sensor Test (Rain & Fog) ---")
    
    # 1. Modify Configuration for Realistic Noise BEFORE Interface Init
    # LiDAR: Atmosphere Attenuation (Fog/Rain effect) + Dropoff (Rain)
    # Note: 'atmosphere_attenuation_rate' requires CARLA 0.9.13+. If older, we rely on 'dropoff' and 'noise_stddev'.
    # We will try setting standard noise params.
    
    print("Configuring Sensors for Adverse Weather...")
    
    # Add noise params to config.SENSORS in memory
    # config.SENSORS['lidar']['atmosphere_attenuation_rate'] = 0.04 # 0.004 is varying, 0.04 is heavy
    # config.SENSORS['lidar']['noise_stddev'] = 0.05 # 5cm noise
    # config.SENSORS['lidar']['dropoff_general_rate'] = 0.45 # Drop points in rain
    # config.SENSORS['lidar']['dropoff_intensity_limit'] = 0.8
    # config.SENSORS['lidar']['dropoff_zero_intensity'] = 0.4
    
    # Radar: Add noise
    # config.SENSORS['radar']['noise_type'] = 'Ricean' # or Normal
    # config.SENSORS['radar']['noise_range'] = 2.0 # meters error
    
    interface = CarlaInterface()
    
    try:
        # 2. Setup World
        interface.connect()
        interface.setup_world()
        
        # 3. Set Aggressive Weather
        weather = carla.WeatherParameters(
            cloudiness=100.0,
            precipitation=100.0, # Heavy Rain
            sun_altitude_angle=10.0, # Gloomy
            fog_density=50.0,
            fog_distance=10.0, # Approx visibility 10m
            wetness=100.0
        )
        interface.world.set_weather(weather)
        print("Weather set to: HEAVY RAIN, FOG, WETNESS")

        # 4. Spawn Ego Vehicle
        spawn_points = interface.map.get_spawn_points()
        # Pick a straight road logic if possible, or just index 10 like before
        interface.spawn_ego_vehicle(spawn_point_index=10) 
        interface.spawn_sensors()
        interface.world.tick()
        
        # 5. Spawn Static Target 15m Ahead
        ego_wp = interface.map.get_waypoint(interface.ego_vehicle.get_location())
        target_wps = ego_wp.next(15.0)
        target_actor = None
        
        if target_wps:
            trans = target_wps[0].transform
            trans.location.z += 0.5
            bp = interface.world.get_blueprint_library().find('vehicle.nissan.patrol')
            bp.set_attribute('color', '255,0,0')
            target_actor = interface.world.try_spawn_actor(bp, trans)
            if target_actor:
                print(f"Spawned TARGET VEHICLE (Nissan Patrol) at 15m ahead.")
            else:
                print("Failed to spawn target vehicle.")
        else:
             print("Could not find waypoint 15m ahead.")

        # 6. Initialize Autonomy Stack
        from planner import BehaviorPlanner, MotionPlanner
        from controller import VehicleController
        
        bp_planner = BehaviorPlanner(interface)
        spawn_points = interface.map.get_spawn_points()
        dest = random.choice(spawn_points).location
        bp_planner.global_route = interface.generate_route(interface.ego_vehicle.get_location(), dest)
        
        m_planner = MotionPlanner(interface.map)
        controller = VehicleController()
        
        print("\nStarting Autonomous Overtake Test (Press Ctrl+C to stop)...")
        print("-" * 60)
        
        while True:
            data = interface.get_data()
            if not data: continue
            
            # --- Sensor Analysis (Background) ---
            lidar_data = data.get('lidar')
            radar_data = data.get('radar')
            
            lidar_target_hits = 0
            if lidar_data:
                points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0] / 4), 4))
                # Filter for target ahead (approx location relative to moving ego)
                # This is tricky as ego moves. We just log general counts.
                lidar_target_hits = points.shape[0]

            radar_detected = False
            if radar_data:
                for detect in radar_data:
                    if detect.depth < 20.0: radar_detected = True

            # --- Autonomous Control ---
            ego_transform = data['ego_transform']
            ego_vel = data['ego_velocity']
            ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)
            
            # Plan
            state, target_speed = bp_planner.plan(ego_transform, ego_speed, data['nearby_vehicles'])
            
            # Route Refresh (Infinite)
            dist_to_goal = 0
            if bp_planner.global_route:
                dist_to_goal = ego_transform.location.distance(bp_planner.global_route[-1][0].transform.location)
            if dist_to_goal < 20.0 or not bp_planner.global_route:
                 dest = random.choice(spawn_points).location
                 bp_planner.global_route = interface.generate_route(ego_transform.location, dest)
            
            # Generate Path & Control
            waypoints = m_planner.generate_path(ego_transform, state, bp_planner.target_lane_wp, bp_planner.global_route)
            control = controller.run_step(ego_speed, ego_transform, target_speed, waypoints)
            interface.apply_control(control)

            # Log
            if data['frame'] % 20 == 0:
                lead_dist, _ = bp_planner._get_lead_vehicle(ego_transform, data['nearby_vehicles'])
                print(f"[Frame {data['frame']}] State: {state.name} | Speed: {ego_speed*3.6:.1f} km/h | Lead Dist: {lead_dist:.1f}m")
                print(f"  Sensors: LiDAR Pts={lidar_target_hits} | Radar={'YES' if radar_detected else 'NO'}")
                print("-" * 20)

            
    except KeyboardInterrupt:
        pass
    finally:
        if target_actor and target_actor.is_alive:
            target_actor.destroy()
        interface.cleanup()

if __name__ == "__main__":
    main()
