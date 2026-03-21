import time
import math
import random
import carla
import config
from carla_interface import CarlaInterface
from planner import BehaviorPlanner, MotionPlanner, LocalGridPlanner
from controller import VehicleController
from perception import CameraPerceptionModule, PerceptionModule
from learning import ImitationLearner

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

        # RAIL Integration: Camera Perception
        camera_perception = CameraPerceptionModule() if config.CAMERA_PERCEPTION_ENABLED else None
        perception = PerceptionModule()

        # RAIL Integration: Local Grid Planner
        local_grid_planner = LocalGridPlanner() if config.LOCAL_GRID_ENABLED else None

        # RAIL Integration: Imitation Learning
        learner = ImitationLearner() if (config.LEARNING_ENABLED or config.LEARNING_COLLECT_DATA) else None
        
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
            
            # B. RAIL: Camera Perception Fusion
            nearby = data['nearby_vehicles']
            if camera_perception and 'camera_front' in data and data['camera_front'] is not None:
                try:
                    cam_detections = camera_perception.process(data['camera_front'], ego_transform)
                    if cam_detections:
                        nearby = perception.fuse_camera_detections(nearby, cam_detections)
                except Exception:
                    pass  # Camera perception failure should not stop driving

            # C. Planning
            # 1. Behavior (Decision)
            state, target_speed = behavior_planner.plan(
                ego_transform,
                ego_speed,
                nearby
            )

            # 2. Motion (Trajectory)
            waypoints = motion_planner.generate_path(
                ego_transform,
                state,
                behavior_planner.target_lane_wp,
                global_route=behavior_planner.global_route
            )

            # 3. RAIL: Local Grid Avoidance Override
            if local_grid_planner and nearby:
                try:
                    local_path = local_grid_planner.plan_local_path(ego_transform, nearby)
                    if local_path and waypoints:
                        # Blend: use local avoidance path for near-field, global for far-field
                        waypoints = local_path + waypoints[len(local_path):]
                except Exception:
                    pass  # Grid planner failure should not stop driving

            # D. Control
            control_cmd = controller.run_step(
                ego_speed,
                ego_transform,
                target_speed,
                waypoints
            )

            # E. RAIL: Imitation Learning (collect or blend)
            if learner:
                try:
                    state_vec = learner.extract_state(ego_speed, ego_transform, target_speed, nearby)
                    if config.LEARNING_COLLECT_DATA:
                        learner.record_step(state_vec, control_cmd)
                    elif config.LEARNING_ENABLED:
                        learned_cmd = learner.predict(state_vec)
                        control_cmd = ImitationLearner.blend_controls(control_cmd, learned_cmd)
                except Exception:
                    pass  # Learning failure should not stop driving

            # F. Actuation
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
        # Save collected driving data if in data-collection mode
        if learner and config.LEARNING_COLLECT_DATA:
            learner.save_data()
            print("Driving data saved for imitation learning training.")

        visualizer.cleanup()
        if lead_vehicle and lead_vehicle.is_alive:
            lead_vehicle.destroy()
            print("Lead vehicle destroyed.")
        interface.cleanup()

if __name__ == '__main__':
    main()
