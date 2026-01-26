import carla
import time
import random
import config
from carla_interface import CarlaInterface
from visualization import SensorVisualizer

def main():
    print("--- Sensor Visualization Mode ---")
    interface = CarlaInterface()
    visualizer = SensorVisualizer()
    
    try:
        # 1. Setup
        interface.connect()
        interface.setup_world()
        
        # 2. Spawn Ego & Sensors
        interface.spawn_ego_vehicle()
        interface.spawn_sensors()
        interface.world.tick()
        
        # 3. Enable Autopilot for motion
        interface.ego_vehicle.set_autopilot(True, interface.tm.get_port())
        print("Autopilot engaged. Visualizer starting...")
        
        # 4. Spawning some traffic for radar/lidar to see
        # Spawn a vehicle in front
        ego_wp = interface.map.get_waypoint(interface.ego_vehicle.get_location())
        spawn_ahead = ego_wp.next(30.0)
        if spawn_ahead:
            bp = interface.world.get_blueprint_library().find('vehicle.nissan.patrol')
            interface.world.try_spawn_actor(bp, spawn_ahead[0].transform)
            print("Spawned dummy vehicle 30m ahead.")

        while True:
            # Get Data
            data = interface.get_data()
            if not data: continue
            
            # Retrieve specific sensor data
            # Note: carla_interface.get_data() puts sensor data in data dict by name
            # Names in config: 'lidar', 'radar', 'camera_front'
            
            # Render
            keep_running = visualizer.render(data)
            if not keep_running:
                break
                
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        visualizer.cleanup()
        interface.cleanup()

if __name__ == "__main__":
    main()
