import carla
import config

def main():
    client = carla.Client(config.CARLA_HOST, config.CARLA_PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    m = world.get_map()
    
    spawn_points = m.get_spawn_points()
    print(f"Total Spawn Points: {len(spawn_points)}")
    
    unique_lane_ids = set()
    
    print("\n--- Spawn Point Analysis ---")
    for i, sp in enumerate(spawn_points):
        wp = m.get_waypoint(sp.location)
        lane_id = wp.lane_id
        unique_lane_ids.add(lane_id)
        
        # Check neighbors
        left = wp.get_left_lane()
        right = wp.get_right_lane()
        
        has_left = left and left.lane_type == carla.LaneType.Driving
        has_right = right and right.lane_type == carla.LaneType.Driving
        
        print(f"Idx {i}: Lane {lane_id} | Left: {has_left} (ID: {left.lane_id if has_left else 'None'}) | Right: {has_right} (ID: {right.lane_id if has_right else 'None'})")
        
    print(f"\nUnique Lane IDs found: {unique_lane_ids}")

if __name__ == '__main__':
    main()
