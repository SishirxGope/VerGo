import math
import heapq
import numpy as np
import carla
import config
from enum import Enum

class BehaviorState(Enum):
    CRUISE = 1
    FOLLOW = 2
    PREPARE_OVERTAKE = 3
    CHASE = 4 # Unused
    EMERGENCY_STOP = 5
    CHANGE_LANE_LEFT = 6
    CHANGE_LANE_RIGHT = 7
    OVERTAKE = 8
    STOP_FOR_TL = 9

class BehaviorPlanner:
    def __init__(self, am_interface):
        self.map = am_interface.map
        self.interface = am_interface
        self.state = BehaviorState.CRUISE
        self.target_speed = config.TARGET_SPEED_KPH / 3.6
        self.target_lane_wp = None # For lane changes
        self.overtake_timer = 0
        self.lane_change_completed = False
        self.global_route = [] # List of (waypoint, road_option)
        self.overtake_victim_id = None # ID of the vehicle we are currently passing

    def plan(self, ego_transform, ego_speed, nearby_vehicles):
        """
        Determines the behavior state and target speed.
        """
        # 0. Observation
        closest_obs_dist, lead_vehicle = self._get_lead_vehicle(ego_transform, nearby_vehicles)
        ttc = float('inf')
        if ego_speed > 0.1:
            ttc = closest_obs_dist / ego_speed
            
        dist_to_goal = 0
        if self.global_route:
             dist_to_goal = ego_transform.location.distance(self.global_route[-1][0].transform.location)
             
        # Log status
        print(f"Nearby: {len(nearby_vehicles)}, Obs: {closest_obs_dist:.0f}m, State: {self.state.name}, Goal: {dist_to_goal:.0f}m")

        # 1. Recovery Logic (Transition out of Stop states)
        if self.state in [BehaviorState.EMERGENCY_STOP, BehaviorState.STOP_FOR_TL]:
            if not self._is_red_light():
                # If path is clear, cruise
                if closest_obs_dist > config.SAFE_FOLLOW_DISTANCE:
                    self.state = BehaviorState.CRUISE
                # If vehicle is there but at a safe distance to resume following/overtaking
                elif closest_obs_dist > 2.0: # Relaxed from MIN_FOLLOW_DISTANCE (5.0) to allow creeping
                    self.state = BehaviorState.FOLLOW
                    
        # 2. Traffic Light Logic (High Priority - Stop if Red/Yellow)
        if self.interface.is_at_traffic_light():
             tl_state = self.interface.get_traffic_light_state()
             if tl_state in [carla.TrafficLightState.Red, carla.TrafficLightState.Yellow]:
                 self.state = BehaviorState.STOP_FOR_TL
                 return self.state, 0.0
             elif self.state == BehaviorState.STOP_FOR_TL and tl_state == carla.TrafficLightState.Green:
                 self.state = BehaviorState.CRUISE # Recover

        # 3. Global Safety Check (Immediate stop if about to hit something)
        # Skip if we are already in a controlled stop state
        # Skip victim masking
        skip_states = [BehaviorState.CHANGE_LANE_LEFT, BehaviorState.CHANGE_LANE_RIGHT, BehaviorState.OVERTAKE]
        is_victim = lead_vehicle and (lead_vehicle.id == self.overtake_victim_id)
        
        if ttc < config.EMERGENCY_BRAKE_TTC and self.state not in [BehaviorState.STOP_FOR_TL]:
             if not (self.state in skip_states and is_victim):
                 self.state = BehaviorState.EMERGENCY_STOP
                 return self.state, 0.0

        # 4. State Machine Logic
        current_wp = self.interface.get_current_lane()
        
        # B. CRUISE / FOLLOW Logic
        if self.state in [BehaviorState.CRUISE, BehaviorState.FOLLOW]:
            # Overtake Trigger Logic
            if closest_obs_dist < config.SAFE_FOLLOW_DISTANCE:
                self.state = BehaviorState.FOLLOW
                
                # If speed is slow, consider overtaking
                if ego_speed < (config.TARGET_SPEED_MPS * 0.8):
                    # Check lanes
                    left_lane = self.interface.get_visual_left_lane(current_wp)
                    left_free = left_lane and self._is_lane_free(left_lane, ego_transform, nearby_vehicles)
                    
                    if left_free:
                        self.state = BehaviorState.PREPARE_OVERTAKE
                        self.target_lane_wp = left_lane
                        self.overtake_victim_id = lead_vehicle.id
                    else:
                        # Try right
                        right_lane = self.interface.get_visual_right_lane(current_wp)
                        right_free = right_lane and self._is_lane_free(right_lane, ego_transform, nearby_vehicles)
                        
                        if right_free:
                             self.state = BehaviorState.PREPARE_OVERTAKE
                             self.target_lane_wp = right_lane
                             self.overtake_victim_id = lead_vehicle.id
            else:
                self.state = BehaviorState.CRUISE
                
        # C. PREPARE_OVERTAKE
        elif self.state == BehaviorState.PREPARE_OVERTAKE:
            # Double check gap safety before committing
            if self.target_lane_wp and self._is_lane_free(self.target_lane_wp, ego_transform, nearby_vehicles):
                # Decide direction based on visual neighbors
                vis_left = self.interface.get_visual_left_lane(current_wp)
                if vis_left and vis_left.lane_id == self.target_lane_wp.lane_id:
                    self.state = BehaviorState.CHANGE_LANE_LEFT
                else:
                    self.state = BehaviorState.CHANGE_LANE_RIGHT
                          
                self.lane_change_completed = False
            else:
                self.state = BehaviorState.FOLLOW # Abort
                self.overtake_victim_id = None

        # D. LANE CHANGES
        elif self.state in [BehaviorState.CHANGE_LANE_RIGHT, BehaviorState.CHANGE_LANE_LEFT]:
            # Motion planner handles the trajectory.
            if current_wp.lane_id == self.target_lane_wp.lane_id:
                 print("DEBUG: Lane change complete. Entering Overtake Stabilization.")
                 self.state = BehaviorState.OVERTAKE

        # E. OVERTAKE MODE
        elif self.state == BehaviorState.OVERTAKE:
            # We must stay in this lane until the victim is well behind us
            victim_far_behind = True
            
            # Identify the original lane we want to return to
            route_lane_id = self.global_route[0][0].lane_id if self.global_route else None
            ego_loc = ego_transform.location
            fwd = ego_transform.get_forward_vector()
            
            for vehicle in nearby_vehicles:
                v_loc = vehicle.get_location()
                v_wp = self.map.get_waypoint(v_loc)
                
                # Check vehicles in the lane we want to return to
                if route_lane_id and v_wp.lane_id == route_lane_id:
                    vec = ego_loc - v_loc
                    long_dist = fwd.x * vec.x + fwd.y * vec.y
                    
                    # If vehicle is within 40m and ego is less than 18m ahead of it
                    if long_dist < 18.0 and ego_loc.distance(v_loc) < 40.0:
                        victim_far_behind = False
                        break
            
            if victim_far_behind:
                print("DEBUG: Overtake stabilization complete. Resuming Cruise.")
                self.state = BehaviorState.CRUISE
                self.overtake_victim_id = None
                  
        # 3. Compute Target Speed
        if self.state == BehaviorState.FOLLOW:
             if lead_vehicle:
                # Speed Matching + Gap Control
                lead_vel = lead_vehicle.get_velocity()
                lead_speed = math.sqrt(lead_vel.x**2 + lead_vel.y**2)
                
                # Simple P-control for gap
                desired_gap = config.MIN_FOLLOW_DISTANCE + 8.0 # Increased buffer
                gap_error = closest_obs_dist - desired_gap
                
                # Final target speed: lead_speed + gap correction
                self.target_speed = lead_speed + (0.7 * gap_error)
                
                # Safety: Don't force momentum if we are too close!
                if closest_obs_dist < 4.0:
                    self.target_speed = max(0.0, self.target_speed)
                else:
                    self.target_speed = max(2.0, self.target_speed) # Maintain momentum otherwise
             else:
                  self.target_speed = config.TARGET_SPEED_MPS
        elif self.state in [BehaviorState.OVERTAKE, BehaviorState.CHANGE_LANE_RIGHT, BehaviorState.CHANGE_LANE_LEFT]:
             self.target_speed = config.TARGET_SPEED_MPS * 1.2 # Assertive passing speed
        else:
             self.target_speed = config.TARGET_SPEED_MPS

        return self.state, self.target_speed

    def _get_lead_vehicle(self, ego_transform, nearby_vehicles):
        """
        Detects the closest vehicle along the global route.
        """
        min_dist = float('inf')
        lead_vehicle = None
        ego_loc = ego_transform.location
        
        # 1. Check against Global Route (Horizon: 60m)
        if self.global_route:
            # Find index of closest point on route to start search
            start_idx = 0
            min_d_to_route = float('inf')
            for i, (wp, _) in enumerate(self.global_route):
                d = wp.transform.location.distance(ego_loc)
                if d < min_d_to_route:
                    min_d_to_route = d
                    start_idx = i
                if d > 10.0 and min_d_to_route < 10.0: break # Optimisation 
            
            # Search ahead from current point
            search_horizon = 60.0
            accum_dist = 0
            
            for i in range(start_idx, len(self.global_route)-1):
                p1 = self.global_route[i][0].transform.location
                p2 = self.global_route[i+1][0].transform.location
                segment_dist = p1.distance(p2)
                accum_dist += segment_dist
                
                if accum_dist > search_horizon: break
                
                # Check each nearby vehicle against this route point
                for vehicle in nearby_vehicles:
                    # Skip the vehicle we are intentionally passing
                    skip_states = [BehaviorState.CHANGE_LANE_LEFT, BehaviorState.CHANGE_LANE_RIGHT, BehaviorState.OVERTAKE]
                    if self.state in skip_states and vehicle.id == self.overtake_victim_id:
                        continue

                    v_loc = vehicle.get_location()
                    
                    # Lane-Aware Filtering: Only care if vehicle is in our "Target Corridor"
                    # If we are in CHANGE_LANE or OVERTAKE, we care about the target lane.
                    # Otherwise we care about the route lane.
                    target_lane_id = None
                    if self.state in skip_states and self.target_lane_wp:
                        target_lane_id = self.target_lane_wp.lane_id
                    else:
                        target_lane_id = self.global_route[i+1][0].lane_id
                    
                    v_wp = self.map.get_waypoint(v_loc)
                    if v_wp.lane_id != target_lane_id:
                        continue

                    # If vehicle is within 5m of any waypoint in our future corridor
                    if p2.distance(v_loc) < 5.0:
                        if accum_dist < min_dist:
                            min_dist = accum_dist
                            lead_vehicle = vehicle
        
        # 2. Emergency Backup (Local Box)
        # Handle vehicles entering from side or very close range before route logic catches them
        if min_dist > 10.0:
            for vehicle in nearby_vehicles:
                # Skip the vehicle we are intentionally passing
                skip_states = [BehaviorState.CHANGE_LANE_LEFT, BehaviorState.CHANGE_LANE_RIGHT, BehaviorState.OVERTAKE]
                if self.state in skip_states:
                    if vehicle.id == self.overtake_victim_id:
                        continue
                    if getattr(self, 'target_lane_wp', None):
                        v_wp = self.map.get_waypoint(vehicle.get_location())
                        if v_wp.lane_id != self.target_lane_wp.lane_id:
                            continue

                dx = vehicle.get_location().x - ego_loc.x
                dy = vehicle.get_location().y - ego_loc.y
                yaw_rad = math.radians(ego_transform.rotation.yaw)
                c, s = math.cos(yaw_rad), math.sin(yaw_rad)
                local_x = dx * c + dy * s
                local_y = -dx * s + dy * c
                
                # Robust corridor: only brake if it's physically in the way
                if local_x > 0 and local_x < 12.0 and abs(local_y) < 1.5:
                    if local_x < min_dist:
                        min_dist = local_x
                        lead_vehicle = vehicle

        return min_dist, lead_vehicle

    def _is_red_light(self):
        if self.interface.is_at_traffic_light():
             tl_state = self.interface.get_traffic_light_state()
             return tl_state == carla.TrafficLightState.Red
        return False
        
    def _is_lane_free(self, target_lane_wp, ego_transform, nearby_vehicles, front_margin=20.0, rear_margin=15.0):
        """Checks if the target lane is free of vehicles within margins."""
        target_lane_id = target_lane_wp.lane_id
        ego_loc = ego_transform.location
        
        for vehicle in nearby_vehicles:
            veh_wp = self.map.get_waypoint(vehicle.get_location())
            if veh_wp.lane_id == target_lane_id:
                # Check longitudinal distance
                dist = vehicle.get_location().distance(ego_loc)
                
                # Check if front or rear
                fwd = ego_transform.get_forward_vector()
                vec = vehicle.get_location() - ego_loc
                dot = fwd.x * vec.x + fwd.y * vec.y
                
                if dot > 0 and dist < front_margin: return False # Blocked in front
                if dot < 0 and dist < rear_margin: return False # Blocked in rear
                
        return True

class MotionPlanner:
    def __init__(self, am_map):
        self.map = am_map
    
    def generate_path(self, ego_transform, behavior_state, target_lane_wp=None, global_route=None, horizon=50):
        """
        Professionally anchors EVERY path to the Global Route. 
        """
        waypoints = []
        ego_loc = ego_transform.location
        
        if not global_route:
            return []

        # 1. Find the current segment on the global route
        start_idx = 0
        min_dist = float('inf')
        for i in range(len(global_route)):
            d = global_route[i][0].transform.location.distance(ego_loc)
            if d < min_dist:
                min_dist = d
                start_idx = i
            if d > 15.0 and min_dist < 15.0: break

        # 2. Project path ahead from route
        accum_dist = 0
        for i in range(start_idx, len(global_route)-1):
            p_wp = global_route[i][0]
            accum_dist += 2.0 
            if accum_dist > horizon: break
            
            # 3. Determine Lateral Shift based on BehaviorState
            if behavior_state in [BehaviorState.CRUISE, BehaviorState.FOLLOW]:
                waypoints.append(p_wp.transform)
                
            elif behavior_state in [BehaviorState.CHANGE_LANE_RIGHT, BehaviorState.CHANGE_LANE_LEFT, BehaviorState.OVERTAKE]:
                if not target_lane_wp:
                    waypoints.append(p_wp.transform)
                    continue
                
                target_id = target_lane_wp.lane_id
                
                # Find the neighbor of the ROUTE waypoint
                shift_wp = None
                r = p_wp.get_right_lane()
                l = p_wp.get_left_lane()
                
                if r and r.lane_id == target_id:
                    shift_wp = r
                elif l and l.lane_id == target_id:
                    shift_wp = l
                else:
                    shift_wp = p_wp # Fallback
                
                waypoints.append(shift_wp.transform)

        return waypoints


class LocalGridPlanner:
    """
    Local occupancy-grid planner adapted from RAIL/planning_module.py.
    Creates a small grid around the ego vehicle, marks obstacles, inflates them
    for safety, and runs A* to find a collision-free local path.
    """
    def __init__(self):
        self.grid_rows = config.LOCAL_GRID_ROWS       # cells in forward direction
        self.grid_cols = config.LOCAL_GRID_COLS       # cells in lateral direction
        self.cell_size = config.LOCAL_GRID_CELL_SIZE  # meters per cell
        self.inflate_cells = config.LOCAL_GRID_INFLATE_CELLS  # obstacle inflation radius

        # Grid covers: forward = grid_rows * cell_size, lateral = grid_cols * cell_size
        # Ego is at (grid_rows - 1, grid_cols // 2)  — bottom center

    def create_grid(self, ego_transform, detected_objects):
        """
        Builds a 2D occupancy grid in ego-local frame.
        0 = free, 1 = occupied.
        """
        grid = np.zeros((self.grid_rows, self.grid_cols), dtype=np.int8)

        ego_loc = ego_transform.location
        yaw_rad = math.radians(ego_transform.rotation.yaw)
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)

        for obj in detected_objects:
            obj_loc = obj.get_location()
            # World to ego-local
            dx = obj_loc.x - ego_loc.x
            dy = obj_loc.y - ego_loc.y
            local_x = dx * cos_y + dy * sin_y   # forward
            local_y = -dx * sin_y + dy * cos_y  # left-positive

            # Convert to grid indices
            row = self.grid_rows - 1 - int(local_x / self.cell_size)
            col = self.grid_cols // 2 + int(local_y / self.cell_size)

            if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
                grid[row][col] = 1

        # Inflate obstacles for vehicle width safety margin
        if self.inflate_cells > 0:
            grid = self._inflate(grid)

        return grid

    def _inflate(self, grid):
        """Dilates obstacles by inflate_cells radius."""
        inflated = grid.copy()
        rows, cols = grid.shape
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == 1:
                    for dr in range(-self.inflate_cells, self.inflate_cells + 1):
                        for dc in range(-self.inflate_cells, self.inflate_cells + 1):
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < rows and 0 <= nc < cols:
                                inflated[nr][nc] = 1
        return inflated

    def astar(self, grid, start, goal):
        """
        A* search on occupancy grid. Adapted from RAIL/planning_module.py
        with added diagonal movement (8-connected) for smoother paths.
        """
        rows, cols = grid.shape
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g = {start: 0}

        # 8-connected neighbors (diagonals cost sqrt(2))
        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                      (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dr, dc, cost in neighbors:
                nr, nc = current[0] + dr, current[1] + dc
                neighbor = (nr, nc)

                if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                    temp_g = g[current] + cost
                    if neighbor not in g or temp_g < g[neighbor]:
                        came_from[neighbor] = current
                        g[neighbor] = temp_g
                        h = math.sqrt((goal[0] - nr)**2 + (goal[1] - nc)**2)
                        heapq.heappush(open_set, (temp_g + h, neighbor))

        return []  # No path found

    def plan_local_path(self, ego_transform, detected_objects):
        """
        Main entry point: creates grid, runs A*, converts path to world waypoints.
        Returns list of carla.Transform representing the local avoidance path.
        Returns empty list if no avoidance needed or path cannot be found.
        """
        # Safety: too many detections likely means false positives
        if len(detected_objects) > 15:
            return []

        grid = self.create_grid(ego_transform, detected_objects)

        # Start: ego position (bottom center of grid)
        start = (self.grid_rows - 1, self.grid_cols // 2)
        # Goal: top center (farthest forward point)
        goal = (0, self.grid_cols // 2)

        # Only plan if there are obstacles in the grid
        obstacle_count = np.sum(grid == 1)
        if obstacle_count == 0:
            return []  # No obstacles, let global planner handle it

        # Safety: if too much of the grid is occupied, likely false positives
        grid_total = self.grid_rows * self.grid_cols
        if obstacle_count > grid_total * 0.3:
            return []  # More than 30% occupied = probably noise

        # Safety: if start or goal is blocked, can't plan
        if grid[start[0]][start[1]] == 1 or grid[goal[0]][goal[1]] == 1:
            return []

        try:
            path = self.astar(grid, start, goal)
        except Exception:
            return []

        if not path or len(path) < 3:
            return []

        # Convert grid path back to world coordinates
        ego_loc = ego_transform.location
        yaw_rad = math.radians(ego_transform.rotation.yaw)
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)

        world_path = []
        for row, col in path[::3]:  # Subsample every 3rd point for smoothness
            local_x = (self.grid_rows - 1 - row) * self.cell_size
            local_y = (col - self.grid_cols // 2) * self.cell_size

            world_x = ego_loc.x + local_x * cos_y - local_y * sin_y
            world_y = ego_loc.y + local_x * sin_y + local_y * cos_y

            wp_transform = carla.Transform(
                carla.Location(world_x, world_y, ego_loc.z),
                carla.Rotation(0, ego_transform.rotation.yaw, 0)
            )
            world_path.append(wp_transform)

        return world_path
