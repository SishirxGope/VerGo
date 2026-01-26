import math
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
            if self.overtake_victim_id:
                for vehicle in nearby_vehicles:
                    if vehicle.id == self.overtake_victim_id:
                        v_loc = vehicle.get_location()
                        ego_loc = ego_transform.location
                        # Check longitudinal distance (Ego in front of Victim)
                        fwd = ego_transform.get_forward_vector()
                        vec = ego_loc - v_loc
                        long_dist = fwd.x * vec.x + fwd.y * vec.y
                        
                        if long_dist < 30.0: # Professional lead: 30m before clearing
                            victim_far_behind = False
            
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
                if self.state in skip_states and vehicle.id == self.overtake_victim_id:
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
