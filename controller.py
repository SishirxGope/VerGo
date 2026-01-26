import math
import numpy as np
import config

class VehicleController:
    def __init__(self):
        self._lon_controller = PIDLongitudinalController(
            config.PID_LONGITUDINAL['K_P'],
            config.PID_LONGITUDINAL['K_D'],
            config.PID_LONGITUDINAL['K_I'],
            config.PID_LONGITUDINAL['dt']
        )
        self._lat_controller = PurePursuitLateralController(
            config.PURE_PURSUIT_LOOKAHEAD
        )

    def run_step(self, current_speed, current_transform, target_speed, waypoints):
        """
        Computes control commands (throttle, brake, steer).
        
        Args:
            current_speed (float): m/s
            current_transform (carla.Transform): Vehicle pose
            target_speed (float): m/s
            waypoints (list): List of carla.Transform (the path)
            
        Returns:
            dict: {'throttle': float, 'brake': float, 'steer': float}
        """
        
        # Longitudinal Control
        acceleration = self._lon_controller.run_step(target_speed, current_speed)
        
        throttle = 0.0
        brake = 0.0
        
        if acceleration >= 0.0:
            throttle = min(acceleration, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(abs(acceleration), 1.0)
            
        # Lateral Control
        steer = self._lat_controller.run_step(current_transform, waypoints, current_speed)
        
        return {
            'throttle': throttle,
            'brake': brake,
            'steer': steer
        }

class PIDLongitudinalController:
    def __init__(self, K_P, K_D, K_I, dt):
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._dt = dt
        self._error_buffer = queue_buffer(maxlen=10)
        self._error_integral = 0.0

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

class PurePursuitLateralController:
    def __init__(self, lookahead_dist):
        self.lookahead_dist = lookahead_dist
        self.wheelbase = 2.8 # Approximate for Model 3

    def run_step(self, vehicle_transform, waypoints, current_speed):
        # 1. Find the target waypoint (closest point ahead by lookahead distance)
        # Assuming waypoints are ordered
        
        target_wp = None
        min_dist = float('inf')
        
        ego_loc = vehicle_transform.location
        
        # Simple search for the point comfortably ahead
        # Ideally, we should do arc-length search, but closest euclidean that is > lookahead works for basic paths
        
        for wp in waypoints:
            dist = wp.location.distance(ego_loc)
            # We want point closest to lookahead distance, but MUST be ahead?
            # Actually pure pursuit usually picks the intersection of the circle with the path.
            # Simplified: Pick first point > lookahead distance
            if dist > self.lookahead_dist:
                target_wp = wp
                break
        
        if target_wp is None and waypoints:
            target_wp = waypoints[-1] # End of path
        elif target_wp is None:
            return 0.0 # No path
            
        # 2. Convert to vehicle coordinates
        # Vector to target
        v_vec = np.array([target_wp.location.x - ego_loc.x, target_wp.location.y - ego_loc.y])
        
        # Vehicle Forward Vector
        yaw_rad = math.radians(vehicle_transform.rotation.yaw)
        f_vec = np.array([math.cos(yaw_rad), math.sin(yaw_rad)])
        
        # 3. Calculate alpha (angle between heading and target)
        # Cross product (2D) to get signed angle? 
        # Or rotate v_vec by -yaw
        
        # Rotate point to vehicle frame
        # x' = x cos(-t) - y sin(-t)
        # y' = x sin(-t) + y cos(-t)
        
        # local_x is forward, local_y is right (Handedness check: CARLA is LHS? No, RHS usually. Z up.)
        # Let's trust standard rotation matrix
        
        dx = target_wp.location.x - ego_loc.x
        dy = target_wp.location.y - ego_loc.y
        
        local_x = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
        local_y = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
        
        alpha = math.atan2(local_y, local_x)
        
        # 4. Steering angle command
        # delta = atan(2 * L * sin(alpha) / lookahead)
        steer = math.atan((2.0 * self.wheelbase * math.sin(alpha)) / self.lookahead_dist)
        
        # Normalize to [-1, 1] assuming max steer 70 deg (1.22 rad)
        steer = np.clip(steer / 1.22, -1.0, 1.0)
        
        return steer

class queue_buffer(list):
    def __init__(self, maxlen):
        self.maxlen = maxlen
    def append(self, item):
        list.append(self, item)
        if len(self) > self.maxlen:
            self.pop(0)
