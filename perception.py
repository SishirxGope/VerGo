import numpy as np
import carla
from sklearn.cluster import DBSCAN
import math

class DetectedObject:
    """
    A wrapper class that mimics the interface of a carla.Actor (Vehicle),
    but is constructed from raw sensor data.
    """
    def __init__(self, obj_id, location, velocity=None, bbox=None):
        self.id = obj_id
        self.location = location # carla.Location
        self.velocity = velocity if velocity else carla.Vector3D(0, 0, 0)
        self.bbox = bbox # carla.BoundingBox (optional)
        
        # Mimic attributes
        self.type_id = "vehicle.unknown" # Placeholder
        
    def get_location(self):
        return self.location

    def get_velocity(self):
        return self.velocity

    def get_transform(self):
        # We might not know rotation from a simple point cluster, 
        # but we can try to estimate it from velocity or just default to 0.
        # For now, 0 rotation is safer than noise.
        rotation = carla.Rotation(0, 0, 0)
        if hasattr(self.velocity, 'x') and (abs(self.velocity.x) > 0.5 or abs(self.velocity.y) > 0.5):
             yaw = math.degrees(math.atan2(self.velocity.y, self.velocity.x))
             rotation = carla.Rotation(0, yaw, 0)
             
        return carla.Transform(self.location, rotation)
    
    def get_acceleration(self):
        return carla.Vector3D(0,0,0) # Not estimated yet

class PerceptionModule:
    def __init__(self):
        # DBSCAN Parameters
        # eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
        # vehicles are typically 2-5 meters long. Points on a vehicle should be within 0.5m - 1.0m of each other.
        self.dbscan = DBSCAN(eps=1.5, min_samples=5)
        self.next_obj_id = 1000 # Start IDs high to avoid conflict with CARLA actors if mixed
        
        # Track previous objects for velocity estimation if Radar fails (Simple Tracking)
        # But for now we rely on Radar for velocity.
        pass

    def process(self, lidar_data, radar_data, ego_transform):
        """
        Main pipeline:
        1. Process LiDAR -> Clusters (Potential Objects)
        2. Process Radar -> Velocity Measurements
        3. Fusion -> Assign Velocity to Clusters
        4. Output -> List of DetectedObject
        """
        detected_objects = []
        
        # --- 1. LiDAR Processing ---
        if not lidar_data:
            return []

        # Convert buffer to numpy (N, 4) [x, y, z, intensity]
        points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        
        # Coordinate System:
        # LiDAR points are in SENSOR Frame.
        # We need them in WORLD Frame for the planner to work easily? 
        # OR the planner handles everything relative?
        # Planner uses `vehicle.get_location()` which returns WORLD coordinates.
        # So we MUST convert Sensor Points -> World Points.
        
        # 1.1 Transform to World
        # Get sensor transform relative to world is hard because lidar_data doesn't store it explicitly per frame easily
        # without looking up the actor. 
        # But `lidar_data.transform` gives the transform of the sensor at that timestamp!
        sensor_transform = lidar_data.transform
        
        # Apply transformation
        # CARLA Python API doesn't have a fast batch transform for numpy.
        # We have to do it manually or assume Planner can work in Ego Frame.
        # Planner `_get_lead_vehicle` compares distances.
        # Let's converting to World Frame is safest to match `get_location()`.
        
        # Transform Matrix (4x4)
        sensor_matrix = self.get_matrix(sensor_transform)
        
        # Filter: Downsample for speed?
        points = points[::5] # 1/5th of points
        
        # Filter: Ground Plane Removal (in Sensor Frame, Z is up)
        # Lidar is at Z=2.4m. Ground should be around Z=-2.4.
        # Let's say anything below -1.5 is ground/road.
        mask = points[:, 2] > -2.0 # Keep objects above ground
        obstacle_points = points[mask]

        if obstacle_points.shape[0] < 5:
            return []
            
        # Extract XYZ only
        xyz = obstacle_points[:, :3]
        
        # 1.2 Clustering
        labels = self.dbscan.fit_predict(xyz)
        unique_labels = set(labels)
        
        clusters = []
        for label in unique_labels:
            if label == -1: continue # Noise
            
            cluster_points = xyz[labels == label]
            
            # Centroid in Sensor Frame
            centroid_local = np.mean(cluster_points, axis=0) # [x, y, z]
            
            # Simple Bounding Box check
            # Discard if too small (noise) or too huge (walls?)
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            dims = max_pt - min_pt
            
            if dims[0] < 0.2 and dims[1] < 0.2: continue # Too small
            
            clusters.append({
                'centroid': centroid_local,
                'points': cluster_points
            })

        # --- 2. Radar Processing ---
        # Radar data gives us detections with Velocity in Sensor Frame.
        radar_detections = []
        if radar_data:
            # radar_data is carla.RadarMeasurement
            for detect in radar_data:
                # depth, azimuth, altitude, velocity
                # detect.velocity is radial velocity (m/s).
                
                # Convert to local Cartesian
                azi = detect.azimuth
                alt = detect.altitude
                depth = detect.depth
                
                x = depth * math.cos(azi) * math.cos(alt)
                y = depth * math.sin(azi) * math.cos(alt)
                z = depth * math.sin(alt)
                
                radar_detections.append({
                    'pos': np.array([x, y, z]),
                    'velocity': detect.velocity # Scalar radial velocity
                })

        # --- 3. Fusion (Deep Association) ---
        for cluster in clusters:
            c_pos = cluster['centroid'] # Local xyz
            
            # Find closest radar point
            best_vel = 0.0
            min_dist = 2.0 # Association Radius (2 meters)
            
            for r in radar_detections:
                r_pos = r['pos']
                dist = np.linalg.norm(c_pos - r_pos)
                
                if dist < min_dist:
                    min_dist = dist
                    best_vel = r['velocity']
            
            # Construct Velocity Vector
            # Radar only gives Radial velocity (towards/away).
            # We assume the object handles roughly along the line of sight or just use it as magnitude?
            # Actually, `radar.velocity` in CARLA is "relative velocity". 
            # If we are following a car at same speed, rel_vel ~ 0.
            # Planner expects ABSOLUTE velocity for some calculations?
            # Planner: `lead_speed = math.sqrt(lead_vel.x**2 + ...)`
            # Then self.target_speed = lead_speed.
            # So we need ABSOLUTE speed of the target.
            # Absolute Speed = Ego Speed + Relative Speed.
            
            # Get Ego Velocity Vector (World Frame)
            ego_v_vec = ego_transform.get_forward_vector()
            # Approximation: Ego Speed scalar
            # We don't have ego speed passed in explicitly, but can derive from ego_transform if we had history,
            # but usually we assume `process` is called with access to ego info. 
            # passed in `ego_transform` doesn't have velocity.
            
            # FIX: We need Ego Velocity to compute Absolute Velocity.
            # Ideally passed in `process` arguments. 
            # For now, let's look at relative velocity.
            # If Radar says -5 m/s, it means it's coming towards us (or we are catching up) at 5 m/s.
            # If we cruise at 30 km/h (8.3 m/s) and radar says -2 m/s, target is going 6.3 m/s.
            
            # We need EGO SPEED. 
            # I will modify the input to include `ego_velocity`.
            pass 
            
            # --- 4. Create Object (World Frame) ---
            # Transform Centroid Local -> World
            # P_world = Transform * P_local
            # carla.Transform.transform(carla.Location)
            
            c_loc_local = carla.Location(float(c_pos[0]), float(c_pos[1]), float(c_pos[2]))
            sensor_transform.transform(c_loc_local) # Transforms in-place
            c_loc_world = c_loc_local
            
            # Create Detection
            obj_id = self.next_obj_id
            self.next_obj_id += 1
            
            # Store Relative Velocity for now, and we will patch Absolute Velocity outside or guess it
            # Actually, let's keep it simple.
            # Allow logic later to patch it.
            # We store the detected object.
            
            obj = DetectedObject(obj_id, c_loc_world)
            # Temporary storage of relative velocity for the interface to patch
            obj.relative_velocity = best_vel 
            
            detected_objects.append(obj)
            
        return detected_objects

    def get_matrix(self, transform):
        """Creates a 4x4 transformation matrix from a carla.Transform."""
        # This is expensive in Python. Use CARLA's transform method on individual Points if Count < 1000? 
        # Or just use the native `transform(location)` one by one for Centroids.
        # Since we only transform Centroids (few objects), we don't need numpy-batch transform.
        pass

