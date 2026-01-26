import carla
import queue
import time
import random
import math
import config
import heapq

# Custom A* Global Router to avoid dependency issues

class GlobalRouter:
    def __init__(self, map_obj, sampling_resolution=2.0):
        self.map = map_obj
        self.sampling_resolution = sampling_resolution
        self.topology = self._build_topology()
        
    def _build_topology(self):
        """Builds a directed graph from the map topology."""
        topology = []
        # get_topology returns list of (wp, wp) tuples
        for segment in self.map.get_topology():
            wp1, wp2 = segment
            l1 = wp1.transform.location
            l2 = wp2.transform.location
            dist = l1.distance(l2)
            topology.append((wp1, wp2, dist))
        return topology

    def _get_successors(self, node_wp):
        """Returns reachable next waypoints."""
        successors = []
        # In a real implementation we would optimize this lookup
        # For small maps, linear scan is okay-ish, but let's use next() if possible
        # Check standard next
        next_wps = node_wp.next(self.sampling_resolution)
        for n in next_wps:
             successors.append((n, self.sampling_resolution))
             
        # Check lane changes (optional for global route, but good for connectivity)
        # Using topology is better for intersections
        # Ideally we map the topology graph. For simplicity, we trust CARLA's wp.next() 
        # but huge gaps (intersections) are handled by topology.
        # Let's rely on map.get_topology to bridge gaps.
        return successors

    def trace_route(self, origin, destination):
        """
        A* search to find a sequence of waypoints.
        Simplified: We just query the map for a path if available, or build a simple graph.
        Actually, CARLA map has loose topology.
        Alternative: Create a dense graph of waypoints.
        
        BETTER PIVOT: Use carla.Map.get_topology() to essentially "Dijkstra" the road network.
        Nodes = Waypoints (Start of segments).
        Edges = Segments.
        """
        # Node format: (x, y, z) rounded or ID? Waypoint ID is not unique across re-query.
        # Let's use `agents.navigation` if possible.
        # Since we can't, we will implement a very basic "Next waypoint that gets closer" heuristic
        # checking intersections.
        
        # ACTUALLY, implementing full A* on CARLA Waypoints from scratch is complex due to
        # lack of a discrete graph.
        #
        # Let's try to locate the 'agents' folder one more time, or assume standard path.
        # If this fails, we will fall back to "Greedy Routing":
        # At every intersection, pick the 'next' option that minimizes distance to goal.
        
        return self._greedy_route(origin, destination)

    def _greedy_route(self, origin_loc, dest_loc, timeout=200):
        """
        Generates a route by selecting the 'next' waypoint that is closest to destination.
        This resolves circles because at the junction, it will pick the exit towards the goal.
        """
        route = []
        current_wp = self.map.get_waypoint(origin_loc)
        accum_dist = 0
        
        for _ in range(timeout):
             route.append((current_wp, None)) # Format: (wp, road_option)
             
             if current_wp.transform.location.distance(dest_loc) < 5.0:
                 break
                 
             next_wps = current_wp.next(2.0)
             
             if not next_wps:
                 break
             
             if len(next_wps) == 1:
                 current_wp = next_wps[0]
             else:
                 # Intersection / Branch: Choice!
                 # Pick the one that minimizes distance to goal
                 best_wp = None
                 min_dist = float('inf')
                 
                 for wp in next_wps:
                     d = wp.transform.location.distance(dest_loc)
                     if d < min_dist:
                         min_dist = d
                         best_wp = wp
                 
                 current_wp = best_wp
                 
        return route

class CarlaInterface:
    def __init__(self):
        self.client = None
        self.world = None
        self.map = None
        self.tm = None
        self.router = None 
        self.ego_vehicle = None
        self.sensors = {}
        self.sensor_queues = {}
        self.data_queue = queue.Queue()

    def connect(self):
        """Connects to the CARLA server."""
        try:
            self.client = carla.Client(config.CARLA_HOST, config.CARLA_PORT)
            self.client.set_timeout(config.TIMEOUT)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            
            # Initialize Custom Router
            self.router = GlobalRouter(self.map)
            print("Connected to CARLA Server & Custom Router initialized.")
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            raise

    def generate_route(self, start_loc, end_loc):
        """Generates a route from start to end location."""
        if not self.router:
            return []
        
        return self.router.trace_route(start_loc, end_loc)


    def setup_world(self):
        """Configures the world for Synchronous Mode."""
        settings = self.world.get_settings()
        settings.synchronous_mode = config.SYNC_MODE
        settings.fixed_delta_seconds = config.FIXED_DELTA_SECONDS
        settings.substepping = True
        settings.max_substep_delta_time = config.MAX_SUBSTEP_DELTA_TIME
        settings.max_substeps = 10
        self.world.apply_settings(settings)

        # Setup Traffic Manager
        self.tm = self.client.get_trafficmanager(8005)
        self.tm.set_synchronous_mode(True)
        self.tm.set_random_device_seed(0)
        print("World settings applied (Sync Mode).")

    def spawn_ego_vehicle(self, spawn_point_index=0):
        """Spawns the ego vehicle at a specified spawn point, preferring outer (left) lanes."""
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.find(config.VEHICLE_MODEL)
        
        spawn_points = self.map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found in this map!")

        # Filter for "Visual Left" lanes on MULTI-LANE roads.
        # User reported spawning on "single lane leading to right lane".
        # We ensure:
        # 1. We are in the Inner Lane (abs(lane_id) == 1)
        # 2. We have a neighbor to our Visual Right (Outer Lane exists)
        # This guarantees we are not on a single-lane ramp.
        preferred_indices = []
        for i, sp in enumerate(spawn_points):
            wp = self.map.get_waypoint(sp.location)
            
            # Check 1: Inner Lane
            if abs(wp.lane_id) == 1:
                # Check 2: Multi-lane verification
                # Does a lane exist to our Visual Right?
                outer_lane = self.get_visual_right_lane(wp)
                if outer_lane is not None:
                     preferred_indices.append(i)
        
        if preferred_indices:
             # Randomize to avoid "Index 0" bad luck
             actual_index = random.choice(preferred_indices)
             spawn_point = spawn_points[actual_index]
             print(f"Spawning at preferred Index {actual_index} (Lane ID: {self.map.get_waypoint(spawn_point.location).lane_id}) | Multi-lane verified.")
        else:
             print("Warning: No multi-lane inner spawn points found. Trying loose filter...")
             # Fallback to just Inner Lane
             fallback_indices = [i for i, sp in enumerate(spawn_points) if abs(self.map.get_waypoint(sp.location).lane_id) == 1]
             if fallback_indices:
                  actual_index = random.choice(fallback_indices)
                  spawn_point = spawn_points[actual_index]
             else:
                  spawn_point = spawn_points[spawn_point_index % len(spawn_points)]
        
        # Try to spawn
        self.ego_vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
        if self.ego_vehicle is None:
            # If failed, try random
            print("Preferred spawn point blocked, trying random...")
            self.ego_vehicle = self.world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
        
        if self.ego_vehicle is None:
            raise RuntimeError("Could not spawn Ego Vehicle!")

        # Ego physics setup if needed
        print(f"Ego Vehicle spawned at {self.ego_vehicle.get_location()}")
        
        # Spectator follow
        spectator = self.world.get_spectator()
        transform = self.ego_vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=30), carla.Rotation(pitch=-90)))

        return self.ego_vehicle

    def spawn_sensors(self):
        """Spawns sensors attached to the ego vehicle based on config."""
        if not self.ego_vehicle:
            raise RuntimeError("Ego vehicle must be spawned before sensors.")

        bp_lib = self.world.get_blueprint_library()

        for name, spec in config.SENSORS.items():
            bp = bp_lib.find(spec['type'])
            
            # Set attributes
            for attr, val in spec.items():
                if attr in ['type', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                    continue
                bp.set_attribute(str(attr), str(val))
            
            # Location
            loc = carla.Location(x=spec.get('x', 0), y=spec.get('y', 0), z=spec.get('z', 0))
            rot = carla.Rotation(roll=spec.get('roll', 0), pitch=spec.get('pitch', 0), yaw=spec.get('yaw', 0))
            transform = carla.Transform(loc, rot)

            # Spawn and Attach
            sensor = self.world.spawn_actor(bp, transform, attach_to=self.ego_vehicle)
            self.sensors[name] = sensor
            
            # Create Queue
            q = queue.Queue()
            self.sensor_queues[name] = q
            sensor.listen(q.put)
            
            print(f"Spawned sensor: {name}")

    def get_data(self):
        """Ticks the world and retrieves sensor data."""
        self.world.tick()
        
        data = {'frame': self.world.get_snapshot().frame}
        
        # Retrieve data from all sensors
        for name, q in self.sensor_queues.items():
            try:
                # Wait for data with timeout matching loop time
                sensor_data = q.get(timeout=2.0)
                data[name] = sensor_data
            except queue.Empty:
                print(f"Warning: Sensor {name} missing data frame.")
                data[name] = None
        
        # Add Ground Truth Data
        data['ego_transform'] = self.ego_vehicle.get_transform()
        data['ego_velocity'] = self.ego_vehicle.get_velocity()
        data['ego_accel'] = self.ego_vehicle.get_acceleration()
        
        # Get Nearby Vehicles (Ground Truth Perception Phase 1)
        # Scan within 100m
        actors = self.world.get_actors().filter('vehicle.*')
        nearby = []
        ego_loc = data['ego_transform'].location
        for actor in actors:
            if actor.id != self.ego_vehicle.id:
                loc = actor.get_location()
                dist = loc.distance(ego_loc)
                if dist < 100.0:
                    nearby.append(actor)
        data['nearby_vehicles'] = nearby
        
        data['nearby_vehicles'] = nearby
        
        # Spectator Follow (TPS View)
        spectator = self.world.get_spectator()
        ego_t = self.ego_vehicle.get_transform()
        
        # Calculate TPS camera position
        # Move back 5.5m and up 2.8m relative to car
        yaw = math.radians(ego_t.rotation.yaw)
        pitch = math.radians(ego_t.rotation.pitch)
        
        # Simple offset calculation ignoring pitch for "following" distance to prevent nausea
        offset_x = -5.5 * math.cos(yaw)
        offset_y = -5.5 * math.sin(yaw)
        
        cam_loc = carla.Location(
            x=ego_t.location.x + offset_x,
            y=ego_t.location.y + offset_y,
            z=ego_t.location.z + 2.8
        )
        
        cam_rot = carla.Rotation(pitch=-15.0, yaw=ego_t.rotation.yaw, roll=0.0)
        spectator.set_transform(carla.Transform(cam_loc, cam_rot))
        
        return data

    def cleanup(self):
        """Despawns all actors."""
        print("Cleaning up...")
        
        # Sensors first
        for s in self.sensors.values():
            if s and s.is_alive:
                s.stop()
                s.destroy()
        
        # Ego
        if self.ego_vehicle and self.ego_vehicle.is_alive:
            self.ego_vehicle.destroy()
            
        print("Cleanup complete.")

    def apply_control(self, control_dict):
        """Applies control to the ego vehicle."""
        if not self.ego_vehicle:
            return
            
        control = carla.VehicleControl()
        control.throttle = control_dict.get('throttle', 0.0)
        control.steer = control_dict.get('steer', 0.0)
        control.brake = control_dict.get('brake', 0.0)
        control.hand_brake = control_dict.get('hand_brake', False)
        control.reverse = control_dict.get('reverse', False)
        
        self.ego_vehicle.apply_control(control)

    # ==============================================================================
    # -- Perception Layer (Phase 2) ------------------------------------------------
    # ==============================================================================
    
    def get_traffic_light_state(self):
        """Returns the state of the traffic light if the vehicle is affected by one."""
        if self.ego_vehicle.is_at_traffic_light():
            tl = self.ego_vehicle.get_traffic_light()
            if tl:
                return tl.get_state()
        return None

    def is_at_traffic_light(self):
        """Returns True if the ego vehicle is at a traffic light."""
        return self.ego_vehicle.is_at_traffic_light()

    def get_current_lane(self, location=None):
        """Returns the current lane (waypoint) of the ego vehicle."""
        if location is None:
            location = self.ego_vehicle.get_location()
        return self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    def get_left_lane(self, wp):
        """Returns the left lane waypoint if it exists and is drivable."""
        left_lane = wp.get_left_lane()
        if self.is_lane_drivable(left_lane):
            return left_lane
        return None
        
    def get_right_lane(self, wp):
        """Returns the right lane waypoint if it exists and is drivable."""
        right_lane = wp.get_right_lane()
        if self.is_lane_drivable(right_lane):
            return right_lane
        return None

    def is_lane_drivable(self, wp):
        """Checks if a waypoint is in a valid driving lane."""
        return wp is not None and wp.lane_type == carla.LaneType.Driving

    def get_visual_left_lane(self, wp):
        """
        Returns the lane to the VISUAL LEFT of the driver.
        Stops at the center line (sign change).
        """
        neighbor = None
        if wp.lane_id < 0:
            neighbor = self.get_left_lane(wp) # Inner
        else:
            neighbor = self.get_right_lane(wp) # Inner (Positive lanes)
            
        # Check for Median Crossing (Sign Change)
        if neighbor and (wp.lane_id * neighbor.lane_id < 0): 
            return None
        return neighbor
            
    def get_visual_right_lane(self, wp):
        """
        Returns the lane to the VISUAL RIGHT of the driver.
        """
        # Right (Visual) is usually Safe (Shoulder), unless one-way road has neighbor.
        # But generally we don't cross sign boundary on visual right (Outward).
        if wp.lane_id < 0:
            return self.get_right_lane(wp)
        else:
            return self.get_left_lane(wp)
