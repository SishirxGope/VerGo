import pygame
import numpy as np
import math
import cv2

class SensorVisualizer:
    def __init__(self, width=1200, height=800, scale=10.0):
        self.width = width
        self.height = height
        self.scale = scale # Pixels per meter
        
        # Colors
        self.COLOR_BG = (10, 10, 10)
        self.COLOR_EGO = (0, 255, 0)
        self.COLOR_RADAR = (255, 0, 0)
        self.COLOR_LIDAR_GROUND = (100, 100, 100)
        self.COLOR_LIDAR_OBSTACLE = (0, 255, 255)
        self.COLOR_TEXT = (255, 255, 255)
        
        # Init Pygame
        pygame.init()
        self.display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Sensor Fusion Debugger: LiDAR + Radar")
        self.font = pygame.font.SysFont('Arial', 14)
        self.clock = pygame.time.Clock()

    def render(self, data):
        """
        Main render loop.
        data: Dictionary containing sensor data buffers.
        """
        # Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        
        self.display.fill(self.COLOR_BG)
        
        # 1. Draw BEV Grid (Right Side)
        bev_center = (self.width * 0.75, self.height // 2)
        self._draw_grid(bev_center)
        
        # 2. Draw Ego Vehicle
        self._draw_ego(bev_center)
        
        # 3. Draw LiDAR (Points)
        if data.get('lidar') is not None:
            self._draw_lidar(data['lidar'], bev_center)
            
        # 4. Draw Radar (Points)
        if data.get('radar') is not None:
            self._draw_radar(data['radar'], bev_center)
            
        # 5. Draw Camera (Left Side)
        if data.get('camera_front') is not None:
            self._draw_camera(data['camera_front'])
            
        # 6. Status Text
        speed = 0.0
        if data.get('ego_velocity'):
             vel = data['ego_velocity']
             speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2)
        self._draw_text(f"Ego Speed: {speed:.1f} km/h", (10, 10))
        
        pygame.display.flip()
        self.clock.tick(60)
        return True

    def _draw_grid(self, center):
        cx, cy = center
        # Circles at 10m, 20m, 50m
        for r in [10, 20, 50]:
            radius = int(r * self.scale)
            pygame.draw.circle(self.display, (50, 50, 50), (int(cx), int(cy)), radius, 1)

    def _draw_ego(self, center):
        cx, cy = center
        # Draw Arrow
        # Assuming Ego is facing UP in BEV (Standard automotive convention in plots)
        # But CARLA x-forward means "Right" if we map (x,y) -> (x,y)
        # Let's verify mapping: 
        # Screen Y is Down. World Z is Up.
        # BEV: World X (Forward) -> Screen Y (Up/Down?)
        # Convention: Forward is usually UP on screen.
        # So: Screen X = World Y, Screen Y = -World X
        
        length = 4.7 * self.scale # ~4.7m Tesla Model 3
        width = 1.85 * self.scale
        
        rect = pygame.Rect(cx - width/2, cy - length/2, width, length)
        pygame.draw.rect(self.display, self.COLOR_EGO, rect)
        
        # Direction indicator
        pygame.draw.line(self.display, (0, 0, 0), (cx, cy), (cx, cy - length/2), 2)

    def _world_to_screen(self, x, y, center):
        """
        Converts sensor coords (vehicle frame) to screen coords.
        Auto: X=Forward, Y=Right, Z=Up.
        Screen: Forward=Up (-y), Right=Right (+x).
        Transformation:
        ScreenX = CenterX + (WorldY * scale)
        ScreenY = CenterY - (WorldX * scale)
        """
        cx, cy = center
        sx = cx + (y * self.scale)
        sy = cy - (x * self.scale)
        return int(sx), int(sy)

    def _draw_lidar(self, lidar_data, center):
        """
        lidar_data: carla.LidarMeasurement
        We need to parse raw data to points.
        """
        # Parse points (Numpy buffer access)
        # Layout: x, y, z, intensity
        points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        
        # Optimize: Filter for display range (ROI)
        # Only draw points within 60m
        points = points[points[:, 0] > -20] # Filter behind
        points = points[points[:, 0] < 80] # Filter too far
        
        # Heavy Optimization: Downsample visual points
        # If we have too many points, Pygame pixel setting is SLOW.
        # Take every 5th point
        points = points[::5]
        
        for p in points:
            x, y, z = p[0], p[1], p[2]
            
            # Simple ground filter visual
            # Z < -1.5 is usually ground/road
            color = self.COLOR_LIDAR_GROUND
            if z > -1.2:
                color = self.COLOR_LIDAR_OBSTACLE
            
            # Project
            sx, sy = self._world_to_screen(x, y, center)
            self.display.set_at((sx, sy), color)

    def _draw_radar(self, radar_data, center):
        """
        radar_data: carla.RadarMeasurement
        Contains list of Detections.
        Each: altitude, azimuth, depth, velocity
        """
        for detect in radar_data:
            azi = detect.azimuth
            alt = detect.altitude
            depth = detect.depth
            
            # Spherical to Cartesian
            # x = depth * cos(alt) * cos(azi)
            # y = depth * cos(alt) * sin(azi)
            # z = depth * sin(alt)
            
            # Note: Carla Radar angles are in RADIANS? No, docs say Radians. But double check.
            # Usually detection.azimuth is rad.
            
            x = depth * math.cos(azi) * math.cos(alt)
            y = depth * math.sin(azi) * math.cos(alt)
            
            sx, sy = self._world_to_screen(x, y, center)
            
            # Draw larger Circle
            pygame.draw.circle(self.display, self.COLOR_RADAR, (sx, sy), 3)

    def _draw_camera(self, image_data):
        """
        Convert CARLA RAW image to Pygame Surface.
        """
        # Layout: BGRA (Carla is raw BGRA)
        array = np.frombuffer(image_data.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image_data.height, image_data.width, 4))
        
        # RGBA correction: CARLA is BGRA, Pygame expects RGB usually.
        # Let's slice: array[:, :, :3] is BGR. 
        # Convert BGR to RGB
        rgb = array[:, :, :3][:, :, ::-1] # BGR -> RGB
        
        # Rotate logic? Coordinate systems?
        # Pygame surface from array
        # Transpose might be needed depending on array shape vs surf expectations
        # CV2 to Pygame: flip, rot
        
        # Resize to fit half screen
        target_h = self.height
        target_w = int(self.width / 2)
        
        # Using cv2 for fast resize
        if rgb.shape[0] > 0 and rgb.shape[1] > 0:
            resized = cv2.resize(rgb, (target_w, int(target_w * 3 / 4))) # 4:3 aspect
            # Blit
            surf = pygame.surfarray.make_surface(resized.swapaxes(0, 1))
            self.display.blit(surf, (0, (self.height - surf.get_height()) // 2))
    
    def _draw_text(self, text, pos):
        surf = self.font.render(text, True, self.COLOR_TEXT)
        self.display.blit(surf, pos)
    
    def cleanup(self):
        pygame.quit()
