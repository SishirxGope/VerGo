import os
import math

# ==============================================================================
# -- Global Simulation Constants -----------------------------------------------
# ==============================================================================
CARLA_HOST = 'localhost'
CARLA_PORT = 2000
TIMEOUT = 20.0  # seconds

# Sync Mode Settings
SYNC_MODE = True
FIXED_DELTA_SECONDS = 0.05  # 20 FPS (Real-time target)
SUBSTEP_DELTA_SECONDS = 0.01 
MAX_SUBSTEP_DELTA_TIME = 0.05

# ==============================================================================
# -- India Driving Rules -------------------------------------------------------
# ==============================================================================
LEFT_LANE_DRIVING = True  # True = India/UK/Japan, False = US/EU
OVERTAKE_ON_RIGHT = True  # Opposite of driving side

# ==============================================================================
# -- Weather Configuration -----------------------------------------------------
# ==============================================================================
# Presets: 'ClearNoon', 'HeavyRain', 'Storm'
WEATHER_PRESET = 'ClearNoon' 

# ==============================================================================
# -- Vehicle Configuration -----------------------------------------------------
# ==============================================================================
VEHICLE_MODEL = 'vehicle.tesla.model3'

# Sensor Positions (x, y, z under ego vehicle coordinates)
SENSORS = {
    'lidar': {
        'type': 'sensor.lidar.ray_cast',
        'x': 0.0, 'y': 0.0, 'z': 2.4,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'channels': 32,
        'range': 100,
        'upper_fov': 10.0,
        'lower_fov': -30.0,
        'points_per_second': 200000,
        'rotation_frequency': 20,
        # Bad Weather Simulation (Noise)
        'atmosphere_attenuation_rate': 0.04, # Fog/Rain attenuation
        'dropoff_general_rate': 0.45,
        'dropoff_intensity_limit': 0.8,
        'dropoff_zero_intensity': 0.4,
        'noise_stddev': 0.02 # 2cm jitter
    },
    'camera_front': {
        'type': 'sensor.camera.rgb',
        'x': 2.0, 'y': 0.0, 'z': 1.5,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'image_size_x': 320, 'image_size_y': 240, 'fov': 90
    },
    'camera_rear': {
        'type': 'sensor.camera.rgb',
        'x': -2.0, 'y': 0.0, 'z': 1.5,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
        'image_size_x': 320, 'image_size_y': 240, 'fov': 90
    },
    'camera_tps': {
        'type': 'sensor.camera.rgb',
        'x': -5.5, 'y': 0.0, 'z': 2.8,
        'roll': 0.0, 'pitch': -15.0, 'yaw': 0.0,
        'image_size_x': 320, 'image_size_y': 240, 'fov': 90
    },
    'gnss': {
        'type': 'sensor.other.gnss',
        'x': 0.0, 'y': 0.0, 'z': 0.0
    },
    'imu': {
        'type': 'sensor.other.imu',
        'x': 0.0, 'y': 0.0, 'z': 0.0
    },
    'radar': {
        'type': 'sensor.other.radar',
        'x': 2.0, 'y': 0.0, 'z': 1.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'horizontal_fov': 35,
        'vertical_fov': 20,
        'range': 100
    }
}

# ==============================================================================
# -- Planning & Control Constants ----------------------------------------------
# ==============================================================================
TARGET_SPEED_KPH = 30.0
TARGET_SPEED_MPS = TARGET_SPEED_KPH / 3.6

# Safety
EMERGENCY_BRAKE_TTC = 1.5  # seconds
MIN_FOLLOW_DISTANCE = 5.0 # meters (bumper to bumper)
SAFE_FOLLOW_DISTANCE = 15.0 # meters

# Control Gains
PID_LONGITUDINAL = {
    'K_P': 1.0,
    'K_D': 0.05,
    'K_I': 0.05,
    'dt': FIXED_DELTA_SECONDS
}

PURE_PURSUIT_LOOKAHEAD = 6.0 # meters

# ==============================================================================
# -- Camera Perception (RAIL Integration) -------------------------------------
# ==============================================================================
CAMERA_PERCEPTION_ENABLED = False   # Experimental — set True to enable camera obstacle detection
CAMERA_MIN_CONTOUR_AREA = 3000     # Minimum contour area in pixels (high to filter road texture)
CAMERA_ADAPTIVE_BLOCK_SIZE = 31    # Block size for adaptive thresholding (must be odd)
CAMERA_ADAPTIVE_C = 10             # Constant subtracted from mean in adaptive threshold
CAMERA_MAX_DETECTIONS = 5          # Cap detections per frame to prevent false-positive flooding

# ==============================================================================
# -- Local Grid Planner (RAIL Integration) ------------------------------------
# ==============================================================================
LOCAL_GRID_ENABLED = False          # Experimental — set True to enable A* local avoidance
LOCAL_GRID_ROWS = 50               # Grid cells in forward direction
LOCAL_GRID_COLS = 30               # Grid cells in lateral direction
LOCAL_GRID_CELL_SIZE = 1.0         # Meters per cell (50m x 30m coverage)
LOCAL_GRID_INFLATE_CELLS = 2       # Obstacle inflation radius in cells (vehicle width margin)

# ==============================================================================
# -- Imitation Learning (RAIL Integration) ------------------------------------
# ==============================================================================
LEARNING_ENABLED = False            # Set True to enable learned control blending
LEARNING_COLLECT_DATA = False       # Set True to record driving data for training
LEARNING_INPUT_DIM = 5             # [ego_speed, heading_error, obs_dist, obs_angle, target_speed]
LEARNING_LR = 0.001                # Learning rate
LEARNING_EPOCHS = 100              # Training epochs
LEARNING_BLEND_ALPHA = 0.2         # Blend weight: 0=full classical, 1=full learned
