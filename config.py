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
WEATHER_PRESET = 'HeavyRain' 

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
        'image_size_x': 640, 'image_size_y': 480, 'fov': 90
    },
    'camera_rear': {
        'type': 'sensor.camera.rgb',
        'x': -2.0, 'y': 0.0, 'z': 1.5,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
        'image_size_x': 640, 'image_size_y': 480, 'fov': 90
    },
    'camera_tps': {
        'type': 'sensor.camera.rgb',
        'x': -5.5, 'y': 0.0, 'z': 2.8,
        'roll': 0.0, 'pitch': -15.0, 'yaw': 0.0,
        'image_size_x': 640, 'image_size_y': 480, 'fov': 90
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
