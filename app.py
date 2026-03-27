#!/usr/bin/env python3
"""
Autonomous UAV Safety System v4.1 - WITH DATA LOGGING
Complete IoT Dashboard with AI Person Detection & Pixhawk Control

Features:
- DHT11 Temperature & Humidity Monitoring (GPIO 17)
- MQ Gas Sensor Detection (GPIO 23)
- AI-powered Person Detection (YOLOv4-tiny)
- Distance Estimation with 2m Safety Threshold
- Pixhawk MAVLink Control via UART (GPIO 14/15)
- Real-time ARM/DISARM Status Monitoring
- Real-time Flight Mode Display
- Real-time Video Streaming with Overlays
- RESTful API for Sensor Data
- Thread-safe Operations
- Comprehensive Error Handling
- Bluetooth MAVLink Bridge Support
- CSV Data Logging for Historical Analysis

Hardware Requirements:
- Raspberry Pi 5 (4GB+ recommended)
- DHT11 Temperature/Humidity Sensor
- MQ-series Gas Sensor (Digital output)
- USB Camera (640x480 minimum)
- Pixhawk Flight Controller (optional - runs in simulation mode without)

Author: saiprasad-tech
Date: 2025-11-18
Version: 4.1 WITH DATA LOGGING
"""

import os
os.environ['BLINKA_LGPIO'] = '1'  # Enable RPi 5 GPIO support

from flask import Flask, Response, jsonify, send_from_directory
import time
import cv2
import board
import digitalio
import adafruit_dht
import numpy as np
import threading
from collections import deque
import sys
import csv
from datetime import datetime
from pathlib import Path

# ========================================
# STARTUP BANNER
# ========================================

print("\n" + "="*70)
print("   =  AUTONOMOUS UAV SAFETY SYSTEM v4.1 WITH DATA LOGGING")
print("   =h=ª Developer: saiprasad-tech")
print("   =≈ Date: 2025-11-18 17:45:26 UTC")
print("   <◊  Production Build - All Features Enabled")
print("   =  Data Logging: ENABLED")
print("="*70)

# ========================================
# CONFIGURATION PARAMETERS
# ========================================

# Hardware Configuration
DHT_PIN = board.D17              # GPIO 17 (Physical Pin 11)
DHT_TYPE = "DHT11"               # DHT11 sensor type
GAS_PIN = board.D23              # GPIO 23 (Physical Pin 16)
PIXHAWK_PORT = '/dev/ttyAMA0'    # GPIO UART (GPIO 14/15)
PIXHAWK_BAUD = 57600             # MAVLink baud rate

# AI Configuration
AI_CONFIDENCE_THRESHOLD = 0.4    # 40% confidence for detection
YOLO_MODEL_WEIGHTS = "yolov4-tiny.weights"
YOLO_MODEL_CONFIG = "yolov4-tiny.cfg"
YOLO_CLASS_NAMES = "coco.names"

# Safety Parameters
DANGER_DISTANCE_METERS = 2.0     # Trigger avoidance within 2m
SAFE_DISTANCE_METERS = 2.0       # Move backward 2m when avoiding
AVOIDANCE_COOLDOWN = 5.0         # Seconds between avoidance maneuvers
CONSECUTIVE_DETECTIONS = 3       # Require 3 detections to trigger

# Distance Estimation Calibration
KNOWN_PERSON_HEIGHT = 1.7        # Average person height in meters
CAMERA_FOCAL_LENGTH = 500        # Camera focal length in pixels (adjust if needed)

# Sensor Read Intervals
DHT_READ_INTERVAL = 3.0          # Seconds between DHT reads (don't go below 2s)
GAS_READ_INTERVAL = 0.5          # Seconds between gas reads

# Camera Settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_WARMUP_FRAMES = 10        # Discard first N frames

# ========================================
# DATA LOGGING CONFIGURATION (NEW)
# ========================================

DATA_LOG_ENABLED = True                          # Enable/disable logging
DATA_LOG_DIRECTORY = "sensor_logs"               # Directory for log files
DATA_LOG_INTERVAL = 1.0                          # Log every N seconds
DATA_LOG_MAX_ROWS_PER_FILE = 100000              # Max rows before creating new file
DATA_LOG_INCLUDE_DETECTION_EVENTS = True         # Log person detection events separately

# ========================================
# GLOBAL VARIABLES (DECLARED EARLY)
# ========================================

# Hardware objects
pixhawk = None
mavlink_available = False
dht_sensor = None
gas_sensor = None
camera = None
net = None
output_layers = None
classes = []

# Pixhawk state
pixhawk_armed = False
pixhawk_mode = "UNKNOWN"
pixhawk_battery_voltage = 0.0
pixhawk_battery_percent = 0

# Sensor state
last_temp = None
last_humidity = None
last_dht_read_time = 0
dht_read_failures = 0
last_gas_level = 0.0

# Detection state
person_detected = False
person_distance = 999.0
person_detection_count = 0
detection_history = deque(maxlen=10)

# Avoidance state
avoidance_active = False
last_avoidance_time = 0

# Sensor data cache
sensor_data_lock = threading.Lock()

# Statistics
frame_count = 0
detection_count = 0
avoidance_count = 0
system_start_time = time.time()

# ========================================
# DATA LOGGING VARIABLES (NEW)
# ========================================

data_log_lock = threading.Lock()
current_log_file = None
current_log_writer = None
current_log_row_count = 0
last_log_time = 0
total_logged_entries = 0
detection_log_file = None
detection_log_writer = None

# ========================================
# DATA LOGGING FUNCTIONS (NEW)
# ========================================

def initialize_data_logging():
    """
    Initialize the data logging system. 
    Creates log directory and opens initial CSV file.
    """
    global current_log_file, current_log_writer, current_log_row_count
    global detection_log_file, detection_log_writer
    
    if not DATA_LOG_ENABLED:
        print("\n[DATA LOG] Data logging is DISABLED")
        return False
    
    try:
        # Create log directory if it doesn't exist
        log_dir = Path(DATA_LOG_DIRECTORY)
        log_dir.mkdir(parents=True, exist_ok=True)
        
        # Create main sensor data log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        sensor_log_path = log_dir / f"sensor_data_{timestamp}.csv"
        
        current_log_file = open(sensor_log_path, 'w', newline='', buffering=1)
        current_log_writer = csv.writer(current_log_file)
        
        # Write header row
        current_log_writer. writerow([
            'timestamp',
            'datetime',
            'temperature_c',
            'humidity_percent',
            'gas_level',
            'person_detected',
            'person_distance_m',
            'avoidance_active',
            'pixhawk_connected',
            'pixhawk_armed',
            'pixhawk_mode',
            'pixhawk_battery_voltage',
            'pixhawk_battery_percent',
            'ai_enabled',
            'frame_count',
            'detection_count',
            'avoidance_count'
        ])
        
        current_log_row_count = 0
        
        print(f"\n[DATA LOG]  Sensor data logging initialized")
        print(f"           í Log file: {sensor_log_path}")
        
        # Create detection events log file if enabled
        if DATA_LOG_INCLUDE_DETECTION_EVENTS:
            detection_log_path = log_dir / f"detection_events_{timestamp}.csv"
            detection_log_file = open(detection_log_path, 'w', newline='', buffering=1)
            detection_log_writer = csv.writer(detection_log_file)
            
            # Write header for detection events
            detection_log_writer.writerow([
                'timestamp',
                'datetime',
                'event_type',
                'person_distance_m',
                'confidence',
                'bbox_x',
                'bbox_y',
                'bbox_width',
                'bbox_height',
                'avoidance_triggered',
                'pixhawk_armed',
                'pixhawk_mode'
            ])
            
            print(f"           í Detection events log: {detection_log_path}")
        
        return True
        
    except Exception as e:
        print(f"\n[DATA LOG]  Failed to initialize: {e}")
        return False


def rotate_log_file_if_needed():
    """
    Check if current log file needs rotation and create new one if necessary.
    """
    global current_log_file, current_log_writer, current_log_row_count
    
    if current_log_row_count >= DATA_LOG_MAX_ROWS_PER_FILE:
        try:
            # Close current file
            if current_log_file:
                current_log_file. close()
            
            # Create new file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_dir = Path(DATA_LOG_DIRECTORY)
            sensor_log_path = log_dir / f"sensor_data_{timestamp}.csv"
            
            current_log_file = open(sensor_log_path, 'w', newline='', buffering=1)
            current_log_writer = csv.writer(current_log_file)
            
            # Write header
            current_log_writer.writerow([
                'timestamp',
                'datetime',
                'temperature_c',
                'humidity_percent',
                'gas_level',
                'person_detected',
                'person_distance_m',
                'avoidance_active',
                'pixhawk_connected',
                'pixhawk_armed',
                'pixhawk_mode',
                'pixhawk_battery_voltage',
                'pixhawk_battery_percent',
                'ai_enabled',
                'frame_count',
                'detection_count',
                'avoidance_count'
            ])
            
            current_log_row_count = 0
            print(f"[DATA LOG] Rotated to new file: {sensor_log_path}")
            
        except Exception as e:
            print(f"[DATA LOG] Error rotating file: {e}")


def log_sensor_data(temperature, humidity, gas_level):
    """
    Log current sensor data to CSV file.
    Called periodically from the data logging thread.
    
    Args:
        temperature: Current temperature reading
        humidity: Current humidity reading
        gas_level: Current gas sensor reading
    """
    global current_log_writer, current_log_row_count, total_logged_entries
    global pixhawk, pixhawk_armed, pixhawk_mode, pixhawk_battery_voltage, pixhawk_battery_percent
    
    if not DATA_LOG_ENABLED or current_log_writer is None:
        return
    
    try:
        with data_log_lock:
            # Check if rotation needed
            rotate_log_file_if_needed()
            
            # Prepare data row
            current_time = time.time()
            current_datetime = datetime.now().strftime("%Y-%m-%d %H:%M:%S. %f")[:-3]
            
            row = [
                round(current_time, 3),
                current_datetime,
                round(temperature, 1) if temperature != -999.0 else None,
                round(humidity, 1) if humidity != -999.0 else None,
                round(gas_level, 1) if gas_level != -999.0 else None,
                person_detected,
                round(person_distance, 2) if person_distance < 999 else None,
                avoidance_active,
                pixhawk is not None,
                pixhawk_armed if pixhawk else False,
                pixhawk_mode if pixhawk else "N/A",
                round(pixhawk_battery_voltage, 2) if pixhawk else 0.0,
                pixhawk_battery_percent if pixhawk else 0,
                net is not None,
                frame_count,
                detection_count,
                avoidance_count
            ]
            
            current_log_writer.writerow(row)
            current_log_row_count += 1
            total_logged_entries += 1
            
    except Exception as e:
        print(f"[DATA LOG] Error writing data: {e}")


def log_detection_event(event_type, distance, confidence=None, bbox=None, avoidance_triggered=False):
    """
    Log a person detection event to the detection events CSV. 
    
    Args:
        event_type: Type of event ('DETECTION', 'DANGER', 'AVOIDANCE')
        distance: Distance to detected person
        confidence: Detection confidence (0-1)
        bbox: Bounding box tuple (x, y, w, h)
        avoidance_triggered: Whether avoidance was triggered
    """
    global detection_log_writer, pixhawk_armed, pixhawk_mode
    
    if not DATA_LOG_INCLUDE_DETECTION_EVENTS or detection_log_writer is None:
        return
    
    try:
        with data_log_lock:
            current_time = time.time()
            current_datetime = datetime.now().strftime("%Y-%m-%d %H:%M:%S. %f")[:-3]
            
            row = [
                round(current_time, 3),
                current_datetime,
                event_type,
                round(distance, 2) if distance < 999 else None,
                round(confidence, 3) if confidence else None,
                bbox[0] if bbox else None,
                bbox[1] if bbox else None,
                bbox[2] if bbox else None,
                bbox[3] if bbox else None,
                avoidance_triggered,
                pixhawk_armed if pixhawk else False,
                pixhawk_mode if pixhawk else "N/A"
            ]
            
            detection_log_writer. writerow(row)
            
    except Exception as e:
        print(f"[DATA LOG] Error writing detection event: {e}")


def data_logging_thread():
    """
    Background thread for periodic data logging.
    Runs continuously and logs sensor data at specified intervals.
    """
    global last_log_time
    
    print("[DATA LOG] Background logging thread started")
    
    while True:
        try:
            current_time = time.time()
            
            # Check if it's time to log
            if current_time - last_log_time >= DATA_LOG_INTERVAL:
                last_log_time = current_time
                
                # Read current sensor values
                with sensor_data_lock:
                    temperature, humidity = read_dht_sensor()
                    gas_level = read_gas_sensor()
                
                # Log the data
                log_sensor_data(temperature, humidity, gas_level)
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.1)
            
        except Exception as e:
            print(f"[DATA LOG] Thread error: {e}")
            time.sleep(1)


def close_data_logging():
    """
    Safely close all log files.
    Called during system shutdown.
    """
    global current_log_file, detection_log_file
    
    try:
        if current_log_file:
            current_log_file.close()
            print("[DATA LOG]  Sensor log file closed")
        
        if detection_log_file:
            detection_log_file.close()
            print("[DATA LOG]  Detection events log file closed")
            
    except Exception as e:
        print(f"[DATA LOG] Error closing files: {e}")


# ========================================
# PRE-FLIGHT CHECKS
# ========================================

print("\n[PRE-FLIGHT] System Checks...")

# Check UART availability
try:
    import serial
    if os.path.exists('/dev/ttyAMA0'):
        print("       UART device found at /dev/ttyAMA0")
    else:
        print("      † UART device not found - enable with raspi-config")
except ImportError:
    print("      † pyserial not installed")

# Check if running as root (not recommended)
if os.geteuid() == 0:
    print("      † Running as root - not recommended for security")
else:
    print("       Running as normal user")

# Check required files
missing_files = []
for file in [YOLO_MODEL_WEIGHTS, YOLO_MODEL_CONFIG, YOLO_CLASS_NAMES]:
    if not os. path.exists(file):
        missing_files.append(file)

if missing_files:
    print(f"      † Missing AI model files: {', '.join(missing_files)}")
    print(f"      í System will run without AI detection")
else:
    print("       All AI model files present")

# Check data logging directory
if DATA_LOG_ENABLED:
    print(f"       Data logging enabled í {DATA_LOG_DIRECTORY}/")
else:
    print(f"      † Data logging disabled")

# ========================================
# SYSTEM INITIALIZATION
# ========================================

# DHT11 Temperature & Humidity Sensor
print("\n[1/7] Initializing DHT11 Sensor (GPIO 17)...")

try:
    dht_sensor = adafruit_dht.DHT11(DHT_PIN, use_pulseio=False)
    
    # Initial stabilization period
    print("      í Waiting for sensor to stabilize (2 seconds)...")
    time.sleep(2)
    
    # Test read with retry
    for attempt in range(3):
        try:
            test_temp = dht_sensor.temperature
            test_hum = dht_sensor.humidity
            
            if test_temp is not None and test_hum is not None:
                print(f"       DHT11 initialized successfully")
                print(f"      í Initial reading: {test_temp}∞C, {test_hum}% RH")
                last_temp = test_temp
                last_humidity = test_hum
                break
        except RuntimeError as e:
            if attempt < 2:
                time.sleep(1)
            else:
                print(f"      † DHT11 initialized but unstable readings")
        
except Exception as e:
    print(f"       DHT11 initialization failed: {e}")
    dht_sensor = None

# AI Model Loading
print("\n[2/7] Loading AI Person Detection Model...")

try:
    net = cv2.dnn.readNet(YOLO_MODEL_WEIGHTS, YOLO_MODEL_CONFIG)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    
    with open(YOLO_CLASS_NAMES, "r") as f:
        classes = [line.strip() for line in f.readlines()]
    
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net. getUnconnectedOutLayers()]
    
    print(f"       YOLOv4-tiny model loaded successfully")
    print(f"      í {len(classes)} object classes available")
    print(f"      í Person detection class ID: 0")
    
except Exception as e:
    print(f"       AI model loading failed: {e}")
    print(f"      í System will run without AI detection")
    net = None

# Pixhawk MAVLink Connection
print("\n[3/7] Connecting to Pixhawk Autopilot...")

try:
    from pymavlink import mavutil
    mavlink_available = True
    
    print(f"      í Attempting connection to {PIXHAWK_PORT} @ {PIXHAWK_BAUD} baud...")
    pixhawk = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
    
    print(f"      í Waiting for heartbeat (10 second timeout)...")
    pixhawk.wait_heartbeat(timeout=10)
    
    print(f"       Pixhawk connected successfully")
    print(f"      í System ID: {pixhawk.target_system}")
    print(f"      í Component ID: {pixhawk.target_component}")
    
    # Get initial arming state
    msg = pixhawk.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg:
        pixhawk_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        print(f"      í Initial State: {'ARMED †' if pixhawk_armed else 'DISARMED '}")
    
except Exception as e:
    print(f"      † Pixhawk not connected: {e}")
    print(f"      í Running in SIMULATION mode")
    print(f"      í Avoidance commands will be logged but not executed")
    pixhawk = None

# Gas Sensor Initialization
print("\n[4/7] Initializing Gas Sensor (GPIO 23)...")

try:
    gas_pin = digitalio.DigitalInOut(GAS_PIN)
    gas_pin.direction = digitalio.Direction.INPUT
    gas_sensor = gas_pin
    
    # Test read
    test_value = gas_pin.value
    print(f"       Gas sensor initialized successfully")
    print(f"      í Current state: {'HIGH (GAS DETECTED)' if test_value else 'LOW (CLEAR)'}")
    
except Exception as e:
    print(f"       Gas sensor initialization failed: {e}")
    gas_sensor = None

# Camera Initialization
print("\n[5/7] Initializing USB Camera...")
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print(f"       Camera initialization failed")
    print(f"      í Check USB connection and permissions")
else:
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    camera. set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency
    
    # Warm up camera (discard initial frames)
    print(f"      í Warming up camera ({CAMERA_WARMUP_FRAMES} frames)...")
    for _ in range(CAMERA_WARMUP_FRAMES):
        camera.read()
    
    ret, test_frame = camera.read()
    if ret:
        h, w, _ = test_frame.shape
        print(f"       Camera initialized successfully")
        print(f"      í Resolution: {w}x{h}")
        print(f"      í FPS target: {CAMERA_FPS}")
    else:
        print(f"      † Camera opened but cannot read frames")

# Flask Web Server
print("\n[6/7] Initializing Web Server...")
app = Flask(__name__)
app.config['JSON_SORT_KEYS'] = False  # Preserve JSON order
print(f"       Flask server initialized")

# Data Logging Initialization (NEW)
print("\n[7/7] Initializing Data Logging System...")
if initialize_data_logging():
    # Start background logging thread
    logging_thread = threading.Thread(target=data_logging_thread, daemon=True)
    logging_thread.start()
    print(f"       Background logging thread started")
    print(f"      í Logging interval: {DATA_LOG_INTERVAL} seconds")
else:
    print(f"      † Data logging not available")

print("\n" + "="*70)
print("    SYSTEM INITIALIZATION COMPLETE")
print("="*70)

# ========================================
# PIXHAWK STATUS MONITORING
# ========================================

def monitor_pixhawk_status():
    """
    Background thread to continuously monitor Pixhawk status. 
    Updates: armed state, flight mode, battery voltage/percent. 
    """
    global pixhawk_armed, pixhawk_mode, pixhawk_battery_voltage, pixhawk_battery_percent, pixhawk
    
    print("\n[MONITOR] Pixhawk status monitoring thread started")
    
    while True:
        if pixhawk and mavlink_available:
            try:
                from pymavlink import mavutil
                
                # Check for heartbeat (arming status and mode)
                msg = pixhawk.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
                if msg:
                    # Check if armed (bit 7 of base_mode)
                    pixhawk_armed = (msg.base_mode & mavutil.mavlink. MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    
                    # Get flight mode
                    mode_mapping = pixhawk.mode_mapping()
                    if mode_mapping:
                        for mode_name, mode_number in mode_mapping.items():
                            if mode_number == msg.custom_mode:
                                pixhawk_mode = mode_name
                                break
                
                # Check for battery status
                battery_msg = pixhawk.recv_match(type='SYS_STATUS', blocking=False, timeout=0.1)
                if battery_msg:
                    pixhawk_battery_voltage = battery_msg.voltage_battery / 1000.0  # mV to V
                    pixhawk_battery_percent = battery_msg. battery_remaining
                
                time.sleep(0.5)  # Check twice per second
                
            except Exception as e:
                print(f"[MONITOR] Error: {e}")
                time.sleep(2)
        else:
            time.sleep(5)  # No Pixhawk, check less frequently

# Start monitoring thread after Pixhawk initialization
if pixhawk:
    monitor_thread = threading.Thread(target=monitor_pixhawk_status, daemon=True)
    monitor_thread.start()

# ========================================
# DHT SENSOR FUNCTIONS
# ========================================

def read_dht_sensor():
    """
    Read temperature and humidity from DHT11 sensor.
    Implements rate limiting and error handling.
    
    Returns:
        tuple: (temperature_celsius, humidity_percent)
    """
    global last_temp, last_humidity, last_dht_read_time, dht_read_failures
    
    if dht_sensor is None:
        return -999.0, -999.0
    
    current_time = time.time()
    
    # Rate limiting (DHT11 needs 2+ seconds between reads)
    if current_time - last_dht_read_time < DHT_READ_INTERVAL:
        return last_temp if last_temp is not None else -999.0, \
               last_humidity if last_humidity is not None else -999.0
    
    last_dht_read_time = current_time
    
    try:
        temperature = dht_sensor.temperature
        humidity = dht_sensor. humidity
        
        if temperature is not None and humidity is not None:
            # DHT11 valid ranges: 0-50∞C, 20-90% RH (typical)
            # But sensor can read 0-100%, so we allow it
            if -40 <= temperature <= 80 and 0 <= humidity <= 100:
                last_temp = temperature
                last_humidity = humidity
                dht_read_failures = 0  # Reset failure counter
                return temperature, humidity
        
        # If we get here, reading was invalid
        dht_read_failures += 1
        
        # Return last good values if available
        return last_temp if last_temp is not None else -999.0, \
               last_humidity if last_humidity is not None else -999.0
               
    except RuntimeError as e:
        # DHT sensors occasionally timeout - this is normal
        dht_read_failures += 1
        
        # If too many failures, log it
        if dht_read_failures > 10:
            print(f"† DHT sensor unstable ({dht_read_failures} failures)")
            dht_read_failures = 0  # Reset counter
        
        return last_temp if last_temp is not None else -999.0, \
               last_humidity if last_humidity is not None else -999.0
               
    except Exception as e:
        print(f"DHT Read Error: {e}")
        return -999.0, -999.0

# ========================================
# GAS SENSOR FUNCTIONS
# ========================================

def read_gas_sensor():
    """
    Read gas detection state from MQ sensor.
    
    Returns:
        float: 0.0 (clear) or 100.0 (gas detected)
    """
    global last_gas_level
    
    if gas_sensor is None:
        return -999.0
    
    try:
        pin_state = gas_sensor.value
        
        # Digital MQ sensors: HIGH = gas detected, LOW = clear
        # If your sensor is inverted, change this line
        gas_level = 100.0 if pin_state else 0.0
        
        last_gas_level = gas_level
        return gas_level
        
    except Exception as e:
        print(f"Gas Sensor Read Error: {e}")
        return last_gas_level

# ========================================
# DISTANCE ESTIMATION
# ========================================

def estimate_distance_from_bbox(bbox_height_pixels):
    """
    Estimate distance to person using bounding box height.
    
    Uses pinhole camera model:
    Distance = (Real_Height * Focal_Length) / Pixel_Height
    
    Calibration tips:
    - Stand exactly 2 meters from camera
    - Measure bounding box height in pixels
    - Calculate: CAMERA_FOCAL_LENGTH = (2 * bbox_height) / 1.7
    
    Args:
        bbox_height_pixels: Height of bounding box in pixels
        
    Returns:
        float: Estimated distance in meters
    """
    if bbox_height_pixels == 0 or bbox_height_pixels < 20:
        return 999.0
    
    distance = (KNOWN_PERSON_HEIGHT * CAMERA_FOCAL_LENGTH) / bbox_height_pixels
    
    # Clamp to reasonable range (0.5m to 20m)
    distance = max(0.5, min(distance, 20.0))
    
    return distance

# ========================================
# PIXHAWK CONTROL FUNCTIONS
# ========================================

def send_velocity_command(vx, vy, vz):
    """
    Send velocity command to Pixhawk in body frame.
    
    Args:
        vx: Forward velocity (m/s) - negative = backward
        vy: Right velocity (m/s)
        vz: Down velocity (m/s)
        
    Returns:
        bool: Success status
    """
    global pixhawk, mavlink_available
    
    if not pixhawk or not mavlink_available:
        print(f"   [SIM] Would send velocity: vx={vx}, vy={vy}, vz={vz}")
        return False
    
    try:
        from pymavlink import mavutil
        
        pixhawk.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            pixhawk.target_system,
            pixhawk.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # Type mask (only velocities enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
        return True
        
    except Exception as e:
        print(f"Pixhawk Command Error: {e}")
        return False

def move_backward(distance_meters):
    """Execute backward movement command."""
    print(f"=Å COMMAND: Move backward {distance_meters}m")
    return send_velocity_command(vx=-1.0, vy=0, vz=0)  # -1 m/s backward

def stop_movement():
    """Stop all movement."""
    print(f"=Å COMMAND: Stop all movement")
    return send_velocity_command(vx=0, vy=0, vz=0)

# ========================================
# AI PERSON DETECTION
# ========================================

def detect_persons_in_frame(frame):
    """
    Detect persons in video frame using YOLO. 
    
    Args:
        frame: OpenCV image (BGR format)
        
    Returns:
        tuple: (annotated_frame, person_detected, closest_distance)
    """
    global person_detected, person_distance, person_detection_count
    global avoidance_active, last_avoidance_time, detection_count
    
    if net is None:
        return frame, False, 999.0
    
    try:
        height, width, channels = frame.shape
        
        # Prepare image for YOLO
        blob = cv2.dnn.blobFromImage(
            frame, 
            1/255.0,           # Scale factor
            (416, 416),        # Input size
            swapRB=True,       # BGR to RGB
            crop=False
        )
        
        net. setInput(blob)
        outputs = net.forward(output_layers)
        
        # Process detections
        boxes = []
        confidences = []
        distances = []
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                # Class 0 = person in COCO dataset
                if class_id == 0 and confidence > AI_CONFIDENCE_THRESHOLD:
                    # Bounding box coordinates
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    
                    # Ensure box is within frame
                    x = max(0, x)
                    y = max(0, y)
                    w = min(w, width - x)
                    h = min(h, height - y)
                    
                    # Estimate distance
                    estimated_distance = estimate_distance_from_bbox(h)
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    distances. append(estimated_distance)
        
        # Apply Non-Maximum Suppression
        person_found = False
        closest_distance = 999.0
        
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(
                boxes, 
                confidences, 
                AI_CONFIDENCE_THRESHOLD, 
                0.4  # NMS threshold
            )
            
            if len(indices) > 0:
                person_found = True
                detection_count += 1
                
                for i in indices.flatten():
                    x, y, w, h = boxes[i]
                    dist = distances[i]
                    conf = confidences[i]
                    
                    # Track closest person
                    if dist < closest_distance:
                        closest_distance = dist
                    
                    # Log detection event (NEW)
                    event_type = "DANGER" if dist <= DANGER_DISTANCE_METERS else "DETECTION"
                    log_detection_event(
                        event_type=event_type,
                        distance=dist,
                        confidence=conf,
                        bbox=(x, y, w, h),
                        avoidance_triggered=False
                    )
                    
                    # Color coding based on distance
                    if dist <= DANGER_DISTANCE_METERS:
                        color = (0, 0, 255)      # RED - DANGER
                        thickness = 3
                        status = "† DANGER"
                    elif dist <= 4.0:
                        color = (0, 165, 255)    # ORANGE - WARNING
                        thickness = 2
                        status = "† WARNING"
                    else:
                        color = (0, 255, 255)    # YELLOW - SAFE
                        thickness = 2
                        status = " SAFE"
                    
                    # Draw bounding box
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)
                    
                    # Draw label with distance
                    label = f'{status} | {int(conf*100)}% | {dist:.1f}m'
                    label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                    
                    # Label background
                    cv2.rectangle(frame, 
                                (x, y - label_size[1] - 10), 
                                (x + label_size[0], y), 
                                color, -1)
                    
                    # Label text
                    cv2.putText(frame, label, (x, y - 5), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    # Draw warning if too close
                    if dist <= DANGER_DISTANCE_METERS:
                        cv2.putText(frame, "† OBSTACLE TOO CLOSE!", 
                                  (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.8, (0, 0, 255), 3)
        
        # Update global state
        person_distance = closest_distance
        detection_history.append(person_found and closest_distance <= DANGER_DISTANCE_METERS)
        
        # Trigger avoidance logic
        current_time = time.time()
        
        if person_found and closest_distance <= DANGER_DISTANCE_METERS:
            person_detection_count += 1
            
            # Check if enough consecutive detections
            danger_count = sum(detection_history)
            
            if (danger_count >= CONSECUTIVE_DETECTIONS and 
                not avoidance_active and 
                current_time - last_avoidance_time > AVOIDANCE_COOLDOWN):
                
                # Trigger avoidance
                avoidance_active = True
                person_detected = True
                last_avoidance_time = current_time
                
                # Log avoidance event (NEW)
                log_detection_event(
                    event_type="AVOIDANCE",
                    distance=closest_distance,
                    confidence=None,
                    bbox=None,
                    avoidance_triggered=True
                )
                
                # Execute in separate thread
                threading.Thread(target=execute_avoidance_maneuver, daemon=True).start()
        else:
            person_detection_count = max(0, person_detection_count - 1)
            if person_detection_count == 0:
                person_detected = False
        
        return frame, person_found, closest_distance
        
    except Exception as e:
        print(f"AI Detection Error: {e}")
        return frame, False, 999.0

# ========================================
# AVOIDANCE MANEUVER
# ========================================

def execute_avoidance_maneuver():
    """
    Execute obstacle avoidance maneuver. 
    Moves UAV backward by SAFE_DISTANCE_METERS. 
    """
    global avoidance_active, avoidance_count, pixhawk
    
    avoidance_count += 1
    
    print("\n" + "="*70)
    print(f"†  AVOIDANCE MANEUVER #{avoidance_count}")
    print(f"   Person Distance: {person_distance:.2f}m")
    print(f"   Danger Threshold: {DANGER_DISTANCE_METERS}m")
    print(f"   Action: Moving backward {SAFE_DISTANCE_METERS}m")
    print("="*70)
    
    if pixhawk:
        # Real flight mode
        if move_backward(SAFE_DISTANCE_METERS):
            print("   í Command sent to Pixhawk")
            time.sleep(3)  # Wait for movement
            stop_movement()
            print("   í Movement complete")
        else:
            print("   í Command failed")
    else:
        # Simulation mode
        print("   í [SIMULATION] Avoidance executed")
        time.sleep(2)
    
    print("="*70 + "\n")
    
    time.sleep(AVOIDANCE_COOLDOWN)
    avoidance_active = False

# ========================================
# FLASK WEB SERVER ROUTES
# ========================================

@app.route('/api/sensors')
def api_get_sensors():
    """
    REST API endpoint for sensor data.
    
    Returns:
        JSON: Current sensor readings
    """
    global pixhawk, pixhawk_armed, pixhawk_mode, pixhawk_battery_voltage, pixhawk_battery_percent
    
    with sensor_data_lock:
        # Read all sensors
        temperature, humidity = read_dht_sensor()
        gas_level = read_gas_sensor()
        
        # Prepare response
        response = {
            'temperature': round(temperature, 1) if temperature != -999.0 else -999.0,
            'humidity': round(humidity, 1) if humidity != -999.0 else -999.0,
            'gas_level': round(gas_level, 1),
            'person_detected': person_detected,
            'person_distance': round(person_distance, 2) if person_distance < 999 else None,
            'avoidance_active': avoidance_active,
            'pixhawk_connected': pixhawk is not None,
            'pixhawk_armed': pixhawk_armed if pixhawk else False,
            'pixhawk_mode': pixhawk_mode if pixhawk else "N/A",
            'pixhawk_battery_voltage': round(pixhawk_battery_voltage, 2) if pixhawk else 0.0,
            'pixhawk_battery_percent': pixhawk_battery_percent if pixhawk else 0,
            'ai_enabled': net is not None,
            'timestamp': time.time()
        }
        
        return jsonify(response)

@app. route('/video_feed')
def video_feed():
    """
    Video streaming route with AI overlay.
    Returns MJPEG stream.
    """
    return Response(
        generate_video_stream(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/')
def index():
    """Serve main dashboard HTML."""
    return send_from_directory('.', 'index.html')

@app.route('/api/status')
def api_status():
    """System status endpoint."""
    global pixhawk, pixhawk_armed, pixhawk_mode
    
    uptime_seconds = time.time() - system_start_time
    return jsonify({
        'system': 'Autonomous UAV Safety System',
        'version': '4.1 WITH DATA LOGGING',
        'developer': 'saiprasad-tech',
        'uptime_seconds': round(uptime_seconds, 1),
        'components': {
            'dht11': dht_sensor is not None,
            'gas_sensor': gas_sensor is not None,
            'camera': camera. isOpened() if camera else False,
            'ai_model': net is not None,
            'pixhawk': pixhawk is not None,
            'pixhawk_armed': pixhawk_armed if pixhawk else False,
            'pixhawk_mode': pixhawk_mode if pixhawk else "N/A",
            'data_logging': DATA_LOG_ENABLED and current_log_writer is not None
        },
        'statistics': {
            'frames_processed': frame_count,
            'persons_detected': detection_count,
            'avoidance_maneuvers': avoidance_count,
            'logged_entries': total_logged_entries
        },
        'data_logging': {
            'enabled': DATA_LOG_ENABLED,
            'directory': DATA_LOG_DIRECTORY,
            'interval_seconds': DATA_LOG_INTERVAL,
            'total_entries': total_logged_entries,
            'current_file_rows': current_log_row_count
        }
    })


# NEW: API endpoint to get logged data statistics
@app.route('/api/logging/stats')
def api_logging_stats():
    """
    REST API endpoint for data logging statistics.
    
    Returns:
        JSON: Logging statistics and file information
    """
    try:
        log_dir = Path(DATA_LOG_DIRECTORY)
        
        # Get list of log files
        sensor_files = list(log_dir.glob("sensor_data_*.csv")) if log_dir.exists() else []
        detection_files = list(log_dir.glob("detection_events_*.csv")) if log_dir.exists() else []
        
        # Calculate total size
        total_size = sum(f.stat().st_size for f in sensor_files + detection_files)
        
        return jsonify({
            'enabled': DATA_LOG_ENABLED,
            'directory': DATA_LOG_DIRECTORY,
            'interval_seconds': DATA_LOG_INTERVAL,
            'total_logged_entries': total_logged_entries,
            'current_file_rows': current_log_row_count,
            'sensor_log_files': len(sensor_files),
            'detection_log_files': len(detection_files),
            'total_size_bytes': total_size,
            'total_size_mb': round(total_size / (1024 * 1024), 2),
            'files': {
                'sensor_logs': [str(f. name) for f in sorted(sensor_files, reverse=True)[:5]],
                'detection_logs': [str(f.name) for f in sorted(detection_files, reverse=True)[:5]]
            }
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# NEW: API endpoint to download recent log data as JSON
@app.route('/api/logging/recent')
def api_logging_recent():
    """
    REST API endpoint to get recent logged data.
    Useful for quick visualization without downloading CSV. 
    
    Returns:
        JSON: Last 100 logged entries
    """
    try:
        log_dir = Path(DATA_LOG_DIRECTORY)
        sensor_files = sorted(log_dir.glob("sensor_data_*.csv"), reverse=True)
        
        if not sensor_files:
            return jsonify({'data': [], 'count': 0})
        
        # Read last 100 entries from most recent file
        recent_data = []
        with open(sensor_files[0], 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            recent_data = rows[-100:]  # Last 100 rows
        
        return jsonify({
            'data': recent_data,
            'count': len(recent_data),
            'source_file': str(sensor_files[0].name)
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

# ========================================
# VIDEO STREAMING GENERATOR
# ========================================

def generate_video_stream():
    """
    Generate video stream with AI overlays.
    Yields MJPEG frames for HTTP streaming.
    """
    global frame_count, pixhawk, pixhawk_armed, pixhawk_mode
    
    while True:
        if not camera or not camera.isOpened():
            # Generate error frame
            error_frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
            cv2.putText(error_frame, "Camera Offline", 
                       (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (255, 255, 255), 2)
            
            ret, jpeg = cv2.imencode('.jpg', error_frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(1)
            continue
        
        # Capture frame
        success, frame = camera.read()
        if not success:
            time.sleep(0.1)
            continue
        
        # AI person detection
        frame, detected, distance = detect_persons_in_frame(frame)
        
        # Add timestamp overlay
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, timestamp, 
                   (10, frame.shape[0] - 10), 
                   cv2. FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Add AI status overlay
        if net:
            if detected and distance <= DANGER_DISTANCE_METERS:
                ai_text = f"AI: DANGER {distance:.1f}m!"
                color = (0, 0, 255)
            elif detected:
                ai_text = f"AI: TRACKING {distance:.1f}m"
                color = (0, 255, 255)
            else:
                ai_text = "AI: MONITORING"
                color = (0, 255, 0)
        else:
            ai_text = "AI: OFFLINE"
            color = (128, 128, 128)
        
        cv2.putText(frame, ai_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Add Pixhawk status with ARM/DISARM
        if pixhawk:
            if pixhawk_armed:
                px_text = f"PIXHAWK: ARMED ({pixhawk_mode})"
                px_color = (0, 0, 255)  # RED when armed
            else:
                px_text = f"PIXHAWK: DISARMED ({pixhawk_mode})"
                px_color = (0, 255, 0)  # GREEN when disarmed
        else:
            px_text = "PIXHAWK: SIM"
            px_color = (255, 165, 0)  # ORANGE for simulation
        
        cv2. putText(frame, px_text, (10, 60), 
                   cv2. FONT_HERSHEY_SIMPLEX, 0.7, px_color, 2)
        
        # Add data logging indicator (NEW)
        if DATA_LOG_ENABLED and current_log_writer:
            log_text = f"LOG: {total_logged_entries} entries"
            log_color = (0, 255, 0)  # Green
        else:
            log_text = "LOG: OFF"
            log_color = (128, 128, 128)  # Gray
        
        cv2.putText(frame, log_text, (frame.shape[1] - 180, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, log_color, 2)
        
        # Encode frame as JPEG
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue
        
        frame_count += 1
        
        # Yield frame in MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg. tobytes() + b'\r\n')

# ========================================
# MAIN ENTRY POINT
# ========================================

if __name__ == '__main__':
    print("\n" + "="*70)
    print("   =Ä STARTING WEB SERVER")
    print("="*70)
    print(f"\n   =· Dashboard URL: http://<YOUR_PI_IP>:5000")
    print(f"   =  API Endpoint: http://<YOUR_PI_IP>:5000/api/sensors")
    print(f"   =» Status Endpoint: http://<YOUR_PI_IP>:5000/api/status")
    print(f"   =¡ Logging Stats: http://<YOUR_PI_IP>:5000/api/logging/stats")
    print(f"   =À Recent Data: http://<YOUR_PI_IP>:5000/api/logging/recent")
    print(f"\n   =À System Status:")
    print(f"      <!  DHT11 Sensor: {' ONLINE' if dht_sensor else ' OFFLINE'}")
    print(f"      =® Gas Sensor: {' ONLINE' if gas_sensor else ' OFFLINE'}")
    print(f"      =˘ Camera: {' ONLINE' if camera and camera.isOpened() else ' OFFLINE'}")
    print(f"      > AI Model: {' LOADED' if net else ' NOT LOADED'}")
    print(f"      =Å Pixhawk: {' CONNECTED' if pixhawk else '† SIMULATION'}")
    if pixhawk:
        print(f"        Armed: {'YES †' if pixhawk_armed else 'NO '}")
        print(f"        Mode: {pixhawk_mode}")
    print(f"      =  Data Logging: {' ENABLED' if DATA_LOG_ENABLED else ' DISABLED'}")
    if DATA_LOG_ENABLED:
        print(f"        Directory: {DATA_LOG_DIRECTORY}/")
        print(f"        Interval: {DATA_LOG_INTERVAL}s")
    print(f"\n   ô  Configuration:")
    print(f"      =œ Danger Distance: {DANGER_DISTANCE_METERS}m")
    print(f"      =·  Safe Distance: {SAFE_DISTANCE_METERS}m")
    print(f"      Ò  Avoidance Cooldown: {AVOIDANCE_COOLDOWN}s")
    print(f"      <Ø AI Confidence: {AI_CONFIDENCE_THRESHOLD*100}%")
    print(f"      = Consecutive Detections: {CONSECUTIVE_DETECTIONS}")
    
    # Bluetooth info
    print(f"\n   =Ò Bluetooth MAVLink Bridge:")
    print(f"      To connect Mission Planner via Bluetooth:")
    print(f"      1. Run: python3 mavlink_bt_bridge.py (in another terminal)")
    print(f"      2.Pair your PC with this Raspberry Pi")
    print(f"      3. In Mission Planner: Connect í Bluetooth í 'Pixhawk MAVLink'")
    
    print("\n" + "="*70 + "\n")
    
    try:
        app.run(
            host='0.0.0.0',      # Listen on all interfaces
            port=5000,           # HTTP port
            debug=False,         # Production mode
            threaded=True,       # Enable threading
            use_reloader=False   # Disable auto-reloader
        )
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("   =— SHUTDOWN INITIATED")
        print("="*70)
        
        # Print final statistics
        uptime = time.time() - system_start_time
        print(f"\n   =  Session Statistics:")
        print(f"      Ò  Uptime: {uptime:.1f} seconds ({uptime/60:.1f} minutes)")
        print(f"      <¨ Frames Processed: {frame_count}")
        print(f"      =d Persons Detected: {detection_count}")
        print(f"      =Å Avoidance Maneuvers: {avoidance_count}")
        print(f"      =› Data Entries Logged: {total_logged_entries}")
        
        # Close data logging (NEW)
        close_data_logging()
        
        # Cleanup
        if camera and camera.isOpened():
            camera.release()
            print(f"\n    Camera released")
        
        if pixhawk:
            pixhawk.close()
            print("    Pixhawk disconnected")
        
        if dht_sensor:
            dht_sensor.exit()
            print("    DHT sensor closed")
        
        print("\n   =K System shutdown complete.Goodbye!")
        print("="*70 + "\n")
