#!/usr/bin/python3

# --- YEH SCRIPT AAPKE RASPBERRY PI PAR CHALEGA ---
# 1. AI Person Detection (USB Webcam)
# 2. DHT11/22 Sensor (Temp/Humidity)
# 3. MQ Gas Sensor (Digital Pin)
# 4. WebSocket Server (Dashboard ko data bhejega)

import time
import adafruit_circuitpython_dht as adafruit_dht # YEH HAI SAHI WALA DHT LIBRARY
import RPi.GPIO as GPIO
import threading
import board
import numpy as np
import cv2
import tensorflow.lite as tflite # YEH HAI SAHI WALA TENSORFLOW IMPORT
import asyncio
import websockets
import json

# --- 1. PIN & SENSOR SETUP ---
DHT_PIN = board.D17     # GPIO 17 (Pin 11)
GAS_ALARM_PIN = 27    # GPIO 27 (Pin 13)
MODEL_FILENAME = 'person_detection.tflite' # AI model file
clients = set()

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(GAS_ALARM_PIN, GPIO.IN)

# Setup DHT Sensor
try:
    dht_sensor = adafruit_dht.DHT22(DHT_PIN, use_pulseio=False)
except RuntimeError:
    print("Failed to find DHT22, trying DHT11...")
    try:
        dht_sensor = adafruit_dht.DHT11(DHT_PIN, use_pulseio=False)
    except Exception as e:
        print(f"FATAL: Could not find DHT sensor. Error: {e}")
        exit()

# Shared data jo dashboard ko bhejenge
data_lock = threading.Lock()
sensor_data = {
    "temperature": 0.0,
    "humidity": 0.0,
    "gas_alarm": "CLEAR",
    "ai_status": "STARTING"
}

# --- 2. SENSOR THREADS ---
def read_climate_sensors():
    while True:
        try:
            temp = dht_sensor.temperature
            hum = dht_sensor.humidity
            if hum is not None and temp is not None:
                with data_lock:
                    sensor_data["temperature"] = temp
                    sensor_data["humidity"] = hum
            time.sleep(2)
        except RuntimeError:
            time.sleep(2)
        except Exception as e:
            dht_sensor.exit()
            return

def read_gas_alarm():
    while True:
        try:
            gas_detected = GPIO.input(GAS_ALARM_PIN)
            with data_lock:
                # Active-low (0 = ALARM)
                if gas_detected == GPIO.LOW: 
                    sensor_data["gas_alarm"] = "!!! ALARM: GAS DETECTED !!!"
                else:
                    sensor_data["gas_alarm"] = "Clear"
            time.sleep(0.1)
        except Exception:
            return

def run_ai_vision():
    try:
        interpreter = tflite.Interpreter(model_path=MODEL_FILENAME)
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]
        camera = cv2.VideoCapture(0) # USB Webcam
        
        if not camera.isOpened():
            with data_lock:
                sensor_data["ai_status"] = "WEBCAM FAILED"
            return
            
    except Exception as e:
        with data_lock:
            sensor_data["ai_status"] = "AI FAILED TO LOAD"
        return

    while True:
        try:
            ret, frame = camera.read()
            if not ret:
                time.sleep(0.1)
                continue
            image_resized = cv2.resize(frame, (width, height))
            input_data = np.expand_dims(image_resized, axis=0)
            interpreter.set_tensor(input_details[0]['index'], input_data)
            interpreter.invoke()
            output_data = interpreter.get_tensor(output_details[0]['index'])
            scores = output_data[0]
            person_score = scores[1]
            
            with data_lock:
                if person_score > 0.6:
                    sensor_data["ai_status"] = "!!! ALERT: PERSON DETECTED !!!"
                else:
                    sensor_data["ai_status"] = "Clear"
        except Exception:
            time.sleep(1)

# --- 3. THE WEB SERVER (Dashboard ko data bhejega) ---
async def data_server(websocket, path):
    clients.add(websocket)
    print("Dashboard connected!")
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)

async def broadcast_data():
    while True:
        with data_lock:
            data_copy = sensor_data.copy()
        
        message = json.dumps(data_copy)
        if clients:
            await asyncio.wait([client.send(message) for client in clients])
        await asyncio.sleep(0.5)

# --- 4. MAIN PROGRAM ---
async def main():
    print("Sensor threads shuru ho rahe hain...")
    threading.Thread(target=read_climate_sensors, daemon=True).start()
    threading.Thread(target=read_gas_alarm, daemon=True).start()
    threading.Thread(target=run_ai_vision, daemon=True).start()
    
    print("WebSocket server shuru ho raha hai port 8765 par...")
    server = await websockets.serve(data_server, "localhost", 8765)
    
    await broadcast_data()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nBand kar raha hoon...")
        GPIO.cleanup()
        print("Goodbye.")