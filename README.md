# Autonomous UAV Safety System (Major Project)

This repository contains my **Major Project**: an onboard safety + monitoring system where a **Raspberry Pi acts as a companion computer for a Pixhawk flight controller**.

The Pi reads sensors (temperature/humidity + gas), runs real-time **AI person detection**, estimates distance, and shows everything on a **web dashboard**. If a person/obstacle is detected within a safety threshold, the system can trigger an avoidance action (or run in simulation mode if Pixhawk is not connected).

---

## What this project does

A Raspberry Pi 5 runs a Flask server that streams the camera feed, overlays AI detections (YOLOv4-tiny), reads environment sensors, and (optionally) communicates with Pixhawk over MAVLink via UART. The dashboard shows live sensor values, AI status, Pixhawk connection/arming/mode, and the system logs sensor + detection events to CSV for later analysis.

---

## Hardware used

- Raspberry Pi 5 (recommended)
- Pixhawk flight controller (connected via UART / MAVLink)
- USB camera
- DHT11 / DHT22 temperature & humidity sensor
- MQ-series gas sensor module (digital output)

---

## How the system is arranged (Pi as companion computer)

- **Pixhawk** handles flight stabilization and control.
- **Raspberry Pi** runs the dashboard + AI + sensor monitoring.
- Communication is done using **MAVLink** over the Pi’s UART (`/dev/ttyAMA0`) at **57600 baud** (configurable in `app.py`).

If Pixhawk is not connected, the project runs in **Simulation Mode** and logs what it would have sent to Pixhawk.

---

## Repository files

- `app.py` — Main program (Flask dashboard, REST API, camera stream, YOLO detection, distance estimation, Pixhawk monitoring/control, CSV logging).
- `index.html` — Dashboard UI (polls `/api/sensors`).
- `yolov4-tiny.cfg` — YOLOv4-tiny config file.
- `coco.names` — COCO class labels used by YOLO (person class = 0).
- `labels.txt` — Alternate labels list (kept for reference).
- `setup_bluetooth.sh` — Bluetooth discoverable script (useful for Mission Planner Bluetooth workflows).
- `final_dht_test.py` — Comprehensive DHT11/DHT22 test script.
- `fix_app.py` — Utility script to fix spacing issues if `app.py` gets corrupted during copy/paste.
- `working_brain.py` — Experimental/reference script (WebSocket + TFLite approach).

---

## Setup (Raspberry Pi)

### 1) Enable UART

Enable serial hardware in `raspi-config`:
- Interface Options → Serial Port
- Disable login shell over serial
- Enable serial port hardware

Reboot after enabling.

### 2) Install dependencies

Recommended base packages:

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3-opencv
```

Create a virtual environment and install Python packages:

```bash
python3 -m venv .venv
source .venv/bin/activate

pip install flask numpy opencv-python adafruit-blinka adafruit-circuitpython-dht pymavlink lgpio
```

> Note: On Raspberry Pi, OpenCV from apt (`python3-opencv`) is often smoother than pip builds.

---

## YOLO weights (required for AI detection)

This repo includes the YOLO config (`yolov4-tiny.cfg`) but does **not** include `yolov4-tiny.weights` (large file).

Download YOLOv4-tiny weights and place it next to `app.py`:
- File name expected by `app.py`: `yolov4-tiny.weights`

If the weights file is missing, the system will still run but AI detection will be disabled.

---

## Run

```bash
python3 app.py
```

Open:
- Dashboard: `http://<PI_IP>:5000/`
- Sensors API: `http://<PI_IP>:5000/api/sensors`
- Status API: `http://<PI_IP>:5000/api/status`

---

## GPIO / wiring notes

Defaults in `app.py`:
- DHT: **GPIO17** (physical pin 11)
- MQ gas digital: **GPIO23** (physical pin 16)
- Pixhawk UART: GPIO14/15 → `/dev/ttyAMA0`

Always share **GND** between Raspberry Pi and Pixhawk.

---

## Data logging

When enabled, logs are saved in `sensor_logs/` as CSV files:
- `sensor_data_<timestamp>.csv`
- `detection_events_<timestamp>.csv`

Useful for reports/graphs during project submission.

---

## Author

**saiprasad-tech** — Major Project
