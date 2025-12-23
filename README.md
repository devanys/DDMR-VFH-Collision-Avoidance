# 🤖 DDMR-VFH-Collision-Avoidance


https://github.com/user-attachments/assets/d18cc994-d7cc-46e8-96df-8fb7225a1beb


Sistem navigasi robot autonomous menggunakan **Vector Field Histogram (VFH)** untuk obstacle avoidance dan **ArUco marker tracking** untuk goal-seeking navigation.

## 📋 Fitur

- ✅ **VFH Obstacle Avoidance**: Deteksi dan hindari obstacle secara real-time
- ✅ **ArUco Marker Tracking**: Track dan approach ArUco marker sebagai target
- ✅ **Hybrid Decision Logic**: Smart priority-based decision making
- ✅ **Modular Architecture**: Clean, maintainable, dan extensible code
- ✅ **Real-time Visualization**: Point cloud + ArUco overlay + status display
- ✅ **Arduino Integration**: Serial communication untuk motor control

## 🏗️ Arsitektur Sistem

```
┌─────────────────────────────────────────┐
│         MAIN CONTROLLER                 │
│      (Decision Logic)                   │
└───────────┬─────────────────────────────┘
            │
    ┌───────┴───────┬──────────┬──────────┐
    │               │          │          │
    ▼               ▼          ▼          ▼
┌────────┐   ┌──────────┐ ┌────────┐ ┌────────┐
│ Kinect │   │  ArUco   │ │Arduino │ │ Config │
│  VFH   │   │ Tracker  │ │  Comm  │ │Settings│
└────────┘   └──────────┘ └────────┘ └────────┘
```

## 🎯 Mode Operasi

### 1. VFH_ACTIVE (Priority 1)
- **Trigger**: Obstacle terdeteksi di salah satu sector
- **Behavior**: Hindari obstacle dengan turn left/right
- **Priority**: Tertinggi (Safety First!)

### 2. ARUCO_TRACKING (Priority 2)
- **Trigger**: Path clear + ArUco marker detected
- **Behavior**: Follow marker (left/center/right zone)
- **Priority**: Sedang (Goal Seeking)

### 3. SEARCH (Priority 3)
- **Trigger**: Path clear + No ArUco detected
- **Behavior**: Stop atau rotate (configurable)
- **Priority**: Terendah (Default State)

## 📦 Instalasi

### 1. Requirements

```bash
# Hardware
- Kinect v1 sensor (Xbox 360 Kinect)
- Arduino Uno dengan motor driver
- 2 DC motors

# Software
- Python 3.8+
- OpenNI2 drivers
- Arduino IDE
```

### 2. Install Dependencies

```bash
cd robot_navigation_mvc
pip install -r requirements.txt
```

### 3. OpenNI2 Setup

Download dan install OpenNI2 SDK dari:
- Windows: https://structure.io/openni
- Linux: `sudo apt-get install libopenni2-dev`

### 4. Upload Arduino Code

Upload `arduino_motor_control.ino` ke Arduino board.

## 🚀 Cara Menjalankan

### Test Individual Modules

```bash
# Test Arduino communication
python tests/test_arduino_comm.py

# Test VFH only
python tests/test_vfh_only.py

# Test ArUco tracking only
python tests/test_aruco_only.py
```

### Run Full System

```bash
python main.py
```

## ⌨️ Keyboard Controls

### Main System
- **Q**: Quit program
- **1**: VFH Only mode (disable ArUco)
- **2**: ArUco Only mode (disable VFH)
- **3**: Hybrid mode (default)
- **Space**: Emergency stop

### VFH Test
- **A/D**: Rotate view left/right
- **W/S**: Zoom in/out
- **Q**: Quit

## ⚙️ Configuration

Edit `config/settings.py` untuk tuning parameters:

```python
# VFH Settings
VFH_THRESHOLD_DISTANCE = 1.0  # meter
VFH_TURN_DURATION = 0.2       # seconds
VFH_FORWARD_DURATION = 0.4    # seconds

# ArUco Settings
ARUCO_DICT_TYPE = 'DICT_5X5_1000'
ARUCO_APPROACH_DISTANCE = 0.5  # meter

# Arduino Settings
ARDUINO_PORT = 'COM5'  # Ganti sesuai port Arduino Anda
```

## 📊 Decision Logic

```python
IF obstacle_detected:
    MODE = VFH_ACTIVE
    COMMAND = vfh_command
    
ELIF aruco_detected AND path_clear:
    IF distance < approach_distance:
        MODE = ARUCO_REACHED
        COMMAND = STOP
    ELSE:
        MODE = ARUCO_TRACKING
        COMMAND = aruco_command
        
ELSE:
    MODE = SEARCH
    COMMAND = STOP
```

## 🧪 Testing Scenarios

### Scenario 1: VFH Obstacle Avoidance
1. Run system tanpa ArUco marker
2. Letakkan obstacle di depan sensor
3. Verify: Robot belok kiri/kanan hindari obstacle

### Scenario 2: ArUco Tracking
1. Pastikan path clear (no obstacle)
2. Show ArUco marker ke camera
3. Verify: Robot turn untuk face marker
4. Verify: Robot forward saat marker di center

### Scenario 3: Hybrid Navigation
1. Show ArUco marker + place obstacle
2. Verify: VFH takes priority (hindari obstacle dulu)
3. Remove obstacle
4. Verify: Resume ArUco tracking

## 📁 Struktur File

```
robot_navigation_mvc/
├── main.py                      # Main controller
├── requirements.txt             # Dependencies
├── README.md                    # Documentation
│
├── config/
│   ├── __init__.py
│   └── settings.py             # Configuration parameters
│
├── modules/
│   ├── __init__.py
│   ├── kinect_vfh.py          # VFH navigation module
│   ├── aruco_tracker.py       # ArUco tracking module
│   └── arduino_comm.py        # Arduino communication
│
├── utils/
│   ├── __init__.py
│   └── logger.py              # Logging utility
│
└── tests/
    ├── test_arduino_comm.py   # Test Arduino
    ├── test_vfh_only.py       # Test VFH only
    └── test_aruco_only.py     # Test ArUco only
```

## 🐛 Troubleshooting

### Kinect tidak terdeteksi
```bash
# Check OpenNI2 installation
# Windows: Check Device Manager
# Linux: lsusb | grep Kinect
```

### Arduino tidak connect
```bash
# Check COM port di Device Manager
# Edit config/settings.py: ARDUINO_PORT = 'COM_YOUR_PORT'
# Test: python tests/test_arduino_comm.py
```

### ArUco tidak terdeteksi
- Pastikan lighting cukup
- Print marker minimal 10cm x 10cm
- Gunakan high contrast print (hitam-putih)
- Check dictionary type: DICT_5X5_1000

### VFH selalu detect obstacle
- Tuning threshold: `VFH_THRESHOLD_DISTANCE` di settings.py
- Check Kinect depth data valid (tidak all zeros)

## 📝 Logs

System log tersimpan di folder `logs/`:
```
logs/robot_nav_YYYYMMDD_HHMMSS.log
```
