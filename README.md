# 🤖 DDMR-VFH-Collision-Avoidance


https://github.com/user-attachments/assets/d18cc994-d7cc-46e8-96df-8fb7225a1beb


The autonomous robot navigation system uses **Vector Field Histogram (VFH)** for obstacle avoidance and **ArUco marker tracking** for goal-seeking navigation.

## 📋 Features

- ✅ **VFH Obstacle Avoidance**: Detect and avoid obstacles in real time
- ✅ **ArUco Marker Tracking**: Track and approach ArUco markers as targets
- ✅ **Hybrid Decision Logic**: Smart priority-based decision making
- ✅ **Modular Architecture**: Clean, maintainable, and extensible code
- ✅ **Real-time Visualization**: Point cloud + ArUco overlay + status display
- ✅ **Arduino Integration**: Serial communication for motor control

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

## 🎯 Operation Mode

### 1. VFH_ACTIVE (Priority 1)
- **Trigger**: Obstacle detected in one of the sectors
- **Behavior**: Avoid the obstacle by turning left/right
- **Priority**: Highest (Safety First!)

### 2. ARUCO_TRACKING (Priority 2)
- **Trigger**: Path clear + ArUco marker detected
- **Behavior**: Follow marker (left/center/right zone)
- **Priority**: Medium (Goal Seeking)

### 3. SEARCH (Priority 3)
- **Trigger**: Path clear + No ArUco detected
- **Behavior**: Stop or rotate (configurable)
- **Priority**: Lowest (Default State)
- 
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

Download and install the OpenNI2 SDK from:
- Windows: https://structure.io/openni
- Linux: `sudo apt-get install libopenni2-dev`


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
ARDUINO_PORT = 'COM5' 
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

## 🧪 Test Scenarios

### Scenario 1: VFH Obstacle Avoidance
1. Run the system without ArUco markers
2. Place an obstacle on the front sensor
3. Verify: The robot turns left/right to avoid obstacles

### Scenario 2: ArUco Tracking
1. Ensure the path is clear (no obstacles)
2. Show the ArUco marker to the camera
3. Verify: The robot turns to face the marker
4. Verify: The robot advances when the marker is centered

### Scenario 3: Hybrid Navigation
1. Show the ArUco marker + place an obstacle
2. Verify: VFH takes priority (avoid obstacles first)
3. Remove the obstacle
4. Verify: Continue ArUco tracking

## 📁 File Structure

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

### Kinect not detected
```bash
# Check OpenNI2 installation
# Windows: Check Device Manager
# Linux: lsusb | grep Kinect
```

### Arduino not connected
```bash
# Check COM port in Device Manager
# Edit config/settings.py: ARDUINO_PORT = 'COM_YOUR_PORT'
# Test: python tests/test_arduino_comm.py
```

### ArUco not detected
- Ensure sufficient lighting
- Print markers at least 10cm x 10cm
- Use high-contrast print (black and white)
- Check dictionary type: DICT_5X5_1000

### VFH always detects obstacles
- Tune threshold: `VFH_THRESHOLD_DISTANCE` in settings.py
- Check Kinect depth data is valid (not all zeros)

## 📝 Logs

System log tersimpan di folder `logs/`:
```
logs/robot_nav_YYYYMMDD_HHMMSS.log
```
