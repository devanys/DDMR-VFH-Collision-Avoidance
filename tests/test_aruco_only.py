"""
Test ArUco Tracking Module Only
Test detection dan tracking ArUco marker menggunakan Kinect RGB
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules import ArucoTracker, ArduinoComm
from config import Config
from openni import openni2
import numpy as np
import cv2
import time

def test_aruco_tracking():
    print("=" * 60)
    print("🧪 TEST: ArUco Tracking Only")
    print("=" * 60)
    print()
    
    print("Initializing Kinect RGB camera...")
    try:
        openni2.initialize()
        dev = openni2.Device.open_any()
        color_stream = dev.create_color_stream()
        color_stream.start()
        print("✅ Kinect RGB initialized")
    except Exception as e:
        print(f"❌ Kinect initialization failed: {e}")
        return
    
    print("Initializing ArUco tracker...")
    aruco = ArucoTracker(
        dict_type=Config.ARUCO_DICT_TYPE,
        frame_width=Config.ARUCO_FRAME_WIDTH,
        frame_height=Config.ARUCO_FRAME_HEIGHT
    )
    
    print("Initializing Arduino...")
    arduino = ArduinoComm(
        port=Config.ARDUINO_PORT,
        baudrate=Config.ARDUINO_BAUDRATE
    )
    
    print()
    print("=" * 60)
    print("📸 ArUco Tracking Active")
    print("   Show ArUco marker to camera")
    print("   [Q] = Quit")
    print("=" * 60)
    print()
    
    cv2.namedWindow("ArUco Tracking Test", cv2.WINDOW_NORMAL)
    
    last_command = None
    
    try:
        while True:
            color_frame = color_stream.read_frame()
            color_data = color_frame.get_buffer_as_uint8()
            rgb = np.frombuffer(color_data, dtype=np.uint8).reshape(480, 640, 3)
            
            result = aruco.detect(rgb)
            
            command = aruco.get_tracking_command(result)
            
            if command and command != last_command:
                arduino.send_command(command)
                last_command = command
                print(f"Command: {command}")
            elif not result['detected'] and last_command != "STOP":
                arduino.send_command("STOP")
                last_command = "STOP"
                print("Command: STOP (no marker)")
            
            display_frame = aruco.draw_overlay(rgb, result, draw_zones=True)
            
            h, w = display_frame.shape[:2]
            status = "DETECTED" if result['detected'] else "NOT DETECTED"
            color = (0, 255, 0) if result['detected'] else (0, 0, 255)
            
            cv2.putText(display_frame, f"Status: {status}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            if result['detected']:
                info = f"ID: {result['marker_id']} | Zone: {result['zone']} | Cmd: {command}"
                cv2.putText(display_frame, info, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            cv2.imshow("ArUco Tracking Test", display_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    
    finally:
        print("\nCleaning up...")
        arduino.send_command("STOP")
        arduino.close()
        color_stream.stop()
        openni2.unload()
        cv2.destroyAllWindows()
        print("✅ Test completed")

if __name__ == "__main__":
    test_aruco_tracking()
