"""
Test Kinect VFH Module Only
Test obstacle detection dan VFH navigation
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules import KinectVFH, ArduinoComm
from config import Config
import cv2
import numpy as np
import time

def test_vfh_navigation():
    print("=" * 60)
    print("🧪 TEST: VFH Navigation Only")
    print("=" * 60)
    print()
    
    print("Initializing Kinect VFH...")
    vfh = KinectVFH(
        threshold_distance=Config.VFH_THRESHOLD_DISTANCE,
        num_sectors=Config.VFH_NUM_SECTORS,
        turn_duration=Config.VFH_TURN_DURATION,
        forward_duration=Config.VFH_FORWARD_DURATION
    )
    
    print("Initializing Arduino...")
    arduino = ArduinoComm(
        port=Config.ARDUINO_PORT,
        baudrate=Config.ARDUINO_BAUDRATE
    )
    
    print()
    print("=" * 60)
    print("🚗 VFH Navigation Active")
    print("   Place obstacles in front of Kinect")
    print("   [Q] = Quit")
    print("   [A/D] = Rotate view")
    print("   [W/S] = Zoom in/out")
    print("=" * 60)
    print()
    
    cv2.namedWindow("VFH Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("VFH Test", 1280, 720)
    
    try:
        while True:
            rgb, depth = vfh.get_frames()
            
            if rgb is None or depth is None:
                continue
            
            sector_status = vfh.analyze_sectors(depth)
            histogram = vfh.calculate_histogram(depth)
            path_clear = vfh.is_all_clear(sector_status)
            command = vfh.get_vfh_command(sector_status)
            
            arduino.send_command(command)
            
            canvas = vfh.get_point_cloud_canvas(rgb, depth, 1280, 720)
            
            positions = [128, 384, 640, 896, 1152]
            for i, label in enumerate(Config.VFH_SECTOR_LABELS):
                status = sector_status[label]
                color = (0, 0, 255) if status == "High" else (0, 255, 0)
                
                x = positions[i]
                cv2.circle(canvas, (x, 80), 15, color, -1)
                cv2.circle(canvas, (x, 80), 18, (255, 255, 255), 2)
                cv2.putText(canvas, label, (x - 40, 110),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            center = (1080, 600)
            radius = 100
            cv2.circle(canvas, center, radius, (255, 255, 255), 1)
            for i, val in enumerate(histogram):
                theta = np.radians(i * (180 / len(histogram)))
                r = radius * (val / 2.0)
                x = int(center[0] + r * np.cos(theta))
                y = int(center[1] - r * np.sin(theta))
                cv2.line(canvas, center, (x, y), (0, 255, 0), 2)
            
            status_text = "PATH CLEAR" if path_clear else "OBSTACLE DETECTED"
            status_color = (0, 255, 0) if path_clear else (0, 0, 255)
            
            cv2.rectangle(canvas, (20, 150), (400, 280), (0, 0, 0), -1)
            cv2.rectangle(canvas, (20, 150), (400, 280), (255, 255, 255), 2)
            
            cv2.putText(canvas, status_text, (30, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(canvas, f"Command: {command}", (30, 220),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            elapsed = time.time() - vfh.command_start_time
            cv2.putText(canvas, f"Timer: {elapsed:.2f}s", (30, 260),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            
            cv2.imshow("VFH Test", canvas)
            
            print(f"Sectors: {sector_status} | Command: {command}")
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('a'):
                vfh.angle_y -= 5
            elif key == ord('d'):
                vfh.angle_y += 5
            elif key == ord('w'):
                vfh.zoom *= 1.1
            elif key == ord('s'):
                vfh.zoom /= 1.1
    
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    
    finally:
        print("\nCleaning up...")
        arduino.send_command("STOP")
        arduino.close()
        vfh.cleanup()
        cv2.destroyAllWindows()
        print("✅ Test completed")

if __name__ == "__main__":
    test_vfh_navigation()
