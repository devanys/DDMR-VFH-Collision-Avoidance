"""
Test Arduino Communication Module
Test koneksi dan command ke Arduino secara manual
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules import ArduinoComm
from config import Config
import time

def test_arduino():
    print("=" * 60)
    print("🧪 TEST: Arduino Communication")
    print("=" * 60)
    print()
    
    print("Initializing Arduino...")
    arduino = ArduinoComm(
        port=Config.ARDUINO_PORT,
        baudrate=Config.ARDUINO_BAUDRATE
    )
    
    if not arduino.is_connected():
        print("⚠️ Arduino not connected - Running in simulation mode")
    
    print()
    print("Manual Command Test")
    print("Commands:")
    print("  F = FORWARD")
    print("  L = LEFT")
    print("  R = RIGHT")
    print("  S = STOP")
    print("  Q = QUIT")
    print("=" * 60)
    print()
    
    try:
        while True:
            key = input("Enter command (F/L/R/S/Q): ").strip().upper()
            
            if key == 'Q':
                print("Quitting...")
                break
            
            elif key == 'F':
                arduino.send_command("FORWARD")
                print("✅ Sent: FORWARD")
            
            elif key == 'L':
                arduino.send_command("LEFT")
                print("✅ Sent: LEFT")
            
            elif key == 'R':
                arduino.send_command("RIGHT")
                print("✅ Sent: RIGHT")
            
            elif key == 'S':
                arduino.send_command("STOP")
                print("✅ Sent: STOP")
            
            else:
                print("❌ Invalid command")
            
            print(f"Last command: {arduino.get_last_command()}")
            print()
    
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    
    finally:
        print("\nClosing connection...")
        arduino.close()
        print("✅ Test completed")

if __name__ == "__main__":
    test_arduino()
