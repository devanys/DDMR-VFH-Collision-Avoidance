"""
Arduino Communication Module
Handle serial communication dengan Arduino motor controller
"""

import serial
import time

class ArduinoComm:
    def __init__(self, port='COM5', baudrate=9600, timeout=1):
        """
        Initialize Arduino serial connection
        
        Args:
            port (str): COM port Arduino (e.g., 'COM5', '/dev/ttyUSB0')
            baudrate (int): Communication speed (default 9600)
            timeout (int): Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.last_command = None
        self.connected = False
        
        self._connect()
    
    def _connect(self):
        """Internal method to establish serial connection"""
        try:
            self.serial = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=self.timeout
            )
            time.sleep(2) 
            self.connected = True
            print(f"✅ Arduino connected on {self.port}")
        except Exception as e:
            self.serial = None
            self.connected = False
            print(f"❌ Arduino connection failed: {e}")
            print(f"   System will run without Arduino (simulation mode)")
    
    def send_command(self, command):
        """
        Send command to Arduino
        
        Args:
            command (str): Command to send - "FORWARD", "LEFT", "RIGHT", "STOP"
        
        Returns:
            bool: True if sent successfully, False otherwise
        """
        valid_commands = ["FORWARD", "LEFT", "RIGHT", "STOP"]
        if command not in valid_commands:
            print(f"⚠️ Invalid command: {command}")
            return False
        
        if command == self.last_command:
            return True
        
        if self.connected and self.serial and self.serial.is_open:
            try:
                full_cmd = f"{command}\n"
                self.serial.write(full_cmd.encode())
                self.last_command = command
                print(f"📤 Arduino << {command}")
                return True
            except Exception as e:
                print(f"❌ Send error: {e}")
                self.connected = False
                return False
        else:
            print(f"🔵 [SIM] Arduino << {command}")
            self.last_command = command
            return True
    
    def is_connected(self):
        """
        Check if Arduino is connected
        
        Returns:
            bool: True if connected, False otherwise
        """
        return self.connected
    
    def get_last_command(self):
        """
        Get last sent command
        
        Returns:
            str: Last command sent, or None if no command sent yet
        """
        return self.last_command
    
    def reconnect(self):
        """Attempt to reconnect to Arduino"""
        print("🔄 Attempting to reconnect...")
        self.close()
        time.sleep(1)
        self._connect()
    
    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(b"STOP\n")
                time.sleep(0.1)
                self.serial.close()
                print("✅ Arduino connection closed")
            except Exception as e:
                print(f"⚠️ Error closing connection: {e}")
        
        self.serial = None
        self.connected = False
        self.last_command = None
