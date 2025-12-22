"""
Logger Utility
Simple logging untuk debugging dan monitoring
"""

import os
from datetime import datetime

class Logger:
    def __init__(self, enabled=True, log_folder="logs"):
        """
        Initialize logger
        
        Args:
            enabled (bool): Enable/disable logging
            log_folder (str): Folder untuk menyimpan log files
        """
        self.enabled = enabled
        self.log_folder = log_folder
        self.log_file = None
        
        if self.enabled:
            self._create_log_file()
    
    def _create_log_file(self):
        """Create log file dengan timestamp"""
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robot_nav_{timestamp}.log"
        self.log_file = os.path.join(self.log_folder, filename)
        
        with open(self.log_file, 'w') as f:
            f.write("="*60 + "\n")
            f.write("ROBOT NAVIGATION SYSTEM LOG\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("="*60 + "\n\n")
    
    def log(self, message, level="INFO"):
        """
        Write log message
        
        Args:
            message (str): Message to log
            level (str): Log level - INFO, WARNING, ERROR, DEBUG
        """
        if not self.enabled:
            return
        
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {level}: {message}\n"
        
        try:
            with open(self.log_file, 'a') as f:
                f.write(log_entry)
        except Exception as e:
            print(f"⚠️ Logging error: {e}")
    
    def info(self, message):
        """Log INFO level message"""
        self.log(message, "INFO")
    
    def warning(self, message):
        """Log WARNING level message"""
        self.log(message, "WARNING")
    
    def error(self, message):
        """Log ERROR level message"""
        self.log(message, "ERROR")
    
    def debug(self, message):
        """Log DEBUG level message"""
        self.log(message, "DEBUG")
