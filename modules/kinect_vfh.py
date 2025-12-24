import cv2
import numpy as np
from openni import openni2
import math
import time

class KinectVFH:
    def __init__(self, threshold_distance=1.0, num_sectors=5, 
                 turn_duration=0.2, forward_duration=0.4):
                     
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        
        self.threshold_distance = threshold_distance
        self.num_sectors = num_sectors
        self.turn_duration = turn_duration
        self.forward_duration = forward_duration
        
        self.sector_labels = ["Kiri", "Kiri-Depan", "Tengah", "Kanan-Depan", "Kanan"]
        
        self.current_command = "FORWARD"
        self.command_start_time = time.time()
        self.turn_direction = None
        
        self.angle_y = 0
        self.zoom = 1.5
        
        self._init_kinect()
        
        print("✅ Kinect VFH Module initialized")
        print(f"   Threshold: {threshold_distance}m | Sectors: {num_sectors}")
        print(f"   Turn duration: {turn_duration}s | Forward duration: {forward_duration}s")
    
    def _init_kinect(self):
        """Initialize Kinect sensor"""
        try:
            openni2.initialize()
            self.dev = openni2.Device.open_any()
            self.color_stream = self.dev.create_color_stream()
            self.depth_stream = self.dev.create_depth_stream()
            self.color_stream.start()
            self.depth_stream.start()
            print("✅ Kinect sensor initialized")
        except Exception as e:
            print(f"❌ Kinect initialization failed: {e}")
            raise
    
    def get_frames(self):

        try:
            color_frame = self.color_stream.read_frame()
            depth_frame = self.depth_stream.read_frame()
            
            color_data = color_frame.get_buffer_as_uint8()
            depth_data = depth_frame.get_buffer_as_uint16()
            
            rgb = np.frombuffer(color_data, dtype=np.uint8).reshape(480, 640, 3)
            depth = np.frombuffer(depth_data, dtype=np.uint16).reshape(480, 640)
            depth = depth.astype(np.float32) / 1000.0  # Convert mm to meters
            
            return rgb, depth
        
        except Exception as e:
            print(f"❌ Error reading frames: {e}")
            return None, None
    
    def analyze_sectors(self, depth_frame):

        if depth_frame is None:
            return {label: "Low" for label in self.sector_labels}
        
        rows, cols = depth_frame.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows))
        
        valid = depth_frame > 0
        z = depth_frame[valid]
        x = (c[valid] - self.cx) * z / self.fx
        y = (r[valid] - self.cy) * z / self.fy
        
        theta = math.radians(self.angle_y)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        x_rot = cos_t * x + sin_t * z
        z_rot = -sin_t * x + cos_t * z
        
        theta_point = np.arctan2(x_rot, z_rot)
        theta_point = theta_point % np.pi  # 0 to π
        
        sector_status = {}
        for i, label in enumerate(self.sector_labels):
            theta_min = i * (np.pi / self.num_sectors)
            theta_max = (i + 1) * (np.pi / self.num_sectors)
            
            mask_sector = (theta_point >= theta_min) & (theta_point < theta_max)
            
            if np.any(mask_sector):
                min_distance = np.min(z[mask_sector])
                sector_status[label] = "High" if min_distance < self.threshold_distance else "Low"
            else:
                sector_status[label] = "Low"
        
        return sector_status
    
    def calculate_histogram(self, depth_frame):
        histogram = np.zeros(self.num_sectors)
        
        if depth_frame is None:
            return histogram
        
        rows, cols = depth_frame.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows))
        
        valid = depth_frame > 0
        z = depth_frame[valid]
        x = (c[valid] - self.cx) * z / self.fx
        
        theta = math.radians(self.angle_y)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        x_rot = cos_t * x + sin_t * z
        z_rot = -sin_t * x + cos_t * z
        
        theta_point = np.arctan2(x_rot, z_rot)
        theta_point = theta_point % np.pi
        
        for i in range(self.num_sectors):
            theta_min = i * (np.pi / self.num_sectors)
            theta_max = (i + 1) * (np.pi / self.num_sectors)
            mask_sector = (theta_point >= theta_min) & (theta_point < theta_max)
            
            if np.any(mask_sector):
                min_distance = np.min(z[mask_sector])
                histogram[i] = min_distance if min_distance < 2 else 2
            else:
                histogram[i] = 2
        
        return histogram
    
    def is_all_clear(self, sector_status):
        return all(status == "Low" for status in sector_status.values())
    
    def get_vfh_command(self, sector_status):
        now = time.time()
        elapsed = now - self.command_start_time
        
        obstacle_detected = any(
            status == "High" for status in sector_status.values()
        )
        
        if not obstacle_detected:
            if self.current_command != "FORWARD":
                self.current_command = "FORWARD"
                self.command_start_time = now
            return self.current_command
        
        if sector_status.get("Kiri", "Low") == "High" or \
           sector_status.get("Kiri-Depan", "Low") == "High":
            self.turn_direction = "RIGHT"
        elif sector_status.get("Kanan", "Low") == "High" or \
             sector_status.get("Kanan-Depan", "Low") == "High":
            self.turn_direction = "LEFT"
        elif sector_status.get("Tengah", "Low") == "High":
            if sector_status.get("Kiri-Depan", "Low") == "Low":
                self.turn_direction = "LEFT"
            else:
                self.turn_direction = "RIGHT"
        
        if self.current_command == "FORWARD":
            if elapsed >= self.forward_duration:
                self.current_command = self.turn_direction
                self.command_start_time = now
        
        elif self.current_command in ["LEFT", "RIGHT"]:
            if elapsed >= self.turn_duration:
                self.current_command = "FORWARD"
                self.command_start_time = now
        
        return self.current_command
    
    def get_point_cloud_canvas(self, rgb_frame, depth_frame, 
                               display_width=1280, display_height=720):

        if rgb_frame is None or depth_frame is None:
            return np.zeros((display_height, display_width, 3), dtype=np.uint8)
        
        rows, cols = depth_frame.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows))
        
        valid = depth_frame > 0
        z = depth_frame[valid]
        x = (c[valid] - self.cx) * z / self.fx
        y = (r[valid] - self.cy) * z / self.fy
        
        theta = math.radians(self.angle_y)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        x_rot = cos_t * x + sin_t * z
        z_rot = -sin_t * x + cos_t * z
        
        x_vis = -x_rot
        x_2d = (x_vis * self.fx * self.zoom / (z_rot + 0.5)) + 320
        y_2d = (y * self.fy * self.zoom / (z_rot + 0.5)) + 240
        colors = rgb_frame[valid]
        
        canvas = np.zeros((display_height, display_width, 3), dtype=np.uint8)
        scale_x = display_width / 640
        scale_y = display_height / 480
        
        x_2d = (x_2d * scale_x).astype(np.int32)
        y_2d = (y_2d * scale_y).astype(np.int32)
        
        mask = (x_2d >= 0) & (x_2d < display_width) & \
               (y_2d >= 0) & (y_2d < display_height)
        
        canvas[y_2d[mask], x_2d[mask]] = colors[mask]
        
        return canvas
    
    def cleanup(self):
        """Stop streams and cleanup Kinect"""
        try:
            self.color_stream.stop()
            self.depth_stream.stop()
            openni2.unload()
            print("✅ Kinect cleanup completed")
        except Exception as e:
            print(f"⚠️ Cleanup error: {e}")
