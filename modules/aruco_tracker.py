"""
ArUco Marker Tracking Module
Detect dan track ArUco markers untuk target navigation
"""

import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoTracker:
    def __init__(self, dict_type='DICT_5X5_1000', frame_width=640, frame_height=480):
        """
        Initialize ArUco tracker
        
        Args:
            dict_type (str): ArUco dictionary type
            frame_width (int): Frame width untuk zone calculation
            frame_height (int): Frame height
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        dict_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_4X4_250': aruco.DICT_4X4_250,
            'DICT_4X4_1000': aruco.DICT_4X4_1000,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_5X5_250': aruco.DICT_5X5_250,
            'DICT_5X5_1000': aruco.DICT_5X5_1000,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_100': aruco.DICT_6X6_100,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_6X6_1000': aruco.DICT_6X6_1000,
        }
        
        if dict_type not in dict_map:
            print(f"⚠️ Unknown dictionary type: {dict_type}, using DICT_5X5_1000")
            dict_type = 'DICT_5X5_1000'
        
        self.aruco_dict = aruco.getPredefinedDictionary(dict_map[dict_type])
        self.parameters = aruco.DetectorParameters()
        
        self.left_bound = int(frame_width * 0.33)   # 1/3
        self.right_bound = int(frame_width * 0.67)  # 2/3
        
        print(f"✅ ArUco Tracker initialized ({dict_type})")
        print(f"   Zone boundaries: LEFT < {self.left_bound} | CENTER: {self.left_bound}-{self.right_bound} | RIGHT > {self.right_bound}")
    
    def detect(self, rgb_frame):
        """
        Detect ArUco marker dalam frame
        
        Args:
            rgb_frame: RGB image (numpy array)
        
        Returns:
            dict: Detection result dengan keys:
                - detected (bool): True jika marker terdeteksi
                - marker_id (int): ID marker (None jika tidak detect)
                - center_x (int): X coordinate center marker
                - center_y (int): Y coordinate center marker
                - corners (array): Corner points marker
                - zone (str): "LEFT", "CENTER", atau "RIGHT"
                - distance (float): Distance ke marker (None jika tidak ada depth)
        """
        result = {
            'detected': False,
            'marker_id': None,
            'center_x': None,
            'center_y': None,
            'corners': None,
            'zone': None,
            'distance': None
        }
        
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected = aruco.detectMarkers(
            gray, 
            self.aruco_dict, 
            parameters=self.parameters
        )
        
        if ids is not None and len(ids) > 0:
            marker_corners = corners[0][0]
            marker_id = ids[0][0]
            
            center_x = int(marker_corners[:, 0].mean())
            center_y = int(marker_corners[:, 1].mean())
            
            zone = self._get_zone(center_x)
            
            result['detected'] = True
            result['marker_id'] = int(marker_id)
            result['center_x'] = center_x
            result['center_y'] = center_y
            result['corners'] = corners[0]
            result['zone'] = zone
        
        return result
    
    def _get_zone(self, x_position):
        """
        Determine zone dari X position
        
        Args:
            x_position (int): X coordinate
        
        Returns:
            str: "LEFT", "CENTER", atau "RIGHT"
        """
        if x_position < self.left_bound:
            return "LEFT"
        elif x_position > self.right_bound:
            return "RIGHT"
        else:
            return "CENTER"
    
    def get_tracking_command(self, detection_result):
        """
        Generate command untuk tracking marker
        
        Args:
            detection_result (dict): Result dari detect()
        
        Returns:
            str: Command - "LEFT", "RIGHT", "FORWARD", atau None
        """
        if not detection_result['detected']:
            return None
        
        zone = detection_result['zone']
        
        if zone == "LEFT":
            return "RIGHT" 
        elif zone == "RIGHT":
            return "LEFT"  
        else:  
            return "FORWARD"
    
    def calculate_distance(self, depth_frame, center_x, center_y, sample_radius=5):
        """
        Calculate distance ke marker dari depth frame
        
        Args:
            depth_frame: Depth image (numpy array) dalam meter
            center_x (int): X coordinate marker center
            center_y (int): Y coordinate marker center
            sample_radius (int): Radius untuk sampling depth values
        
        Returns:
            float: Distance dalam meter, atau None jika invalid
        """
        if depth_frame is None:
            return None
        
        h, w = depth_frame.shape
        
        if center_x < 0 or center_x >= w or center_y < 0 or center_y >= h:
            return None
        
        y_min = max(0, center_y - sample_radius)
        y_max = min(h, center_y + sample_radius)
        x_min = max(0, center_x - sample_radius)
        x_max = min(w, center_x + sample_radius)
        
        depth_sample = depth_frame[y_min:y_max, x_min:x_max]
        
        valid_depths = depth_sample[depth_sample > 0]
        
        if len(valid_depths) == 0:
            return None
        
        return float(np.median(valid_depths))
    
    def draw_overlay(self, frame, detection_result, draw_zones=True):
        """
        Draw ArUco detection overlay pada frame
        
        Args:
            frame: Image frame (numpy array)
            detection_result (dict): Result dari detect()
            draw_zones (bool): Draw zone lines atau tidak
        
        Returns:
            frame: Frame dengan overlay
        """
        h, w = frame.shape[:2]
        
        if draw_zones:
            cv2.line(frame, (self.left_bound, 0), (self.left_bound, h), (0, 255, 255), 2)
            cv2.line(frame, (self.right_bound, 0), (self.right_bound, h), (0, 255, 255), 2)
            
            cv2.putText(frame, "LEFT", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, "CENTER", (self.left_bound + 50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, "RIGHT", (self.right_bound + 50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        if detection_result['detected']:
            corners = detection_result['corners']
            aruco.drawDetectedMarkers(frame, [corners], 
                                     np.array([[detection_result['marker_id']]]))
            
            cx, cy = detection_result['center_x'], detection_result['center_y']
            cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), 12, (255, 255, 255), 2)
            
            info_text = f"ID: {detection_result['marker_id']} | Zone: {detection_result['zone']}"
            cv2.putText(frame, info_text, (10, h - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if detection_result['distance'] is not None:
                dist_text = f"Distance: {detection_result['distance']:.2f}m"
                cv2.putText(frame, dist_text, (10, h - 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame
