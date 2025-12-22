import cv2
import numpy as np
import time
from datetime import datetime

from modules import KinectVFH, ArucoTracker, ArduinoComm
from config import Config
from utils import Logger

class RobotController:
    def __init__(self):
        print("=" * 60)
        print("🤖 ROBOT NAVIGATION SYSTEM - STARTING")
        print("=" * 60)
        
        # Initialize logger
        self.logger = Logger(
            enabled=Config.ENABLE_LOGGING, 
            log_folder=Config.LOG_FOLDER
        )
        self.logger.info("System initialization started")
        
        try:
            self.vfh = KinectVFH(
                threshold_distance=Config.VFH_THRESHOLD_DISTANCE,
                num_sectors=Config.VFH_NUM_SECTORS,
                turn_duration=Config.VFH_TURN_DURATION,
                forward_duration=Config.VFH_FORWARD_DURATION
            )
            self.logger.info("Kinect VFH Module initialized")
            
            self.aruco = ArucoTracker(
                dict_type=Config.ARUCO_DICT_TYPE,
                frame_width=Config.ARUCO_FRAME_WIDTH,
                frame_height=Config.ARUCO_FRAME_HEIGHT
            )
            self.logger.info("ArUco Tracker Module initialized")
            
            self.arduino = ArduinoComm(
                port=Config.ARDUINO_PORT,
                baudrate=Config.ARDUINO_BAUDRATE,
                timeout=Config.ARDUINO_TIMEOUT
            )
            self.logger.info("Arduino Communication initialized")
            
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            self.logger.error(f"Initialization failed: {e}")
            raise
        
        self.mode = "INIT"
        self.current_command = "STOP"
        self.running = True
        
        self.fps = 0
        self.frame_count = 0
        self.fps_start_time = time.time()
        
        self.force_mode = None 
        
        print("=" * 60)
        print("✅ All modules initialized successfully")
        print("=" * 60)
        print()
        
        self.logger.info("System ready")
    
    def run(self):
        """Main control loop"""
        print("🚀 System running...")
        print("   [Q] = Quit")
        print("   [1] = VFH Only Mode")
        print("   [2] = ArUco Only Mode")
        print("   [3] = Hybrid Mode (Default)")
        print("   [Space] = Emergency Stop")
        print("=" * 60)
        print()
        
        cv2.namedWindow("Robot Navigation System", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Robot Navigation System", Config.DISPLAY_WIDTH, Config.DISPLAY_HEIGHT)
        
        frame_time = 1.0 / Config.TARGET_FPS
        
        try:
            while self.running:
                loop_start_time = time.time()
                
                rgb_frame, depth_frame = self.vfh.get_frames()
                
                if rgb_frame is None or depth_frame is None:
                    print("⚠️ Failed to get frames")
                    continue
                
                sector_status = self.vfh.analyze_sectors(depth_frame)
                histogram = self.vfh.calculate_histogram(depth_frame)
                path_clear = self.vfh.is_all_clear(sector_status)
                vfh_command = self.vfh.get_vfh_command(sector_status)
                
                aruco_result = self.aruco.detect(rgb_frame)
                
                if aruco_result['detected']:
                    distance = self.aruco.calculate_distance(
                        depth_frame,
                        aruco_result['center_x'],
                        aruco_result['center_y']
                    )
                    aruco_result['distance'] = distance
                
                aruco_command = self.aruco.get_tracking_command(aruco_result)
                
                final_command = self.decision_logic(
                    path_clear,
                    sector_status,
                    vfh_command,
                    aruco_result,
                    aruco_command
                )
                
                if final_command != self.current_command:
                    self.arduino.send_command(final_command)
                    self.current_command = final_command
                    self.logger.info(f"Mode: {self.mode} | Command: {final_command}")
                
                display_frame = self.create_visualization(
                    rgb_frame,
                    depth_frame,
                    sector_status,
                    histogram,
                    aruco_result,
                    vfh_command,
                    aruco_command,
                    final_command
                )
                
                self.frame_count += 1
                if time.time() - self.fps_start_time >= 1.0:
                    self.fps = self.frame_count
                    self.frame_count = 0
                    self.fps_start_time = time.time()
                
                cv2.imshow("Robot Navigation System", display_frame)
                
                key = cv2.waitKey(1) & 0xFF
                if not self.handle_keyboard(key):
                    break
                
                elapsed = time.time() - loop_start_time
                if elapsed < frame_time:
                    time.sleep(frame_time - elapsed)
        
        except KeyboardInterrupt:
            print("\n⚠️ Interrupted by user")
            self.logger.warning("System interrupted by user")
        
        except Exception as e:
            print(f"\n❌ Error in main loop: {e}")
            self.logger.error(f"Main loop error: {e}")
        
        finally:
            self.cleanup()
    
    def decision_logic(self, path_clear, sector_status, vfh_cmd, 
                      aruco_result, aruco_cmd):
        """
        DECISION LOGIC - Tentukan command final
        
        Priority:
        1. VFH (Obstacle Avoidance) - Safety First!
        2. ArUco Tracking - Goal Seeking
        3. Search/Stop - Default
        
        Args:
            path_clear (bool): Semua sector clear?
            sector_status (dict): VFH sector status
            vfh_cmd (str): VFH command
            aruco_result (dict): ArUco detection result
            aruco_cmd (str): ArUco tracking command
        
        Returns:
            str: Final command
        """
        if self.force_mode == "VFH_ONLY":
            self.mode = "VFH_ACTIVE"
            return vfh_cmd
        
        elif self.force_mode == "ARUCO_ONLY":
            if aruco_result['detected']:
                self.mode = "ARUCO_TRACKING"
                return aruco_cmd
            else:
                self.mode = "SEARCH"
                return "STOP"
        
        if not path_clear:
            self.mode = "VFH_ACTIVE"
            return vfh_cmd
        
        if aruco_result['detected']:
            if aruco_result['distance'] is not None:
                if aruco_result['distance'] < Config.ARUCO_APPROACH_DISTANCE:
                    self.mode = "ARUCO_REACHED"
                    return "STOP"
            
            self.mode = "ARUCO_TRACKING"
            return aruco_cmd
        
        self.mode = "SEARCH"
        if Config.SEARCH_MODE_BEHAVIOR == "STOP":
            return "STOP"
        else:
            return "STOP"
    
    def create_visualization(self, rgb_frame, depth_frame, sector_status, 
                            histogram, aruco_result, vfh_cmd, aruco_cmd, final_cmd):
        """
        Create complete visualization
        
        Returns:
            numpy array: Display frame
        """
        canvas = self.vfh.get_point_cloud_canvas(
            rgb_frame, 
            depth_frame,
            Config.DISPLAY_WIDTH,
            Config.DISPLAY_HEIGHT
        )
        
        if aruco_result['detected']:
            scale_x = Config.DISPLAY_WIDTH / Config.ARUCO_FRAME_WIDTH
            scale_y = Config.DISPLAY_HEIGHT / Config.ARUCO_FRAME_HEIGHT
            
            cx = int(aruco_result['center_x'] * scale_x)
            cy = int(aruco_result['center_y'] * scale_y)
            
            cv2.circle(canvas, (cx, cy), 15, (0, 0, 255), -1)
            cv2.circle(canvas, (cx, cy), 20, (255, 255, 255), 3)
            
            left_bound = int(Config.ARUCO_LEFT_BOUNDARY * Config.DISPLAY_WIDTH)
            right_bound = int(Config.ARUCO_RIGHT_BOUNDARY * Config.DISPLAY_WIDTH)
            
            cv2.line(canvas, (left_bound, 0), (left_bound, Config.DISPLAY_HEIGHT), 
                    Config.COLOR_ZONE_LINE, 2)
            cv2.line(canvas, (right_bound, 0), (right_bound, Config.DISPLAY_HEIGHT), 
                    Config.COLOR_ZONE_LINE, 2)
        
        self._draw_vfh_sectors(canvas, sector_status)
        
        self._draw_polar_histogram(canvas, histogram)
        
        self._draw_status_panel(canvas, sector_status, aruco_result, 
                               vfh_cmd, aruco_cmd, final_cmd)
        
        return canvas
    
    def _draw_vfh_sectors(self, canvas, sector_status):
        """Draw VFH sector indicators"""
        h, w = canvas.shape[:2]
        
        positions = [
            int(w * 0.1),   # Kiri
            int(w * 0.3),   # Kiri-Depan
            int(w * 0.5),   # Tengah
            int(w * 0.7),   # Kanan-Depan
            int(w * 0.9)    # Kanan
        ]
        
        for i, label in enumerate(Config.VFH_SECTOR_LABELS):
            status = sector_status.get(label, "Low")
            color = Config.COLOR_OBSTACLE if status == "High" else Config.COLOR_CLEAR
            
            x_pos = positions[i]
            cv2.circle(canvas, (x_pos, 80), 12, color, -1)
            cv2.circle(canvas, (x_pos, 80), 15, Config.COLOR_TEXT, 2)
            
            cv2.putText(canvas, label, (x_pos - 40, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, Config.COLOR_TEXT, 1)
    
    def _draw_polar_histogram(self, canvas, histogram):
        """Draw polar histogram (VFH visualization)"""
        center = (Config.DISPLAY_WIDTH - 200, Config.DISPLAY_HEIGHT - 200)
        radius = 100
        
        cv2.circle(canvas, center, radius, Config.COLOR_TEXT, 1)
        
        sectors = len(histogram)
        for i, val in enumerate(histogram):
            theta = np.radians(i * (180 / sectors))
            r = radius * (val / 2.0)  
            x = int(center[0] + r * np.cos(theta))
            y = int(center[1] - r * np.sin(theta))
            cv2.line(canvas, center, (x, y), Config.COLOR_CLEAR, 2)
        
        cv2.putText(canvas, "VFH Histogram", 
                   (center[0] - 60, center[1] + radius + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, Config.COLOR_TEXT, 1)
    
    def _draw_status_panel(self, canvas, sector_status, aruco_result, 
                          vfh_cmd, aruco_cmd, final_cmd):
        """Draw status information panel"""
        h, w = canvas.shape[:2]
        
        panel_height = 180
        cv2.rectangle(canvas, (10, 150), (500, 150 + panel_height), (0, 0, 0), -1)
        cv2.rectangle(canvas, (10, 150), (500, 150 + panel_height), 
                     Config.COLOR_TEXT, 2)
        
        y_offset = 180
        line_height = 30
        
        mode_color = {
            "VFH_ACTIVE": (0, 165, 255),      # Orange
            "ARUCO_TRACKING": (0, 255, 0),    # Green
            "ARUCO_REACHED": (255, 0, 255),   # Magenta
            "SEARCH": (0, 255, 255)           # Yellow
        }.get(self.mode, Config.COLOR_TEXT)
        
        cv2.putText(canvas, f"MODE: {self.mode}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        y_offset += line_height
        
        cv2.putText(canvas, f"Command: {final_cmd}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, Config.COLOR_TEXT, 1)
        y_offset += line_height
        
        vfh_status = "CLEAR" if self.vfh.is_all_clear(sector_status) else "OBSTACLE"
        vfh_color = Config.COLOR_CLEAR if vfh_status == "CLEAR" else Config.COLOR_OBSTACLE
        cv2.putText(canvas, f"VFH: {vfh_status} -> {vfh_cmd}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, vfh_color, 1)
        y_offset += line_height
        
        if aruco_result['detected']:
            aruco_text = f"ArUco: ID={aruco_result['marker_id']} "
            aruco_text += f"Zone={aruco_result['zone']} -> {aruco_cmd}"
            if aruco_result['distance']:
                aruco_text += f" ({aruco_result['distance']:.2f}m)"
            cv2.putText(canvas, aruco_text, (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, Config.COLOR_ARUCO, 1)
        else:
            cv2.putText(canvas, "ArUco: NOT DETECTED", (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
        y_offset += line_height
        
        if Config.SHOW_FPS:
            cv2.putText(canvas, f"FPS: {self.fps}", (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, Config.COLOR_TEXT, 1)
        
        cv2.putText(canvas, "[Q]=Quit [1]=VFH [2]=ArUco [3]=Hybrid [Space]=Stop",
                   (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                   Config.COLOR_TEXT, 1)
    
    def handle_keyboard(self, key):
        """
        Handle keyboard input
        
        Returns:
            bool: True to continue, False to quit
        """
        if key == ord('q'):
            print("🛑 Quit requested")
            return False
        
        elif key == ord('1'):
            self.force_mode = "VFH_ONLY"
            print("📍 Mode: VFH ONLY")
            self.logger.info("Mode switched to VFH_ONLY")
        
        elif key == ord('2'):
            self.force_mode = "ARUCO_ONLY"
            print("📍 Mode: ARUCO ONLY")
            self.logger.info("Mode switched to ARUCO_ONLY")
        
        elif key == ord('3'):
            self.force_mode = None
            print("📍 Mode: HYBRID")
            self.logger.info("Mode switched to HYBRID")
        
        elif key == ord(' '):
            print("🛑 EMERGENCY STOP")
            self.arduino.send_command("STOP")
            self.logger.warning("Emergency stop triggered")
        
        return True
    
    def cleanup(self):
        """Cleanup all resources"""
        print("\n" + "=" * 60)
        print("🛑 SYSTEM SHUTDOWN")
        print("=" * 60)
        
        print("Stopping robot...")
        self.arduino.send_command("STOP")
        time.sleep(0.5)
        
        print("Cleaning up modules...")
        self.vfh.cleanup()
        self.arduino.close()
        
        cv2.destroyAllWindows()
        
        self.logger.info("System shutdown completed")
        print("✅ Cleanup completed")
        print("=" * 60)


def main():
    """Main entry point"""
    try:
        controller = RobotController()
        controller.run()
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
