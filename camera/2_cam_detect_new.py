# ZED Libraries 
import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ZedObstacleDetector(Node):
    def __init__(self):
        super().__init__('zed_obstacle_detector')
        
        # Publisher for obstacle detection signal
        self.obstacle_publisher = self.create_publisher(Int32, '/zed_obstacle_signal', 10)
        
        # Initialize cameras
        self.zed_obstacle = sl.Camera()  # Camera for obstacle detection (SN: 30855840)
        self.zed_video = sl.Camera()     # Camera for video output (SN: 35455187)
        
        # Track which cameras are available
        self.obstacle_cam_available = False
        self.video_cam_available = False

        # Configuration for obstacle detection camera
        init_obstacle = sl.InitParameters()
        init_obstacle.camera_resolution = sl.RESOLUTION.VGA
        init_obstacle.depth_mode = sl.DEPTH_MODE.NEURAL
        init_obstacle.set_from_serial_number(35455187)

        # Configuration for video-only camera
        init_video = sl.InitParameters()
        init_video.camera_resolution = sl.RESOLUTION.VGA
        init_video.depth_mode = sl.DEPTH_MODE.NONE
        init_video.set_from_serial_number(30855840)

        # Try to open obstacle camera
        err = self.zed_obstacle.open(init_obstacle)
        if err == sl.ERROR_CODE.SUCCESS:
            self.obstacle_cam_available = True
            self.get_logger().info("Obstacle camera opened successfully.")
        else:
            self.get_logger().warn(f"Failed to open obstacle camera (SN: 35455187). Error: {err}")
            self.get_logger().warn("Continuing without obstacle camera...")

        # Try to open video camera
        err = self.zed_video.open(init_video)
        if err == sl.ERROR_CODE.SUCCESS:
            self.video_cam_available = True
            self.get_logger().info("Video camera opened successfully.")
        else:
            self.get_logger().warn(f"Failed to open video camera (SN: 30855840). Error: {err}")
            self.get_logger().warn("Continuing without video camera...")

        # Check if at least one camera is available
        if not self.obstacle_cam_available and not self.video_cam_available:
            self.get_logger().error("No cameras available. Exiting.")
            exit(1)

        self.runtime_params = sl.RuntimeParameters()
        self.image_obstacle = sl.Mat()
        self.image_video = sl.Mat()
        self.depth_obstacle = sl.Mat()
        
        # Timer for processing frames
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30Hz

    def process_frame(self):
        obstacle_detected = False
        img_obstacle = None
        img_video = None
        
        # Process obstacle camera if available
        if self.obstacle_cam_available:
            if self.zed_obstacle.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve obstacle cam data (image + depth)
                self.zed_obstacle.retrieve_image(self.image_obstacle, sl.VIEW.LEFT)
                img_obstacle = self.image_obstacle.get_data()
                if img_obstacle.shape[2] == 4:
                    img_obstacle = cv2.cvtColor(img_obstacle, cv2.COLOR_BGRA2BGR)
                
                # Retrieve depth
                self.zed_obstacle.retrieve_measure(self.depth_obstacle, sl.MEASURE.DEPTH)
                
                # Obstacle Detection
                height, width, _ = img_obstacle.shape
                grid_size = 20  # Size of each small rectangle
                
                for y in range(0, height, grid_size):
                    for x in range(0, width, grid_size):
                        depth_value = self.depth_obstacle.get_value(x, y)[1]
                        if 0 < depth_value < 400:  # Detection range in mm
                            obstacle_detected = True
                            cv2.rectangle(img_obstacle, (x, y), (x + grid_size, y + grid_size), (0, 0, 255), 1)
        
        # Process video camera if available
        if self.video_cam_available:
            if self.zed_video.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                self.zed_video.retrieve_image(self.image_video, sl.VIEW.LEFT)
                img_video = self.image_video.get_data()
                if img_video.shape[2] == 4:
                    img_video = cv2.cvtColor(img_video, cv2.COLOR_BGRA2BGR)

        # Prepare display
        if self.obstacle_cam_available and self.video_cam_available:
            # Both cameras available
            cv2.putText(img_obstacle, "Back Camera", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img_video, "Front Camera", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            combined = cv2.vconcat([img_obstacle, img_video])
        elif self.obstacle_cam_available:
            # Only obstacle camera available
            cv2.putText(img_obstacle, "Back Camera", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            combined = img_obstacle
        elif self.video_cam_available:
            # Only video camera available
            cv2.putText(img_video, "Front Camera", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            combined = img_video
        else:
            return

        # Resize to fit screen
        scale_percent = 100
        width = int(combined.shape[1] * scale_percent / 100)
        height = int(combined.shape[0] * scale_percent / 100)
        resized = cv2.resize(combined, (width, height))

        # Display the window
        cv2.imshow("Camera Feeds", resized)
        cv2.waitKey(1)

        # Publish obstacle detection signal
        signal = Int32()
        signal.data = 1 if obstacle_detected else 0
        self.obstacle_publisher.publish(signal)
        
        if obstacle_detected:
            self.get_logger().info("Obstacle detected! Signal: 1")
        else:
            self.get_logger().info("No obstacle detected. Signal: 0")

    def __del__(self):
        # Close the cameras when node is destroyed
        if self.obstacle_cam_available:
            self.zed_obstacle.close()
        if self.video_cam_available:
            self.zed_video.close()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ZedObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()