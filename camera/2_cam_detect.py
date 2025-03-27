import cv2
import numpy as np
import pyzed.sl as sl

def main():
    # Initialize cameras
    zed_obstacle = sl.Camera()  # Camera for obstacle detection (SN: 30855840)
    zed_video = sl.Camera()      # Camera for video output (SN: 35455187)

    # Configuration for obstacle detection camera (higher specs)
    init_obstacle = sl.InitParameters()
    init_obstacle.camera_resolution = sl.RESOLUTION.VGA
    init_obstacle.depth_mode = sl.DEPTH_MODE.NEURAL
    init_obstacle.set_from_serial_number(35455187)  # Assign specific SN

    # Configuration for video-only camera (lower specs to reduce load)
    init_video = sl.InitParameters()
    init_video.camera_resolution = sl.RESOLUTION.VGA
    init_video.depth_mode = sl.DEPTH_MODE.NONE      # No depth needed
    init_video.set_from_serial_number(30855840)      # Assign specific SN

    # Open obstacle camera first
    err = zed_obstacle.open(init_obstacle)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open obstacle camera (SN: 35455187). Error: {err}")
        exit(1)

    # Open video camera
    err = zed_video.open(init_video)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open video camera (SN: 30855840). Error: {err}")
        zed_obstacle.close()  # Release the first camera
        exit(1)

    print("Both cameras opened successfully. Press 'ctrl+c' to exit.")

    runtime_params = sl.RuntimeParameters()
    image_obstacle = sl.Mat()
    image_video = sl.Mat()
    depth_obstacle = sl.Mat()  # Only used for obstacle cam

    while True:
        # Grab frames from both cameras
        if (zed_obstacle.grab(runtime_params) == sl.ERROR_CODE.SUCCESS and 
            zed_video.grab(runtime_params) == sl.ERROR_CODE.SUCCESS):
            
            # Retrieve obstacle cam data (image + depth)
            zed_obstacle.retrieve_image(image_obstacle, sl.VIEW.LEFT)
            zed_obstacle.retrieve_measure(depth_obstacle, sl.MEASURE.DEPTH)
            
            # Retrieve video cam data (image only)
            zed_video.retrieve_image(image_video, sl.VIEW.LEFT)

            # Convert to OpenCV format
            img_obstacle = image_obstacle.get_data()
            img_video = image_video.get_data()

            if img_obstacle.shape[2] == 4:
                img_obstacle = cv2.cvtColor(img_obstacle, cv2.COLOR_BGRA2BGR)
            if img_video.shape[2] == 4:
                img_video = cv2.cvtColor(img_video, cv2.COLOR_BGRA2BGR)

            # --- Obstacle Detection (on first camera) ---
            height, width, _ = img_obstacle.shape
            obstacle_points = []
            
            for y in range(0, height, 20):
                for x in range(0, width, 20):
                    depth_value = depth_obstacle.get_value(x, y)[1]
                    if 0 < depth_value < 500:  # Detect within 1 meter
                        obstacle_points.append((x, y))
            
            # Draw bounding box if obstacles found
            if obstacle_points:
                obstacle_points = np.array(obstacle_points)
                x_min, y_min = np.min(obstacle_points, axis=0)
                x_max, y_max = np.max(obstacle_points, axis=0)
                cv2.rectangle(img_obstacle, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

            # Display one feed above the other (vertical stack)
            # Stack feeds vertically
            combined = cv2.vconcat([img_obstacle, img_video])

            # Resize to fit screen (e.g., 50% scale)
            scale_percent = 100  # Adjust this value as needed
            width = int(combined.shape[1] * scale_percent / 100)
            height = int(combined.shape[0] * scale_percent / 100)
            resized = cv2.resize(combined, (width, height))

            # Display the resized window
            cv2.imshow("Camera Feeds", resized)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    zed_obstacle.close()
    zed_video.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()