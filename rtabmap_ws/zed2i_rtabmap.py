import rclpy
from rclpy.node import Node
from subprocess import Popen

class RTABMapLauncher(Node):
    def __init__(self):
        super().__init__('rtabmap_visual_slam')

        self.get_logger().info("Starting RTAB-Map with ZED 2i RGB-D input...")

        self.process = Popen([
            '/home/epriauto/rtabmap_ws/install/rtabmap/bin/rtabmap',
            '--ros-args',
            '-p', 'frame_id:=zed_camera_link',
            '-p', 'subscribe_depth:=true',
            '-p', 'subscribe_scan:=false',
            '-p', 'subscribe_stereo:=false',
            '-p', 'approx_sync:=true',
            '-p', 'queue_size:=30',
            '-p', 'use_sim_time:=false',
            '-p', 'rtabmap_args:=--delete_db_on_start',
            '--remap', 'rgb/image:=/zed/zed_node/rgb/image_rect_color',
            '--remap', 'depth/image:=/zed/zed_node/depth/depth_registered',
            '--remap', 'rgb/camera_info:=/zed/zed_node/rgb/camera_info',
            '--remap', 'odom:=/zed/zed_node/odom'
        ])

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("Shutting down RTAB-Map...")
        self.process.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = RTABMapLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

