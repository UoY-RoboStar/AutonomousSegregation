import cvb
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def discover_device():
    # Discover devices
    discovered_devices = cvb.DeviceFactory.discover_from_root(cvb.DiscoverFlags.IgnoreVins)
    if not discovered_devices:
        raise RuntimeError("No devices found.")
    # Select the first discovered device
    device_info = discovered_devices[0]
    return device_info.access_token

def configure_camera(camera):
    try:
        # Access the correct node map
        node_map = camera.node_maps["Device"]
        # Set resolution and ROI
        node_map["Width"].value = 2560  # Full width
        node_map["Height"].value = 2048  # Full height
        node_map["OffsetX"].value = 0  # Ensure no cropping on X-axis
        node_map["OffsetY"].value = 0  # Ensure no cropping on Y-axis
        node_map["PixelFormat"].value = "BayerGR8"
    except KeyError as e:
        print(f"Configuration error: {e}. Verify the node names in your camera's GenICam settings.")

def correct_color_balance(frame):
    if len(frame.shape) == 2:  # Single-channel Bayer image
        try:
            frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_GR2BGR)
        except cv2.error as e:
            print(f"Error during Bayer conversion: {e}")
            return None
    avg_r = np.mean(frame[:, :, 2])
    avg_g = np.mean(frame[:, :, 1])
    avg_b = np.mean(frame[:, :, 0])
    scale_r = avg_g / avg_r if avg_r > 0 else 1
    scale_b = avg_g / avg_b if avg_b > 0 else 1
    frame[:, :, 2] = np.clip(frame[:, :, 2] * scale_r, 0, 255).astype(np.uint8)
    frame[:, :, 0] = np.clip(frame[:, :, 0] * scale_b, 0, 255).astype(np.uint8)
    return frame

class OverheadCamera(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.pose_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.puck_publisher = self.create_publisher(Image, '/robot1/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_camera_data)  # 10Hz
        self.frame = None
        self.access_token = discover_device()
        self.get_logger().info("Camera node initialized.")
        self.initialize_camera()

    def initialize_camera(self):
        self.camera = cvb.DeviceFactory.open(self.access_token, cvb.AcquisitionStack.Vin)
        configure_camera(self.camera)
        self.stream = self.camera.stream()
        self.stream.start()

    def publish_camera_data(self):
        image, status = self.stream.wait()
        if status == cvb.WaitStatus.Ok:
            frame = cvb.as_array(image)
            corrected_frame = correct_color_balance(frame)
            if corrected_frame is not None:
                try:
                    # Convert to ROS2 Image message
                    image_msg = self.bridge.cv2_to_imgmsg(corrected_frame, "bgr8")
                    self.pose_publisher.publish(image_msg)
                    self.puck_publisher.publish(image_msg)
                except CvBridgeError as e:
                    self.get_logger().error(f"CvBridge error: {e}")

def main(args=None):
    rclpy.init(args=args)
    camera_node = OverheadCamera()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info("Shutting down camera node.")
    finally:
        camera_node.stream.abort()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
