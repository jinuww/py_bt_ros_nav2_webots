import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2


class ImageCaptureNode(Node):


    def __init__(self):
        super().__init__("image_capture_node")

        self.last_image_msg = None
        self.bridge = CvBridge()

        self.save_dir = "/home/jinwoo/webots_ros2_ws/src/bt_image_capture/bt_image_capture/bt_images"

        os.makedirs(self.save_dir, exist_ok=True)

        self.create_subscription(
            Image,
            "/TurtleBot3Burger/front_camera/image_color",
            self.image_callback,
            10,
        )

        self.srv = self.create_service(
            Trigger,
            "/bt/capture_image",
            self.handle_capture_image,
        )

        self.get_logger().info(
            "ImageCaptureNode started. Waiting for /bt/capture_image requests..."
        )

    def image_callback(self, msg: Image):
        self.last_image_msg = msg

    def handle_capture_image(self, request, response):
        if self.last_image_msg is None:
            response.success = False
            response.message = "No image received yet."
            self.get_logger().warn("Capture request but no image received yet.")
            return response

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.last_image_msg, desired_encoding="bgr8")
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{ts}.png"
            filepath = os.path.join(self.save_dir, filename)
            cv2.imwrite(filepath, cv_img)

            response.success = True
            response.message = filepath
            self.get_logger().info(f"Image captured and saved to {filepath}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to save image: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
