#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class YoloImageNode(Node):
    def __init__(self):
        super().__init__('yolo_image_node')

        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',   # change if your camera topic is different
            self.image_callback,
            10)

        self.bridge = CvBridge()

        # Load YOLOv8 model
        yolo_pkg_path = get_package_share_directory('slam_based_nav')
        model_path = os.path.join(yolo_pkg_path, 'yolo', 'yolov8s-oiv7.pt')
        self.model = YOLO(model_path)
        self.get_logger().info("✅ YOLOv8 model loaded, waiting for images...")

    def image_callback(self, msg):
        # Convert ROS2 Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLO inference
        results = self.model(frame, verbose=False)

        # Draw detections on the frame
        annotated = results[0].plot()

        # Show in OpenCV window
        cv2.imshow("YOLOv8 Detection", annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
