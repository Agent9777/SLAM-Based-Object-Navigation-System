
#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import json
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math


class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')

        # Subscribers
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        # Publishers
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # keep last message for new subscribers
        self.objects_pub = self.create_publisher(MarkerArray, "/detected_objects_map", 10)
        self.objects_data_pub = self.create_publisher(String, "/detected_objects_data", qos)

        # Store last LiDAR scan
        self.latest_scan = None  

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Load YOLOv8 model
        yolo_pkg_path = get_package_share_directory('slam_based_nav')
        model_path = os.path.join(yolo_pkg_path, 'yolo', 'yolov8s-oiv7.pt')
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # Frame skipping (~10–15 FPS)
        self.frame_count = 0
        self.skip_rate = 3  

        # Detection distance limit
        self.max_range = 4.0  

        # Object deduplication (hitbox)
        self.detected_objects = []  # store objects with map hitboxes

        # Store object coordinates for other nodes
        self.object_positions = {}  # key = label, value = list of positions

        self.get_logger().info("✅ ObjectMapper node started (publishing object data to ROS topic)")

    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def detect_ground(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=50, maxLineGap=20)
        img_h, img_w = frame.shape[:2]
        ground_y = int(img_h * 0.75)  # fallback
        if lines is not None:
            horiz_lines = [line for line in lines if abs(line[0][1] - line[0][3]) < 10]
            if horiz_lines:
                lowest_line = max(horiz_lines, key=lambda l: l[0][1])
                ground_y = lowest_line[0][1]
        return ground_y

    def is_inside_hitbox(self, x, y):
        """Check if point (x,y) is inside any existing object's hitbox"""
        for obj in self.detected_objects:
            if obj['x_min'] <= x <= obj['x_max'] and obj['y_min'] <= y <= obj['y_max']:
                return True
        return False

    def get_lidar_distance(self, angle):
        """ Interpolate LiDAR distance at a specific angle """
        if self.latest_scan is None:
            return None

        idx_float = (angle - self.latest_scan.angle_min) / self.latest_scan.angle_increment
        idx_low = int(np.floor(idx_float))
        idx_high = int(np.ceil(idx_float))

        if 0 <= idx_low < len(self.latest_scan.ranges) and 0 <= idx_high < len(self.latest_scan.ranges):
            r_low = self.latest_scan.ranges[idx_low]
            r_high = self.latest_scan.ranges[idx_high]

            # Linear interpolation
            alpha = idx_float - idx_low
            dist = (1 - alpha) * r_low + alpha * r_high

            if np.isinf(dist) or np.isnan(dist) or dist > self.max_range:
                return None
            return dist
        return None

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.skip_rate != 0:
            return

        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img_h, img_w, _ = frame.shape

        # Ground detection
        ground_y = self.detect_ground(frame)

        # YOLO detection
        results = self.model(frame, verbose=False)
        detections = results[0].boxes.data.cpu().numpy()

        marker_array = MarkerArray()
        marker_id = len(self.detected_objects)  # continue marker IDs

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            label = self.model.names[int(cls)]
            conf = float(conf)

            # Draw bounding box + label on camera feed
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Skip if object bottom not touching ground
            if y2 < ground_y - 10:
                continue

            # LiDAR matching: compute angle of object center
            cx = (x1 + x2) / 2
            norm = cx / img_w
            # FIX: flip horizontal mapping so center of image → forward
            norm = (cx - img_w / 2) / img_w  # range approx -0.5 .. 0.5

            # Map to 0..2π LiDAR
            angle = (norm + 0.5) * (self.latest_scan.angle_max - self.latest_scan.angle_min)
            angle = (angle + math.pi) % (2 * math.pi)

            self.get_logger().info(f"angle-prev {angle}")
            # angle += np.pi

            # angle = (angle + np.pi) % (2 * np.pi) - np.pi
            # self.get_logger().info(f"angle-aft {angle}")

            dist = self.get_lidar_distance(angle)
            if dist is None:
                continue

            # Convert polar to local coordinates
            local_x = dist * np.cos(angle)
            local_y = dist * np.sin(angle)

            point_in_base = PointStamped()
            point_in_base.header.stamp = msg.header.stamp
            point_in_base.header.frame_id = "base_link"
            point_in_base.point.x = local_x
            point_in_base.point.y = local_y
            point_in_base.point.z = 0.0

            try:
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform("map", "base_link", now)
                point_in_map = tf2_geometry_msgs.do_transform_point(point_in_base, transform)
                px = point_in_map.point.x
                py = point_in_map.point.y

                # Deduplication using hitbox for publishing markers
                if self.is_inside_hitbox(px, py):
                    self.get_logger().info(f"⛔ Skipped publishing duplicate {label} at ({px:.2f}, {py:.2f})")
                    continue

                # Compute hitbox size based on bounding box size in meters
                width_m = (x2 - x1) / img_w * dist
                height_m = (y2 - y1) / img_h * dist
                x_min = px - width_m / 2
                x_max = px + width_m / 2
                y_min = py - height_m / 2
                y_max = py + height_m / 2

                # Save object hitbox
                self.detected_objects.append({
                    "label": label,
                    "x_min": x_min, "x_max": x_max,
                    "y_min": y_min,
                    "y_max": y_max,
                    "marker_id": marker_id
                })

                # Store coordinates for other nodes (persistent append)
                if label not in self.object_positions:
                    self.object_positions[label] = []
                self.object_positions[label].append({'x': px, 'y': py, 'z': 0.0})

                # Publish object positions as JSON string
                msg_data = String()
                msg_data.data = json.dumps(self.object_positions)
                self.objects_data_pub.publish(msg_data)

                # Create Marker for RViz
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = marker_id
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = px
                marker.pose.position.y = py
                marker.pose.position.z = 0.5
                marker.scale.z = 0.4
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.text = label

                marker_array.markers.append(marker)
                marker_id += 1
                self.get_logger().info(f"📍 {label} at ({px:.2f}, {py:.2f})")

            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {e}")

        # Publish markers
        if marker_array.markers:
            self.objects_pub.publish(marker_array)

        # Draw ground line for debugging
        cv2.line(frame, (0, ground_y), (img_w, ground_y), (255, 0, 0), 2)
        cv2.imshow("YOLOv8 Ground Objects", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

