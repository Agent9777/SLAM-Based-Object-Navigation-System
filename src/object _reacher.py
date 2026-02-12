#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import json

class GoToObject(Node):
    def __init__(self):
        super().__init__('go_to_object_node')

        # Subscribe to detected object data
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  

        self.objects_sub = self.create_subscription(String, '/detected_objects_data', self.objects_callback, qos)
        self.objects_data = {}  # latest object data

        # Navigation2 action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for NavigateToPose action server...")
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info("Connected to NavigateToPose action server.")

        # Start user input loop
        self.user_timer = self.create_timer(1.0, self.ask_user_input)
        self.user_input_requested = True

    def objects_callback(self, msg):
         try:
             self.objects_data = json.loads(msg.data)
         except Exception as e:
             self.get_logger().warn(f"Failed to parse object data: {e}")

    def ask_user_input(self):
        if not self.user_input_requested:
            return
        if not self.objects_data:
            self.get_logger().info("No detected objects yet.")
            return

        # List available objects
        print("\nDetected objects:")
        for label in self.objects_data.keys():
            print(f"- {label}")
        print("----------------------")

        # Ask for input
        obj_name = input("Enter object name to go to: ").strip()
        if obj_name in self.objects_data and self.objects_data[obj_name]:
            target = self.objects_data[obj_name][0]  # pick first detected coordinate
            self.get_logger().info(f"Going to {obj_name} at ({target['x']:.2f}, {target['y']:.2f})")
            self.send_goal(target['x'], target['y'])
        else:
            self.get_logger().warn(f"Object '{obj_name}' not found in detected objects.")
        self.user_input_requested = True  # ready for next input

    def send_goal(self, x, y):
        # Stop a little before the object (offset in meters)
        offset = 0.1  # distance to stop in front

        goal_x = x - offset
        goal_y = y

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # facing forward

        self.get_logger().info(f"Sending goal: ({goal_x:.2f}, {goal_y:.2f})")
        self.nav_to_pose_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
