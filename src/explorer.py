#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
import math
import random

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.map_data = None
        self.map_info = None
        self.exploring = False

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info

        if not self.exploring:
            self.exploring = True
            self.send_next_goal()

    def send_next_goal(self):
        frontier_points = self.find_frontiers()
        if not frontier_points:
            self.get_logger().info("No frontiers found — exploration complete.")
            return

        target = random.choice(frontier_points)  # pick random frontier
        world_x = self.map_info.origin.position.x + target[1] * self.map_info.resolution
        world_y = self.map_info.origin.position.y + target[0] * self.map_info.resolution

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = world_x
        goal_pose.pose.position.y = world_y
        goal_pose.pose.orientation.w = 1.0
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Navigating to frontier: ({world_x:.2f}, {world_y:.2f})")

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            self.exploring = False
            return
        self.get_logger().info('Goal accepted, navigating...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Navigation finished with status: {result.status}')
        self.exploring = False
        self.send_next_goal()

    def find_frontiers(self):
        """Finds cells that are unexplored (=-1) adjacent to free space (=0) and not walls (>50)."""
        frontiers = []
        for r in range(1, self.map_data.shape[0] - 1):
            for c in range(1, self.map_data.shape[1] - 1):
                if self.map_data[r, c] == -1:  # unexplored
                    neighbors = self.map_data[r-1:r+2, c-1:c+2].flatten()
                    if 0 in neighbors and np.all(neighbors != 100):  # free neighbor, no walls touching
                        frontiers.append((r, c))
        return frontiers

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
