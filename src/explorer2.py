#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
from tf2_ros import Buffer, TransformListener
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.map_data = None
        self.map_info = None
        self.exploring = False
        self.last_frontiers = []
        self.blocked_points = [] 
        self.skip_radius = 0.5 
        self.frontier_radius_check = 0.8 
        self.min_unexplored_fraction = 0.20 

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info

        if not self.exploring:
            self.exploring = True
            self.last_frontiers = self.find_frontiers()
            self.send_next_goal()

    def send_next_goal(self):
        if not self.last_frontiers:
            self.get_logger().info("No frontiers found — exploration complete.")
            self.exploring = False
            return

        available_frontiers = [
            pt for pt in self.last_frontiers
            if not self.is_near_blocked_point(pt) and self.has_sufficient_unexplored_area(pt)
        ]

        if not available_frontiers:
            self.get_logger().info("No reachable frontiers left — exploration complete.")
            self.exploring = False
            return

        robot_x, robot_y = self.get_robot_position()
        target = min(available_frontiers, key=lambda p: math.dist((p[0], p[1]), (robot_x, robot_y)))

        world_x = self.map_info.origin.position.x + target[0] * self.map_info.resolution
        world_y = self.map_info.origin.position.y + target[1] * self.map_info.resolution

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = world_x
        goal_pose.pose.position.y = world_y
        goal_pose.pose.orientation.w = 1.0
        goal_msg.pose = goal_pose

        self.current_goal = (world_x, world_y)

        self.get_logger().info(f"Navigating to nearest reachable frontier: ({world_x:.2f}, {world_y:.2f})")

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            self.handle_aborted_goal()
            return
        self.get_logger().info('Goal accepted, navigating...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4: 
            self.get_logger().warn('Goal aborted, skipping location.')
            self.handle_aborted_goal()
        elif result.status == 0: 
            self.get_logger().info('Goal reached successfully.')
            self.last_frontiers = self.find_frontiers()
            self.send_next_goal()
        else:
            self.get_logger().warn(f"Goal ended with status: {result.status}")
            self.handle_aborted_goal()

    def handle_aborted_goal(self):
        self.blocked_points.append(self.current_goal)
        self.send_next_goal()

    def is_near_blocked_point(self, point):
        world_x = self.map_info.origin.position.x + point[0] * self.map_info.resolution
        world_y = self.map_info.origin.position.y + point[1] * self.map_info.resolution
        for bx, by in self.blocked_points:
            if math.dist((world_x, world_y), (bx, by)) <= self.skip_radius:
                return True
        return False

    def get_robot_position(self):
        trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        robot_x = float(trans.transform.translation.x)
        robot_y = float(trans.transform.translation.y)
        return (robot_x,robot_y)

    def find_frontiers(self):
        """Find unexplored cells (=-1) adjacent to free space (=0)."""
        frontiers = []
        for r in range(1, self.map_data.shape[0] - 1):
            for c in range(1, self.map_data.shape[1] - 1):
                if self.map_data[r, c] == -1:  # unexplored
                    neighbors = self.map_data[r-1:r+2, c-1:c+2].flatten()
                    if 0 in neighbors and np.all(neighbors != 100):  # free neighbor, no walls
                        frontiers.append((r, c))
        return frontiers
    def has_sufficient_unexplored_area(self, frontier):
        """Check if at least min_unexplored_fraction of area around frontier is unexplored."""
        radius_cells = int(self.frontier_radius_check / self.map_info.resolution)
        r, c = frontier
        unexplored_count = 0
        total_count = 0

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr*dr + dc*dc <= radius_cells * radius_cells:
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < self.map_data.shape[0] and 0 <= cc < self.map_data.shape[1]:
                        total_count += 1
                        if self.map_data[rr, cc] == -1:
                            unexplored_count += 1

        if total_count == 0:
            return False
        return (unexplored_count / total_count) >= self.min_unexplored_fraction

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
