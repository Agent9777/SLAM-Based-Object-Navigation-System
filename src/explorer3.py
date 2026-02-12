#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import numpy as np
from math import sqrt
import os

class mapinfo(Node):
    def __init__(self):
        super().__init__("mapinfo")
        self.mapsub=self.create_subscription(OccupancyGrid,"/map",self.mapcallback,10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.map_data = None
        self.map_info = None
        self.exploring = False
        self.blocked_points = []  # store aborted goals
        self.chosen_point=None
        self.skip_radius = 0.5  # meters
        self.frontier_radius_check = 0.8  # meters
        self.min_unexplored_fraction = 0.20  # 20%
    
    def grid_to_world(self, x_idx, y_idx):
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        res = self.map_info.resolution
        wx = origin_x + (x_idx + 0.5) * res
        wy = origin_y + (y_idx + 0.5) * res
        return wx, wy
        
    def mapcallback(self,msg):
        self.map_data=np.array(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width)
        self.map_info=msg.info       
        if not self.exploring:
            self.exploring = True
            self.chosen_point=self.find_point(self.map_data,msg.info.height,msg.info.width)
            
    def find_point(self,grid,height,width):
        free_mask = (grid == 0)        # free cells
        unknown_mask = (grid == -1)    # unknown cells

        neighbors_unknown = np.zeros_like(grid, dtype=bool)
        neighbors_unknown[:-1, :] |= unknown_mask[1:, :]   # up
        neighbors_unknown[1:, :]  |= unknown_mask[:-1, :]  # down
        neighbors_unknown[:, :-1] |= unknown_mask[:, 1:]   # left
        neighbors_unknown[:, 1:]  |= unknown_mask[:, :-1]  # right

        frontier_mask = free_mask & neighbors_unknown

        unknown_int = unknown_mask.astype(int)
        cumsum = unknown_int.cumsum(axis=0).cumsum(axis=1)
        fraction_unknown = np.zeros_like(grid, dtype=float)

        for y, x in zip(*np.where(frontier_mask)):
            y_min = int(max(0, y - self.frontier_radius_check))
            y_max = int(min(height, y + self.frontier_radius_check + 1))
            x_min = int(max(0, x - self.frontier_radius_check))
            x_max = int(min(width, x + self.frontier_radius_check + 1))

            total_cells = (y_max - y_min) * (x_max - x_min)

            s = cumsum[y_max-1, x_max-1]
            if y_min > 0:
                s -= cumsum[y_min-1, x_max-1]
            if x_min > 0:
                s -= cumsum[y_max-1, x_min-1]
            if y_min > 0 and x_min > 0:
                s += cumsum[y_min-1, x_min-1]

            fraction_unknown[y, x] = s / total_cells

        frontier_mask &= (fraction_unknown >= self.min_unexplored_fraction)

        frontiers = np.argwhere(frontier_mask)
        world_frontiers = []
        for (fx, fy) in frontiers:
            wx, wy = self.grid_to_world(fx, fy)
    
            cond1 = (wx, wy) not in self.blocked_points
    
            cond2 = all(sqrt((wx - bx)**2 + (wy - by)**2) > self.skip_radius 
                for (bx, by) in self.blocked_points)
    
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = float(trans.transform.translation.x)
            robot_y = float(trans.transform.translation.y)
            min_dist =self.skip_radius
            cond3 = sqrt((wx - robot_x)**2 + (wy - robot_y)**2) > min_dist
    
            if cond1  and cond2 and cond3:
                world_frontiers.append((wx, wy))


        if not world_frontiers:
            self.get_logger().info("No valid (non-blocked) frontiers")
            self.exploring = False 
            return
        
        # trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        # robot_x =float(trans.transform.translation.x)
        # robot_y = float(trans.transform.translation.y)
        world_frontiers.sort(key=lambda p: sqrt((p[0] - robot_x) ** 2 + (p[1] - robot_y) ** 2))
        next_pos_x, next_pos_y = world_frontiers[0]
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = next_pos_x
        goal_pose.pose.position.y = next_pos_y
        goal_pose.pose.orientation.w = 1.0
        goal_msg.pose = goal_pose

        self.current_goal = (next_pos_x, next_pos_y)

        self.get_logger().info(f"Navigating to nearest reachable frontier: ({next_pos_x:.2f}, {next_pos_y:.2f})")

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
        if result.status == 4:  # Aborted
            self.get_logger().warn('Goal aborted, skipping location.')
            self.handle_aborted_goal()
        elif result.status == 0:  # Succeeded
            self.get_logger().info('Goal reached successfully.')
            # Refresh frontiers with updated map
        else:
            self.get_logger().warn(f"Goal ended with status: {result.status}")
            self.handle_aborted_goal()
        self.exploring = False
    def handle_aborted_goal(self):
        self.blocked_points.append(self.current_goal)
        self.exploring = False
        
        
        
        

def main(args=None):
    rclpy.init(args=args)
    node=mapinfo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__=="__main__":
    main()
    