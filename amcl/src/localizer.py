#!/usr/bin/env python3

from __future__ import annotations

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Odometry
from std_msgs.msg import Header
from rclpy.callback_groups import ReentrantCallbackGroup
from launch.actions import ExecuteProcess

class Localizer(Node):
    def __init__(self):
        
        # Initialize node, name it "mapper"
        super().__init__('localizer')
        
        # Create callback group
        self.cb_group = ReentrantCallbackGroup()
        
        # Publishing Twists to /cmd_vel
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        # Create command for /request_nomotion_update
        self.update = ExecuteProcess(
            cmd=['ros2',
                'service',
                'call',
                '/request_nomotion_update',
                'std_srvs/Empty',
                ],
        )
        
        # Create twist message
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.twist.angular.z = 1.0
        
        # Wait for other processes to finish elsewhere
        time.sleep(5)
        
        # Start spinning!
        start_time = self.get_clock().now().to_msg().sec
        while True:
            
            if self.get_clock().now().to_msg().sec - start_time >= 21.0:
                msg_cmd_vel.twist.angular.z = 0.0
                break
            elif self.get_clock().now().to_msg().sec - start_time >= 11.0:
                msg_cmd_vel.twist.angular.z = -1.0
            elif self.get_clock().now().to_msg().sec - start_time >= 10.0:
                msg_cmd_vel.twist.angular.z = 0.0
            
            self.publisher.publish(msg_cmd_vel)
            self.update
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop the robot, assume we have finished localizing
        msg_cmd_vel.twist.angular.z = 0.0
        self.publisher.publish(msg_cmd_vel)
        self.update
        
        self.get_logger().info("FINISHED LOCALIZING")
        
        # WE ASSUME LOCALIZATION HAS FINISHED
        
        # Create our QoS profiles
        self.QoS_Policy_rviz = QoSProfile(durability=DurabilityPolicy.VOLATILE, depth=10)
        self.QoS_Policy_map = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10)
        
        # Subscribe to /map, /amcl_pose, and /clicked_point
        self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.handle_map, 
            self.QoS_Policy_map, 
            callback_group=self.cb_group
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.handle_covariant_pose,
            self.QoS_Policy_rviz,
            callback_group=self.cb_group
        )
        self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.handle_clicked_point,
            self.QoS_Policy_rviz,
            callback_group=self.cb_group
        )
        
        # Publish to /odom and /mapper_frontiers
        self.update_pose = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        self.create_frontier = self.create_publisher(
            GridCells,
            '/mapper_frontiers',
            self.QoS_Policy_map
        )
        
        # Initialize mapdata and origin
        self.mapdata = None


    def handle_map(self, mapdata: OccupancyGrid):
        '''
        Continuously update our map so we can transform clicked points
        :param mapdata [OccupancyGrid] The map message
        '''
        
        # Update the mapdata
        self.mapdata = mapdata
    

    def handle_covariant_pose(self, pose: PoseWithCovarianceStamped):
        '''
        Send the covariant pose to /odom for the controller to handle
        :param msg [PoseWithCovarianceStamped] The pose message
        '''
        
        # Create an origin from the pose
        origin = Odometry()
        origin.header.frame_id = 'map'
        origin.header.stamp = self.get_clock().now().to_msg()
        origin.pose = pose.pose
        
        # Publish to /odom
        self.update_pose.publish(origin)
    
    
    def handle_clicked_point(self, point: PointStamped): 
        '''
        Send the clicked point to /mapper_frontiers for the path planner to handle
        :param msg [PointStamped] The point message
        '''
        
        # Create a frontier from the clicked point
        frontier = GridCells()
        frontier.header = Header()
        frontier.header.stamp = self.get_clock().now().to_msg()
        frontier.header.frame_id = self.mapdata.header.frame_id
        frontier.cell_width = float(self.mapdata.info.resolution)
        frontier.cell_height = float(self.mapdata.info.resolution)
        frontier.cells = [point.point]
        
        # Publish to /mapper_frontiers
        self.create_frontier.publish(frontier)


def main(args=None):
    rclpy.init(args=args)
    
    node = Localizer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: # when 'ctrl + C' is pressed
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
