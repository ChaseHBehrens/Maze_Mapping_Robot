#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from scipy.spatial.transform import Rotation 
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
import math

def normalize_angle(angle: float):
    return ((angle + 3.14159) % 6.28318) - 3.14159

class Controller(Node):
    def __init__(self):
        
        # Initialize node, name it 'controller'
        super().__init__('controller')

        # Create callback group
        self.cb_group = ReentrantCallbackGroup()
        
        self.QoS_Policy_path = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        # Subscribe to '/odom', '/nav_path', and 'move_base_simple/goal'
        self.create_subscription(
            Odometry,
            '/odom',
            self.update_odometry,
            10,
            callback_group=self.cb_group
        )
        self.create_subscription(
            Path,
            '/nav_path',
            self.handle_path,
            self.QoS_Policy_path,
            callback_group=self.cb_group
        )
        self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.go_to,
            10,
            callback_group=self.cb_group
        )

        # Publish to '/cmd_vel'
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Create a transform listener + TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize attributes
        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0
        self.send_speed(0.0, 0.0)


    def update_odometry(self, msg: Odometry):
        '''
        A callback that updates the current pose of the robot
        :param msg [Odometry] the current odometry information
        '''
        
        # Create quaternion 
        quat = msg.pose.pose.orientation
        (_, _, yaw) = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=False)

        # Assign odom values
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pth = yaw


    def send_speed(self, linear_speed: float, angular_speed: float): 
        '''
        Sends speed to the /cmd_vel topic, which runs the turtlebot motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        '''

        # Create twist message
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.header.frame_id = 'base_link'

        # Linear velocity
        msg_cmd_vel.twist.linear.x = linear_speed
        msg_cmd_vel.twist.linear.y = 0.0
        msg_cmd_vel.twist.linear.z = 0.0

        # Angular velocity
        msg_cmd_vel.twist.angular.x = 0.0
        msg_cmd_vel.twist.angular.y = 0.0
        msg_cmd_vel.twist.angular.z = angular_speed

        # Publish message
        self.publisher.publish(msg_cmd_vel)


    def drive(self, distance: float, linear_speed: float):
        '''
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        '''
        
        # Save the destination
        dest_x = self.px + distance * math.cos(self.pth)
        dest_y = self.py + distance * math.sin(self.pth)

        # Drive forward
        self.send_speed(linear_speed, 0.0)

        while True:

            # Calculate the distance error between your current pose and the destination
            error_x = abs(dest_x - self.px)
            error_y = abs(dest_y - self.py)

            # If the error is smaller than a certain tolerance, stop the robot; otherwise keep waiting
            if max(error_x, error_y) < 0.01:
                self.send_speed(0.0, 0.0)
                break

            rclpy.spin_once(self, timeout_sec=0.1)


    def drive_arc(self, x: float, y: float, linear_speed: float):
        '''
        Drives the robot in a straight line.
        :param x            [float] [m]   The x-coordinate
        :param y            [float] [m]   The y-coordinate
        :param linear_speed [float] [m/s] The forward linear speed.
        '''

        # Find distance and angle to destination
        distance = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
        robot_angle = math.atan2(y, x)

        # Find radius of curvature
        radius = distance / (2*math.cos(robot_angle))

        # Find angular velocity
        angular_speed = linear_speed / radius

        # Drive along arc
        self.send_speed(linear_speed, angular_speed)

        while True:

            # Calculate the distance error between your current pose and the destination
            error_x = abs(x - self.px)
            error_y = abs(y - self.py)
            
            # If the error is smaller than a certain tolerance, stop the robot; otherwise keep waiting
            if max(error_x, error_y) < 0.1:
                self.send_speed(0.0, 0.0)
                break

            rclpy.spin_once(self, timeout_sec=0.1)


    def rotate(self, angle: float, angular_speed: float):
        '''
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        '''
        
        # Find the change in angle and final orientation
        delta_th = Controller.normalize_angle(angle)
        final_th = self.pth + delta_th
        
        # Rotate in the appropriate direction
        self.send_speed(0.0, math.copysign(angular_speed, delta_th))
        
        while True:

            # Calculate the angle error between your current pose and the destination
            error = abs(final_th - self.pth)

            # If the error is smaller than a certain tolerance, stop the robot; otherwise keep waiting
            if error < 0.01:
                self.send_speed(0.0, 0.0)
                break

            rclpy.spin_once(self, timeout_sec=0.1)


    def go_to(self, msg: PoseStamped):
        '''
        Uses rotate() and drive() to get to a specific pose.
        :param msg [PoseStamped] The target or "goal" pose.
        '''

        # Create quaternion
        quat = msg.pose.orientation
        (_, _, yaw) = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=False)

        # Find the difference in x and y in the worldframe between the robot and its destination
        world_x = msg.pose.position.x
        world_y = msg.pose.position.y
        world_th = yaw

        error_x = world_x - self.px
        error_y = world_y - self.py
        heading = math.atan2(error_y, error_x)

        # Rotate robot to face destination
        self.smooth_rotate(heading - self.pth, 0.5)

        # Drive to destination
        distance = math.sqrt(math.pow(error_x, 2) + math.pow(error_y, 2))
        self.smooth_drive(distance, 0.75)

        # Rotate robot to complete pose
        self.smooth_rotate(world_th - self.pth, 0.5)
    
    
    def handle_path(self, path:Path):
        '''
        A callback function that handles iterating through a path.
        :param path [Path] The path that the robot will drive.
        '''

        # Drive to each pose in path
        for pose in path.poses:
            self.go_to(pose)

    def smooth_rotate(self, angle: float, angular_speed: float):
        '''
        Smoothly rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        '''
        # Save the starting and final angles
        th_start = self.pth
        th_dest = normalize_angle(self.pth + angle)
        
        # Determine how far the robot should rotate before reaching its top speed
        ramp_dist = 0.5
        
        while True:
            
            # Calculate the error between your current angle and the final angle
            error = abs(th_dest - self.pth)
            
            # Determine if the robot has reached its goal
            polarity = math.copysign(1, th_dest - th_start) == 1
            offset = self.pth >= th_dest
            arrived = polarity == offset
            
            # If the error is smaller than a certain tolerance, stop the robot; otherwise keep updating
            if error < 0.01 or arrived:
                self.send_speed(0.0, 0.0)
                break
            
            # Find the distance we have traveled and the distance remaining
            traveled_dist = abs(normalize_angle(self.pth - th_start))
            remaining_dist = abs(normalize_angle(th_dest - self.pth))
            
            # Create a proportional value that corresponds to where the robot is within the trapezoidal profile
            # The angular speed of the robot is the product of this coefficient and the given angular speed
            # We want to floor the angular speed as it approaches zero to avoid the robot rotating infinitely slow
            coeff = min(traveled_dist, ramp_dist, remaining_dist) / ramp_dist
            current_speed = max(0.05, coeff*angular_speed)
            
            # Drive at the appropriate speed in the appropriate direction
            self.send_speed(0.0, math.copysign(current_speed, th_dest - th_start))
            
            rclpy.spin_once(self, timeout_sec=0.1)



    def smooth_drive(self, distance: float, linear_speed: float):
        '''
        Smoothly drives the robot in a straight line by regulating its speed.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        '''
        
        # Save the starting and final positions
        x_start = self.px
        y_start = self.py
        x_dest = self.px + (distance * math.cos(self.pth))
        y_dest = self.py + (distance * math.sin(self.pth))
        
        # Determine how far the robot should travel before reaching its top speed
        ramp_dist = 0.25
        
        while True:         
            
            # Calculate the distance error between your current pose and the destination
            error_x = abs(x_dest - self.px)
            error_y = abs(y_dest - self.py)
            
            # Determine if the robot has reached its goal
            polarity_x = math.copysign(1, x_dest - x_start) == 1
            offset_x = self.px >= x_dest
            polarity_y = math.copysign(1, y_dest - y_start) == 1
            offset_y = self.py >= y_dest
            
            arrived = (polarity_x == offset_x) and (polarity_y == offset_y)
            
            # If the error is smaller than a certain tolerance, stop the robot; otherwise keep updating
            if max(error_x, error_y) < 0.01 or arrived:
                self.send_speed(0.0, 0.0)
                break
            
            # Find the distance we have traveled and the distance remaining
            traveled_dist = math.sqrt(((self.px - x_start) ** 2) + ((self.py - y_start) ** 2))
            remaining_dist = math.sqrt(((x_dest - self.px) ** 2) + ((y_dest - self.py) ** 2))
            
            # Create a proportional value that corresponds to where the robot is within the trapezoidal profile
            # The linear speed of the robot is the product of this coefficient and the given linear speed
            # We want to floor the linear speed as it approaches zero to avoid the robot moving infinitely slow
            coeff = min(traveled_dist, ramp_dist, remaining_dist) / ramp_dist
            current_speed = max(0.02, coeff*linear_speed)
            
            # Drive at the appropriate speed
            self.send_speed(current_speed, 0.0)
            
            rclpy.spin_once(self, timeout_sec=0.1)
    
    
def main(args=None):
    rclpy.init(args=args)
    
    node = Controller()
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
