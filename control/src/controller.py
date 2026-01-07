#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
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
        self.cb_group_path = MutuallyExclusiveCallbackGroup()
        
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
            1,
            callback_group=self.cb_group
        )
        self.create_subscription(
            String,
            '/fault',
            self.handle_fault,
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

        self.path = []
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.create_timer(0.5, self.drive_path, callback_group=self.cb_group_path)
        self.last_path_time_stamp = self.get_clock().now().to_msg()


    def handle_fault(self, msg: String): 
        if msg.data == "unwalkable goal" or msg.data == "far goal":
            self.get_logger().info("fault stopping robot")
            self.send_speed(0.0, 0.0)
            self.path = []


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


    def handle_path(self, path: Path):
        '''
        A callback function that handles iterating through a path.
        :param path [Path] The path that the robot will drive.
        '''
        
        self.get_logger().info("Handling path")
        
        # if path is older than last recieved skip processing
        if path.header.stamp.sec < self.last_path_time_stamp.sec:
            self.get_logger().info("Outdated path")
            return
        
        # set the last message time to this message
        self.last_path_time_stamp = path.header.stamp
        
        # transform frame
        new_path = []
        for pose in path.poses:  
            stamp = rclpy.time.Time()
            pose.header.stamp = stamp.to_msg()

            if self.tf_buffer.can_transform('odom', pose.header.frame_id, stamp, timeout=rclpy.duration.Duration(seconds=1)):
                transformed_pose = self.tf_buffer.transform(pose, 'odom', timeout=rclpy.duration.Duration(seconds=1))
            else:
                self.get_logger().warn(f"Transform from {pose.header.frame_id} to 'odom' not yet available. Skipping path.")
                return
            new_path.append(transformed_pose)

        # update path
        self.path = new_path
            
            
    def drive_path(self):
        if len(self.path) != 0:
            
            # if first point of path is behind robot remove it
            if len(self.path) > 1:
                distance_to_point2 = (((self.px - self.path[1].pose.position.x) ** 2) + ((self.py - self.path[1].pose.position.y) ** 2)) ** 0.5 
                distance_point1_to_point2 = (((self.path[0].pose.position.x - self.path[1].pose.position.x) ** 2) + ((self.path[0].pose.position.y - self.path[1].pose.position.y) ** 2)) ** 0.5
                
                if distance_to_point2 < distance_point1_to_point2:
                    self.path.pop(0)

            # begin navigating to start of path 
            target_pose = self.path[0].pose

            # Find the difference in x and y in the worldframe between the robot and its destination
            world_x = target_pose.position.x
            world_y = target_pose.position.y

            error_x = world_x - self.px
            error_y = world_y - self.py
            distance = math.sqrt(math.pow(error_x, 2) + math.pow(error_y, 2))
            heading = math.atan2(error_y, error_x)
            error_heading = normalize_angle(self.pth - heading)
            
            # self.get_logger().info(f"LS: {self.linear_speed} AS: {self.angular_speed}")
            # self.get_logger().info(f"pth: {self.pth} heading: {heading}")
            # self.get_logger().info(f"error: {abs(error_heading)}")

            max_angular_speed = 0.8
            max_linear_speed = 0.070
           
            # set angular target speed based on angle to next point
            target_angular_speed = 1 * abs(error_heading)
            target_angular_speed = math.copysign(min(max_angular_speed, target_angular_speed), -error_heading)
            
            # set linear target speed based on angle to next point
            if (abs(error_heading) > 1.05):
               target_linear_speed = 0.0
            else:
                target_linear_speed_th = 0.075 * ((1 - (abs(error_heading) / 3.14159)) ** 0.1)
                target_linear_speed = min(target_linear_speed_th, max_linear_speed)
            
            # remove point from path once reached
            if (distance < 0.03):
                self.path.pop(0)
             
            # send message
            self.send_speed(float(target_linear_speed), float(target_angular_speed))
        else:
            # path is empty stay where you are
            self.send_speed(0.0, 0.0)
                
    
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
