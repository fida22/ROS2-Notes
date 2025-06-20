#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2
import math

class ObstacleStopNode(Node):
    def __init__(self):
        super().__init__('obstacle_stop_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/gazebo_ros_laser_controller/out', self.lidar_callback, 10)
        self.teleop_sub = self.create_subscription(
            Twist, '/cmd_vel_input', self.teleop_callback, 10)

        self.last_teleop_cmd = Twist()
        self.safe_to_move = True
        self.min_safe_distance = 0.5  # Stop if obstacle < 0.5m
        self.resume_distance = 0.7    # Resume if obstacle > 0.7m 
        self.obstacle_detected = False

        self.timer = self.create_timer(0.1, self.drive_loop)
        self.get_logger().info("ObstaceStopNode started — press any key to move")

    def teleop_callback(self, msg):
        self.last_teleop_cmd = msg
        # Force resume if a new command is received (even if obstacle was detected)
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            if not self.safe_to_move:
                self.get_logger().info("New manual command overriding obstacle stop!")
                self.safe_to_move = True

    def lidar_callback(self, msg):
        min_distance = float('inf')

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, _ = point  
            if -1.0 < y < 1.0 and x > 0:  # Check 1m left/right in front
                distance = math.sqrt(x**2 + y**2)
                if distance < min_distance:
                    min_distance = distance

        if min_distance < self.min_safe_distance:
            if self.safe_to_move:
                self.get_logger().warn(f"Obstacle at {min_distance:.2f}m! Stopping.")
            self.safe_to_move = False
            self.obstacle_detected = True
        elif min_distance > self.resume_distance and self.obstacle_detected:
            self.get_logger().info("Path clear, resuming movement.")
            self.safe_to_move = True
            self.obstacle_detected = False

    def drive_loop(self):
        cmd = Twist()
        if self.safe_to_move:
            cmd = self.last_teleop_cmd
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()