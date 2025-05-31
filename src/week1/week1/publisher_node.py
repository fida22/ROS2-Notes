#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    LivelinessPolicy,
    Duration
)

class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher_node")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            deadline=Duration(nanoseconds=10_000_000),  
            liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
            liveliness_lease_duration=Duration(seconds=1)
        )
        self.num_pub=self.create_publisher(Int32,"/number",10)
        self.num=1
        self.timer=self.create_timer(1.0,self.publish_num)
    
    def publish_num(self):
            msg=Int32()
            msg.data=self.num
            self.num_pub.publish(msg)
            self.get_logger().info(f"Number: {self.num}")
            self.num+=1
      

def main(args=None):
    rclpy.init(args=args)
    node=PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':   
    main()
