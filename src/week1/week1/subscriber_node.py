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

class SubscriberNode(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            deadline=Duration(nanoseconds=10_000_000),  
            liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
            liveliness_lease_duration=Duration(seconds=1),
        )
        
        self.num_sub=self.create_subscription(Int32,"/number",self.square,10)
    
    def square(self,msg):
        square=msg.data*msg.data
        self.get_logger().info(f'Squared Number: {square}')

def main(args=None):
    rclpy.init(args=args)
    node=SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':   
    main()
