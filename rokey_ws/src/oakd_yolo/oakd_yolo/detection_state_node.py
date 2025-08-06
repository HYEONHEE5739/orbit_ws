#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class DetectionStateNode(Node):
    def __init__(self):
        super().__init__('detection_state_node')

        # 1) Subscriber: artwork_id (String)
        self.artwork_sub = self.create_subscription(
            String,
            '/robot2/artwork_id',
            self.artwork_callback,
            10
        )

        # 2) Publishers: person_detected, artwork_detected (Bool)
        self.person_pub  = self.create_publisher(Bool, '/robot2/person_detected', 10)
        self.artwork_pub = self.create_publisher(Bool, '/robot2/artwork_detected', 10)

    def artwork_callback(self, msg: String):
        artwork_id = msg.data.strip()
        # 사람 감지 여부: artwork_id 메시지가 들어오면 True
        person_msg = Bool(data=True)
        self.person_pub.publish(person_msg)
        self.get_logger().info(f'Published /robot2/person_detected: True')

        # 작품 감지 여부: 유효한 ID가 들어왔을 때만 True
        has_artwork = bool(artwork_id)
        artwork_msg = Bool(data=has_artwork)
        self.artwork_pub.publish(artwork_msg)
        self.get_logger().info(f'Published /robot2/artwork_detected: {has_artwork}')

def main():
    rclpy.init()
    node = DetectionStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
