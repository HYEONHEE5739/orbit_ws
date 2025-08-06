import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped


class MarkerTransformNode(Node):
    def __init__(self):
        super().__init__('marker_transform_node')

        # 원본 마커 구독
        self.marker_sub = self.create_subscription(
            Marker,
            '/robot2/oakd/marker',
            self.marker_callback,
            10
        )

        # 변환 마커 퍼블리셔
        self.marker_pub = self.create_publisher(
            Marker,
            '/marker_transformed',
            10
        )

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def marker_callback(self, msg: Marker):
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = msg.header.frame_id
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()  # 🔑 최신 시간으로 고정
            pose_stamped.pose = msg.pose

            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time(),  # 🔑 최신 시간 사용
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            transformed_pose = self.tf_buffer.transform(
                pose_stamped,
                'map',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )


            self.get_logger().info(
                f"📍 map 좌표: x={transformed_pose.pose.position.x:.2f}, "
                f"y={transformed_pose.pose.position.y:.2f}, "
                f"z={transformed_pose.pose.position.z:.2f}"
            )

            # 변환된 마커 발행
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = transformed_pose.pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = MarkerTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
