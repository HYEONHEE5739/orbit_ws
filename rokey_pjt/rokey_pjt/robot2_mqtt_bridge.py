#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from multi_robot_interface.msg import MissionStatus
from paho.mqtt import client as mqtt_client
import random

class Robot2MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros_bridge2')

        # ROS2 publisher (MQTT → ROS2)
        self.status_pub = self.create_publisher(MissionStatus, '/robot2/mission_status_from_mqtt', 10)
        self.painting_keep_watching_pub = self.create_publisher(Bool, '/robot2/painting_keep_watching', 1)
        # ROS2 subscriber (ROS2 → MQTT)
        self.status_sub = self.create_subscription(MissionStatus, '/robot2/mission_status', self.status_cb, 10)
        self.person_detected_sub = self.create_subscription(
            Bool, '/robot2/person_detected', self.person_detected_cb, 1)
        self.artwork_detected_sub = self.create_subscription(
            Bool, '/robot2/artwork_detected', self.artwork_detected_cb, 1)
        # MQTT 설정
        self.broker = 'xbe65600.ala.us-east-1.emqxsl.com'
        self.port = 8883
        self.username = 'parkjihye'
        self.password = 'rokey1234'
        self.client_id = f'mqtt-ros2-{random.randint(0, 1000)}'

        # MQTT publish 토픽
        self.mqtt_arrived_pub = '/robot2/arrived'
        self.mqtt_arrived_painting_pub = '/robot2/arrived_painting'
        self.mqtt_artwork_det_pub = '/robot2/artwork_detected'
        self.mqtt_person_det_pub = '/robot2/person_detected'

        self.mqtt_client = mqtt_client.Client(client_id=self.client_id, protocol=mqtt_client.MQTTv311)
        self.mqtt_client.tls_set()
        self.mqtt_client.username_pw_set(self.username, self.password)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.broker, self.port)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("✅ MQTT - ROBOT2 연결 성공")
            client.subscribe([
                ("/robot2/artwork_id", 0),
                ("/floor_changed", 0),
                ("/robot2/gohome", 0),
                ("/robot2/painting_keep_watching", 0),
            ])
        else:
            self.get_logger().error(f"❌ MQTT - ROBOT2 연결 실패, 코드: {rc}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic

        # 작품ID 보내기
        if topic == "/robot2/artwork_id":
            payload = msg.payload.decode()
            ros_msg = MissionStatus()
            ros_msg.artwork_id = payload
            self.get_logger().info(f"📥 MQTT → ROS2: artwork_id = {payload}")
            self.status_pub.publish(ros_msg)
        
        # 작품의 층이 바뀌는 경우 -> 인계장소로 모이기
        elif topic == "/floor_changed":
            value = bool(int.from_bytes(msg.payload, 'big'))
            ros_msg = MissionStatus()
            ros_msg.floor_changed = value
            self.get_logger().info(f"📥 MQTT → ROS2: floor_changed = {value}")
            self.status_pub.publish(ros_msg)

        # 큐레이팅 종료 -> home으로 이동
        elif topic == "/robot2/gohome":
            value = bool(int.from_bytes(msg.payload, 'big'))
            ros_msg = MissionStatus()
            ros_msg.gohome = value
            self.get_logger().info(f"📥 MQTT → ROS2: gohome = {value}")
            self.status_pub.publish(ros_msg)

        elif topic == "/robot2/painting_keep_watching":
            value = bool(int.from_bytes(msg.payload, 'big'))
            ros_msg = Bool()
            ros_msg.data = value
            self.painting_keep_watching_pub(ros_msg)
            self.get_logger().info(f"📥 MQTT → ROS2: painting_keep_watching = {value}")

    def status_cb(self, msg: MissionStatus):
        try:
            # 각각의 필드가 True일 경우 MQTT 전송
            if msg.arrived:
                self.mqtt_client.publish(self.mqtt_arrived_pub, b'\x01')
                self.get_logger().info(f"📤 ROS2 → MQTT: arrived = True")

            if msg.arrived_painting:
                self.mqtt_client.publish(self.mqtt_arrived_painting_pub, b'\x01')
                self.get_logger().info(f"📤 ROS2 → MQTT: arrived_painting = True")

        except Exception as e:
            self.get_logger().error(f"❌ ROS2 → MQTT 전송 실패: {e}")

    # ---------- 사람 인식 : 카메라단과 인터페이스 합의하기 ------------
    def person_detected_cb(self, msg):
        try:
            self.mqtt_client.publish(self.mqtt_person_det_pub, msg.data)
            self.get_logger().info(f":보낼_편지함_트레이: ROS2 → MQTT ( /robot2/person_detected ): {msg.data}")
        except Exception as e:
            self.get_logger().error(f":x: ROS → MQTT 전송 실패: {e}")
    # ---------- 작품 인식 : 카메라단과 인터페이스 합의하기 ------------
    def artwork_detected_cb(self, msg):
        try:
            self.mqtt_client.publish(self.mqtt_artwork_det_pub, msg.data)  # data : True
            self.get_logger().info(f":보낼_편지함_트레이: ROS2 → MQTT ( /robot2/artwork_detected ): {msg.data}")
        except Exception as e:
            self.get_logger().error(f":x: ROS → MQTT 전송 실패: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = Robot2MqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 브리지 노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
