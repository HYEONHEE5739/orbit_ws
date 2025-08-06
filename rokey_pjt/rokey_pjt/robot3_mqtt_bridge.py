#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from multi_robot_interface.msg import MissionStatus
from paho.mqtt import client as mqtt_client
import random

class Robot3MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros_bridge3')

        # ROS2 publisher (MQTT → ROS2)
        self.status_pub = self.create_publisher(MissionStatus, '/robot3/mission_status_from_mqtt', 10)
        # ROS2 subscriber (ROS2 → MQTT)
        self.status_sub = self.create_subscription(MissionStatus, '/robot3/mission_status', self.status_cb, 10)

        # MQTT 설정
        self.broker = 'xbe65600.ala.us-east-1.emqxsl.com'
        self.port = 8883
        self.username = 'parkjihye'
        self.password = 'rokey1234'
        self.client_id = f'mqtt-ros2-{random.randint(0, 1000)}'
        self.mqtt_arrived_pub = '/robot3/arrived'

        self.mqtt_client = mqtt_client.Client(client_id=self.client_id, protocol=mqtt_client.MQTTv311)
        self.mqtt_client.tls_set()
        self.mqtt_client.username_pw_set(self.username, self.password)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect(self.broker, self.port)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("✅ MQTT - ROBOT3 연결 성공")
            client.subscribe([
                ("/robot3/artwork_id", 0),
                ("/floor_changed", 0),
                ("/robot3/gohome", 0)
            ])
        else:
            self.get_logger().error(f"❌ MQTT - ROBOT3 연결 실패, 코드: {rc}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        ros_msg = MissionStatus()  # 콜백마다 생성하여 초기화 시키기
        
        # 작품ID 보내기 --> 로봇은 작품 ID로 판단해서 자율주행 동작
        if topic == "/robot3/artwork_id":
            data = msg.payload.decode()
            ros_msg.artwork_id = data
            self.get_logger().info(f"📥 MQTT → ROS2: artwork_id = {data}")

        # 작품의 층이 바뀌는 경우 -> 인계장소로 모이기
        elif topic == "/floor_changed":
            value = bool(int.from_bytes(msg.payload, 'big'))
            ros_msg.floor_changed = value
            self.get_logger().info(f"📥 MQTT → ROS2: floor_changed = {value}")

        # 두 로봇 인계장소 도착 -> home으로 이동
        elif topic == "/robot3/gohome":
            value = bool(int.from_bytes(msg.payload, 'big'))
            ros_msg.gohome = value
            self.get_logger().info(f"📥 MQTT → ROS2: gohome = {value}")

        self.status_pub.publish(ros_msg)

    def status_cb(self, msg: MissionStatus):
        try:
            self.mqtt_client.publish(self.mqtt_arrived_pub, b'\x01' if msg.arrived else b'\x00')
            self.get_logger().info(f"📤 ROS2 → MQTT (arrived): {msg.arrived}")
        except Exception as e:
            self.get_logger().error(f"❌ ROS2 → MQTT 전송 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Robot3MqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 브리지 노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
