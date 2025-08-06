#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from multi_robot_interface.msg import MissionStatus
import time
from rclpy.executors import MultiThreadedExecutor

INITIAL_POSE_POSITION = [-0.95, 0.25]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH
LAST_POSE_POSITION = [-0.75, 0.552]
LAST_POSE_DIRECTION = TurtleBot4Directions.NORTH

GOAL_POSES = [([-1.75, 2.21], TurtleBot4Directions.SOUTH)]

ARTWORK_POSITIONS = {
    '1': [[-0.6, 3.0], TurtleBot4Directions.WEST],
    '2': [[-0.48, 0.153], TurtleBot4Directions.EAST],
    '3': [[-2.0, 0.02], TurtleBot4Directions.EAST]
}

class ArtworkNavigator(Node):
    def __init__(self):
        super().__init__('artwork_navigator')

        self.navigator = TurtleBot4Navigator(namespace='/robot2')
        self.painting_pose = None
        self.paint_check = False
        self.person_detected = False
        self.paint_keep_watching_state = False
        self.detecting = False
        self.start_time = None

        self.initial_pose = self.navigator.getPoseStamped(INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION)
        self.last_pose = self.navigator.getPoseStamped(LAST_POSE_POSITION, LAST_POSE_DIRECTION)
        self.goal_pose = self.navigator.getPoseStamped(*GOAL_POSES[0])

        self.status_pub = self.create_publisher(MissionStatus, '/robot2/mission_status', 10)
        
        self.status_sub = self.create_subscription(MissionStatus, '/robot2/mission_status_from_mqtt', self.status_callback, 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/robot2/person_detected', self.person_callback, 10)
        self.painting_keep_watching_subscription = self.create_subscription(Bool, '/robot2/painting_keep_watching', self.painting_keep_watching_callback, 1)
    
    
    def status_callback(self, msg: MissionStatus):
        if msg.floor_changed:
            self.handle_floor_change()

        if msg.artwork_id:
            self.set_painting_pose(msg.artwork_id)



    def painting_keep_watching_callback(self, msg: Bool):
        if msg.data == True:
            self.paint_keep_watching_state = True
            self.start_time = time.time()

        if time.time() - self.start_time > 20 or msg.data==False:
            self.navigator.startToPose(self.last_pose)
            self.paint_keep_watching_state = False
            self.navigator.dock()

    def handle_floor_change(self):
        self.get_logger().info("📥 층 변경 감지 → 초기화 후 목표 지점으로 이동")
        if not self.navigator.getDockedStatus():
            self.navigator.info('🔌 도킹 중...')
            self.navigator.dock()
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()
        self.navigator.startToPose(self.goal_pose)

        msg = MissionStatus()
        msg.arrived = True
        self.status_pub.publish(msg)
        self.detecting = True

    def set_painting_pose(self, artwork_id):
        if artwork_id not in ARTWORK_POSITIONS:
            self.get_logger().warn(f"❌ 존재하지 않는 artwork_id: {artwork_id}")
            self.navigator.startToPose(self.initial_pose)
            self.navigator.dock()

        coords, direction = ARTWORK_POSITIONS[artwork_id]
        self.painting_pose = self.navigator.getPoseStamped(coords, direction)
        self.paint_check = True
        self.get_logger().info(f"🎯 artwork_id {artwork_id} → painting_pose 설정 완료")

        if self.paint_keep_watching_state:
            self.navigator.startToPose(self.painting_pose)
            msg = MissionStatus()
            msg.arrived_painting = True
            self.status_pub.publish(msg)
            time.sleep(10)
            self.get_logger().info("작품을 다시 보시겠습니까?")
            self.start_time = time.time()

    def person_callback(self, msg: Bool):

        if self.detecting == True and self.paint_check == True:
            self.person_detected = msg.data
            self.get_logger().info(f"사람 감지 상태: {self.person_detected}")
            
            if self.painting_pose is None:
                self.get_logger().warn("⚠️ painting_pose가 아직 설정되지 않았습니다.")
                return


            if self.person_detected:
                self.navigator.info("🎯 탐지 성공! 3초 대기후 그림 앞으로 이동")
                time.sleep(3)
                msg2 = MissionStatus()
                msg2.arrived_painting = True
                self.status_pub.publish(msg2)
                self.navigator.startToPose(self.painting_pose)
               
                time.sleep(10)
                self.detecting = False
                self.start_time = time.time()
                self.get_logger().info("다시 보실건가요?")

            else:
                self.navigator.info("❌ 탐지 실패, 바로 복귀")
                self.detecting = False
                self.navigator.startToPose(self.last_pose)
                self.navigator.dock()


def main():
    rclpy.init()
    node = ArtworkNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
