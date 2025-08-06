#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from multi_robot_interface.msg import MissionStatus
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import time
import sys
from rclpy.utilities import remove_ros_args

# 작품 ID → 좌표 매핑
GOAL_COORDINATES = {
    '1': [-0.155, -5.78],
    '2': [-0.155, -5.78],
    '3': [-0.155, -5.78],
}

# 초기 위치 및 방향
INITIAL_POSE_POSITION = [-0.245, -3.11]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

class ArtworkNavigator(Node):
    def __init__(self, goal_coordinates, initial_pose_position, initial_pose_direction, artwork_id=None):
        super().__init__('artwork_navigator')

        self.goal_coordinates = goal_coordinates
        self.initial_pose_position = initial_pose_position
        self.initial_pose_direction = initial_pose_direction

        self.navigator = TurtleBot4Navigator()
        self.initial_pose = self.navigator.getPoseStamped(
            self.initial_pose_position,
            self.initial_pose_direction
        )

        self.goal_pose = None
        self.goal_received = False
        self.waiting_for_gohome = False

        self.latest_artwork_id = None
        self.floor_changed_flag = False


        # arrived_pub
        self.status_pub = self.create_publisher(MissionStatus, '/robot3/mission_status', 1)
        
        self.status_sub = self.create_subscription(MissionStatus, '/robot3/mission_status_from_mqtt', self.status_cb, 1)
        
        # CLI 인자로 artwork_id 수신 가능 (테스트용)
        if artwork_id:
            self.get_logger().info(f"[CLI] artwork_id = {artwork_id}")
            self.latest_artwork_id = artwork_id
            self.try_navigation()

    def status_callback(self, msg: MissionStatus):
        # artwork_id가 있다면 저장
        if msg.artwork_id:
            self.latest_artwork_id = msg.artwork_id
            self.get_logger().info(f"📥 artwork_id 수신: {msg.artwork_id}")
            self.try_navigation()

        # floor_changed가 True이면
        if msg.floor_changed:
            self.floor_changed_flag = True
            self.get_logger().info(f"📥 floor_changed: True")
            self.try_navigation()

        # gohome 수신 처리
        if msg.gohome and self.waiting_for_gohome:
            self.get_logger().info("🏠 gohome 명령 수신 → 초기 위치로 복귀 시작")
            self.return_to_start()
            self.waiting_for_gohome = False


    def try_navigation(self):
        if self.floor_changed_flag and self.latest_artwork_id:
            self.get_logger().info("🚀 조건 만족: floor_changed + artwork_id → 이동 시작")
            self.process_artwork_id(self.latest_artwork_id)
            # 조건 충족 후 플래그 초기화
            self.floor_changed_flag = False
            self.latest_artwork_id = None

    def process_artwork_id(self, artwork_id):
        if artwork_id not in self.goal_coordinates:
            self.get_logger().warn(f"❌ 알 수 없는 artwork ID: {artwork_id}")
            return

        coords = self.goal_coordinates[artwork_id]
        self.goal_pose = self.navigator.getPoseStamped(coords, TurtleBot4Directions.NORTH)
        self.goal_received = True

        self.get_logger().info(f"✅ 목적지 설정: ID {artwork_id} → 좌표 {coords}")

        if not self.navigator.getDockedStatus():
            self.navigator.info('도킹되지 않음 → 도킹 중...')
            self.navigator.dock()

        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.navigate_to_goal()

    def navigate_to_goal(self):
        self.navigator.goToPose(self.goal_pose)
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        self.navigator.info("✅ 목표 지점 도착 완료")
        arrived_msg = MissionStatus()
        arrived_msg.arrived = True
        self.status_pub.publish(arrived_msg)
        self.waiting_for_gohome = True

    def return_to_start(self):
        self.navigator.info("초기 위치로 복귀 중...")
        self.navigator.goToPose(self.initial_pose)
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        self.navigator.dock()
        self.get_logger().info("✅ 초기 위치 복귀 및 도킹 완료")
        self.goal_received = False

def main(args=None):
    rclpy.init(args=args)

    cli_args = remove_ros_args(sys.argv)
    artwork_id = None
    if len(cli_args) > 1:
        artwork_id = cli_args[1]

    node = ArtworkNavigator(
        goal_coordinates=GOAL_COORDINATES,
        initial_pose_position=INITIAL_POSE_POSITION,
        initial_pose_direction=INITIAL_POSE_DIRECTION,
        artwork_id=artwork_id
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()