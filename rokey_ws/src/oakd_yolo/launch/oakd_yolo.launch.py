#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import rclpy
from std_srvs.srv import Trigger

# 카메라 종료 서비스 호출
def call_stop_camera_service(context, *args, **kwargs):
    rclpy.init()
    node = rclpy.create_node('stop_camera_launcher')

    client = node.create_client(Trigger, '/robot2/oakd/stop_camera')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('서비스 없음: /robot2/oakd/stop_camera')
        rclpy.shutdown()
        return []

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"stop_camera 호출 성공: {future.result().message}")
    else:
        node.get_logger().error("stop_camera 호출 실패")

    node.destroy_node()
    rclpy.shutdown()
    return []

# YOLO 노드를 런타임에 생성
def launch_yolo_node(context, *args, **kwargs):
#    param_file = LaunchConfiguration('param_file').perform(context)
    debug_mode = LaunchConfiguration('debug_mode').perform(context)

    return [
        Node(
            package='oakd_yolo',
            executable='yolo_depth_oakd',
            name='yolo_depth_oakd',
            namespace='robot2',
            output='screen',
            parameters=[
                #param_file,
                {'debug_mode': debug_mode.lower() == 'true'}
            ]
        )
    ]

# 최종 런치 구성
def generate_launch_description():
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/home/ubuntu/rokey_ws/src/oakd_yolo/config/oakd_yolo.yaml'
    )
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false'
    )

    return LaunchDescription([
       # param_file_arg,
        debug_mode_arg,
        OpaqueFunction(function=call_stop_camera_service),
        TimerAction(period=5.0, actions=[OpaqueFunction(function=launch_yolo_node)])
    ])
