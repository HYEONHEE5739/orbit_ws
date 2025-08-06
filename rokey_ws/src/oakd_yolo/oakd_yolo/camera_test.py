#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class OakdRgbDepthNode(Node):
    def __init__(self):
        super().__init__('oakd_rgb_depth_node')

        # -----------------------
        # Publishers
        # -----------------------
        self.pub_rgb_comp   = self.create_publisher(CompressedImage, '/oakd/rgb/image_raw/compressed', 1)
        self.pub_depth_raw  = self.create_publisher(Image,           '/oakd/depth/image_raw',          1)
        self.pub_depth_comp = self.create_publisher(CompressedImage, '/oakd/depth_color/image_raw/compressed', 1)

        self.bridge = CvBridge()

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter('rgb.i_fps',            30.0)
        self.declare_parameter('rgb.i_width',          640)
        self.declare_parameter('rgb.i_height',         400)
        self.declare_parameter('rgb.i_interleaved',    False)
        self.declare_parameter('rgb.i_resolution',     '720')  # '720' or '1080' (센서 해상도)
        self.declare_parameter('stereo.i_align_depth', True)
        self.declare_parameter('depth.colorize',       False)  # depth를 컬러맵으로도 퍼블리시할지 여부

        self.fps          = float(self.get_parameter('rgb.i_fps').value)
        self.width        = int(self.get_parameter('rgb.i_width').value)
        self.height       = int(self.get_parameter('rgb.i_height').value)
        self.interleaved  = bool(self.get_parameter('rgb.i_interleaved').value)
        self.resolution   = self.get_parameter('rgb.i_resolution').value.upper()
        self.align_depth  = bool(self.get_parameter('stereo.i_align_depth').value)
        self.colorize     = bool(self.get_parameter('depth.colorize').value)

        # -----------------------
        # DepthAI pipeline
        # -----------------------
        self.device, self.qRgb, self.qDepth = self._create_pipeline()

        # -----------------------
        # Timer
        # -----------------------
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        self.get_logger().info("oakd_rgb_depth_node started (RGB + Depth only).")

    # ---------------------------------------------------------
    # DepthAI 파이프라인 생성
    # ---------------------------------------------------------
    def _create_pipeline(self):
        pipeline = dai.Pipeline()

        camRgb = pipeline.create(dai.node.ColorCamera)
        monoL  = pipeline.create(dai.node.MonoCamera)
        monoR  = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        xoutRgb   = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")

        # -----------------------
        # Color camera
        # -----------------------
        camRgb.setFps(self.fps)
        camRgb.setPreviewSize(self.width, self.height)
        camRgb.setInterleaved(self.interleaved)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        if self.resolution == "720":
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        else:
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        # -----------------------
        # StereoDepth
        # -----------------------
        monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoL.setCamera("left")
        monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoR.setCamera("right")

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        if self.align_depth:
            stereo.setDepthAlign(dai.CameraBoardSocket.RGB)  # RGB와 정렬
        stereo.setOutputSize(self.width, self.height)

        # -----------------------
        # Linking
        # -----------------------
        monoL.out.link(stereo.left)
        monoR.out.link(stereo.right)

        camRgb.preview.link(xoutRgb.input)
        stereo.depth.link(xoutDepth.input)

        # -----------------------
        # Start device & queues
        # -----------------------
        device = dai.Device(pipeline)
        qRgb   = device.getOutputQueue(name="rgb",   maxSize=1, blocking=False)
        qDepth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

        return device, qRgb, qDepth

    # ---------------------------------------------------------
    # Timer 콜백
    # ---------------------------------------------------------
    def timer_callback(self):
        inRgb   = self.qRgb.tryGet()
        inDepth = self.qDepth.tryGet()

        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id="oakd_rgb_frame")

        # 1) RGB (Compressed)
        if inRgb is not None:
            frame = inRgb.getCvFrame()  # BGR
            ret, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ret:
                comp = CompressedImage()
                comp.header = header
                comp.format = 'jpeg'
                comp.data = np.array(buf).tobytes()
                self.pub_rgb_comp.publish(comp)

        # 2) Depth (mono16, mm)
        if inDepth is not None:
            depth_frame = inDepth.getFrame()  # uint16 (mm)

            # raw depth
            depth_img = self.bridge.cv2_to_imgmsg(depth_frame, encoding='mono16')
            depth_img.header = header
            self.pub_depth_raw.publish(depth_img)

            # optional: colorized depth as compressed jpeg
            if self.colorize:
                # 시각화용으로 0~5000mm 클립 후 8bit로 노멀라이즈
                depth_vis = np.clip(depth_frame, 0, 5000).astype(np.float32)
                depth_vis = (depth_vis / 5000.0 * 255.0).astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                ret, buf = cv2.imencode('.jpg', depth_vis, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ret:
                    comp = CompressedImage()
                    comp.header = header
                    comp.format = 'jpeg'
                    comp.data = np.array(buf).tobytes()
                    self.pub_depth_comp.publish(comp)


def main():
    rclpy.init()
    node = OakdRgbDepthNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Stopping OAK-D...")
        node.device.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
