#!/usr/bin/env python3
import cv2
import depthai as dai
from pathlib import Path
import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from visualization_msgs.msg import Marker

from std_msgs.msg import Bool, Header
from geometry_msgs.msg import PointStamped, Pose2D
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String


# 모델 라벨 선정
LABELS = ["Yan","Moonk","Da_Vinci","Customer"]
CONF_THRESH = 0.8
BLOB_PATH = "/home/ubuntu/rokey_ws/models/best_museum2_openvino_2022.1_6shave.blob"
CLASS_TO_WEB_ID ={
    '0' : 3,
    '1' : 1,
    '2' : 2
}
class OakdYoloNode(Node):
    def __init__(self):
        super().__init__('oakd_yolo_node')

        # Publishers
        self.pub_dets       = self.create_publisher(Detection2DArray, 'oakd/yolo_detections', 1)
        self.pub_center     = self.create_publisher(PointStamped,      'oakd/target_center',  1)
        self.pub_has_det    = self.create_publisher(Bool,              'oakd/has_detection',  1)
        self.pub_detect_img  = self.create_publisher(CompressedImage,   'oakd/detect_image/compressed',1)
        self.pub_detect_img_raw = self.create_publisher(Image, 'oakd/detect_image', 1)
        self.pub_person_detected = self.create_publisher(Bool, '/robot2/person_detected', 1)
        self.pub_artwork_detected = self.create_publisher(Bool, '/robot2/artwork_detected', 1)
        self.pub_marker = self.create_publisher(Marker, "oakd/marker", 1)

        self.sub_artwork_id = self.create_subscription(String, '/robot2/artwork_id', self.artwork_id_callback, 1)

        self.bridge = CvBridge()

        self.frame_counter = 0
        self.fps_start_time = time.time()

        # Parameters
#        self.fps         = self.get_parameter_or('rgb.i_fps', 30.0)
#        self.width       = self.get_parameter_or('rgb.i_width', 320)
#        self.height      = self.get_parameter_or('rgb.i_height', 320)
#        self.interleaved = self.get_parameter_or('rgb.i_interleaved', False)
#        self.resolution  = self.get_parameter_or('rgb.i_resolution', '720').upper()
#        self.align_depth = self.get_parameter_or('stereo.i_align_depth', True)
        self.debug_mode  = self.get_parameter_or('debug_mode', False)
        self.fps          = 10.0           # 고정 FPS
        self.width        = 320
        self.height       = 320
        self.interleaved  = False
        self.resolution   = '720'          # '720' or '1080'
        self.align_depth  = True
        self.jpeg_q       = 70
        self.jpeg_q = 70


        self.current_target_artwork_id = None
        # DepthAI pipeline
        self.device, self.qRgb, self.qDet = self._create_pipeline()
        time.sleep(4.0)
        # Timer (30FPS)
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.get_logger().info("oakd_yolo_node started (Timer + SingleThreadedExecutor).")
        if self.debug_mode:
            self.get_logger().debug("Debug_mode") 

    #구독박은 ID를 콜뱆
    def artwork_id_callback(self, msg: String):
        self.current_target_artwork_id = msg.data
        self.get_logger().info(f"Target artwork ID set to: {self.current_target_artwork_id}")

    # ---------------------------------------------------------
    # DepthAI 파이프라인 생성
    # ---------------------------------------------------------
    def _create_pipeline(self):
        if not Path(BLOB_PATH).exists():
            raise FileNotFoundError(f"Blob not found: {BLOB_PATH}")

        pipeline = dai.Pipeline()

        camRgb = pipeline.create(dai.node.ColorCamera)
        monoL  = pipeline.create(dai.node.MonoCamera)
        monoR  = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        yolo   = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        xoutDet = pipeline.create(dai.node.XLinkOut)
        xoutDet.setStreamName("detections")

        # Color camera
        camRgb.setFps(self.fps)
        camRgb.setPreviewSize(self.width, self.height)
        camRgb.setInterleaved(self.interleaved)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        if "720" in self.resolution:
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        elif "1080" in self.resolution:
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        else:
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_400_P)

        # Stereo 
        monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoL.setCamera("left")
        monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoR.setCamera("right")

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        if self.align_depth:
            stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(self.width, self.height)

        # YOLO Spatial
        yolo.setBlobPath(BLOB_PATH)
        yolo.setConfidenceThreshold(CONF_THRESH)
        yolo.setIouThreshold(0.5)
        yolo.setNumClasses(len(LABELS))
        yolo.setCoordinateSize(4)
        yolo.setNumInferenceThreads(2)
        yolo.input.setBlocking(False)

        yolo.setDepthLowerThreshold(100)
        yolo.setDepthUpperThreshold(5000)

        # Linking
        monoL.out.link(stereo.left)
        monoR.out.link(stereo.right)

        camRgb.preview.link(yolo.input)
        stereo.depth.link(yolo.inputDepth)

        yolo.passthrough.link(xoutRgb.input)
        yolo.out.link(xoutDet.input)

        # Start
        device = dai.Device(pipeline)
        qRgb = device.getOutputQueue(name="rgb",        maxSize=1, blocking=False)
        qDet = device.getOutputQueue(name="detections", maxSize=1, blocking=False)

        return device, qRgb, qDet

    # ---------------------------------------------------------
    # Timer 콜백: 최신 프레임만 꺼내서 publish
    # ---------------------------------------------------------
    def timer_callback(self):
        if not self.qRgb.has() or not self.qDet.has():
            return
        # 최신 프레임 가져오기
        inRgb = self.qRgb.tryGet()
        inDet = self.qDet.tryGet()

        if inRgb is None or inDet is None:
            return

        frame = inRgb.getCvFrame()
        dets  = inDet.detections
        has_detection = len(dets) > 0

        now = self.get_clock().now().to_msg()

        # 1) has_detection
        self.pub_has_det.publish(Bool(data=has_detection))

        # 2) Detection2DArray
        det_array_msg = self._make_detection_array_msg(dets, frame.shape[1], frame.shape[0], now)
        if det_array_msg is not None:
            self.pub_dets.publish(det_array_msg)

        if has_detection:
            self.get_logger().info("detected")
            # 3) center(가장 conf 높은 detection 기준)
            best = max(dets, key=lambda d: d.confidence)
            x_m, y_m, z_m = self._center_and_depth(best)
            pt = PointStamped()
            pt.header.stamp = now
            pt.header.frame_id = "oakd_rgb_camera_optical_frame"
            pt.point.x = float(x_m)
            pt.point.y = float(y_m)
            pt.point.z = float(z_m)
            self.pub_center.publish(pt)

            x_m, y_m, z_m = self._center_and_depth(best)

            # === FPS 측정 ===
            self.frame_counter += 1
            now_time = time.time()
            elapsed = now_time - self.fps_start_time

            if elapsed >= 5.0:  # 5초 단위로 평균 FPS 출력
                fps = self.frame_counter / elapsed
                self.get_logger().info(f"[FPS 측정] 실제 처리 FPS: {fps:.2f}")
                self.fps_start_time = now_time
                self.frame_counter = 0

            # Marker publish
            label = LABELS[best.label] if best.label < len(LABELS) else str(best.label)
            marker_msg = self._make_marker_msg(x_m, y_m, z_m, label, now)
            self.pub_marker.publish(marker_msg)

            # Person_detacted publish
            person_detected = Bool(data=False)
            if best.label < len(LABELS) and LABELS[best.label] == "Customer":
                person_detected.data = True
                self.pub_person_detected.publish(person_detected)

            # Person_detacted publish
            artwork_detected = Bool(data=False)
            best_class_str = str(best.label)
            if best_class_str in CLASS_TO_WEB_ID:
                mapped_web_id = str(CLASS_TO_WEB_ID[best_class_str])
                if self.current_target_artwork_id == mapped_web_id:
                    self.get_logger().info(f"Artwork matched: class_id {best.label} → web_id {mapped_web_id}")
                    artwork_detected.data = True
                    self.pub_artwork_detected.publish(artwork_detected)

        publish_image = self.debug_mode or has_detection
        if publish_image:
            if has_detection:
                detect_img = self._draw(frame, dets, frame.shape[1], frame.shape[0])
            else:
                detect_img = frame

            ret, buf = cv2.imencode('.jpg', detect_img,
                                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_q])
            if ret:
                comp = CompressedImage()
                comp.header.stamp = now
                comp.header.frame_id = "oakd_rgb_frame"
                comp.format = 'jpeg'
                comp.data = np.array(buf).tobytes()
                self.pub_detect_img.publish(comp)

                img_msg = self.bridge.cv2_to_imgmsg(detect_img, encoding="bgr8")
                img_msg.header.stamp = now
                img_msg.header.frame_id = "oakd_rgb_frame"
                self.pub_detect_img_raw.publish(img_msg)

    # ---------------------------------------------------------
    # Detection2DArray 생성
    # ---------------------------------------------------------
    def _make_detection_array_msg(self, dets, W, H, stamp):
        header = Header(stamp=stamp, frame_id="oakd_rgb_frame")
        det_array = Detection2DArray()
        det_array.header = header

        for d in dets:
            # 픽셀 좌표로 변경
            x1 = int(d.xmin * W); y1 = int(d.ymin * H)
            x2 = int(d.xmax * W); y2 = int(d.ymax * H)

            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w  = (x2 - x1)
            h  = (y2 - y1)

            det_msg = Detection2D()
            det_msg.header = header
            det_msg.bbox = BoundingBox2D()
            det_msg.bbox.center.position.x = float(cx)
            det_msg.bbox.center.position.y = float(cy)
            det_msg.bbox.center.theta = 0.0
            det_msg.bbox.size_x = float(w)
            det_msg.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = LABELS[d.label] if d.label < len(LABELS) else str(d.label) # 클래스 이름
            hyp.hypothesis.score = float(d.confidence)
            det_msg.results.append(hyp)

            det_array.detections.append(det_msg)

        return det_array


    # ---------------------------------------------------------
    # draw annotations
    # ---------------------------------------------------------
    @staticmethod
    def _draw(frame, dets, W, H):
        color = (0, 255, 0)
        for d in dets:
            x1 = int(d.xmin * W); y1 = int(d.ymin * H)
            x2 = int(d.xmax * W); y2 = int(d.ymax * H)
            cx = (x1 + x2) // 2; cy = (y1 + y2) // 2

            label = LABELS[d.label] if d.label < len(LABELS) else str(d.label)
            conf  = int(d.confidence * 100)
            z_m   = float(d.spatialCoordinates.z) / 1000.0
            z_m   = z_m - 0.04

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            y_text = y1 - 10 if y1 - 10 > 10 else y1 + 20
            cv2.putText(frame, f"{label} {conf}%", (x1, y_text),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Z: {z_m:.3f} m", (cx, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        return frame

    # ---------------------------------------------------------
    # center pixel & depth(m)
    # ---------------------------------------------------------
    @staticmethod
    def _center_and_depth(d):
        # x1 = int(d.xmin * W); y1 = int(d.ymin * H)
        # x2 = int(d.xmax * W); y2 = int(d.ymax * H)
        # cx = (x1 + x2) // 2
        # cy = (y1 + y2) // 2
        x_m = float(d.spatialCoordinates.x) / 1000.0  # m
        y_m= float(d.spatialCoordinates.y) / 1000.0  # m
        z_m = float(d.spatialCoordinates.z) / 1000.0  # mm -> m
        z_m = z_m -0.04 # offset 
        return x_m, y_m, z_m
    
    def _make_marker_msg(self, x, y, z, label, now):
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = "oakd_rgb_camera_optical_frame"
        marker.ns = "oakd_detections"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        # marker.pose.position.z = z + 0.1  # 텍스트가 객체 위에 떠 있도록
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        # marker.scale.z = 0.1  # 텍스트 크기
        marker.color.a = 1.0

        # 🎨 색상 지정
        if label == "Customer":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0  # 초록색
        elif label in ["Yan", "Moonk", "Da_Vinci"]:  # 필요한 경우 label 이름 추가
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # 파란색
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0  # 기본 노란색
            
            marker.text = label
        return marker

def main():
    rclpy.init()
    node = OakdYoloNode()

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
