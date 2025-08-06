#!/usr/bin/env python3
import os, sys, threading, queue, json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# =================================================
# 1) 클래스 번호 → 작품 ID 매핑 정의
#    예: YOLO가 class 0→작품1, class1→작품2, class2→작품3
CLASS_TO_PIECE_ID = {
    0: 3,   
    2: 1,    
    1: 2,     
}

MODEL_PATH          = '/home/rokey/orbit_ws'
IMAGE_TOPIC         = '/robot2/oakd/rgb/preview/image_raw'
CONFIDENCE_THRESHOLD = 0.90

# ROS 토픽 이름
MATCH_BOOL_TOPIC    = '/object_match'     # 기존 Bool 신호
MATCH_ID_TOPIC      = '/matched_piece'    # 새로 추가할 Int32 신호

class YOLOTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)

        # 모델 & 브릿지
        self.model = YOLO(MODEL_PATH)
        self.bridge = CvBridge()

        # 퍼블리셔: Bool(객체 유무), Int32(작품 ID)
        self.bool_pub = self.create_publisher(Bool, MATCH_BOOL_TOPIC, 10)
        self.id_pub   = self.create_publisher(Int32, MATCH_ID_TOPIC, 10)

        # 이미지 구독 + 처리 스레드
        self.image_queue = queue.Queue(maxsize=1)
        self.create_subscription(Image, IMAGE_TOPIC, self.image_callback, 10)
        thread = threading.Thread(target=self.tracking_loop, daemon=True)
        thread.start()

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.image_queue.full():
            _ = self.image_queue.get_nowait()
        self.image_queue.put(frame)

    def tracking_loop(self):
        while rclpy.ok():
            try:
                frame = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            results = self.model.track(source=frame, stream=True, persist=True, verbose=False)
            found_high_conf = False

            for r in results:
                for box in r.boxes:
                    cls  = int(box.cls[0])
                    conf = float(box.conf[0])
                    if conf < CONFIDENCE_THRESHOLD:
                        continue

                    # ====================================================
                    # 2) 클래스 번호 → 작품 ID 변환
                    if cls in CLASS_TO_PIECE_ID:
                        piece_id = CLASS_TO_PIECE_ID[cls]
                        # Bool 신호 발행
                        found_high_conf = True
                        # 작품 ID 발행
                        id_msg = Int32()
                        id_msg.data = piece_id
                        self.id_pub.publish(id_msg)
                        self.get_logger().info(f"Published {MATCH_ID_TOPIC}: {piece_id}")
                    # ====================================================

                    # 시각화 (기존 코드)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    label = f"{self.model.names[cls]}:{piece_id if cls in CLASS_TO_PIECE_ID else cls}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,0,255), 2)
                    cv2.putText(frame, f"{label} ({conf:.2f})",
                                (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)

            # ========================================================
            # 3) Bool 신호 발행 (신뢰도 조건 만족 시)
            if found_high_conf:
                bool_msg = Bool()
                bool_msg.data = True
                self.bool_pub.publish(bool_msg)
            # ========================================================

            cv2.imshow("YOLO Tracking", cv2.resize(frame, (0,0), fx=2, fy=2))
            if cv2.waitKey(1) == ord('q'):
                break

def main():
    rclpy.init()
    node = YOLOTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
