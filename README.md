# orbit_ws

로봇 시스템 통합 워크스페이스

---

## 📦 패키지 구성

### 🔹 multi_robot_interface  
- 🧩 **robot custom interface**

### 🔹 rokey_pjt (PC 제어용 패키지)
- 🚗 `nav_to_pose` : 큐레이팅 로봇의 내비게이션 노드  
- 🚶 `nav_to_pose_topic` : 안내 로봇의 내비게이션 노드  
- 🔁 `robot2_mqtt_bridge` : 큐레이팅 로봇 MQTT–ROS2 브릿지  
- 🔁 `robot3_mqtt_bridge` : 안내 로봇 MQTT–ROS2 브릿지  

### 🔹 rokey_ws (TurtleBot4 워크스페이스)
- 🧠 `oakd_yolo/yolo_depth_oakd` : OAK-D 카메라 기반 YOLO 탐지 + Depth 추정 노드  
- 🎯 `oakd_yolo/opainting_tf_marker` : 탐지된 그림 위치를 마커로 표시하는 노드     

---

### 🔹 orbit(ui)  
- react(javascript + styled-components)
- tts

## 📌 설명

- 본 워크스페이스는 **멀티로봇 협업**, **OAK-D 카메라 기반 인식**, **MQTT 기반 분산 제어** 등을 통합한 로봇 시스템입니다.
- ROS 2 기반으로 구현되었으며, TurtleBot4 및 DepthAI(OAK-D)를 활용합니다.
