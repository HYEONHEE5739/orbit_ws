import mqtt from "mqtt";

// 연결 설정
const options = {
  username: "parkjihye",
  password: "rokey1234",
  protocol: "wss",  // 또는 "mqtts"
};

const client = mqtt.connect("wss://xbe65600.ala.us-east-1.emqxsl.com:8084/mqtt", options);

// 기본 연결 이벤트
client.on("connect", () => {
  console.log("✅ MQTT 연결 성공");
});

export default client;
