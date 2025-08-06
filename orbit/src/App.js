import React, { useState, useEffect, useRef, useCallback } from "react";
import styled from "styled-components";
import ArtworkList from "./components/ArtworkList";
import ArtworkDetail from "./components/ArtworkDetail";
import GlobalStyle from "./styles/GlobalStyle";
import client from "./mqttClient";

function App() {
  const [step, setStep] = useState("mode");
  const [selectedArtwork, setSelectedArtwork] = useState(null);
  const [floor, setFloor] = useState(1);

  // 상세 화면 진입 조건(AND)
  const [detected, setDetected] = useState(false);
  const [artworkDetectedFlag, setArtworkDetectedFlag] = useState(false);   // /robot2/artwork_detected
  const [arrivedPaintingFlag, setArrivedPaintingFlag] = useState(false);   // /robot2/arrived_painting

  // 보조 상태
  const [arrived, setArrived] = useState(false); // 일반 도착 표시용(옵션)
  const [gohomeSent, setGohomeSent] = useState(false);

  // 동시조건용 플래그: /robot2/person_detected, /robot3/arrived
  const [personDetected, setPersonDetected] = useState(false);
  const [robot3ArrivedFlag, setRobot3ArrivedFlag] = useState(false);

  // 최신값을 위해 ref 동기화
  const selectedRef = useRef(null);
  useEffect(() => { selectedRef.current = selectedArtwork; }, [selectedArtwork]);

  const personDetectedRef = useRef(false);
  useEffect(() => { personDetectedRef.current = personDetected; }, [personDetected]);

  const robot3ArrivedRef = useRef(false);
  useEffect(() => { robot3ArrivedRef.current = robot3ArrivedFlag; }, [robot3ArrivedFlag]);

  const gohomeSentRef = useRef(false);
  useEffect(() => { gohomeSentRef.current = gohomeSent; }, [gohomeSent]);

  // MQTT payload → boolean 파서
  const parseBoolPayload = (payload) => {
    if (payload && typeof payload.length === "number") {
      if (payload.length === 1 && (payload[0] === 0 || payload[0] === 1)) {
        return payload[0] === 1;
      }
    }
    try {
      const txt = new TextDecoder().decode(payload).trim().toLowerCase();
      if (txt === "true" || txt === "1") return true;
      if (txt === "false" || txt === "0") return false;
      const obj = JSON.parse(txt);
      if (typeof obj === "boolean") return obj;
      if (obj && typeof obj.detected === "boolean") return obj.detected;
      if (obj && typeof obj.arrived === "boolean") return obj.arrived;
    } catch (_) {}
    return false;
  };

  // 두 신호가 모두 true일 때만 상세 본문 표시/전환
  const tryEnterDetail = useCallback(() => {
    if (selectedRef.current && artworkDetectedFlag && arrivedPaintingFlag) {
      setDetected(true);
      setStep((prev) => (prev === "detail" ? prev : "detail"));
    }
  }, [artworkDetectedFlag, arrivedPaintingFlag]);

  // /robot3/gohome: /robot2/person_detected && /robot3/arrived 동시에 true일 때 한 번만
  const trySendGohome = useCallback(() => {
    if (
      personDetectedRef.current &&
      robot3ArrivedRef.current &&
      !gohomeSentRef.current
    ) {
      const one = Uint8Array.from([1]);
      client.publish("/robot3/gohome", one);
      console.log("📤 /robot3/gohome → <0x01> (person_detected && robot3/arrived)");
      setGohomeSent(true);
      gohomeSentRef.current = true;
    }
  }, []);

  // 구독
  useEffect(() => {
    client.subscribe("/robot2/arrived", (err) =>
      err ? console.error("❌ /robot2/arrived 구독 실패", err)
          : console.log("📡 /robot2/arrived 구독 성공")
    );
    client.subscribe("/robot3/arrived", (err) =>
      err ? console.error("❌ /robot3/arrived 구독 실패", err)
          : console.log("📡 /robot3/arrived 구독 성공")
    );
    client.subscribe("/robot2/artwork_detected", (err) =>
      err ? console.error("❌ /robot2/artwork_detected 구독 실패", err)
          : console.log("📡 /robot2/artwork_detected 구독 성공")
    );
    client.subscribe("/robot2/arrived_painting", (err) =>
      err ? console.error("❌ /robot2/arrived_painting 구독 실패", err)
          : console.log("📡 /robot2/arrived_painting 구독 성공")
    );
    client.subscribe("/robot2/person_detected", (err) =>
      err ? console.error("❌ /robot2/person_detected 구독 실패", err)
          : console.log("📡 /robot2/person_detected 구독 성공")
    );

    const onMessage = (topic, payload) => {
      // (보조) 도착 신호
      if (topic === "/robot2/arrived") {
        const isArrived = parseBoolPayload(payload);
        console.log(`📥 /robot2/arrived =`, isArrived);
        setArrived(isArrived);
        return;
      }

      if (topic === "/robot3/arrived") {
        const isArrived = parseBoolPayload(payload);
        console.log(`📥 /robot3/arrived =`, isArrived);
        setArrived(isArrived);
        setRobot3ArrivedFlag(isArrived);
        robot3ArrivedRef.current = isArrived;
        // 조건 충족 시 gohome 시도
        trySendGohome();
        return;
      }

      // 상세 진입 AND 조건의 첫 번째 신호
      if (topic === "/robot2/artwork_detected") {
        const ok = parseBoolPayload(payload);
        console.log("📥 /robot2/artwork_detected =", ok);
        setArtworkDetectedFlag(!!ok);
        if (!ok) setDetected(false); // 둘 다 true 되기 전까지 '이동 중' 유지
        tryEnterDetail();
        return;
      }

      // 상세 진입 AND 조건의 두 번째 신호
      if (topic === "/robot2/arrived_painting") {
        const ok = parseBoolPayload(payload);
        console.log("📥 /robot2/arrived_painting =", ok);
        setArrivedPaintingFlag(!!ok);
        tryEnterDetail();
        return;
      }

      // 동시 조건 1/2: 사람 인식
      if (topic === "/robot2/person_detected") {
        const v = parseBoolPayload(payload);
        console.log("📥 /robot2/person_detected =", v);
        setPersonDetected(v);
        // 조건 충족 시 gohome 시도
        trySendGohome();
        return;
      }
    };

    client.on("message", onMessage);
    return () => client.off("message", onMessage);
  }, [tryEnterDetail, trySendGohome]);

  const handleModeSelect = (mode) => {
    if (mode === "맞춤 관람") setStep("list");
    else alert("전체 관람 모드는 현재 준비 중입니다.");
  };

  // 층 변경 플래그 발행
  const publishFloorChangedFlag = (changed) => {
    const buf = Uint8Array.from([changed ? 1 : 0]);
    client.publish("/floor_changed", buf);
    console.log("📤 /floor_changed →", changed ? "<0x01>" : "<0x00>");
  };

  const handleSelect = (art) => {
    // 새 사이클 초기화
    setDetected(false);
    setArrived(false);
    setGohomeSent(false);
    setArtworkDetectedFlag(false);
    setArrivedPaintingFlag(false);
    setPersonDetected(false);
    setRobot3ArrivedFlag(false);

    if (art.floor !== floor) {
      if (window.confirm(`${art.floor}층으로 이동할까요?`)) {
        publishFloorChangedFlag(true);
        setFloor(art.floor);

        const message = String(art.id);
        client.publish("/robot3/artwork_id", message);
        client.publish("/robot2/artwork_id", message);
        console.log("📤 /robot3/artwork_id →", message);
        console.log("📤 /robot2/artwork_id →", message);

        setSelectedArtwork(art);
        selectedRef.current = art;
        setStep("detail"); // 먼저 '이동 중' 화면
      }
    } else {
      const message = String(art.id);
      client.publish("/robot3/artwork_id", message);
      client.publish("/robot2/artwork_id", message);
      console.log("📤 /robot3/artwork_id →", message);
      console.log("📤 /robot2/artwork_id →", message);

      setSelectedArtwork(art);
      selectedRef.current = art;
      setStep("detail");
    }
  };

  const handleDone = () => setStep("more"); // 필요 시 "list" 등으로

  // "더 둘러보시겠어요?" 버튼 로직
  const handleMoreClick = useCallback(() => {
    const yes = window.confirm("더 둘러보시겠어요?\n예(계속 둘러보기) / 아니오(그만 둘러보기)");
    if (yes) {
     // 예 → keep_watching = true, 감지 플래그 초기화, 목록으로 이동
      const one = Uint8Array.from([1]);
      client.publish("/robot2/painting_keep_watching", one);
      console.log("📤 /robot2/painting_keep_watching → <0x01> (예)");

      setDetected(false);
      setArtworkDetectedFlag(false);
      setArrivedPaintingFlag(false);
      setStep("list");
    } else {
      // 아니오 → keep_watching = false (화면 유지)
      const zero = Uint8Array.from([0]);
      client.publish("/robot2/painting_keep_watching", zero);
      console.log("📤 /robot2/painting_keep_watching → <0x00> (아니오)");

      setDetected(false);
      setArtworkDetectedFlag(false);
      setArrivedPaintingFlag(false);
      setSelectedArtwork(null);
      setArrived(false);
      setGohomeSent(false);
      setPersonDetected(false);
      setRobot3ArrivedFlag(false);

      setStep("mode");
    }
  }, []);

  return (
    <AppContainer>
      <GlobalStyle />
      <Layout>
        <Title>ORBIT</Title>

        {step === "mode" && (
          <>
            <Subtitle>관람 방법을 선택해 주세요</Subtitle>
            <ModeSelectContainer>
              <Button onClick={() => handleModeSelect("전체 관람")}>전체 관람</Button>
              <Button onClick={() => handleModeSelect("맞춤 관람")}>맞춤 관람</Button>
            </ModeSelectContainer>
          </>
        )}

        {step === "list" && (
          <ArtworkList onSelect={handleSelect} floor={floor} />
        )}

        {step === "detail" && (
          <ArtworkDetail
            artwork={selectedArtwork}
            onDone={handleDone}
            detected={detected} // /robot2/artwork_detected && /robot2/arrived_painting 둘 다 true일 때 본문
            onMore={handleMoreClick}
          />
        )}
      </Layout>
    </AppContainer>
  );
}

export default App;
// (스타일은 기존 그대로)

const AppContainer = styled.div`
  width: 100vw;
  height: 100vh;
  background-color: #f8f8f8;
  display: flex;
  justify-content: center;
  align-items: flex-start;
  padding-top: 15vh;
`;

const Layout = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  width: 100%;
`;

const Title = styled.h1`
  font-size: 5rem;
  color: #222;
  margin-bottom: 8vh; 
  letter-spacing: 0.1rem; 
`;

const Subtitle = styled.h2`
  font-size: 2.5rem;
  margin-bottom: 6vh;
`;

const ModeSelectContainer = styled.div`
  display: flex;
  gap: 10rem;
  margin-top: 3vh; 

  @media (max-width: 768px) {
    flex-direction: column;
    align-items: center;
  }
`;

const Button = styled.button`
  font-size: 2rem;
  padding: 2rem 3.5rem;  
  border-radius: 16px;   
  border: none;
  background-color: #333;
  color: white;
  cursor: pointer;
  transition: background-color 0.3s;

  &:hover {
    background-color: #555;
  }

  &:active {
    background-color: #111;
  }
`;
