// ArtworkDetail.js
import React, { useEffect, useRef, useCallback } from "react";
import styled from "styled-components";

function ArtworkDetail({ artwork, detected, onMore }) {
  const spokeRef = useRef(false); // 중복 자동재생 방지
  const utterRef = useRef(null);

  const stopTTS = useCallback(() => {
    if ("speechSynthesis" in window) {
      window.speechSynthesis.cancel();
    }
  }, []);

  const speak = useCallback((text) => {
    if (!("speechSynthesis" in window)) {
      console.warn("Web Speech API 미지원 브라우저입니다.");
      return;
    }
    stopTTS();
    const u = new SpeechSynthesisUtterance(text);
    u.lang = "ko-KR";
    u.rate = 1.0;
    u.pitch = 1.0;
    u.volume = 1.0;
    utterRef.current = u;
    window.speechSynthesis.speak(u);
  }, [stopTTS]);

  // 상세가 처음 표시될 때 자동으로 한 번만 읽기
  useEffect(() => {
    if (!artwork || !detected) {
      // 화면 벗어나면 읽기 중지
      stopTTS();
      spokeRef.current = false;
      return;
    }
    if (!spokeRef.current) {
      const composed =
        `${artwork.name}. 작가 ${artwork.author}. ` +
        (artwork.description || "");
      speak(composed);
      spokeRef.current = true;
    }
  }, [artwork, detected, speak, stopTTS]);

  if (!artwork) return null;

  return (
    <Container>
      {!detected ? (
        <StatusText>로봇이 작품 위치로 이동하고 있어요</StatusText>
      ) : (
        <Content>
          <HeaderRow>
            <Title>{artwork.name}</Title>
            <AudioButtons>
              <IconButton
                title="다시 듣기"
                onClick={() => {
                  const composed =
                    `${artwork.name}. 작가 ${artwork.author}. ` +
                    (artwork.description || "");
                  speak(composed);
                }}
              >
                🔊
              </IconButton>
              <IconButton title="정지" onClick={stopTTS}>⏹</IconButton>
            </AudioButtons>
          </HeaderRow>

          <MainRow>
            <ImageWrap>
              <Image
                src={require(`../images/picture${artwork.id}.jpg`)}
                alt={artwork.name}
              />
            </ImageWrap>

            <InfoWrap>
              <Author>{artwork.author}</Author>
              <Description>{artwork.description}</Description>

              <MoreButton onClick={onMore}>더 둘러보시겠어요?</MoreButton>
            </InfoWrap>
          </MainRow>
        </Content>
      )}
    </Container>
  );
}

export default ArtworkDetail;

// ===== Styles =====
const Container = styled.div`
  width: 100%;
  padding: 2rem;
`;

const StatusText = styled.p`
  font-size: 1.8rem;
  margin-top: 4rem;
  color: #333;
  text-align: center;
`;

const Content = styled.div`
  max-width: 960px;
  margin: 0 auto;
`;

const HeaderRow = styled.div`
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 1rem;
`;

const AudioButtons = styled.div`
  display: flex;
  gap: 0.5rem;
`;

const IconButton = styled.button`
  font-size: 1.2rem;
  padding: 0.5rem 0.8rem;
  border: none;
  border-radius: 8px;
  background: #eee;
  cursor: pointer;
  &:hover { background: #ddd; }
`;

const MainRow = styled.div`
  display: flex;
  gap: 2rem;
  align-items: flex-start;
  justify-content: center;
  margin-top: 1rem;
`;

const ImageWrap = styled.div`
  flex: 0 0 300px;
`;

const Image = styled.img`
  width: 100%;
  height: auto;
  max-height: 520px;
  object-fit: cover;
  border-radius: 12px;
  box-shadow: 0 6px 12px rgba(0,0,0,0.2);
`;

const InfoWrap = styled.div`
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 1rem;
`;

const Title = styled.h2`
  font-size: 2rem;
  margin: 0.5rem 0 0.25rem 0;
`;

const Author = styled.p`
  font-size: 1.2rem;
  color: #666;
  margin: 0 0 0.5rem 0;
`;

const Description = styled.p`
  font-size: 1.05rem;
  line-height: 1.7;
  color: #444;
  white-space: pre-wrap;
`;

const MoreButton = styled.button`
  margin-top: 2rem;
  align-self: flex-start;
  font-size: 1.2rem;
  padding: 0.9rem 1.6rem;
  background-color: #444;
  color: white;
  border: none;
  border-radius: 10px;
  cursor: pointer;
  &:hover { background-color: #666; }
`;
