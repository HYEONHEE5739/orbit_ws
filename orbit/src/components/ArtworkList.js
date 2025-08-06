import React from "react";
import styled from "styled-components";
import artworks from "../data/artworks";
import GlobalStyle from "../styles/GlobalStyle";

function ArtworkList({ onSelect }) {
  return (
    <ListContainer>
        <GlobalStyle />
            <Title>작품을 선택해주세요</Title>
            <CardList>
                {artworks.map((art, index) => (
                <Card key={art.id} onClick={() => onSelect(art)}>
                    <Image src={require(`../images/picture${index + 1}.jpg`)} alt={art.name} />
                    <ArtTitle>{art.name}</ArtTitle>
                    <Author>{art.author}</Author>
                </Card>
                ))}
            </CardList>
        </ListContainer>
    );
}

export default ArtworkList;

const ListContainer = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  width: 100%;
`;

const Title = styled.h2`
  font-size: 2.5rem;
  margin-bottom: 9vh;
`;

const CardList = styled.ul`
  display: flex;
  flex-wrap: wrap;
  justify-content: center;
  gap: 3rem;
  list-style: none;
  max-width: calc((280px + 3rem) * 4); 
  margin-bottom: 9rem;
`;

const Card = styled.li`
  display: flex;
  flex-direction: column;
  align-items: center;
  width: 280px;
  cursor: pointer;
  background-color: #fff;
  border-radius: 10px;
  box-shadow: 0 6px 12px rgba(0,0,0,0.1);
  padding: 1rem;
  transition: transform 0.2s ease;

  &:hover {
    transform: scale(1.03);
  }
`;

const Image = styled.img`
  width: 200px;
  height: 250px;
  object-fit: cover;
  border-radius: 5px;
  margin-bottom: 1rem;
`;

const ArtTitle = styled.p`
  font-size: 1.2rem;
  text-align: center;
`;

const Author = styled.p`
  font-size: 1rem;
  color: #555;
  margin-top: 0.5rem;
`;
