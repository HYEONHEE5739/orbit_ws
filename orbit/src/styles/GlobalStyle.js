// src/styles/GlobalStyle.js
import { createGlobalStyle } from 'styled-components';
import AppleFont from '../fonts/AppleSDGothicNeoR.ttf';

const GlobalStyle = createGlobalStyle`
  @font-face {
    font-family: 'AppleSDGothicNeoR';
    src: url(${AppleFont}) format('truetype');
    font-weight: normal;
    font-style: normal;
  }

  * {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
    font-family: 'AppleSDGothicNeoR', sans-serif;
  }

  body {
    background-color: #f8f8f8;
  }
`;

export default GlobalStyle;
