from google.cloud import texttospeech
import json
import os
import tempfile
import subprocess

# artwork.json 경로 직접 지정
json_path = "/home/rokey/orbit/artwork.json"

# JSON 파일 로드
with open(json_path, "r", encoding="utf-8") as f:
    artwork_descriptions = json.load(f)

# 재생할 artwork ID
artwork_id = "art001"  # 필요시 다른 ID로 바꿔보세요

# 설명 가져오기
text = artwork_descriptions.get(artwork_id)
if not text:
    raise ValueError(f"'{artwork_id}'에 대한 설명이 없습니다.")

# Google TTS로 음성 재생
def play_tts(text, lang="ko-KR", voice="ko-KR-Neural2-A"):
    client = texttospeech.TextToSpeechClient()

    synthesis_input = texttospeech.SynthesisInput(text=text)
    voice_params = texttospeech.VoiceSelectionParams(
        language_code=lang,
        name=voice
    )
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.MP3
    )

    response = client.synthesize_speech(
        input=synthesis_input,
        voice=voice_params,
        audio_config=audio_config
    )

    with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as out:
        out.write(response.audio_content)
        temp_path = out.name

    subprocess.run(["mpg123", temp_path])
    os.remove(temp_path)

# 실행
print(f"🔊 '{artwork_id}' 설명 재생 중...")
play_tts(text)
