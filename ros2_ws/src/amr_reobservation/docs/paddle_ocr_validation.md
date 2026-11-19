# PaddleOCR 한국어 번호판 오프라인 검증

검증일: 2026-11-19 · 브랜치: `feature/reobservation-recovery`

**PaddleOCR는 이동 후 사진 5장 중 4장과 원본 번호판 텍스처 1장을 정확히 판독했다.**
바깥쪽 시작 배치 사진 1장에서는 `123가4567`을 `12374567`로 읽어 한 글자가 틀렸다.
동일 crop을 사용한 어제 EasyOCR의 전체 문자열 일치는 이동 후 0/5, 원본 텍스처 0/1이었다.

[비교 그림](../../../../local_artifacts/paddle_ocr_validation_2026-11-19/comparison.png)

![동일 입력의 EasyOCR와 PaddleOCR 결과](../../../../local_artifacts/paddle_ocr_validation_2026-11-19/comparison.png)

## 방법

- 모델: **`korean_PP-OCRv5_mobile_rec`**, 공식 공개 추론 가중치. 별도 학습 없음.
- CPU 실행, Paddle predictor 2 threads, MKL-DNN/HPI 비활성화, batch size 1.
- 기존 [EasyOCR 시험](ocr_validation.md)의 PNG crop 파일을 그대로 사용했다.
  원본 이미지와 crop의 SHA-256을 기준 보고서와 대조했다.
- 깨끗한 원본 텍스처 1장과 기존 이동 후 사진 5장을 모두 시험했다.
  5장은 바깥쪽 시작 배치, 반대쪽 후방 대각선, 동일 배치 재방문 3회다.
- `TextRecognition`에 번호판 전체 crop을 넣었다. 추가 글자 검출기·수동 자르기·
  기하 보정·정답 기반 후보 선택·한글 치환·허용 문자 제한을 적용하지 않았다.
  리사이즈·정규화·디코딩은 배포 모델의 기본 인식 처리에 맡겼다.
- 정답 `123가4567`은 추론 후 채점에만 사용했다.
  Unicode NFC 및 공백 제거 후 **전체 문자열 일치**를 판정했다.
- EasyOCR의 전체 crop 인식과 CRAFT 글자 영역 분리 결과를 모두 비교 그림에 표시했다.
  세 방법의 입력 PNG는 동일하지만 내부 글자 검출·리사이즈 과정은 다르다.
- 패키지는 프로젝트의 별도 가상환경에 설치했다. 모델을 내려받은 뒤 추론은 로컬에서 실행했다.
  ROS 노드·AMR 이동·기존 검출 임계값은 변경하지 않았다.

## 결과

| 입력 | PaddleOCR 원문 | 전체 일치 | 인식 점수 | 인식 시간 |
|---|---|---|---:|---:|
| 바깥쪽 시작 배치 | `12374567` | 실패 | 0.807 | 47.6ms |
| 반대쪽 대각선 | `123가4567` | 성공 | 0.927 | 47.8ms |
| 같은 배치 재방문 1 | `123가4567` | 성공 | 0.914 | 46.5ms |
| 같은 배치 재방문 2 | `123가4567` | 성공 | 0.893 | 46.7ms |
| 같은 배치 재방문 3 | `123가4567` | 성공 | 0.967 | 49.1ms |
| 원본 텍스처 — 별도 대조 | `123가4567` | 성공 | 0.991 | 86.7ms |

| 조건 | EasyOCR 전체 crop | EasyOCR + 글자 분리 | PaddleOCR 전체 crop |
|---|---:|---:|---:|
| 이동 후 저장 사진 | 0/5 | 0/5 | **4/5** |
| 원본 텍스처 | 0/1 | 0/1 | **1/1** |

인식 점수는 모델의 출력이며 정답일 확률로 보정된 값이 아니다.
엔진 간 점수도 동일 척도로 비교하지 않는다. 실패 사진도 0.807이므로 높은 점수만으로
성공을 확정할 수 없다. 이번 6장의 정답을 보고 성공 임계값을 맞추지 않았다.

모델 초기화는 약 0.79초, 프로세스 최대 RSS는 약 **515 MiB**였다.
텍스처를 첫 입력으로 실행했으며 인식 시간에는 파일 읽기·전처리·첫 추론 준비 등이
포함된다. 모델 초기화, 원래 번호판 검출, ROS 전송 및 이동 제어는 제외된다.
이를 실제 로봇의 전체 처리 지연이나 모델 간 정밀 속도 비교로 해석하지 않는다.
실행 프로세스의 가상 주소 공간 상한을 5GiB로 제한했다.

## 해석과 다음 검증

PaddleOCR는 현재 합성 번호판의 한글을 읽을 수 있고, 일부 이동 후 사진에서도
전체 판독이 가능하다는 근거를 얻었다. 어제 기본 EasyOCR에서 모든 입력에 나타났던
`가→7` 혼동이 이번에는 1장에 남았다. 실패 원인이 각도·해상도·테두리 중 무엇인지
이 실험만으로 분리하지 못했으므로, 더 가까이 이동하면 해결된다고 단정하지 않는다.

**4/5는 이 저장 사진 집합의 결과이며, 일반적인 한국 번호판 인식률 80%가 아니다.**
한 차량·같은 번호판만 사용했고 재방문 3장이 포함되어 있다.
이동 후 crop에는 초록색 검출 상자가 남아 있으며 원본 센서 영상이 아니다.
원본 텍스처는 별도 대조이며 이동 성공 횟수에 합산하지 않는다.

다음 단계는 모델·처리를 고정한 뒤, 디버그 표시 없는 원본 카메라 영상을 사용해
**이동 전후의 전체 문자 일치, 정지 후 연속 프레임의 유지 여부**를 비교하는 것이다.
검출 실패로 OCR을 수행하지 못한 경우와 OCR의 문자 오인식을 분리해 기록한다.
현재 단계에서는 실시간 OCR 연결이나 재관측 성공 조건을 변경하지 않았다.
실험 평가에 쓰는 정답 문자열을 로봇의 일반적인 성공 판단에 하드코딩하지 않는다.

## 파일 및 재현

- `tools/probe_paddle_plate_ocr.py`: 입력 검증, 로컬 인식, 문자열 채점, JSON 결과 저장.
- `tools/plot_paddle_plate_ocr_comparison.py`: 동일 crop 해시 확인 후 비교 그림 생성.
- `tools/test_probe_paddle_plate_ocr.py`: 고정 입력 선택 및 추론에 정답 미전달 검증.
- [실행 manifest](../../../../local_artifacts/paddle_ocr_validation_2026-11-19/whole_box_01/manifest.json)
- [상세 결과](../../../../local_artifacts/paddle_ocr_validation_2026-11-19/whole_box_01/report.json)
- [모델 다운로드 출처·해시](../../../../local_artifacts/paddle_ocr_validation_2026-11-19/model_download.json)
- [설치 패키지 목록](../../../../local_artifacts/paddle_ocr_validation_2026-11-19/requirements.freeze.txt)

주요 버전: PaddleOCR 3.3.2, PaddleX 3.3.13, PaddlePaddle CPU 3.2.2,
NumPy 1.26.4, OpenCV contrib 4.10.0.84. `pip check` 통과.

```bash
python3 -m unittest discover -s tools -p 'test_probe*ocr.py'

local_artifacts/paddle_ocr_validation_2026-11-19/venv/bin/python \
  tools/probe_paddle_plate_ocr.py \
  --baseline local_artifacts/ocr_validation_2026-11-18/text_regions_01/report.json \
  --model-dir local_artifacts/paddle_ocr_validation_2026-11-19/models/korean_PP-OCRv5_mobile_rec_infer \
  --output local_artifacts/paddle_ocr_validation_2026-11-19/whole_box_02
```

OCR 관련 테스트 총 14개 통과. 재실행은 새 출력 디렉터리를 사용하며 이전 결과를 덮어쓰지 않는다.
모델·가상환경·실험 결과는 기존 `/local_artifacts/` 규칙으로 Git 제외 대상이다.

공식 자료: [문자 인식 모델·사용법](https://www.paddleocr.ai/main/en/version3.x/module_usage/text_recognition.html),
[PP-OCRv5 다국어 모델](https://github.com/PaddlePaddle/PaddleOCR/blob/main/docs/version3.x/algorithm/PP-OCRv5/PP-OCRv5_multi_languages.en.md).

후속 실험: [번호판 원근 보정 비교](plate_rectification_validation.md).
크기·비율 조정, 재자르기, 원근 보정을 분리하여 같은 crop으로 비교했다.
