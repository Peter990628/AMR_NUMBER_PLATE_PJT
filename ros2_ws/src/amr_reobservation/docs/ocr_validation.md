# 번호판 OCR 오프라인 검증

검증일: 2026-11-18 · 브랜치: `feature/reobservation-recovery`

**이동 후 번호판 검출은 회복됐지만, 이번 EasyOCR 시험에서 전체 문자 일치는 0/5였다.**
글자 영역을 분리한 뒤에도 다섯 장 모두 `123가4567`을 `12374567`로 판독했다.
깨끗한 원본 번호판 텍스처에서도 같은 한 글자 오류가 발생했다.
따라서 현재 결과로 OCR 성공을 선언하거나, 재관측 거리를 더 줄이면 해결된다고 판단할 수 없다.

[비교 그림](../../../../local_artifacts/ocr_validation_2026-11-18/comparison.png)

![이동 전후와 OCR 결과](../../../../local_artifacts/ocr_validation_2026-11-18/comparison.png)

## 시험 대상과 판정 방법

- 기존 이동 실험 5개의 **이동 전·이동 후·복귀 사진 각 1장**, 총 15장.
  바깥쪽 시작 배치, 반대쪽 대각선, 같은 배치 재방문 3회다.
- 별도 대조 입력: 시뮬레이션 번호판 원본 텍스처 1장.
  런타임 진단으로 일반 한국어 문장 1장도 추가 확인했다.
- 정답은 `assets/license_plate/mount.json`의 `label`, **123가4567**.
  이 값은 추론이 끝난 뒤 채점에만 사용했다. 모델 입력·허용 문자·후보 선택에 쓰지 않았다.
- EasyOCR 1.7.2의 `korean_g2`, 언어 `ko,en`, CPU 2 threads, 기본 quantization,
  greedy decoding. GPU·ROS 발행·AMR 이동·Google API 호출은 수행하지 않았다.
- 1차: 저장된 번호판 검출 상자를 그대로 잘라 문자 인식.
- 추가 진단: 같은 crop 내부에서 CRAFT가 글자 영역을 찾고 인식.
  `canvas_size=1024`, `mag_ratio=1.0`, 그 외 기본값. 모든 검출 문자열을 좌→우 순서로 보존했다.
- 정규화는 Unicode NFC와 공백 제거만 적용. `7→가` 교정, 숫자 치환, 정답에 가까운 결과 선택은 없다.
- 형식 검사 `[0-9]{2,3}[가-힣][0-9]{4}`는 단순 진단 지표다. 모든 한국 번호판 규격을
  지원하는 검증기가 아니며, 형식 통과와 정답 일치는 별개다.

기존 프로젝트 `Alley_Park_Patrol/detect/plate_ocr_node.py`는 **Google Vision
document_text_detection**과 번호판 crop 여백 30px·8배 확대를 사용한다.
이번 로컬 OCR은 다른 엔진·입력 처리의 기준 시험이며, 그 원본 구현의 성능 검증이 아니다.

후속(2026-11-19): [Google Vision 비교](google_ocr_validation.md)를 준비하고 첫 요청을
보냈으나 서비스 접근 거부(HTTP 403)로 OCR 결과를 얻지 못했다. 정확도 비교는 아직 미완료다.
같은 날 [PaddleOCR 비교](paddle_ocr_validation.md)는 실행을 완료했고,
동일 이동 후 crop에서 4/5, 원본 텍스처에서 1/1 전체 문자열 일치를 확인했다.

## 결과

| 저장 사진 | 번호판 검출 | OCR 시도 | 전체 문자 일치 |
|---|---:|---:|---:|
| 이동 전 5장 | 0/5 | 0/5 | 파이프라인 출력 없음 |
| 이동 후 5장 | 5/5 | 5/5 | **0/5** |
| 복귀 후 5장 | 0/5 | 0/5 | 파이프라인 출력 없음 |

미검출 사진은 번호판 ROI가 없어서 OCR을 실행하지 않았다.
이는 사람이 직접 잘라 주거나 다른 검출기를 써도 판독할 수 없다는 뜻이 아니다.

| 이동 후 장면 | 상자 전체 입력 결과 | 글자 영역 분리 결과 | 분리 후 OCR 점수 | 일치 |
|---|---|---|---:|---|
| 시작점 바깥쪽 이동 | `[[12374567 \|` | `12374567` | 0.940 | 실패 |
| 반대쪽 후방 대각선 | `[[12374567 ]` | `12374567` | 0.877 | 실패 |
| 같은 배치 재방문 1 | `[[12374567` | `12374567` | 0.835 | 실패 |
| 같은 배치 재방문 2 | `[[12374567` | `12374567` | 0.873 | 실패 |
| 같은 배치 재방문 3 | `[[12374567 _` | `12374567` | 0.783 | 실패 |

표는 가독성을 위해 바깥 공백을 생략했다. 원문·좌표·점수·문자 오류율은 JSON에 보존했다.
글자 영역 분리 후에는 8글자 중 1글자 치환으로 문자 오류율(CER)이 각각 12.5%다.
번호판 전체 일치는 여전히 실패다. 숫자만 일치한 것을 판독 성공으로 세지 않는다.

| 추가 대조 | 출력 | 의미 |
|---|---|---|
| 깨끗한 원본 번호판 텍스처 | `12374567` | 카메라 각도·해상도·디버그 상자가 없어도 동일 오류 |
| 일반 한국어 문장 `한국어 인식 테스트` | `한국어 인식 테스트` | 한국어 모델 로딩·기본 실행이 작동함을 확인 |

**원인 해석:** 테두리의 괄호 오인은 글자 영역 분리로 줄었지만, `가→7` 혼동은 남았다.
이 번호판의 글자 모양·배치와 현재 OCR 모델의 조합이 영향을 주는 것으로 보인다.
일반 문장 한 건 통과만으로 모든 글꼴·한국어 인식이 정상이라고 단정하지 않는다.

**OCR 점수가 높아도 정답은 아닐 수 있다.** 이번에는 0.940인 결과도 한 글자가 틀렸다.
또한 다섯 저장 사진에서 같은 오답이 반복됐다. 실시간 연속 프레임 검증은 아니지만,
단순 반복 일치만으로 실제 번호가 맞다고 보장할 수 없음을 보여준다.

## 범위와 다음 작업

1. 우선 같은 저장 입력으로 **기존 Google Vision OCR** 또는 별도의 한국 번호판 인식기를
   비교해 문자 판독 병목을 확인한다. 엔진별 동일 crop 비교와 원래 파이프라인 재현을 구분한다.
2. 판독 가능한 방법을 정한 뒤 디버그 표시가 없는 **원본 카메라 영상**을 저장해
   이동 전후·연속 프레임의 판독률, 다른 번호로의 오인식, 처리 시간을 평가한다.
3. 그 결과로 재관측 종료 기준을 정한다. 현재 `usable` 조건이나 제어 노드는 변경하지 않았다.

한 차량·같은 합성 번호판만 사용했으며 3장은 같은 배치 재방문이다.
이는 독립 차량 5대의 성공률도, 수백 프레임 OCR의 성공률도 아니다.
이동 후 저장물은 `/debug/plate_observation`의 상자·상태 문구가 포함된 영상이다.
이를 원본 센서 영상으로 취급하지 않았고 초록 상자를 임의 제거해 원본으로 복원하지 않았다.

CPU 최대 RSS는 1차 약 626 MiB, CRAFT 추가 시험 약 1,078 MiB였다.
이동 후 crop 5장의 판독 시간은 1차 약 24~181ms, 추가 시험 약 136~285ms였다.
모델 초기화, 번호판 검출·ROS 전송 등은 제외되며 첫 추론 준비 시간이 포함된다.
실시간 전체 처리 성능을 나타내는 값이 아니다.

## 파일과 재현

- 도구: `tools/probe_plate_ocr.py`, `tools/plot_plate_ocr_validation.py`
- 채점·crop 검증: `python3 tools/test_probe_plate_ocr.py` — 5개 통과.
- [고정 입력 manifest](../../../../local_artifacts/ocr_validation_2026-11-18/samples.json)
- [상자 전체 입력 결과](../../../../local_artifacts/ocr_validation_2026-11-18/whole_box_01/report.json)
- [글자 영역 분리 결과](../../../../local_artifacts/ocr_validation_2026-11-18/text_regions_01/report.json)
- [일반 한국어 대조 결과](../../../../local_artifacts/ocr_validation_2026-11-18/language_control_01/report.json)

입력별 원본 이미지 SHA-256, 출처 report SHA-256, 원래 PlateObservation,
crop 좌표·이미지 SHA-256, 출력 문자열·점수, 모델 SHA-256을 기록했다.
별도 라이브러리는 `local_artifacts/ocr_validation_2026-11-18/python_deps`에 설치했고
전역 Python·ROS 환경을 변경하지 않았다. 이 폴더와 다운로드 모델·결과 이미지는 Git 제외 대상이다.

기존 환경: PyTorch 2.6.0+cu124, OpenCV 5.0.0, NumPy 1.24.4.
추가 패키지: EasyOCR 1.7.2, python-bidi 0.6.7, pyclipper 1.3.0.post6,
Shapely 2.0.7, ninja 1.11.1.4. 설치 시 `--no-deps --target`을 사용했다.

공식 모델 파일을 로컬 `models` 폴더에 준비한 뒤 프로젝트 루트에서:

```bash
PYTHONPATH=local_artifacts/ocr_validation_2026-11-18/python_deps \
OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
python3 tools/probe_plate_ocr.py \
  --manifest local_artifacts/ocr_validation_2026-11-18/samples.json \
  --models local_artifacts/ocr_validation_2026-11-18/models \
  --mode text_regions \
  --output local_artifacts/ocr_validation_2026-11-18/new_run
```

`--mode whole_box`는 1차 방법이다. 기존 결과 폴더를 덮어쓰지 않으며,
실행 중 자동 다운로드도 비활성화했다. 모델은 공식 배포의 MD5를 확인했다:

- `korean_g2.pth`: `befecf7b1ca2fffb5af814a51443682d`
- `craft_mlt_25k.pth`: `2f8227d2def4037cdb3b34389dcf9ec1`

참고: [EasyOCR 공식 코드·사용법](https://github.com/JaidedAI/EasyOCR),
[공식 모델 URL·체크섬 정의](https://github.com/JaidedAI/EasyOCR/blob/v1.7.2/easyocr/config.py).
