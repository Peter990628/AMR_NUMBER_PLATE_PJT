# 번호판 원근 보정과 PaddleOCR 비교 — 2026-11-19

저장된 카메라 crop 5장에서 기존 입력은 4/5, 전체 크기·비율 조정은 5/5,
재자르기 후 크기 조정은 5/5, 원근 보정(실패 시 원본 유지)은 5/5가
정답 `123가4567`과 완전히 일치했다. **이 표본에서는 원근 보정 고유의 추가 효과가 확인되지 않았다.**
단순한 크기·비율 조정만으로도 기존의 `가 → 7` 오인식이 해소되었다.

## 비교 조건

동일한 5개 이동 후 카메라 crop과 별도 원본 텍스처 1개를 사용했다.
기존 PaddleOCR 실험과 같은 `korean_PP-OCRv5_mobile_rec` 모델 파일 및 설정을 사용한다.
정답은 추론 후 평가에만 사용하며 모서리 선택, 글자 교정, 후보 선택에 쓰지 않는다.

| 조건 | 처리 | 카메라 완전 일치 |
|---|---|---:|
| original | 기존 crop 그대로 | 4/5 |
| resize_only | 전체 crop을 520×110으로 비등방 크기 조정 | 5/5 |
| recrop_resize | 추정 네 모서리의 축 정렬 바운딩 박스로 재자른 뒤 520×110 | 5/5 |
| perspective | 추정 네 모서리를 520×110 직사각형으로 원근 변환 | 5/5 |

재자르기/원근 보정은 모서리를 못 찾으면 **기존 crop 그대로** 반환한다.
카메라 5장 중 4장에 실제 변환이 적용되었고 `repeat_3_after_0`은 원본 유지였다.
원본 텍스처 대조군은 네 조건 모두 1/1 일치했다. 카메라 집계에 포함하지 않는다.

| 이미지 | 기존 OCR | 크기·비율 조정 | 재자르기 | 원근 보정 |
|---|---|---|---|---|
| lateral | 12374567 | 123가4567 | 123가4567 | 123가4567 |
| opposite | 123가4567 | 123가4567 | 123가4567 | 123가4567 |
| repeat_1 | 123가4567 | 123가4567 | 123가4567 | 123가4567 |
| repeat_2 | 123가4567 | 123가4567 | 123가4567 | 123가4567 |
| repeat_3 | 123가4567 | 123가4567 | 123가4567 (원본 유지) | 123가4567 (원본 유지) |

## 구현

- `tools/rectify_plate_crop.py`: Gaussian blur → Canny → close → 윤곽선 사각형 근사.
  면적, 폭, 변 길이, 볼록성, 가로세로 비율, crop 경계 및 색상 조건으로 필터링한다.
  남은 사각형 중 면적이 가장 큰 것을 선택한다. OCR 점수로 선택하지 않는다.
- `getPerspectiveTransform`으로 네 모서리 대응을 계산하고 `warpPerspective`로 보간한다.
- 출력 비율 520:110은 `assets/license_plate/mount.json`의 0.52m×0.11m 규격을 사용한 사전 정보다.
  이미지에서 실제 치수를 복원한 것이 아니다.
- `tools/probe_plate_rectification.py`: 입력을 먼저 고정한 manifest를 생성한 후 동일 OCR로 네 조건을 평가한다.
  입력·모델·코드 해시, 모서리, 변환 행렬, 처리 시간과 원시 OCR 결과를 기록한다.
- `tools/plot_plate_rectification.py`: 비교 그림 생성. 빨간 선과 파란 점은 설명용이며 OCR 입력에 넣지 않는다.

초기 구현은 OCR 실행 전에 영상의 기하 형태만 보고 정했다. 이후 합성 단위 테스트에서
초록 검출 박스의 안쪽 에지가 통과하는 결함을 발견하여, 색상 검사를 에지 주변 7×7 영역으로
넓혔다. 수정은 OCR 정확도 최적화가 아니라 오버레이 오선택 방지를 위한 것이다.
초기 manifest와 결과는 보존했고 `overlay_guard_v2`에 수정 후 실험을 별도로 저장했다.

## 결과 위치 및 재현

프로젝트 루트 기준(실험 산출물은 git 제외):

- 최종 입력: `local_artifacts/plate_rectification_2026-11-19/overlay_guard_v2/manifest.json`
- 최종 결과: `local_artifacts/plate_rectification_2026-11-19/overlay_guard_v2/run/report.json`
- 비교 그림: `local_artifacts/plate_rectification_2026-11-19/comparison.png`
- 기존 비교: [PaddleOCR 검증](paddle_ocr_validation.md)

```bash
# output은 아직 존재하지 않는 디렉터리로 지정한다.
local_artifacts/paddle_ocr_validation_2026-11-19/venv/bin/python tools/probe_plate_rectification.py prepare \
  --baseline local_artifacts/paddle_ocr_validation_2026-11-19/whole_box_01/report.json \
  --output local_artifacts/plate_rectification_new_run
local_artifacts/paddle_ocr_validation_2026-11-19/venv/bin/python tools/probe_plate_rectification.py run \
  --manifest local_artifacts/plate_rectification_new_run/manifest.json
python3 tools/plot_plate_rectification.py \
  --report local_artifacts/plate_rectification_new_run/run/report.json \
  --output local_artifacts/plate_rectification_new_run/comparison.png
local_artifacts/paddle_ocr_validation_2026-11-19/venv/bin/python -m unittest discover -s tools -p 'test_rectify_plate_crop.py'
```

## 해석과 제한

- 단일 합성 차량·동일 문자열의 소수 표본이다. 이 영상으로 기하 로직을 개발했으므로 별도 미관측 테스트셋이 아니다.
  5/5를 일반 인식 정확도나 이동 성공률로 표현하면 안 된다.
- resize_only는 단순 확대뿐 아니라 가로세로 비율도 변경한다. 확대 효과와 비율 효과는 분리하지 않았다.
- 모서리 추정은 픽셀 정답과 비교하지 않았다. 차량 프레임을 포함하거나 둥근 원본 테두리를
  사각형으로 근사해 불필요한 기울기를 만들 수 있다. 항상 정확한 정면 복원은 아니다.
- 기존 crop에는 초록 디버그 박스가 포함되어 있다. 실시간 적용 전에는 원본 센서 영상에서
  박스를 그리기 전 crop을 사용하고 별도 영상으로 검증해야 한다.
- OCR 점수는 정답 확률이 아니다. 정답 문자열 완전 일치로 평가했다.
- 가려지거나 crop 밖으로 잘린 글자를 원근 보정으로 복원할 수는 없다.
- 현재는 오프라인 비교 도구다. ROS 관측 노드나 이동 제어에 연결하지 않았다.

다음 이동 전후 비교에는 우선 동일한 크기·비율 조정 + PaddleOCR을 고정된 평가 절차로 쓰고,
원근 보정은 더 큰 사선 각도의 새 표본에서 추가 효과를 확인하는 것이 적절하다.

API 참고: [OpenCV 변환](https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html),
[윤곽선 처리](https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html).
