# 후보 한 곳을 사용하는 번호판 재관측

차량 근처에서 판독에 실패한 상태를 입력으로 받아, 현재 위치에서 OCR을 확인하고 필요하면
새 후보 한 곳으로 이동한 뒤 성공·실패를 반환하는 최소 통합 실행기다.
기존 실험과 달리 저장 자료의 사후 정답 비교가 아니라 **실행 중 OCR의 일관성**이 종료를 결정한다.
초기 차량 접근은 상위 로직의 역할이며 이 도구가 구현하지 않는다.

## 실행 흐름

1. 현재 위치에서 정지 안정화 후 4초·2Hz로 8개 이미지/검출 결과 쌍을 수집한다.
2. 고정 PaddleOCR로 읽는다. 이미 일관되게 판독되면 이동 없이 성공한다.
3. 실패하면 현재 정지 영상 3장으로 CFV 방향을 새로 추정하고, LiDAR에서 확인된 표면과
   결합해 45도·관측 거리 2.0m 후보 한 개를 계산한다. 저장된 성공 좌표를 재사용하지 않는다.
4. 기존 관측점 간격 검사·시간 예산·속도 제한을 통과하면 후보로 이동한다.
5. 도착 후 다시 정지하고 동일한 4초 수집/OCR을 수행한다.
6. 성공 또는 후보 소진 실패를 반환하고 **현재 위치에서 정지**한다.
   자동 복귀나 두 번째 후보 시도는 하지 않는다. 입력 오류는 `aborted`로 구분한다.

제어 프로세스는 OCR·CFV 자식 프로세스를 기다리는 동안에도 0 속도 명령, ROS 수신,
요청·센서·시간·정지 상태 검사를 계속 수행한다. 오래 걸리는 인식 계산이 제어 콜백을 막지 않는다.
별도 명령 guard의 0.35초 유효기간도 유지한다.

## 임시 성공 조건

다음 조건을 **관측 창의 마지막 3개 연속 표본**이 모두 만족해야 한다.

- 정확히 같은 원본 시각·frame·해상도의 이미지와 검출 결과를 사용한다.
- 숫자 2~3자리 + 한글 1자 + 숫자 4자리 형식의 동일한 문자열이다.
- 각 표본의 **OCR 점수**가 0.8 이상이다. 검출기의 `usable`이나 검출 점수 0.5는 요구하지 않는다.
- 서로 다른 원본 시각이며 인접 표본 간 원본 시각 차이가 1.5초 이하다.
- 인접 검출 상자의 겹침 비율(IoU)이 0.3 이상이다.
- 중간 미검출·입력 누락·오독·낮은 OCR 점수·무효화는 연속 횟수를 초기화한다.
  앞부분에서 3회 성공했어도 마지막에 실패했다면 성공으로 반환하지 않는다.

정답 문자열을 입력받지 않으며 문자 치환·정답 기반 재시도도 없다. 같은 오독이 반복되는 경우까지
배제한 것은 아니다. 위 값은 초기 실험 기준이며 모든 한국 번호판 형식이나 여러 차량을 지원하는
완성된 판독 정책이 아니다. 차량이 하나인 현재 장면에서 검증을 시작한다.

각 단계는 관측 창을 마친 다음 한 번의 OCR 작업으로 판정한다. 매 카메라 프레임마다 상시
OCR을 수행하거나 세 번째 일치 즉시 중간에 움직이는 구조는 아니다. 전처리는 기존 전체 상자
520×110 조정 방식이며 원근 보정은 사용하지 않는다.

## 실행

현재 PC의 로컬 CFV/PaddleOCR 모델·환경 경로를 기본값으로 사용한다.
다른 PC에서는 `--config paths.json`으로 경로를 지정한다. 재학습·다운로드·외부 OCR API는 없다.
Python 도구와 기존 설치 노드를 연결하므로 이번 추가 기능 때문에 colcon 재빌드는 필요하지 않다.

```bash
cd ~/peter_pjt
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 경로와 설정만 확인. ROS 노드나 이동을 실행하지 않는다.
/usr/bin/python3 tools/run_plate_recovery.py \
  --mock-params ros2_ws/src/amr_reobservation/config/mock_recovery_request.yaml \
  --output local_artifacts/single_recovery_01
```

저장된 실패 배치에서 차량 힌트가 맞는 것을 확인하고 Play를 유지한다.
기존 검출·preview·속도 제어 노드는 종료한다. 실행기는 충돌하는 발행자를 발견하면 중단하며
외부 노드를 강제로 종료하지 않는다. 다음 명령은 실제 시뮬레이션 이동을 허용한다.

```bash
/usr/bin/python3 tools/run_plate_recovery.py --execute \
  --mock-params ros2_ws/src/amr_reobservation/config/mock_recovery_request.yaml \
  --output local_artifacts/single_recovery_01
```

`--mock-params`는 명시적인 수동 차량 힌트다. 저장된 실패 배치와 다른 곳에 AMR을 놓았다면
기본 `(3.5, 0.4)m` 힌트를 그대로 사용하지 않는다. 힌트는 시작 자세에서 한 번 `odom`으로 고정된다.
출력 폴더에는 mock과 preview의 600초 유효기간을 설정한 실행용 복사본을 저장한다.

상위 로직에서 실제 `RecoveryRequest`를 주는 경우에는 `--mock-params`를 생략한다.
요청에는 `odom` 좌표의 차량 힌트, 접근 pose, 요청/목표 ID, 유효기간 등이 필요하다.
실행 중 요청이 변경되거나 비활성화·만료되면 중단한다. 실제 상위 시스템과 연결한 검증은 아직 없다.

결과는 표준 출력과 `<output>/result.json`에 저장한다.

| 상태 | 의미 | 프로세스 종료 코드 |
|---|---|---:|
| `succeeded_without_motion` | 현재 위치에서 OCR 일관성 확인 | 0 |
| `succeeded_after_motion` | 후보 한 곳 이동 후 OCR 일관성 확인 | 0 |
| `candidate_exhausted` | 이동 후에도 판독 조건 불충족 | 2 |
| `no_candidate` | 제한 시간 안에 후보를 확정하지 못함 | 2 |
| `aborted` | 입력·시간·요청·경로·프로세스 오류 또는 사용자 중단 | 1 |

성공 여부는 `success`, 읽은 문자열은 `text`, 시도 수는 `candidate_attempts`다.
성공하지 못하면 `text`는 `null`이다. `request`에 상위 요청의 식별 정보를 보존한다.
이 단계의 인터페이스는 **일회성 CLI와 JSON 결과**다. ROS action/service 및 상위 호출 어댑터는
아직 추가하지 않았다. 실행기가 시작한 검출·preview·제어 프로세스는 종료 시 정리한다.
Ctrl+C로 중단할 수 있으며, 자동으로 출발점까지 복귀시키지 않는다.

상세 상태 전이는 `controller/events.jsonl`, 단계별 원본과 OCR 출력은
`controller/before_capture/`, `before_ocr/`, `after_capture/`, `after_ocr/`에 남긴다.
모든 실행 자료는 Git 제외 경로 `local_artifacts/`에 저장한다.

## 구현 검증 — 2026-11-19

- 순수 정책·흐름·자식 작업 중단 시험: 13개 통과.
- 기존 정확한 이미지/결과 연결 및 평가 시험: 12개 통과.
- 기존 이동 수학·예산·관측 창 및 CFV 선택 회귀 시험: 71개 통과.
- 저장된 이동 전후 20장씩을 새 정답 없는 OCR 작업에 다시 입력했다.
  이동 전은 `not_detected`, 이동 후는 `consistent_ocr`, 문자열 `123가4567`로 판정됐다.
  이동 후에는 마지막 20개가 모두 일치했다. **저장 자료 재생이며 새 이동 실험은 아니다.**
- 입력 없는 별도 ROS 도메인 124에서 약 21초 후 `readiness_clock_stalled`로 중단됐다.
  후보 시도는 0회이고 이동 guard는 시작하지 않았다. 실행기가 시작한 preview·검출 프로세스도 종료됐다.

실제 OCR 재생 결과와 입력 없음 시험 기록:
`local_artifacts/single_recovery_implementation_2026-11-19/`.

새 통합 명령으로 실제 Play 배치에서 이동한 검증은 아직 수행하지 않았다.
현재 이동 제어는 기존 단일 차량 Isaac Sim용이다. 일반 경로 계획·미관측 공간·다중 차량의
목표 신원 연계는 이 최소 구현의 범위에 포함되지 않는다.

```bash
/usr/bin/python3 -m unittest discover -s tools -p 'test_recovery_ocr_policy.py'
/usr/bin/python3 -m unittest discover -s tools -p 'test_plate_evaluation.py'
```
