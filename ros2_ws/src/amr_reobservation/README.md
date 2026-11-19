# AMR 관측 LiDAR C++ 처리

**OCR로 종료하는 최소 재관측 흐름:** [후보 한 곳 이동·판독 통합 실행](docs/single_candidate_ocr_recovery.md).
현재 위치에서 OCR을 먼저 확인하고, 실패하면 새 CFV·LiDAR 후보 한 곳으로 이동해 다시 판독한다.
성공·실패는 JSON과 종료 코드로 반환하며 최종 위치에서 정지한다.

**재관측 후보 미리보기:** 대략적인 차량 위치 주변의 LiDAR 관측 윤곽에서 조준점을 만들고, 조준점마다 현재 위치 근처의 이동 후보 6개와 카메라 방향을 `odom`에서 계산한다. 아래의 새 흐름은 기존 측면 끝점 A/B 후보와 독립적이다.

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amr_reobservation
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch amr_reobservation recovery_preview.launch.py
```

Isaac Sim에서 저장한 **실패 배치**를 열고 Play한다. 기본 launch의 `mock:=true`는 상위 접근 프로그램을 대신하는 명시적인 테스트 입력이다. `config/mock_recovery_request.yaml`의 차량 힌트 `(3.5, 0.4)m`는 시작 시 AMR 기준으로 수동 지정한 대략적 위치이고, 원래 접근 pose는 시작 시 AMR pose로 대신한다. 첫 유효 스캔 시각 TF를 사용해 이 입력을 `odom`에 고정한다. 실제 차량 검출 결과나 저장된 복귀 배치의 정답 위치가 아니다. RViz에서 노란 차량 힌트가 대상 관측점 근처에 있는지 먼저 확인하고, 맞지 않으면 YAML을 수정한 후 다시 실행한다. 동일 입력의 반복 발행은 유효기간을 갱신하지 않으며 기본 300초(ROS 또는 wall 한도) 후 새 실험을 시작해야 한다.

새 입력 `/reobservation/request`는 `RecoveryRequest` 메시지이며, 대상 ID, 차량 관측 위치, 원래 접근 pose, 고정 좌표계의 ROS 시각, 실패 사유, 출처, 유효기간을 명시한다. `enabled=false` 요청은 표시를 취소한다. 차량 방향은 입력에 없으며 접근 yaw로 대체하지 않는다. 실제 상위 로직을 연결할 때는 `mock:=false`로 실행한다. 확인한 외부 코드의 `run_plate_recovery_for_target(target)`에서 초기 번호판 확인이 실패하는 부분이 연결 지점이며, 성공 후 로봇 자세를 전달하는 `/a_to_b`는 실패 요청 입력이 아니다. 웹캠 JSON의 wall `system_time`을 시뮬레이션 ROS stamp로 그대로 복사하지 말고, 추정 좌표의 기준과 시각을 명시적으로 맞춰야 한다.

차량 힌트는 대상 윤곽을 연결하는 기준이다. 유효 LiDAR 점을 원본 빔 순서와 점 사이 거리로 군집화하고, 힌트에 가장 가까운 충분한 크기의 군집을 선택한다. 비슷하게 가까운 군집이 둘 이상이면 `contour_ambiguous_target`으로 거부한다. 선택한 윤곽을 휘어짐에 따라 나눈 뒤, 충분히 긴 구간의 중앙 부근 **실제 관측점**을 최대 3개 조준점으로 사용한다. `A1/A2/A3`는 해당 스캔의 구간 순서일 뿐, 시간에 걸쳐 같은 부위를 추적하는 ID나 차량 전면/후면 분류가 아니다. 유효한 구간이 없으면 후보를 비우며 힌트를 조준점으로 대신 사용하지 않는다. USD 번호판 좌표는 계산에 사용하지 않는다.

이동 후보는 차량 힌트 방향을 기준으로 **제자리 재조준, 0.35m 접근, 좌/우 0.35m, 접근+좌/우**이다. 조준점을 바꿔도 이 6개의 BASE XY는 동일하다. 카메라의 실제 장착 위치와 optical +Z 방향을 스캔 시각 TF로 읽어 각 관측 조준점을 향하는 BASE yaw를 계산한다. 출력은 조준점이 N개이면 `6 × N`개이며 `candidate_aim_indices`로 조준점을 연결한다. 시선은 관측점의 XY를 카메라 높이에 투영한 수평 방향이며 번호판의 위치·높이나 새 위치에서의 가시성을 추정하지 않는다.

RViz는 노란 힌트, 청록색 관측 윤곽, `A1/A2/A3` 조준점과 현재 카메라에서 각 조준점을 향하는 선을 표시한다. 화면이 겹치지 않도록 이동 후보 화살표는 한 조준점의 6개만 표시한다. 다음 명령의 `0/1/2`는 **화면 표시만** 바꾸며 후보 선택·목표 확정·이동을 수행하지 않는다. 없는 조준점을 선택하면 후보 화살표를 지우고 안내를 표시한다.

```bash
ros2 param set /recovery_preview marker_aim_index 0  # A1
ros2 param set /recovery_preview marker_aim_index 1  # A2
ros2 param set /recovery_preview marker_aim_index 2  # A3
```

후보 pose는 평면 모델이다. BASE·스캔 기울기와 카메라 광축의 수평 이탈이 5°를 넘으면 거부하며, 그 이내의 작은 roll/pitch는 후보 yaw·카메라 높이 계산에서 근사한다. 이동 중 스캔 왜곡이나 후보 위치에서 차체에 가려지는 시선은 아직 평가하지 않는다.

윤곽 기본 설정은 점 간격 0.18m 이내, 누락 빔 최대 2개, 군집 최소 12점, 힌트 연관 거리 2m, 군집 선택 거리 차이 0.15m 이하면 모호함으로 처리한다. 구간 분할은 양 끝을 잇는 선에서 관측점이 0.06m를 초과해 벗어날 때 수행한다. 조준점 구간은 최소 12점과 끝점 간 거리 0.45m가 필요하다. 길이가 긴 구간을 최대 3개 남긴 뒤 원래 윤곽 순서로 표시한다. 이 값들은 관측 지지를 확인하기 위한 초기 설정이며 차량 치수나 번호판 정답에 맞춘 값이 아니다. `config/recovery_preview.yaml`에서 변경 후 재시작한다. 실행 중 변경 가능한 항목은 표시용 `marker_aim_index`다.

출력 `/reobservation/preview`는 `RecoveryPreview`, RViz 출력은 `/debug/reobservation/preview`의 `MarkerArray`다. 원래 접근 pose와 현재 pose, 차량 힌트, 카메라 시선과 각 후보의 제외 사유를 함께 표시한다. `valid=true`는 입력·시각·TF와 관측 조준점이 유효하다는 뜻이고 `eligible=true`/`preview_unchecked`도 제한된 기하 조건만 통과했다는 뜻이다. 설정된 관측점 거리 0.75m는 미리보기 제외 기준이며, 정확한 로봇 footprint나 경로·회전 중 충돌·미관측 공간의 안전성을 보증하지 않는다. 번호판 `usable` 상태는 참고 표시일 뿐 OCR 성공이나 후보의 성공 보장이 아니다. 이 노드는 Nav2 목표와 속도 명령을 발행하지 않는다.

입력이 오래됐거나 중단됐을 때, 요청이 만료됐을 때, 정확한 스캔 시각 TF가 없을 때는 빈 결과와 마커 삭제를 발행한다. `/clock`이 뒤로 가면 이전 요청과 후보를 버린다. 기본 mock은 새 스캔/TF로 새 테스트 요청을 만든다. 외부 요청은 새 요청 ID와 해당 실행 구간의 시각을 제공해야 한다. 기존 A/B 안정화 결과와 이번 후보의 유효 판정은 서로 다른 출력이다.

```bash
ros2 topic echo /reobservation/request --once
ros2 topic echo /reobservation/preview --once
# 합성 입력과 TF를 사용하는 격리 도메인 검증 (사용하지 않는 도메인 번호 선택)
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 python3 src/amr_reobservation/test/smoke_recovery_preview.py
```

## 관측 후보 선택과 제한된 시뮬레이터 이동 실험

프로젝트 루트의 `tools/run_recovery_trial.py`는 정지 상태의 관측으로 시험할 후보를 고정하고, 요청한 경우 Isaac Sim에서 순서대로 이동하여 번호판 관측 품질을 비교하는 실험 도구다. **기본 실행은 선택·기록만 수행한다. `--execute`를 붙였을 때만 실제 시뮬레이터에 속도 명령을 보낸다.** 일반 주행 서비스나 Nav2 목표 생성기는 아니다.

CFV 차량 축을 사용하는 제한 시험은 `--cfv-inference-report` 옵션으로 연결한다.
현재 Play에서 수집한 정지 영상 3장의 일관성과 시각·좌표·요청을 확인한 뒤,
새 LiDAR preview에서 확인된 앞뒤 면 후보 한 개를 선택한다.
[수집·추론·시험 명령과 11월 18일 결과](docs/cfv_orientation_test.md#현재-play-데이터로-재개-2026-11-18)를 참고한다.

시작 배치를 바꾼 뒤 새 2.0m 목표를 계산하는 시험은 `--cfv-fresh-close`를 함께 사용한다.
이전 성공 목표를 다시 방문하는 `--repeat-view-reference`와 달리, 새 CFV 축과 현재 확인된
LiDAR 표면에서 목표를 생성하고 10초 관측 후 이번 출발점으로 복귀한다.
두 모드는 함께 사용할 수 없다. 시작 배치 변경 절차와 고정 차량 힌트 처리 방법은
[실험 기록](docs/cfv_orientation_test.md#시작-위치각도-변경-후-새-목표-계산-11월-18일)에 설명했다.
수동 시작 배치 변경은 `--setup-local-offset FORWARD_M LEFT_M YAW_DEG`로 최대 0.5m / ±20°까지
지정하며 변경한 자리에서 정지한다. 이것은 재관측 후보 선택과 별도의 시험 준비 동작이다.

실패 배치를 열어 Play하고, 위의 `recovery_preview.launch.py`와 [번호판 관측 노드](docs/stage3_plate_observation.md)를 실행한다. `/clock`, `/chassis/odom`, 관측 LiDAR와 전후방 기본 LiDAR, `/plate/observation`, `/reobservation/request`, `/reobservation/preview`가 필요하다. 처음 실행할 때 기본 mock의 차량 힌트는 **원래 실패 위치의 AMR 자세에서 한 번** `odom`으로 고정된다. 다른 속도 제어기는 종료한 상태에서 실험한다.

새 ROS 터미널에서 다음과 같이 선택 결과만 먼저 기록할 수 있다. 출력 경로는 아직 없는 디렉터리를 사용한다.

```bash
cd ~/peter_pjt
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 tools/run_recovery_trial.py \
  --output local_artifacts/recovery_trial_select_01
```

다음 명령은 일반 이동 실험에서 후보를 최대 2개 실제로 시험한다. 각 후보 pose는 `odom` 기준 BASE 위치·yaw이며, 이동 중 새 후보로 바꾸지 않는다. 회전·전진·최종 방향 정렬 후 정지하여 관측한다. 영상 품질 조건을 만족하면 그 위치에서 정지한다. 정해진 후보를 모두 시험해도 품질 조건을 만족하지 못하면, 같은 입력·거리 검사를 통과하는 동안 초기 위치로 돌아가 관측한다. 입력 오류나 한도 초과로 중단되면 그 자리에서 정지하며 자동 복귀를 강행하지 않는다.

```bash
python3 tools/run_recovery_trial.py --execute --max-trials 2 \
  --output local_artifacts/recovery_trial_execute_01
```

서로 다른 조준점의 효과부터 비교하려면 회전 비교 모드를 사용한다. 다음 명령은 최소 2개의 확인된 조준점에 대한 **제자리 회전 후보만** 선택하여 두 방향을 모두 관측한다. 이동 후보를 제자리 회전으로 변환하는 기능이 아니라, 원래 BASE XY가 같은 실제 후보만 사용하는 방식이다.

```bash
python3 tools/run_recovery_trial.py --execute --rotation-only \
  --required-aims 2 --max-trials 2 \
  --output local_artifacts/reaim_comparison_01
```

회전 비교에서는 기준 위치나 첫 후보에서 `usable` 품질 조건을 만족해도 조기 종료하지 않는다. 선택한 방향들을 모두 비교한 뒤 입력·거리 검사가 유지되는 동안 **비교 시작 yaw로 돌아와 정지**하고 다시 관측한다. 선속도는 제어기와 guard 모두 0으로 제한하며, 시작점에서 실제 XY가 3cm를 넘게 벗어나면 중단한다. 보고서의 `quality_proxy_pass_labels`는 품질 조건을 통과한 후보 목록이다. 비교가 끝나도 `plate_recovery_verified=false`, `verification_status=requires_visual_or_ocr_review`로 남기며 실제 번호판인지 영상 또는 OCR로 별도 확인해야 한다.

선택은 `tools/recovery_selection.py`에서 수행한다. 시간에 따라 바뀌는 `A1/A2/A3` 이름을 추적 ID로 쓰지 않고, 구간의 방향·길이 방향 겹침·수직 거리를 비교한다. 한 구간이 여러 구간으로 나뉘거나 여러 구간이 합쳐져 연결이 모호하면 해당 관측의 연결을 거부한다. 정지 자세가 3cm·2° 이내에서 유지되고, 서로 다른 스캔 시각으로 최소 3회 연속 확인된 구간만 채택한다. 조준점은 고정된 첫 관측점에서 15cm 이내이고, 관측점끼리의 최대 거리도 20cm 이내여야 한다. 평균이나 계속 움직이는 기준값은 사용하지 않는다. 요청 변경, 시간 역행·공백, 무효 입력 또는 AMR 이동은 확인 횟수를 초기화한다.

새 후보 선택은 기본 `--required-aims 2`에 따라 서로 다른 확정 조준점 최소 2개를 요구한다. **이 조건을 만족하는 첫 신선한 한 메시지에서 즉시 큐를 고정**한다. 추가로 일정 시간을 기다리거나, 더 나중에 한 조준점만 남은 메시지로 바꾸지 않는다. 12초의 선택 한도 안에 확보하지 못하면 `insufficient_aim_diversity`로 중단하며 한 조준점의 여러 이동 위치로 대신 채우지 않는다. 이전 시각의 서로 다른 메시지에서 후보를 모아 합치지도 않는다.

큐는 해당 **한 메시지의 실제 pose와 조준점**을 복사한다. 확인된 조준점들의 제자리 회전을 먼저 배치하고, 서로 다른 조준점의 첫 후보를 넣은 다음 같은 조준점의 추가 이동 위치를 채운다. 이동 후보는 이동 거리가 짧고 방향 변화가 작은 순서를 우선한다. 후보 수는 최대 8개이고 `--max-trials`로 시험 개수를 제한하되 필요한 조준점 수보다 작게 지정할 수 없다. 길이·점 수는 관측 구간의 지지를 나타낼 뿐 번호판의 위치나 성공 확률을 뜻하지 않는다. 선택 후 관측 구간의 분할·연관이 일시적으로 모호해져도 목표를 바꾸지 않지만, 새 센서 입력과 요청·시간·TF 및 이동 거리 검사는 계속 필요하다.

비교는 `tools/recovery_trial_metrics.py`에서 원본 영상 stamp를 기준으로 수행한다. 초기 위치와 각 도착 위치에서 정지 안정화 이후의 새 영상만 센다. 유효한 서로 다른 영상 최소 12장, 원본 시각 범위 최소 1초가 모이면 관측 창이 완료된다. 전체 표본의 `usable` 비율이 80% 이상이고 연속 `usable`이 3장 이상이며 최신 입력도 유효할 때 이 실험의 복귀 조건을 만족한다. 같은 영상을 반복 받은 callback이나 3에서 포화되는 `consecutive_count`를 새 영상으로 세지 않는다. `input_timeout` 등의 무효화는 이미 센 표본에도 반영된다. 검출 비율, 점수 중앙값·하위 10% 값, 박스 크기, 경계 여백, 실패 상태별 횟수를 함께 기록한다. **이 판정은 기존 검출·영상 품질 신호이며 OCR 성공, 번호판 문자 일치 또는 목표 차량의 신원 확인은 아니다.**

확인용 영상도 측정에 포함된 결과와 **원본 stamp·frame·해상도가 정확히 같은 것만** 저장한다. 가까운 시각의 결과나 마지막 수신값을 붙이지 않는다. 무효화된 표본의 영상은 증거로 채택하지 않으며, 일치하는 영상이 하나도 없으면 `no_matched_measurement_image_*`로 중단한다. 각 관측의 `image_evidence`에는 이미지 파일명, 원본 stamp와 그 영상의 `PlateObservation` 메타데이터가 함께 기록된다. 검출 상자가 실제 번호판을 가리키는지와 `usable` 표기가 같은 영상에서 나온 것인지 확인할 때 사용한다.

실제 이동에는 다음 제한을 적용한다.

- 시작 전 `/cmd_vel` 발행자가 없고, 구독자가 Isaac의 `iw_hub_ROS_differential_drive_ros2_subscribe_twist`에 해당하는 노드 하나인지 확인한다. 실행 중에도 전용 guard가 유일한 발행자인지 확인한다. 다른 장치나 제어기가 연결되면 거부·중단한다.
- 실험 프로세스는 전용 내부 토픽에만 명령을 보낸다. 별도 `tools/recovery_command_guard.py`가 `/cmd_vel`을 발행하며, 내부 명령이 실제 시간 0.35초 동안 끊기면 0 속도를 보낸다. 잘못된 값이나 한도를 넘는 명령은 재시작할 때까지 정지 상태로 고정한다. 종료 시에도 정지 명령을 반복 전송한다.
- 일반 모드의 제어 속도 상한은 전진 0.07m/s, 회전 0.25rad/s이며 guard 상한은 각각 0.08m/s·0.30rad/s이다. 목표는 시작점에서 0.51m 이내, 실제 위치는 0.55m 이내로 제한한다. 회전 비교에서는 선속도 상한을 0, 회전 상한을 0.25rad/s로 정하고 시작점에서 실제 XY 변화는 3cm 이내로 제한한다. 세션 실제 시간은 기본 260초이며 `--max-seconds`로 30~280초 범위에서 지정한다.
- 관측·전방·후방 LiDAR를 각 스캔 시각의 TF로 `odom`에 변환하고, 현재 중심부터 목표 중심까지의 선분과 관측점 사이 거리를 1.2m 이상으로 검사한다. 기본 차체의 알려진 자체 반사 영역은 전후방 LiDAR에서 제외한다. 입력 중단·오래된 원본 시각·시계 정지나 역행·요청 변경이나 만료·TF 실패·진행 정체는 정지 사유다.

이 거리 검사는 현재 단일 차량 시뮬레이션에서의 보수적인 관측점 검사다. 미관측 공간, 높이가 다른 장애물, 동적 장애물 및 전체 경로의 안전성을 보증하지 않는다. 실제 로봇에서 사용하거나 일반 주행에 연결하려면 별도 검증과 경로 계획이 필요하다. USD의 차량·번호판 정답 좌표와 저장해 둔 성공 배치 위치는 선택·제어에 사용하지 않는다.

실험 결과는 지정 디렉터리의 `report.json`, 고정 후보 `selection.json`, 수신·명령·판정 기록 `events.jsonl`, 도착 후 디버그 영상과 실행한 경우 `guard.log`에 남긴다. `/local_artifacts/`는 Git에서 제외되어 있다. 중단 이유는 `report.json`의 `status: aborted`와 `error`에서 확인한다.

중단 후 AMR이 이미 회전·이동했다면 기본 mock을 다시 시작하여 힌트를 현재 BASE 기준으로 재설정하면 안 된다. 원래 실패 배치로 되돌려 새 실험을 하거나, **같은 시뮬레이션 실행 구간과 중단 위치를 유지한 채** 다음의 명시적인 고정 좌표 재시도 기능을 사용한다. 먼저 기존 preview launch를 종료하고 mock 없이 실행한다. 이때 다른 상위 요청 발행자도 없어야 한다.

```bash
# ROS 환경을 설정한 ros2_ws 터미널
ros2 launch amr_reobservation recovery_preview.launch.py mock:=false
```

다른 ROS 터미널에서 프로젝트 루트 기준으로 실행한다.

```bash
python3 tools/run_recovery_trial.py --execute --max-trials 2 \
  --retry-report local_artifacts/recovery_trial_execute_01/report.json \
  --output local_artifacts/recovery_trial_retry_01
```

`--retry-report`는 `aborted` 보고서만 받으며, 현재 pose가 기록된 최종 pose의 3cm·2° 이내인지와 현재 ROS 시각이 저장한 후보 시각보다 큰지 검사한다. 이 검사만으로 재시작 여부를 완전히 구별할 수는 없으므로 시뮬레이터를 Stop·재시작하거나 장면을 바꾼 뒤에는 재시도 보고서를 사용하지 않는다. 원래의 `odom` 차량 힌트·접근 정보를 보존한 **새 요청 ID와 새 유효기간**을 발행하므로 만료된 요청을 연장하는 기능이 아니다. 이전에 관측까지 완료한 후보는 제외하고 남은 고정 후보를 검사한다. 각 조준점 주변 15cm 안에 관측점이 최소 3개 있는 상태를 서로 다른 새 스캔 3회로 재확인해야 이동한다. 이 재확인은 새로운 번호판 검출이나 경로 안전 보증을 의미하지 않는다.

이전 보고서의 차량 힌트·접근 정보만 유지하고 **현재 관측에서 후보를 새로 선택**하려면 `--context-report`를 사용한다. 이 옵션도 `mock:=false`와 다른 요청 발행자가 없는 상태가 필요하며, 같은 시뮬레이션 실행 구간과 보고서에 기록된 최종 pose를 유지해야 한다. 완료된 보고서도 사용할 수 있다. 과거 큐나 확인 횟수는 재사용하지 않고 현재 pose를 새 실험의 기준으로 잡는다.

```bash
python3 tools/run_recovery_trial.py --execute --rotation-only \
  --required-aims 2 --max-trials 2 \
  --context-report local_artifacts/recovery_trial_execute_01/report.json \
  --output local_artifacts/reaim_comparison_fixed_context_01
```

`--retry-report`는 과거의 남은 후보를 재검사하는 일반 실험용이고, `--context-report`는 고정된 차량 정보만 넘겨 새 후보를 선택하는 용도다. 두 옵션을 함께 사용할 수 없으며 회전 비교 모드는 `--retry-report`와 함께 사용할 수 없다. 기본 mock을 재시작하여 현재 AMR 자세에서 차량 힌트를 다시 만드는 것과도 구분한다.

**관측 표면 각도에 따른 이동 비교:** `--translation-comparison`은 현재 스냅샷의 최대 18개 후보를 유지한 채 실제 이동 후보를 별도로 평가한다. 각 조준점의 관측 선분에 수직인 방향과 카메라 시선의 각도를 계산하고, 같은 조준점을 제자리에서 바라보는 경우보다 2° 이상 정면에 가까워지는 후보를 남긴다. 관측 선분은 차량 후면이나 번호판으로 분류된 결과가 아니다. 기존 후보의 위치·카메라 방향은 바꾸지 않는다.

서로 다른 조준점 두 개가 각각 3회 이상 확인되고 개선되는 이동 후보를 갖추면 즉시 선택한다. 각 조준점에서 가장 좋은 각도와 1° 이내인 후보 중 이동 거리가 짧은 것을 우선한다. 이후 표면을 더 정면에서 보는 후보 순으로 정한다. 대각선 이동으로 각도가 거의 나아지지 않는 경우 불필요한 이동을 줄이기 위한 기준이다. 아래 명령은 두 조준점을 비교하되 실제 이동은 최상위 후보 하나만 시험한다.

```bash
python3 tools/run_recovery_trial.py --execute --translation-comparison \
  --required-aims 2 --max-trials 1 --max-seconds 280 \
  --context-report local_artifacts/reaim_comparison_2026-11-13/execute_01/report.json \
  --output local_artifacts/translation_comparison_trial_01
```

앞서 설명한 `mock:=false`, 같은 시뮬레이션 실행 구간, 저장한 최종 pose 유지 조건이 적용된다. 출발 전 현재 시뮬레이션 시간 진행률과 제어기 모의 계산으로 이동·측정·복귀 시간을 추정한다. 복귀와 측정 여유가 부족하면 출발하지 않는다. 기존 속도·거리·센서·독립 정지 장치 제한도 그대로 적용된다.

이 비교 모드는 `usable`이 나와도 관측을 기록한 뒤 출발 위치와 방향으로 복귀한다. `quality_proxy_pass_labels`는 관측 품질 조건 통과 목록이며 번호판 검출 복구의 확정 결과가 아니다. 저장된 같은 시각의 영상에서 박스가 실제 번호판에 놓였는지 확인해야 한다. 출발 전 예측은 시간 확보를 위한 추정이며, 실행 중 입력 중단이나 장애물이 생기면 복귀를 강행하지 않고 정지한다.

2026-11-13 실패 장면의 첫 이동 비교에서는 `A2/left` 35cm 후보를 시험했다. 새 관측 35장 모두 `not_detected`였고, 실제 영상의 번호판은 더 좁게 보였다. 관측 표면의 계산상 각도 개선이 번호판 면의 관측 개선을 뜻하지 않음을 보여 주는 사례다. 이 순위는 실험용이며 후면 접근이나 복구 성공을 보장하는 정책으로 사용하지 않는다. 저장 영상의 오프라인 모델 점수는 오히려 상승했으므로, 번호판 크기나 각도 하나만을 실패 원인으로 단정하지 않는다.

**4단계 첫 검증:** [평행 배치의 관측 후보](docs/stage4_parallel_candidates.md). 측면 양 끝의 센서 관측 후보와 시선을 표시하고 실제 LiDAR 데이터에서 거리·방향을 검증한다. 자동 이동 목표는 아직 발행하지 않는다.

**4단계 회전 비교:** [고정 좌표계 기준 수집과 ±10° 비교](docs/stage4_rotation_check.md). 원본 stamp의 TF로 후보를 `odom`에 변환하고, 같은 실행 구간에서 회전 전후 평균과 편차를 비교한다. +10° 자세 → 평행 및 평행 → −10° 평균 비교는 통과했다. 원본 후보의 순간 점프는 별도로 기록했다.

**4단계 연속 관측 확인:** [후보의 시간 일관성 확인](docs/stage4_candidate_stability.md). 기본 launch는 `/observation_candidates/raw`를 검사하고, 3회 일관된 현재 후보만 `/observation_candidates`로 채택한다. 급변·무효·입력 중단 시 빈 후보와 마커 삭제를 발행한다. 원본 기하의 평균 처리나 자동 이동은 수행하지 않는다.

**3단계 실행:** [번호판 관측 품질 판정](docs/stage3_plate_observation.md). ONNX Runtime C++ 추론, 박스 크기·여백·연속 관측 판정, 영상 타임아웃 처리를 제공한다. 전체 패키지를 처음 빌드하기 전에 프로젝트 루트에서 `bash tools/setup_onnxruntime.sh`로 로컬 C++ 런타임을 준비한다.

**2단계 실행:** [차량 측면 후보 추정](docs/stage2_vehicle_side.md). 1단계 전처리 라이브러리를 재사용하여 클러스터링·RANSAC·PCA와 typed ROS 결과를 제공한다. 아래는 독립적으로 실행할 수 있는 1단계 안내다.

## 1단계: 전처리

`/observation_lidar/scan`을 받아 유효한 거리를 XY 점으로 바꾸고, 거리·각도 ROI를 적용한 결과를 `/debug/observation_points`에 표시한다. 현재 출력은 **필터링된 관측 점**이며, 차량 분류나 측면 직선 추정 결과가 아니다.

## 빌드와 실행

ROS 2 Humble 개발 터미널에서 실행한다. Isaac Sim의 Python/라이브러리 환경 대신 `/opt/ros/humble` 환경을 사용한다.

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amr_reobservation
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch amr_reobservation scan_preprocessor.launch.py rviz:=true
```

Isaac Sim에서 `amr_side_lidar_check.usd`를 열고 **Play**한다. AMR을 정지시킨 채 비교한다. 실행 파일은 이동 명령을 발행하지 않는다.

- RViz Fixed Frame: `observation_lidar`
- 흰색 작은 점: 원본 `/observation_lidar/scan` (Best Effort)
- 초록색 큰 점: 처리 결과 `/debug/observation_points` (Reliable)
- 두 표시의 점 위치가 겹쳐야 한다. 흰색 점이 가려지면 Displays에서 초록색 표시를 잠시 끈다.
- 종료: 실행 터미널에서 `Ctrl+C`.

RViz가 이미 열려 있다면 `rviz:=false`(기본값)로 실행하고, RViz에서 **Add → Marker**를 선택해 출력 토픽을 지정해도 된다. 이 경우 RViz에도 `use_sim_time=true`를 설정한다. 제공된 launch는 노드와 RViz 모두에 이를 설정한다.

```bash
ros2 topic info /observation_lidar/scan -v
ros2 topic echo /clock --once
ros2 topic info /debug/observation_points -v
ros2 param get /scan_preprocessor use_sim_time
```

`/clock`이 오지 않으면 Isaac Sim의 Clock 그래프와 Play 상태를 확인한다. 관측 센서·TF·Clock의 Stop 시 시간 초기화 정책을 맞춘다. 노드는 원본 스캔 시각을 그대로 전달하며, 임의로 현재 시각으로 교체하지 않는다.

## 처리 원리와 C++ 역할

```text
LaserScan → ROS 노드 → ScanPreprocessor → ROS Marker → RViz
                        일반 C++17 계산
```

- `include/amr_reobservation/scan_preprocessor.hpp`: 입력·출력 구조체와 클래스 선언.
- `src/scan_preprocessor.cpp`: 유효 거리 검사, 극좌표 변환, ROI 필터. ROS 의존성 없음.
- `src/scan_preprocessor_node.cpp`: ROS 메시지 수신, 파라미터 읽기, Marker 발행, 처리 개수 로그.
- `test/test_scan_preprocessor.cpp`: 센서 없이 실행하는 수학·경계 조건 테스트.

빔 `i`의 각도는 `angle_min + i * angle_increment`이다. 좌표는 `x = r cos(θ)`, `y = r sin(θ)`로 계산한다. `const &`로 거리 배열을 읽어 입력 전체를 복사하지 않고, `std::vector<ScanPoint>`에 선택한 점을 담는다. 각 점의 `beam_index`를 유지하므로 다음 단계의 클러스터링에서 제거된 빔 사이의 간격을 알 수 있다.

거리가 NaN/Inf이거나 센서의 `range_min/range_max` 밖이면 제거한다. 그 후 설정한 거리·각도 범위를 적용한다. 범위의 양 끝은 포함한다. 각도 경계에는 float32 입력의 반올림을 고려해 `1e-6 rad` 허용 오차를 둔다. `angle_max`는 독립적으로 반올림될 수 있어 반복 횟수는 실제 `ranges.size()`를 기준으로 한다.

음수 `angle_increment`도 처리한다. 0인 경우 빔 하나는 허용하지만 빔이 여러 개면 잘못된 metadata로 처리한다. 빈 스캔, 유효점 0개, 잘못된 metadata, 빈 frame_id를 받으면 동일 namespace/id의 Marker를 삭제하여 이전 결과를 새 관측처럼 남기지 않는다.

## 파라미터와 ROI 확인

설정 파일: `config/scan_preprocessor.yaml`. 아래 항목은 **시작 시 읽는 파라미터**이며 실행 중 변경은 거부한다. YAML을 수정하고 노드를 다시 실행하면 적용된다. `--symlink-install`로 빌드했다면 YAML 수정 후 재빌드는 필요하지 않다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `input_topic` | `/observation_lidar/scan` | LaserScan 입력 |
| `marker_topic` | `/debug/observation_points` | Marker 출력 |
| `range_min_m` | `0.1` | 허용할 최소 거리(m) |
| `range_max_m` | `6.0` | 허용할 최대 거리(m) |
| `roi_min_deg` | `-180.0` | 센서 기준 ROI 시작 각도 |
| `roi_max_deg` | `180.0` | 센서 기준 ROI 끝 각도 |
| `marker_size_m` | `0.035` | 표시 점의 폭·높이(m) |
| `marker_lifetime_sec` | `1.0` | 마지막 갱신 이후 Marker 수명(ROS 시간) |

거리 범위는 `0 <= min < max`, 각도는 `[-180, 180]`이어야 한다. 각도 시작과 끝이 같거나 `[180, -180]`이면 폭 0으로 거부한다. 전체 각도는 `[-180, 180]`으로 지정한다.

각도는 **LiDAR 좌표계** 기준이며 0°는 +X, +90°는 +Y이다. 카메라 방향이나 화면의 좌우만으로 ROI 부호를 정하지 않는다. 전체 각도로 점과 좌표축을 먼저 확인한 뒤 좁힌다. 예를 들어 +Y 쪽 관측은 다음처럼 설정할 수 있다.

```yaml
roi_min_deg: 30.0
roi_max_deg: 150.0
```

`150.0`부터 `-150.0`처럼 시작이 끝보다 크면 ±180° 경계를 통과하는 뒤쪽 구간이다. 별도 설정 파일은 다음처럼 지정한다.

```bash
ros2 launch amr_reobservation scan_preprocessor.launch.py \
  params_file:=/absolute/path/to/scan_preprocessor.yaml rviz:=true
```

점 Marker는 입력의 frame/stamp를 유지하고 `z=0`을 사용한다. 이것은 **LiDAR 스캔 평면의 높이**이며 월드 바닥 높이를 뜻하지 않는다. 다른 Fixed Frame을 사용하려면 해당 시각의 TF가 필요하다. 빈 frame_id를 받았을 때의 삭제 메시지는 마지막 유효 frame과 시각 0(최신 TF)을 사용해 RViz의 TF 필터를 통과하도록 한다. 마커 수명은 RViz의 ROS 시간을 따르므로 시뮬레이션 시간이 멈추면 마지막 표시도 유지될 수 있다.

## 검증

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select amr_reobservation --event-handlers console_direct+
colcon test-result --verbose
```

단위 테스트는 좌표·빔 번호, 잘못된 거리, 경계 포함 여부, 일반/전체/경계 통과 ROI, 음수 각도 간격, 빈 입력과 설정 오류를 검증한다. 별도로 실제 장면에서 원본 점과 처리 점의 일치, ROI 변경 결과를 확인한다.

현재는 정지 상태에서 한 스캔을 처리한다. 빔별 취득 시간에 따른 주행 중 왜곡 보정과 다른 좌표계 변환은 수행하지 않는다. 측면 추정은 별도 2단계 노드에서 이 전처리 라이브러리를 재사용한다.

메시지와 QoS 참고: [ROS 2 Humble LaserScan 정의](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg), [Marker 정의](https://github.com/ros2/common_interfaces/blob/humble/visualization_msgs/msg/Marker.msg), [rclcpp QoS 정의](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/qos.hpp).
