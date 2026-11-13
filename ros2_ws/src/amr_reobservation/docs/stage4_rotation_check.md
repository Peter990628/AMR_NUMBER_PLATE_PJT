# 4단계 두 번째 검증: ±10° 회전과 고정 좌표계 비교

2026-11-12 추가한 [연속 관측 확인](stage4_candidate_stability.md) 이후 `/observation_candidates`는 확인된 출력이고 원본은 `/observation_candidates/raw`다. 아래 기존 실측 수치는 확인 단계 도입 전 원본 기록이다. 수집기의 기본 토픽은 `/observation_candidates`이므로 이후 캡처는 확인 결과를 검사한다. 기존 결과와 새 결과의 유효 비율 및 순간 변동을 구분한다.

차량과 AMR을 평행하게 둔 상태를 기준으로 기록하고, AMR을 제자리에서 +10°, −10° 회전했을 때 두 관측 후보가 차량 기준으로 얼마나 변하는지 측정한다. 도구는 ROS 메시지를 구독하고 로컬 결과 파일을 저장한다. 속도 명령, 주행 목표, TF, 시뮬레이터 상태를 발행하지 않는다.

후보는 여전히 **센서의 관측 위치와 시선 후보**다. `odom`으로 변환했다고 해서 `base_link`의 주행 목표가 되는 것은 아니다. 후면 선택, 카메라 장착 위치를 반영한 주행 목표, 차량과의 충돌 여유, 이동 경로는 이 시험에서 검증하지 않는다.

## 같은 실행 구간에서 비교해야 하는 이유

현재 사용하는 `odom`은 시뮬레이터 시작 자세를 기준으로 하는 좌표계다. Isaac Sim 5.1.0의 `Isaac Compute Odometry`는 첫 프레임의 자세를 저장하고 이후 자세를 그 시작 자세에 상대적으로 계산한다. 리셋되면 시작 자세를 다시 초기화한다. [NVIDIA의 v5.1.0 구현 소스](https://raw.githubusercontent.com/isaac-sim/IsaacSim/v5.1.0/source/extensions/isaacsim.core.nodes/nodes/OgnIsaacComputeOdometry.cpp)

따라서 기준 수집부터 두 회전 시험이 끝날 때까지 **Stop을 누르지 않고 같은 Play 구간을 유지**한다. Stop 후 다시 Play하면 원점이 바뀔 수 있어 이전 기준과 비교할 수 없다. Pause 자체는 Stop에 의한 원점 초기화와 다르지만, 수집 중에는 메시지 수신이 끊기므로 Pause하지 않는다.

세션 감시 도구도 시험 전체에서 계속 실행해야 한다. 감시 도구가 재시작되면 새 세션 ID가 생성된다. 시각 역행, `/clock` 또는 odometry 발행자의 변경·소실 등 실행 연속성이 깨진 경우에는 세션 epoch를 바꾸어 이전 기준과의 비교를 거부한다. 이런 경우 평행 상태부터 새로 수집한다.

## 비교에 사용하는 데이터

- `/observation_candidates`: 후보의 원본 stamp와 센서 좌표계.
- `odom → base_link → observation_lidar`: 후보 stamp 시점의 TF.
- `/chassis/odom`: 같은 stamp의 TF와 내부 일관성을 비교하는 보조 입력.
- `/clock`: 실행 구간과 시각 역행 감시.

수집기는 각 후보의 **정확한 원본 stamp**로 TF를 조회한다. 그 시점의 TF가 없으면 해당 샘플을 제외하며, 최신 TF로 대신 계산하지 않는다. `base_link`에서 센서까지의 회전과 위치 오프셋을 모두 적용하므로, 센서가 로봇 중심에 있다고 가정하지 않는다.

후보 A/B 라벨은 차량 앞뒤를 의미하지 않는다. 수집 중 및 두 결과 사이에서 위치를 기준으로 두 후보를 대응시켜 단순 라벨 교환이 위치 점프로 집계되는 것을 막는다.

RViz에서도 고정 좌표계로 보려면 `Global Options → Fixed Frame`을 `odom`으로 변경한다. RViz의 확대·화면 이동은 좌표 측정 결과에 영향을 주지 않는다. 정량 비교는 저장된 JSON으로 수행한다.

`/chassis/odom`과 TF가 일치하는 것은 **출력 간 내부 일관성**을 확인한 것이다. 두 출력이 같은 계산 결과에서 만들어질 수 있으므로, 이것만으로 시뮬레이터의 실제 로봇 이동을 독립적으로 검증했다고 판단하지 않는다. 실제 회전 시험에서 로봇 yaw가 요청한 만큼 변했는지도 반드시 확인한다.

## 준비

Isaac Sim에서 차량과 AMR의 평행 배치를 유지하고 Play한다. 측면 추정 및 후보 노드는 기존 실행을 유지한다. 실행 중인 동일 노드를 추가로 띄우지 않는다. 후보 파이프라인이 종료되어 있다면 [평행 배치 검증 안내](stage4_parallel_candidates.md)의 launch 명령으로 실행한다.

아래 공통 환경 설정은 새 터미널마다 적용한다. 명령의 작업 디렉터리는 프로젝트 루트다.

```bash
cd ~/peter_pjt
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/amr_rotation_check_ros_logs
```

결과 JSON과 그림은 모두 Git에서 제외되는 `local_artifacts/rotation_check_2026-11-12/` 아래에 저장한다. 도구는 기존 캡처 파일을 기본적으로 덮어쓰지 않는다. 재시험할 때에는 다른 파일명을 사용하고, 비교 명령에서도 그 파일명을 사용한다.

## 1. 세션 감시를 계속 실행

터미널 A에서 실행하고, 기준·+10°·−10° 수집이 모두 끝날 때까지 종료하지 않는다.

```bash
python3 tools/capture_candidate_rotation.py watch \
  --session-file local_artifacts/rotation_check_2026-11-12/session.json
```

`Session changed`가 나오면 이전 기준을 계속 사용하지 않는다. 원인을 확인한 뒤 평행 상태로 돌아가 새 기준을 수집한다. 감시 도구가 살아 있어도 `/clock`이 끊겨 있으면 수집기는 유효한 수집으로 인정하지 않는다.

## 2. 평행 상태를 15초 수집

터미널 B에서 공통 환경을 적용한 뒤 실행한다. 수집하는 동안 AMR과 차량을 움직이지 않는다.

```bash
python3 tools/capture_candidate_rotation.py capture \
  --session-file local_artifacts/rotation_check_2026-11-12/session.json \
  --output local_artifacts/rotation_check_2026-11-12/baseline.json \
  --label parallel --duration 15
```

출력의 `usable: true`를 확인한다. 유효한 원본 stamp 샘플이 20개 미만이거나 TF·세션·발행자·정지 조건에 문제가 있으면 `usable: false`와 `errors`가 기록된다. 이 상태에서는 회전 비교를 진행하지 않고 원인을 먼저 해결한다.

기준의 `summary.robot_yaw_rad`, `robot_position_xyz`, 두 후보 평균과 최대 편차를 함께 기록한다. `robot_yaw_rad`의 단위는 radian이다.

## 3. 기준에서 +10° 회전

1. 차량은 그대로 두고 AMR만 Z축으로 +10° 회전한다.
2. AMR의 X·Y 위치와 높이, roll·pitch는 유지한다.
3. 회전이 끝난 뒤 로봇이 정지한 상태에서 수집한다. 수집 중에는 회전시키지 않는다.

편집기에서 외형이 돌아간 것만으로 TF가 반영됐다고 판단하지 않는다. 다른 터미널에서 다음 명령으로 `odom → base_link`의 자세를 살펴볼 수 있다.

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

회전 전후 RPY의 yaw가 약 +10° 변하고 위치가 유지되는지 확인한다. 이 명령의 최신 TF 출력은 빠른 상태 확인용이며, 실제 비교에는 수집기가 조회한 원본 stamp의 TF를 사용한다. 외형만 회전하고 TF yaw가 변하지 않으면 유효한 회전 시험으로 판단하지 않는다.

```bash
python3 tools/capture_candidate_rotation.py capture \
  --session-file local_artifacts/rotation_check_2026-11-12/session.json \
  --output local_artifacts/rotation_check_2026-11-12/plus10.json \
  --label plus10 --duration 15

python3 tools/compare_candidate_rotations.py \
  local_artifacts/rotation_check_2026-11-12/baseline.json \
  local_artifacts/rotation_check_2026-11-12/plus10.json \
  --expected-yaw-deg 10 \
  --output local_artifacts/rotation_check_2026-11-12/compare_plus10.json \
  --plot local_artifacts/rotation_check_2026-11-12/compare_plus10.png
```

## 4. 기준에서 −10° 회전

평행 기준 방향에서 −10°가 되도록 회전한다. 현재 +10° 상태라면 **반대 방향으로 20°** 돌린 상태가 −10°다. X·Y 위치는 계속 유지한다. Stop 없이 같은 실행 구간과 세션 감시를 유지한다.

```bash
python3 tools/capture_candidate_rotation.py capture \
  --session-file local_artifacts/rotation_check_2026-11-12/session.json \
  --output local_artifacts/rotation_check_2026-11-12/minus10.json \
  --label minus10 --duration 15

python3 tools/compare_candidate_rotations.py \
  local_artifacts/rotation_check_2026-11-12/baseline.json \
  local_artifacts/rotation_check_2026-11-12/minus10.json \
  --expected-yaw-deg -10 \
  --output local_artifacts/rotation_check_2026-11-12/compare_minus10.json \
  --plot local_artifacts/rotation_check_2026-11-12/compare_minus10.png
```

두 비교 모두 **평행 기준**과 비교한다. `plus10.json`과 `minus10.json`을 직접 비교한다면 기대 회전량은 −20°가 되므로 위 명령의 의미와 다르다.

## 결과 판정

현재 기준은 초기 검증용이며 센서 성능 보증값이 아니다.

| 항목 | 초기 통과 기준 |
|---|---|
| 캡처 내부 로봇 정지 상태 | 평균 대비 최대 위치 편차 0.02m 이하, yaw 편차 1° 이하 |
| 실제 로봇 yaw 변화 | 요청한 +10° 또는 −10°에서 오차 3° 이하 |
| 기준 대비 로봇 위치 변화 | 0.05m 이하 |
| 대응된 후보의 평균 위치 변화 | 후보별 0.10m 이하 |
| 대응된 후보의 평균 시선 방향 변화 | 후보별 3° 이하 |

위치 변화는 도구에서 XYZ의 3차원 거리로 검사한다. 따라서 X·Y 이동뿐 아니라 높이 변화도 제한하는 기준이다. 센서 장착 오프셋이 1mm 또는 0.1°를 넘게 바뀌면 같은 조건의 비교로 인정하지 않는다.

- `valid: false`: 세션·epoch·좌표계·시간 구간·파라미터·장착 오프셋 불일치 또는 데이터 형식 문제로 비교할 수 없다.
- `valid: true, passed: false`: 비교 가능한 데이터지만 로봇 회전량·이동량·정지 상태 또는 후보 변화가 기준을 벗어났다. `checks`와 `candidate_comparisons`에서 원인을 확인한다.
- `valid: true, passed: true`: 두 수집 구간의 평균 비교가 설정 기준을 통과했다.

실제 로봇 yaw 변화도 검사하므로, odometry가 초기화되어 모든 자세가 다시 0으로 나오는 상황을 “후보가 안정적”이라고 통과시키지 않는다. 스크린샷 하나나 후보 평균만으로 판정하지 않는다.

후보의 최대 위치·방향 편차는 기준과 회전 캡처 각각 따로 기록한다. 한 프레임의 큰 편차와 두 캡처의 평균 위치 변화를 구분한다. 그림의 원은 수집된 최대 위치 편차이며 **차량 형상, 충돌 여유, 통계적 신뢰구간이 아니다.** 그림 생성에만 `matplotlib`이 필요하며, 수치 비교 자체는 Python 표준 라이브러리를 사용한다.

비교 도구의 종료 코드는 통과 `0`, 유효한 비교의 조건 실패 `1`, 비교 불가 `2`다. 실제 회전에서 후보가 크게 움직이면 TF 반영 여부와 관측 선분의 끝점 변화부터 구분해 확인한다. 단순히 허용 오차를 늘려 통과시키지 않는다.

## 자동 검증

```bash
cd ~/peter_pjt
python3 tools/test_candidate_frame_math.py
python3 tools/test_compare_candidate_rotations.py
```

수학 테스트 11개는 알려진 변환, 회전·이동의 역변환, 로봇 중심에서 벗어난 센서 오프셋, 라벨 교환, 각도 경계, 잘못된 입력을 검사한다. 비교기 테스트 15개는 후보 평균 변화, 기대 회전량, 정지 상태, 세션·시각 초기화 거부 및 CLI를 검사한다.

ROS 수집기 smoke는 실제 시뮬레이터의 도메인 0과 분리된 도메인 88에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_LOG_DIR=/tmp/amr_rotation_smoke_ros_logs \
  python3 tools/smoke_capture_candidate_rotation.py
```

자동 테스트는 도구의 계산과 입력 처리 검증이다. Isaac Sim에서 실제로 ±10°를 회전한 결과를 대신하지 않는다.

## 실제 시험 진행 상태

2026-11-12 새 감시 세션에서 **+10° 자세 → 평행 복귀 및 평행 → −10° 평균 비교를 통과**했다. −10° 기록에서는 후보 A가 한 프레임에서 크게 변하는 현상도 확인했다. 정지 자세 간 평균 기하의 일관성과 매 프레임의 후보 안정성을 구분해 판정한다.

### 최초 평행 기준 (이후 비교에는 사용하지 않음)

- 15초 동안 유효 후보 49개를 `odom`에 변환했다. 원본 시각의 TF 누락이나 오래된 입력으로 제외된 유효 후보는 없었다.
- `invalid_side:no_valid_line` 3회는 무효 후보로 수신되어 평균 계산에서 제외됐다. 전체 스트림이 항상 유효하다는 뜻은 아니다.
- 동일 시각의 odometry/TF 쌍 314개가 일치했다. 최대 위치 차이는 0m, yaw 차이는 수치 반올림 수준이다. 실제 회전 반영 여부는 다음 시험에서 확인한다.
- 정지 로봇의 평균 대비 최대 위치 편차는 약 0.092mm, yaw 편차는 0.00049°였다.
- 장착 변환 `base_link → observation_lidar`는 약 (0.255041, 0.008163, 0.720000)m, 회전은 항등이었다.

| 후보 | 평균 X / Y (`odom`, m) | 평균 yaw | 평균 대비 최대 3D 위치 편차 |
|---|---|---|---|
| A | −2.716 / −1.095 | −50.82° | 3.07cm |
| B | 2.445 / −1.048 | −128.14° | 11.98cm |

기준 후보 B의 순간 편차가 약 12cm이므로 이후 시험은 평균의 이동량과 각 구간의 변동을 함께 검토한다. 10cm 기준은 두 구간의 **평균 위치 차이**에 적용하는 초기 기준이며 순간 편차 한계와 다르다. 후보는 측정된 끝점에 의존하므로, 회전으로 관측 영역이 달라지면 TF가 정확해도 후보가 움직일 수 있다.

로컬 결과는 `local_artifacts/rotation_check_2026-11-12/baseline.json`, `baseline_odom.png`에 저장했다. 세션 감시가 재시작되거나 epoch가 변하면 이 기준을 재사용하지 않는다. 수학·비교기 테스트 26개와 별도 도메인 ROS smoke를 통과했으며, smoke는 시각 초기화 후 시간이 다시 기준 시각을 넘어가도 서로 다른 epoch의 비교를 거부함을 확인했다.

### 세션 복구 후 +10° 자세 → 평행 복귀

사용자가 +10° 회전 후 감시 터미널을 닫아 최초 평행 기준과의 실행 연속성을 확인할 수 없게 됐다. 새 감시는 터미널과 독립된 백그라운드 프로세스로 실행하고, 현재 회전 자세부터 새로 수집했다. 이전 세션의 기준을 새 비교에 섞지 않았다.

두 자세 모두 15초씩 수집했다. 회전 자세에서 유효 샘플 38개, 평행 복귀에서 43개를 확보했고, 동일한 세션 ID·epoch·후보 설정·장착 변환을 확인했다. 회전 자세의 유효 입력 39개 중 1개는 원본 시각의 TF가 없어 제외했다. 측면 무효 입력은 각각 6/45회, 4/47회였다.

| 검사 항목 | 실측 결과 | 초기 기준 | 판정 |
|---|---|---|---|
| 로봇 yaw 변화 | −10.041° (+9.835° → −0.206°) | −10°에서 오차 3° 이하 | 통과 |
| 로봇 위치 변화 | 1.69mm | 5cm 이하 | 통과 |
| 후보 A 평균 위치 변화 | 2.00mm | 10cm 이하 | 통과 |
| 후보 B 평균 위치 변화 | 2.65cm | 10cm 이하 | 통과 |
| 두 후보 평균 시선 변화 | 각각 약 −0.018° | 3° 이하 | 통과 |

두 캡처의 정지 조건도 통과했다. A의 평균 대비 최대 위치 편차는 회전/복귀 각각 4.80/3.17cm, B는 14.71/11.54cm였다. 이번 결과는 **두 정지 자세에서 고정 좌표계 후보 평균이 유지됨**을 확인한다. 연속 회전 중의 성능, 순간 후보 안정성, 차량 정답 좌표 대비 정확도나 충돌 여유를 검증한 것은 아니다.

결과 디렉터리는 `local_artifacts/rotation_recovery_2026-11-12_01/`이다.

- `rotated_reference.json`: 새 세션의 첫 회전 자세.
- `parallel_recovered.json`: 평행 복귀 자세. 반대 방향 시험의 기준으로 사용했다.
- `compare_return_parallel.json`, `compare_return_parallel.png`: 이번 비교 결과(`valid: true`, `passed: true`).
- `session.json`, `watcher_process.json`: 감시 상태 및 백그라운드 프로세스 PID.
- `recovery_plan.json`: 다음 캡처·비교 파일과 기대 회전량.

반대 방향 시험에는 위 일반 명령의 세션 경로를 이 복구 디렉터리의 `session.json`으로, 기준 파일을 `parallel_recovered.json`으로 바꿔 사용했다. 감시를 종료하려면 모든 시험 후 `watcher_process.json`의 PID를 확인해 해당 프로세스에 `SIGINT`를 보낸다. 종료 후에는 같은 기준을 이어 쓰지 않는다.

### 평행 → −10°: 평균 비교 통과, 순간 변동 확인

같은 세션 ID·epoch에서 −10° 정지 자세를 15초 수집했다. 후보 메시지 46개 중 유효 44개를 모두 변환했고, 2개(4.35%)는 `invalid_side:no_valid_line`이었다. 원본 시각의 TF 누락은 없었다. 동일 시각 odometry/TF 278쌍은 일치했다.

| 검사 항목 | 실측 결과 | 초기 기준 | 판정 |
|---|---|---|---|
| 로봇 yaw 변화 | −9.831° (−0.206° → −10.037°) | −10°에서 오차 3° 이하 | 통과 |
| 로봇 위치 변화 | 2.62mm | 5cm 이하 | 통과 |
| 후보 A 평균 위치 변화 | 1.16cm | 10cm 이하 | 통과 |
| 후보 B 평균 위치 변화 | 4.03mm | 10cm 이하 | 통과 |
| 두 후보 평균 시선 변화 | 각각 약 −0.0082° | 3° 이하 | 통과 |

**순간 변동은 평균 비교의 통과와 별도로 해결해야 한다.** −10° 캡처에서 A의 평균 대비 최대 위치 편차는 44.42cm, B는 11.58cm였다. A는 44개 샘플 중 1개에서 평균 대비 10cm를 넘었으며, 해당 프레임의 앞뒤 후보와 거리는 각각 46.55cm와 46.60cm였다. 이후 프레임에서는 기존 위치 부근으로 돌아왔다.

그 프레임(시뮬레이션 stamp `271500014159ns`)의 `look_at` 관측 끝점도 앞뒤 프레임 대비 약 45.8/46.0cm 변했다. 같은 구간 로봇 TF 위치 변화는 약 3~4µm였다. 관측 끝점 변동이 후보에 전달된 정황은 확인했지만, 기록에는 원본 측면 적합 결과나 스캔이 포함되지 않아 RANSAC·클러스터 선택 등 세부 원인은 확정하지 않았다. 무효화·급변 검사·연속 관측 등 안정화 방식은 이 사례를 바탕으로 검토해야 한다.

추가 결과는 같은 복구 디렉터리에 저장했다.

- `minus10.json`: −10° 자세 원본 캡처.
- `compare_minus10.json`, `compare_minus10.png`: 평행 대비 평균 비교(`valid: true`, `passed: true`).
- `minus10_variation.json`, `minus10_variation.png`: 최대 편차 프레임 및 이웃 샘플 비교, 시계열 그림.

위 원본 시험 당시 판정은 **평행 및 양쪽 약 10° 정지 자세의 평균 후보 기하 검증 완료, 시간에 따른 후보 안정화는 미완료**였다. 이후 연속 관측 확인을 구현하고 −10°·평행·+10° 정지 자세에서 재검증한 결과는 [후보 시간 일관성 확인 문서](stage4_candidate_stability.md)에 기록했다. 급변 거부와 재확인은 확인했지만 입력 측면 추정의 무효 구간과 작은 끝점 변동은 남아 있다. 평균 비교 통과만으로 후보를 주행 목표로 사용하지 않는다.
