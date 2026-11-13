# 4단계 첫 검증: 평행 배치의 관측 후보

측면 추정 선분의 양 끝에서 센서 관측 후보 두 개를 만들고 RViz에 표시한다. 이 단계는 **센서 평면에서의 후보 기하**를 확인한다. 후보는 `base_link` 주행 목표가 아니며 차량 후면, 실제 범퍼 끝, 차체와의 충돌 여유, 도달 가능한 경로를 보장하지 않는다.

## 실행

Isaac Sim에서 차량 옆에 AMR을 평행하게 배치한 뒤 Play한다. 기존 측면 추정 노드가 실행 중이면 아래 명령에 `start_estimator:=false`를 추가하여 중복 발행을 피한다.

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch amr_reobservation observation_candidates.launch.py
```

현재 launch는 측면 추정, 원본 후보 생성, 연속 관측 확인 노드와 RViz를 실행한다. 원본은 `/observation_candidates/raw`, 확인 결과는 `/observation_candidates`다. 차량 위치나 속도 명령을 발행하지 않는다. 기하 설정은 `config/observation_candidates.yaml`, 확인 기준은 `config/candidate_stability.yaml`에서 변경 후 재시작한다. [연속 관측 확인 안내](stage4_candidate_stability.md)에 현재 출력 계약을 정리했다.

RViz의 고정 프레임은 `observation_lidar`다. 노란 선은 관측된 측면, 초록색 A와 분홍색 B는 확인된 관측 후보, 화살표는 각각의 관측 끝점을 향하는 시선이다. 선택적으로 원본 후보 표시를 켜면 직각으로 꺾인 L·D 보조선과 흰색 LiDAR 원점도 보인다. z=0은 바닥이 아닌 LiDAR 스캔 평면이다. 현재 AMR 외곽선은 표시하지 않는다.

## 계산과 유효성

측면 끝점을 A, B, A→B 단위 접선을 t, 측면에서 센서 원점으로 향하는 단위 법선을 n이라 할 때:

```text
후보 A = A − L·t + D·n
후보 B = B + L·t + D·n
yaw = atan2(해당 끝점.y − 후보.y, 해당 끝점.x − 후보.x)
```

초기값은 L=0.8m, D=1.0m다. A/B는 선분 끝점 라벨이며 차량의 앞/뒤를 뜻하지 않는다. 시선은 번호판의 정답 위치가 아니라 관측된 선분 끝점을 향한다. 실제 카메라와 로봇 기준점 사이의 장착 오프셋을 적용한 주행 목표 변환은 이후 단계다.

- 입력: `/vehicle_side/estimate`, `VehicleSideEstimate`, Reliable depth 1.
- 원본 출력: 현재 launch에서는 `/observation_candidates/raw`, `ObservationCandidates`, Reliable depth 1. 원본 header를 보존하며 유효할 때만 후보 2개와 시선 대상 2개를 제공한다. 원본 노드를 직접 실행할 때의 기본 토픽은 이전과 같은 `/observation_candidates`이므로 확인 노드와 중복 발행하지 않는다.
- 원본 표시: 현재 launch에서는 `/debug/observation_candidates/raw`, `MarkerArray`, Reliable depth 1. 확인된 표시에는 `/debug/observation_candidates`를 사용한다.
- 유한값, 단위 접선·법선, 직교성, 끝점 순서, 법선이 센서를 향하는지 확인한다.
- 측면 추정이 무효이면 즉시 빈 후보와 `DELETEALL`을 발행한다.
- ROS 시각 기준 나이 0.5초 초과, 미래 0.1초 초과, 중복·역행 stamp도 거부한다. 역행 시 새 기준으로 옮겨 후속 증가 프레임에서 복구한다.
- wall clock 수신 중단 1초 후 `input_timeout`과 `DELETEALL`을 발행한다. Pause에서도 작동한다. downstream은 노드 자체 종료에 대비하여 별도 수신 watchdog을 사용해야 한다.

## 평행 배치 실측 방법

측면/후보 메시지를 같은 frame+stamp로 짝지어 확인한다. 아래 도구는 끝점으로부터 접선·법선을 다시 계산하여 거리·방향을 독립적으로 검사한다. 구독만 하며 후보나 센서 입력을 발행하지 않는다.

```bash
cd ~/peter_pjt
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 tools/verify_parallel_candidates.py --duration 15 \
  --output-dir local_artifacts/parallel_check_2026-11-12
```

검사 항목:

1. 두 후보가 관측 측면 기준으로 센서와 같은 쪽에 있는가.
2. 수직 거리 D와 부호를 포함한 길이 방향 연장량 L이 설정값과 일치하는가.
3. yaw가 해당 끝점을 향하는가.
4. 정지 관측 중 후보 위치가 얼마나 변하는가. 후보는 현재 프레임 측정을 그대로 사용하며 평활화하지 않는다.
5. `base_link → observation_lidar` TF가 있으면 측면 축과 로봇 전방 축의 차이를 계산한다. 3°는 초기 평행 배치 확인 기준이며 센서 정확도 보증값이 아니다.

그림의 raw scan은 캡처 마지막 스캔이며, 후보 수치 검사는 정확히 같은 stamp의 측면/후보 쌍에 대해서 수행한다. 여러 자세 비교에서는 고정 좌표계 변환과 TF의 실제 pose 반영 여부를 먼저 확인해야 한다.

## 2026-11-12 첫 실측 결과

아래는 연속 관측 확인을 추가하기 전 원본 후보의 기록이다. 현재 검증 도구의 기본 `/observation_candidates` 구독은 확인된 출력을 평가하므로 유효 비율을 직접 동일시하지 않는다.

- 15초 동안 후보 메시지 49개 수신. 유효 쌍 46개 모두 거리·방향 계산 통과.
- 3개는 `invalid_side:no_valid_line`으로 후보를 무효화했다. 유효 프레임 통과를 전체 스트림 안정성 통과로 해석하지 않는다.
- 측면 방향과 AMR 전방 축 차이 평균 0.538°, 관측 선분 길이 평균 3.558m, 선분 적합 RMSE 평균 14.13mm.
- 후보 A 평균 좌표 (-2.971, -1.104)m, yaw -50.80°. 평균 대비 최대 위치 편차 4.10cm.
- 후보 B 평균 좌표 (2.187, -1.056)m, yaw -128.12°. 평균 대비 최대 위치 편차 10.04cm.
- 양 후보 모두 D=1.000m, L=0.800m. 좌표는 `observation_lidar` 기준이다.

결론: **평행 배치에서의 기본 후보 기하 계산은 통과했다. 측면 추정의 간헐적 무효화와 끝점 변동은 확인되었으므로, 이동 제어 연결 전 안정화가 필요하다.** 실제 차체·범퍼와의 독립적인 간격 측정, AMR 외곽선 검사, 후면 선택 및 경로 검증은 수행하지 않았다.

실측 JSON과 그림은 Git에서 제외되는 `local_artifacts/parallel_check_2026-11-12/`에 저장한다. 사용자에게서 확인받은 3단계 검출 저하·복구, 미검출 및 Pause/Play 복구도 완료되어 있으며, 이번 결과는 그 다음의 기하 계산 검증에 해당한다.

## 자동 테스트

다음 검증은 [고정 좌표계에서 ±10° 회전 비교](stage4_rotation_check.md)다. 평행 기준을 `odom`에 저장한 뒤 같은 Play 구간에서 회전 전후 후보를 비교한다.

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amr_reobservation
source install/setup.bash
colcon test --packages-select amr_reobservation
colcon test-result --verbose
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
  python3 src/amr_reobservation/test/smoke_observation_candidates.py
```

후보 기하 단위 테스트 12개를 포함한 GTest 59개 통과. 별도 도메인 ROS smoke는 알려진 좌표, 마커, 무효 입력, 잘못된 법선, 중복 stamp, 수신 중단과 복구를 검증했다. ±10° 회전의 수학 테스트는 실제 Isaac Sim 회전 배치 시험을 대신하지 않는다.
