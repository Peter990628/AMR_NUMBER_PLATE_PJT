# AMR 관측 LiDAR C++ 처리

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
