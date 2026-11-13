# 2단계: 차량 측면 후보 추정

차량 옆에 정지한 AMR의 관측 LiDAR로 선형 표면 후보를 추정한다. 기존 전처리 라이브러리를 재사용하므로 1단계 노드를 따로 실행할 필요가 없다.

## 실행

Isaac Sim에서 `amr_side_lidar_check.usd`를 열고 **Play**한다. 기존 1단계 launch 터미널은 `Ctrl+C`로 종료하고 새 ROS 2 터미널에서 실행한다.

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amr_reobservation
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch amr_reobservation vehicle_side_estimator.launch.py rviz:=true
```

빌드가 이미 끝났다면 `colcon build`는 생략한다. 시뮬레이션 설정은 1단계와 동일하게 `/clock`과 센서의 시간 정책을 맞추고 `use_sim_time=true`를 사용한다. 노드는 주행 명령을 발행하지 않는다.

| RViz 표시 | 의미 |
|---|---|
| 흰색 점 | 원본 관측 스캔 |
| 여러 색의 점 | 거리·각도 필터를 통과한 점을 나눈 클러스터 |
| 청록색 큰 점 | 선택된 직선의 최종 inlier |
| 노란 선 | 최종 직선에 투영한 관측 구간 |
| 파란 화살표 | 측면 축의 방향, 차량 전방을 뜻하지 않음 |
| 주황 화살표 | 측면에서 LiDAR 원점을 향하는 법선 |
| 글자 | 관측 길이(span), RMSE, inlier 개수와 비율 |

고정 좌표계는 `observation_lidar`다. 표시가 겹치면 `Side estimate → Namespaces`에서 `clusters`, `inliers` 등을 개별적으로 끌 수 있다. 기존 RViz를 사용할 때는 **Add → MarkerArray**에 `/debug/vehicle_side`를 지정한다.

별도 터미널에서 결과를 확인할 수 있다.

```bash
source /opt/ros/humble/setup.bash
source ~/peter_pjt/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic echo /vehicle_side/estimate --once
```

## 계산 순서

```text
LaserScan
  → ScanPreprocessor: 유효 거리·ROI, XY 변환, 원본 beam_index 유지
  → VehicleSideEstimator: 순차 클러스터링
  → 클러스터별 RANSAC 직선 후보
  → PCA 재계산과 inlier 갱신
  → 점 개수·비율·길이·잔차 검사
  → 센서에서 가장 가까운 유효 관측 선분 선택
  → VehicleSideEstimate 메시지와 RViz MarkerArray
```

**클러스터링:** 연속한 점 사이의 XY 거리가 임계값보다 크거나, 원본 스캔에서 빠진 빔 수가 허용치를 넘으면 분리한다. 전체 360° 스캔에서는 첫/마지막 구간에도 같은 기준을 적용해 연결한다. 부분 시야 스캔의 양 끝은 연결하지 않는다.

**RANSAC:** 무작위로 두 점을 골라 직선을 만들고, 그 직선에서 일정 거리 이내에 있는 점을 inlier로 센다. inlier가 가장 많은 모델을 선택하고, 동률이면 제곱 잔차가 작은 모델을 선택한다. 난수 seed와 반복 횟수는 설정으로 고정하여 같은 입력에서 재현할 수 있다.

**PCA:** 선택된 점들의 평균과 2×2 공분산을 계산한다. Eigen의 대칭행렬 고유값 분해에서 가장 큰 고유값에 대응하는 고유벡터가 측면 축이다. 이 직선으로 inlier를 다시 계산하며, `refinement_limit`(기본 20회) 안에 점 집합이 안정되지 않으면 실패로 처리한다. 수렴하면 즉시 종료하므로 매번 20회를 계산하지 않는다. 최종 중심·방향·잔차·끝점은 모두 같은 최종 inlier 집합에서 계산한다.

**관측 길이:** 각 inlier를 측면 축으로 투영하고, 최대 투영값에서 최소 투영값을 뺀다. 출력 끝점은 해당 투영값의 선 위 위치다. 이 값은 관측된 선분의 길이이며 가려진 부분이나 범퍼 곡면을 포함한 차량 전체 길이가 아니다.

**후보 선택:** 품질을 통과한 후보 중 LiDAR 원점에서 관측 선분까지의 거리가 가장 가까운 것을 선택한다. 동률이면 inlier가 많은 후보, 그다음 입력 순서가 빠른 클러스터를 선택한다. 한 클러스터에서 하나의 지배적인 직선만 추정한다.

## 출력 계약과 실패 처리

- 입력: `/observation_lidar/scan` (`sensor_msgs/msg/LaserScan`, Best Effort 구독).
- 결과: `/vehicle_side/estimate` (`amr_reobservation/msg/VehicleSideEstimate`, Reliable).
- 표시: `/debug/vehicle_side` (`visualization_msgs/msg/MarkerArray`, Reliable).
- 진단: `/debug/vehicle_side/fit_diagnostics` (`amr_reobservation/msg/VehicleSideFitDiagnostics`, Reliable, depth 10). [기록 방법과 필드 해석](fit_diagnostics.md).
- 결과의 `header`는 원본 스캔의 frame과 취득 시각을 그대로 유지한다. 좌표는 모두 LiDAR 프레임의 XY 스캔 평면이며, 월드 바닥 높이를 뜻하지 않는다.
- `valid=true`일 때만 중심·방향·끝점·길이·잔차를 사용한다. 실패한 스캔에도 `valid=false`와 상태를 발행하고 기하값을 기본값으로 돌린다. 이전 추정값은 재사용하지 않는다.
- `input_point_count`는 전처리 후 점 수, `cluster_count`는 크기가 작아 탈락한 것까지 포함한 클러스터 수다. `inlier_ratio`의 분모는 선택된 클러스터의 점 수다. 확률 또는 차량 검출 신뢰도를 뜻하지 않는다.
- `tangent`의 부호는 결정적으로 정하며 차량 앞뒤를 의미하지 않는다. `normal_toward_sensor`는 원점 쪽을 향한다. 선이 원점을 지나는 퇴화 배치에서는 센서 쪽이라는 구분이 불가능하므로 결정적인 부호만 사용한다.
- 입력이 오지 않으면 새로운 결과도 발행되지 않는다. **`valid`는 해당 스캔 시각의 유효성**이며, 후속 제어기는 메시지 시각과 실제 수신 경과 시간을 별도로 검사해야 한다.

| status | 의미 |
|---|---|
| `ok` | 품질 검사를 통과한 후보 있음 |
| `empty` | 전처리 후 점이 없거나 빈 스캔 |
| `no_clusters` | 최소 점 수를 채운 클러스터 없음 |
| `no_valid_line` | 클러스터는 있으나 직선 추정 또는 품질 검사 실패 |
| `invalid_input` | 점 순서·beam index·스캔 범위 등 입력 계약 위반 |
| `invalid_scan_metadata` | 원본 거리·각도 metadata 오류 |
| `empty_frame` | 원본 frame_id 누락 |

추정 실패 시 이전 inlier·선·방향·텍스트 Marker를 삭제한다. 클러스터가 있으면 진단용 클러스터 점은 계속 표시한다. 빈 frame을 받은 경우 삭제 메시지만 마지막 유효 frame과 시각 0(최신 TF)을 사용한다. 표시 수명은 1 ROS초이며 시뮬레이션 시간이 멈추면 수명 계산도 멈출 수 있다.

## 파라미터

`config/vehicle_side_estimator.yaml`을 수정하고 노드를 다시 실행한다. 전처리 거리·각도는 1단계와 같은 의미지만 **2단계 YAML에 독립적으로 설정**한다. 시작 기본값은 거리 0.1~6m, 전체 각도다.

| 파라미터 | 기본값 | 역할 |
|---|---:|---|
| `cluster_gap_m` | 0.15 | 연속 점 사이 허용 거리(m) |
| `max_missing_beams` | 2 | 연속 점 사이 허용 누락 빔 수 |
| `min_cluster_points` | 20 | 추정할 클러스터의 최소 점 수 |
| `ransac_iterations` | 100 | RANSAC 최대 시도 수 |
| `refinement_limit` | 20 | 클러스터별 PCA 최대 재적합 횟수(1~100), 수렴 시 조기 종료 |
| `ransac_seed` | 7 | 재현용 난수 seed |
| `inlier_distance_m` | 0.03 | 직선에서 inlier까지 허용 수직 거리(m) |
| `min_inliers` | 20 | 최종 inlier 최소 수 |
| `min_inlier_ratio` | 0.5 | 클러스터 내 최소 inlier 비율 |
| `min_observed_length_m` | 0.5 | 허용할 최소 관측 길이(m) |
| `max_rmse_m` | 0.02 | 최종 수직 거리 RMSE 상한(m) |

이 값들은 시작 설정이다. 여러 장애물이 있는 장면에서는 ROI를 대상 영역으로 좁혀야 한다. 기하만으로 벽과 차량을 구분하지 않으므로 가까운 벽이 선택될 수 있다. 정지 차량 한 대의 측면 확인 장면부터 검증한다. 시간 방향 추적·평활화·주행 중 스캔 왜곡 보정은 현재 범위에 포함되지 않는다.

## 테스트와 확인 순서

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select amr_reobservation --event-handlers console_direct+
colcon test-result --verbose
```

GTest는 잡음·이상치가 섞인 직선의 수치 오차, 최종 inlier와 잔차의 일관성, 클러스터 간격, 전체/부분 시야 경계, 후보 선택, 곡선·짧은 구간·중복 점의 실패, 입력 오류와 재현성을 확인한다.

설치된 launch와 노드를 합성 스캔으로 검증하는 수동 DDS 테스트도 제공한다. 위와 같이 workspace 환경을 source한 뒤 사용하지 않는 별도 도메인에서 실행한다. 아래 환경 변수는 해당 명령에만 적용되므로 기존 Isaac Sim의 도메인 0 설정은 유지된다.

```bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 \
  python3 src/amr_reobservation/test/smoke_vehicle_side_node.py
```

이 테스트는 알려진 직선의 수치 결과, 실패 시 기하값 초기화와 Marker 삭제, 정상 입력 복귀, 시작 파라미터 검사를 확인한다. 테스트용 프로세스는 종료 시 정리한다. ROS 메시지 계약 검증이며 Isaac Sim/RViz의 실제 렌더링 검증과는 별개다.

실제 장면에서는 노란 선이 차량 측면의 직선 구간을 따라가는지, 주황 법선이 LiDAR 쪽인지, `valid`와 잔차가 어떻게 변하는지 확인한다. 그다음 ROI를 차량에서 벗어나게 바꿔 재실행하면 결과가 무효가 되고 이전 선이 남지 않는지 확인한다. 결과가 부정확할 때는 먼저 클러스터 분리와 선택된 inlier를 보고 원인을 구분한다.

구현 파일: [일반 C++ 추정기](../src/vehicle_side_estimator.cpp), [ROS 노드](../src/vehicle_side_estimator_node.cpp), [메시지](../msg/VehicleSideEstimate.msg), [단위 테스트](../test/test_vehicle_side_estimator.cpp).

참고: [Eigen SelfAdjointEigenSolver](https://libeigen.gitlab.io/eigen/docs-3.4/classEigen_1_1SelfAdjointEigenSolver.html), [ROS 2 동일 패키지에서 인터페이스 생성·사용](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.rst).
