# AMR 시작 자세와 독립적인 차량 좌표 입력

`mock_recovery_request.py`의 `vehicle_hint_mode: fixed_frame`은 수동으로 지정한 차량의
대략적인 위치를 고정 좌표계에서 받는다. AMR 시작 위치와 yaw가 달라져도 차량 좌표에
AMR 자세를 다시 더하지 않는다. 원래 접근 자세는 실험 시작 시 AMR의 TF에서 새로 읽는다.
이 노드는 테스트용 상위 입력을 발행하며 이동 명령을 발행하지 않는다.

## 입력 방식

| 파라미터 | 의미 |
|---|---|
| `vehicle_hint_mode: scene_world` | `run_recovery_test.sh`가 사용하는 방식. 장면이 발행하는 `scene_vehicle_frame`의 원점을 현재 실행의 `odom`으로 변환한다. |
| `vehicle_hint_mode: anchor_base` | 기존 방식. 시작 AMR 기준 `(vehicle_hint_x_in_anchor_base_m, vehicle_hint_y_in_anchor_base_m)`를 한 번 변환한다. 기본값이다. |
| `vehicle_hint_mode: fixed_frame` | `(vehicle_hint_x_m, vehicle_hint_y_m)`를 고정 좌표계의 차량 위치로 사용한다. 두 값은 반드시 지정해야 한다. |
| `vehicle_hint_frame` | 고정 입력 좌표의 기준. 빈 문자열이면 출력의 `fixed_frame`을 사용한다. `base_link` 같은 이동 프레임을 사용하면 안 된다. |
| `fixed_frame` | 발행할 `RecoveryRequest`의 좌표계. 기존 preview·제어기는 `odom`을 사용한다. |

차량 좌표는 관측 대상 선택을 위한 대략적인 위치다. 번호판 위치, 차량 방향, 성공한 이동
목표를 입력하는 기능은 아니다. `approach_*_in_anchor_base_m`, `approach_yaw_from_anchor_deg`는
기존 의미를 유지한다. 모두 0이면 이번 실험의 시작 AMR 자세가 접근 자세가 된다.

## 현재 Isaac Sim 배치에서 주행 전 확인

`amr_camera_fail.usd`는 Play 중에 같은 메시지에 `world → base_link`와
`world → sedan_car`를 담아 `/reobservation/scene_tf`로 발행한다. mock 노드는 스캔 시각에
차량과 AMR의 world 좌표 차이를 구하고, 두 좌표계에서 본 AMR의 yaw 차이만큼 회전한 뒤
AMR의 odom 위치를 더한다. 차량의 방향이나 번호판 위치는 이 계산에 쓰이지 않는다.
변환은 요청 내부에서 이루어진다. 로봇의 기존 `/tf` 트리는 그대로 사용한다.

아래 명령은 주행 없이 RViz를 연다.

```bash
cd ~/peter_pjt
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch amr_reobservation recovery_preview.launch.py \
  mock_params_file:="$PWD/ros2_ws/src/amr_reobservation/config/camera_fail_test_hint.yaml"
```

터미널의 `sedan_car at world (…) -> odom (…)`와 RViz의 노란 힌트를 확인한다.
Stop에서 AMR 위치와 방향만 바꾸고 Play했을 때 차량의 world 좌표는 같고 새 odom 좌표가
같은 차체 지점을 가리켜야 한다. 장면 데이터가 없으면 mock 노드는 요청을 보류한다.
RViz 마커는 관측 LiDAR에서 유효한 차체 윤곽을 찾으면 표시된다.

수정 전부터 장면을 열어 두었고 `/reobservation/scene_tf`가 나오지 않으면 Stop 상태에서
Script Editor에 다음을 한 번 실행해 현재 배치에 레이어를 연결한다.

```python
import omni.usd
stage = omni.usd.get_context().get_stage()
layer = stage.GetRootLayer()
if not stage.GetPrimAtPath("/Graph/ROS_Clock/SceneVehiclePoses"):
    layer.subLayerPaths.append("./assets/scene_vehicle_hint.usda")
```

## 고정 좌표 수동 입력과 미리보기

설치한 노드와 새 설정 파일을 반영한다.

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amr_reobservation --executor sequential
source install/setup.bash
```

실험별 설정을 Git 제외 폴더에 복사한다.

```bash
cd ~/peter_pjt
mkdir -p local_artifacts/fixed_vehicle_hint
cp ros2_ws/src/amr_reobservation/config/mock_recovery_request_fixed.yaml \
  local_artifacts/fixed_vehicle_hint/request.yaml
```

`request.yaml`의 `vehicle_hint_x_m`, `vehicle_hint_y_m` 두 줄의 주석을 풀고,
**`vehicle_hint_frame` 기준으로 확인한 차량 좌표**를 적는다. 예제에는 차량 좌표를 미리 채우지
않았으며, 빠뜨리면 오류로 종료한다. `(0.0, 0.0)` 자체는 유효한 고정 좌표다.
기존 `anchor_base` 설정의 `(3.5, 0.4)`를 고정 좌표라고 간주해서 복사하지 않는다.

같은 Play 구간에서 원래 실패 배치로 생성한 차량 힌트를 이미 RViz에서 확인했다면,
`ros2 topic echo /reobservation/request --once`의 `header.frame_id`와 `vehicle_hint.x/y`가
고정 입력의 출발점이 될 수 있다. **다른 Play 구간의 이전 요청은 재사용하지 않는다.**

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch amr_reobservation recovery_preview.launch.py \
  mock_params_file:=$PWD/local_artifacts/fixed_vehicle_hint/request.yaml
```

기존 preview/mock 발행자를 종료한 뒤 실행한다. RViz의 노란 차량 힌트가 대상 차량 근처에
남아 있는지 확인한다. 같은 Play 구간에서 AMR 시작 자세를 바꾼 후 preview/mock을 다시
시작하면 차량 힌트는 같고 `approach_pose`와 요청 ID는 새로 만들어진다. 하나의 실행 중
반복 발행은 차량 힌트·접근 자세·요청 시각을 갱신하지 않으며 유효기간도 연장하지 않는다.

통합 실행기의 `--mock-params`에도 같은 파일을 전달할 수 있다. 아래 명령은 경로 확인만 한다.
실제 이동은 기존과 같이 명시적인 `--execute` 실행에서 수행한다.

```bash
/usr/bin/python3 tools/run_plate_recovery.py \
  --mock-params local_artifacts/fixed_vehicle_hint/request.yaml \
  --output local_artifacts/fixed_hint_recovery_01
```

## Stop → Play 및 다른 고정 좌표계

**`odom` 고정 좌표와 Isaac USD의 world 좌표는 같은 값이라고 가정하면 안 된다.**
[이전 회전 검증](stage4_rotation_check.md)에 기록했듯 Isaac Compute Odometry는 첫 프레임의
자세를 원점으로 삼는다. 따라서 AMR 시작 자세를 바꾸고 Stop → Play하면, 같은 물리적 차량도
새 `odom`에서 다른 좌표를 가질 수 있다. `odom` 입력의 재사용은 원점·축이 유지되는 구간으로
한정한다. 노드 재시작은 좌표의 정확성을 자동으로 확인해 주지 않는다.

`fixed_frame` 수동 입력으로 배치 재시작을 넘어 같은 차량 좌표를 재사용하려면 위치가 유지되는 `map` 또는 `world`를
`vehicle_hint_frame`으로 지정하고, 해당 프레임과 새 `odom` 사이의 **올바른 TF**를 제공해야 한다.
출력 `fixed_frame`은 기존대로 `odom`에 둘 수 있다. 노드는 스캔 시각의 변환을 적용한 뒤
차량 힌트를 고정한다. 변환이 없거나 해당 시각을 지원하지 않으면 요청 발행을 기다린다.
좌표계를 이름만 바꾸거나 단위 변환으로 대신하지 않는다. 변환은 같은 XY 지면을 나타내는
평면 변환이어야 한다.

ROS 시각이 뒤로 가면 기존 요청·스캔·TF를 비우고 새 스캔과 TF로 새 요청을 만든다.
이것은 이전 시각의 데이터를 섞지 않기 위한 처리이며, `odom` 원점이 유지된다는 증거는 아니다.
기본 `scene_world` 실험은 두 좌표계 사이의 평면 변환을 요청마다 계산한다.
센서 기록에는 해당 실험에서 사용한 `/reobservation/scene_tf`도 저장된다.

## 검증

격리된 비어 있는 ROS 도메인에서 다음 계약 테스트를 실행한다.

```bash
ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  ros2_ws/src/amr_reobservation/test/smoke_mock_recovery_request.py
```

합성 스캔·TF·시계를 사용하므로 실제 장면 이동이나 OCR 성공을 검증하는 테스트는 아니다.
실제 Play에서 노란 힌트가 차량 위치에 놓이는지와, 새 시작점의 재관측 성공 여부는 별도로 확인한다.
