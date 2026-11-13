# 3단계: 번호판 검출과 관측 품질 판정

ROS 카메라 영상에서 번호판 후보를 검출하고, 재관측 성공 판단에 사용할 품질 신호를 계산한다. **현재 `usable`은 설정한 영상 품질 조건을 만족했다는 뜻이다. OCR 성공이나 번호판 전체 가시성을 보장하지 않는다.** 자동 이동 명령은 발행하지 않는다.

## 현재 PC에서 실행

모델 변환과 빌드는 완료되어 있다. Isaac Sim에서 번호판이 보이는 차량 뒤쪽 배치를 열고 **Play**한다. 측면 LiDAR 확인 배치에서는 전방 카메라에 번호판이 없을 수 있으며 이때 `not_detected`가 정상이다.

현재 장면에서 재현 가능한 정면 배치를 만들려면 **Stop → Save As `amr_plate_observation_test.usd`**로 시험용 사본을 저장한다. Script Editor에서 아래 도구를 실행하고 저장한 뒤 Play한다. 이 도구는 차량의 위치와 yaw만 변경한다.

```python
import runpy
runpy.run_path('/home/peter-msi/peter_pjt/tools/setup_camera_check.py')
```

별도 ROS 터미널:

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch amr_reobservation plate_observation.launch.py \
  model_path:=$HOME/peter_pjt/assets/models/semi_allimages_v5n.onnx
```

다른 ROS 터미널에서 아래 명령을 실행하고 `/debug/plate_observation`을 선택한다.

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

결과 확인:

```bash
source /opt/ros/humble/setup.bash
source ~/peter_pjt/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic echo /plate/observation --once
```

성공 조건을 연속 3회 채우면 `status: usable`, `input_valid: true`, `detected: true`, `usable: true`가 나온다. 디버그 영상의 박스는 초록색이다. ROS 2 노드와 `/clock`, 카메라 stamp의 시간 기준을 맞춰야 한다. Isaac Sim 환경과 ROS 개발 터미널은 분리한다.

## 계산과 구현 구조

```text
Image → BGR8 → 정사각형 letterbox → RGB float32 /255
      → ONNX Runtime CPU 추론 → Plate 클래스 선택 → NMS
      → 픽셀 좌표 복원 → 크기·경계 여백·모호성 검사
      → 같은 후보의 연속 관측 → PlateObservation + 디버그 Image
```

| 파일 | 역할 |
|---|---|
| `include/amr_reobservation/plate_observation.hpp`, `src/plate_observation.cpp` | ROS/OpenCV 의존성 없는 일반 C++ 품질 판정 및 연속 프레임 상태 |
| `include/amr_reobservation/plate_detector.hpp`, `src/plate_detector.cpp` | C++ 전처리·ONNX Runtime 추론·박스 해석·NMS |
| `src/plate_observation_node.cpp` | ROS 메시지, 시간 검사, watchdog 및 디버그 영상 |
| `src/plate_image_probe.cpp` | ROS 없이 저장 영상 한 장의 C++ 검출 확인 |
| `msg/PlateObservation.msg` | 센서 시각·판정·박스·점수·연속 횟수 |
| `config/plate_observation.yaml` | 시작 파라미터 |
| `tools/export_plate_model.py` (프로젝트 루트 기준) | 오프라인 모델 변환; 런타임 판정은 C++ |

이 모델의 클래스는 `0: Car`, `1: Not_Car`, `2: Plate`, `3: Not_Plate`이다. `Plate`가 해당 후보의 최상위 클래스일 때만 사용한다. 입력은 `[1,3,640,640]`, 출력은 `[1,8,8400]`이며 **별도의 objectness 채널이 없다.** 이름이 YOLOv5라고 해서 구형 `[N,5+classes]` 출력을 가정하면 안 된다. 출력 규격이 다르면 시작 또는 추론 단계에서 오류 처리한다.

시스템 OpenCV 4.5.4의 DNN으로는 이 ONNX 모델 추론이 실패하여 **ONNX Runtime 1.23.2 CPU C++**를 사용한다. OpenCV는 이미지 변환·letterbox·NMS·그리기에 사용한다. ROS의 OpenCV 설치를 교체하지 않는다.

## 판정 기준과 한계

| 파라미터 | 초기값 | 의미 |
|---|---:|---|
| `detector_score_threshold` | 0.25 | 후보로 남길 최소 모델 점수 |
| `min_confidence` | 0.5 | 품질 판정에 필요한 최소 점수 |
| `min_width_px` / `min_height_px` | 80 / 16 | 원본 영상 좌표에서 최소 박스 크기 |
| `border_margin_ratio` | 0.02 | 각 축 크기로 정규화한 최소 경계 여백 |
| `required_consecutive` | 3 | 연속으로 품질을 만족해야 하는 횟수 |
| `min_tracking_iou` | 0.3 | 이전 후보와 같은 대상으로 이어 볼 최소 박스 IoU |
| `image_timeout_sec` | 1.0 | 마지막 영상 callback 이후 최대 실제 시간 간격 |
| `max_image_age_sec` | 0.5 | ROS 시각 기준 최대 영상 나이 |
| `future_tolerance_sec` | 0.1 | 미래 stamp의 허용 오차 |
| `inference_threads` / `opencv_threads` | 2 / 2 | CPU 사용 스레드 제한 |

원본 박스 `(x,y,w,h)`와 영상 크기 `(W,H)`에 대해 여백은 `min(x/W, y/H, (W-x-w)/W, (H-y-h)/H)`이다. 박스를 화면 안으로 자르기 전에 판단하므로 화면 밖으로 넘어간 부분을 숨기지 않는다. 검출기가 잘린 번호판의 보이는 일부만 박스로 잡는 경우까지 완전하게 알아낼 수 있는 기준은 아니다. 후보를 찾지 못한 잘림은 `not_detected`가 될 수 있다.

NMS 후 품질 점수를 넘는 후보가 두 개 이상이면 `ambiguous`로 처리한다. 추후 여러 차량을 다룰 때에는 목표 차량과 번호판의 연결 로직이 별도로 필요하다. IoU는 간단한 연속성 기준이며 객체 ID 추적기는 아니다. 같은 번호판이라도 이동량이 크면 연속 횟수가 다시 시작된다.

이 초기 임계값은 현재 합성 장면용 출발점이다. 실제 카메라·조명·해상도 변화 및 OCR 가독성과의 관계는 별도로 검증해야 한다. 원본 번호판 글자나 USD의 정답 위치를 런타임 판정에 사용하지 않는다.

## 상태와 시간 계약

- `/plate/observation`: Reliable, depth 1. `header`는 원본 영상 stamp/frame을 보존한다.
- `/debug/plate_observation`: BGR8 Image, Best Effort, depth 1. 입력은 `/front_stereo_camera/left/image_raw`의 Best Effort 구독이다.
- `detected`는 후보가 있다는 뜻이다. 낮은 점수·작은 크기·경계 위험에서도 후보의 박스와 점수를 진단용으로 제공할 수 있다.
- `input_valid`는 신선한 영상을 정상 처리했음을 나타낸다. `usable`이 실제 관측 품질 판정 결과다.
- 미검출·입력 오류·timeout에서는 박스와 점수를 0으로 초기화한다. 품질 실패와 입력 오류는 연속 횟수를 초기화한다.
- 같은 stamp를 반복 수신하면 `duplicate_stamp`로 거부한다. 역행 stamp는 `time_reset`으로 한 프레임 거부하고 기준을 새 시각으로 옮겨, 이후 증가하는 프레임들로 복귀할 수 있게 한다.
- frame이나 해상도가 바뀌면 연속 횟수를 다시 시작한다.
- 추론 전후에 ROS 시간 기준 freshness를 확인한다. 별도 callback group의 wall timer와 2-thread executor를 사용하여 추론 중에도 timeout을 발행할 수 있다.
- timeout으로 무효화된 추론의 뒤늦은 결과는 성공으로 발행하지 않는다. 성공 상태에 진입하려면 다시 연속된 정상 프레임이 필요하다.
- **후속 제어기는 자체 수신 watchdog도 사용해야 한다.** 이 노드가 종료되거나 통신이 끊기면 timeout 메시지 자체를 받을 수 없다.

| 상태 | 의미 |
|---|---|
| `warming_up` | 현재 후보는 품질 통과, 연속 횟수 부족 |
| `usable` | 연속 관측 조건까지 통과 |
| `not_detected` / `low_confidence` | 후보 없음 / 품질 기준보다 점수 낮음 |
| `border_risk` / `too_small` | 경계에 닿거나 여백 부족 / 픽셀 크기 부족 |
| `ambiguous` / `invalid_detection` | 다수 후보 / 잘못된 후보 값 |
| `no_image` / `input_timeout` | 시작 후 영상 없음 / 영상 수신 중단 |
| `invalid_image` / `empty_frame` | 영상 형식·배열 오류 / frame_id 없음 |
| `invalid_stamp` / `stale_image` / `future_image` | 영상 시각 오류 |
| `duplicate_stamp` / `time_reset` | 중복 / 역행 stamp |
| `inference_error` / `processing_timeout` | 추론 실패 / 시간 초과·추론 도중 무효화 |

영상 오류·timeout 시 디버그 영상은 검은 배경의 상태 표시로 교체한다. Pause/Stop 중에도 wall watchdog은 동작한다. 별도 입력 timeout 메시지의 header는 마지막 입력 시각이며 새 촬영 시각을 만들어 넣지 않는다. 첫 영상 전에는 빈 header를 사용한다.

## 새 환경에서 모델·런타임 준비

모델 가중치는 기존 `ALLEY_PARK_PATROL_PJT`의 사용자 보유 파일을 재사용한다. 자체 학습 모델로 주장하지 않는다. 원본·변환 모델 및 다운로드한 런타임은 `.gitignore`에 포함되어 있다. 모델 원출처·적용 조건은 재배포 전에 확인하고, 런타임의 LICENSE와 ThirdPartyNotices는 설치 패키지에도 보존한다.

프로젝트 루트에서:

```bash
bash tools/setup_onnxruntime.sh
```

이 스크립트는 Microsoft 공식 Linux x86_64 CPU 배포판 1.23.2를 받고 SHA-256을 확인한다. 설치 경로는 `third_party/onnxruntime-linux-x64-1.23.2`다. 다른 설치를 사용할 경우 CMake의 `ONNXRUNTIME_ROOT`를 지정한다.

모델 export는 기존 PyTorch/Ultralytics 환경을 기반으로 격리된 Python 환경에서 수행한다. 이 PC에서 확인한 버전은 PyTorch 2.6.0, Ultralytics 8.4.100이다.

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install onnx==1.17.0 onnxslim==0.1.82 protobuf==4.25.8 ml_dtypes==0.5.3
.venv/bin/python tools/export_plate_model.py \
  --weights /absolute/path/to/semi_allimages_v5n.pt
```

변환 스크립트는 원본을 `assets/models`로 복사한 뒤 CPU에서 변환하며 자동 패키지 설치를 비활성화한다. `semi_allimages_v5n.onnx`와 클래스·입출력 규격·원본/ONNX 해시가 담긴 JSON을 생성한다. 현재 원본 모델 SHA-256은 `69678e72aa6c2a2efb834e3a7f0aed8037fc60e0ee8155078a1c9de69e8c12b7`이다.

그다음 별도 ROS 터미널에서 빌드한다.

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amr_reobservation --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## 검증

2026-11-12 현재 PC에서 확인한 결과:

- C++ 빌드 완료, 1·2단계를 포함한 GTest 47개 통과.
- 기존 Isaac Sim 카메라 화면에서 추출한 영상 2장으로 실제 모델 추론 확인. C++와 Python 기준 결과의 박스 좌표 차이는 0.005px 미만, 점수 차이는 0.000002 미만이었다.
- 격리된 ROS 도메인에서 저장 영상 재생 테스트 통과: 연속 검출, 미검출, 잘못된 영상, frame 변경, 중복·과거·미래 stamp, 수신 중단 및 복구, `/clock` 역행 후 복구.
- 이후 사용자가 실제 Isaac Sim 카메라에서 `usable → low_confidence → usable`, 화면 밖 `not_detected`, Pause의 `input_timeout` 및 Play 후 복구를 확인했다. 제공된 화면에서도 품질 저하·복구, 미검출 박스 제거, timeout 상태 교체를 확인했다. 다양한 주행·조명 조건의 검출 성능 검증은 별도다.

로컬 검증 결과는 Git에서 제외된 프로젝트 루트의 `local_artifacts/plate_validation/`에 저장했다.

```bash
colcon test --packages-select amr_reobservation --event-handlers console_direct+
colcon test-result --verbose
```

품질 판정 테스트는 연속 성공·실패 초기화·크기 경계·네 영상 경계·화면 밖 박스·점수·다중 후보·대상 변경·설정 오류를 확인한다. 검출 후처리 테스트는 letterbox 역변환, NMS, `Not_Plate` 제외, 비정상 출력 거부를 확인한다. 모델 파일이 없어도 이 수학 테스트들은 실행할 수 있다(빌드용 런타임은 필요).

저장 영상의 실제 C++ 추론:

```bash
ros2 run amr_reobservation plate_image_probe \
  ~/peter_pjt/assets/models/semi_allimages_v5n.onnx /path/to/camera.png /tmp/plate_debug.png
```

한 장만 처리하므로 정상 후보에서도 `warming_up`이 나오는 것이 정상이다. 검출 박스와 점수는 JSON으로 출력한다.

실제 모델을 쓰는 ROS 메시지·시간·실패 복귀 테스트는 사용하지 않는 별도 도메인에서 수행한다. `--image`에는 번호판이 검출되는 카메라 영상 전체를 제공한다.

```bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
  python3 src/amr_reobservation/test/smoke_plate_observation.py \
  --model ~/peter_pjt/assets/models/semi_allimages_v5n.onnx \
  --image /path/to/positive_camera.png
```

이 검증은 같은 저장 영상을 새 stamp로 재생하여 메시지 계약과 연속 판정 상태를 확인한다. 실제 움직이는 카메라의 연속 검출 성능과는 구분한다.

실제 Isaac Sim에서는 다음을 차례로 확인한다: 번호판 정면 → 가장자리 접근 → 일부 잘림 → 먼 거리 → 미검출 → 정상 복귀 → Pause/Play. 화면에서 읽을 수 있는 번호판을 놓치거나 잘린 상태를 성공으로 판정하는 경우를 남겨 임계값과 판정 기준을 검토한다.

참고: [ONNX Runtime C++](https://onnxruntime.ai/docs/get-started/with-cpp.html), [고정한 런타임 릴리스](https://github.com/microsoft/onnxruntime/releases/tag/v1.23.2).
