# AMR 번호판 재관측

차량 근처까지 온 로봇이 비스듬한 각도 때문에 번호판을 읽지 못하면 어디로 움직여야 할까? 이 프로젝트는 Isaac Sim에서 **로봇이 스스로 촬영 위치를 다시 고르고, 그곳에서 같은 번호를 세 번 연속 읽을 수 있는지** 검증한다. 상위 주행 시스템이 차량 근처에 도착한 시점부터 시작한다. 시뮬레이션에서는 장면 속 차량의 위치를 상위 시스템의 힌트로 사용한다.

## 실험 결과

2026년 11월 24일, 같은 세단과 번호판을 두고 **네 출발 자세를 각각 한 번씩** 시험했다. 네 곳 모두 출발 전 사진 8장에서 번호판 상자를 찾지 못했다. 그중 **3곳에서 이동 후 `123가4567`을 연속 세 장 판독**했다. 코드 `3d9a498`과 실행 옵션은 네 번 모두 동일했다.

[![네 출발 자세에서 출발 전 카메라 화면과 최종 화면을 비교한 그림](docs/media/create3-trials.jpg)](docs/media/create3-recovery-demo.mp4)

[실험 영상 재생 또는 다운로드 (MP4)](docs/media/create3-recovery-demo.mp4) · [시작 좌표, 목표, OCR 점수와 중단 사유 (JSON)](docs/media/create3-trials.json)

| 출발 배치 | 월드 X, Y (m) / yaw | 실제 이동 | 판독 결과 | 영상 시작 |
|---|---|---|---|---|
| 측면 | 3.190, 2.986 / -46.0° | 모서리 경유 후 두 번째 자리 | 성공 · 123가4567 | 0:00 |
| 후방 대각선 A | 0.725, 2.831 / -51.3° | 출발 보류 | 후보 없음 | 0:43 |
| 후방 대각선 B · 반대편 | 0.725, -2.670 / +51.3° | 범퍼를 볼 자리로 한 번 | 성공 · 123가4567 | 0:59 |
| A에서 중심선 쪽으로 40cm | 0.725, 2.431 / -51.3° | 범퍼를 볼 자리로 한 번 | 성공 · 123가4567 | 1:23 |

측면 출발에서는 모서리에서도 사진 8장 모두 미검출이었지만, 그 자리에서 다시 관측한 차체 끝면으로 두 번째 목표를 정해 판독했다. 후방 대각선 A에서는 끝면 후보가 잡혀도 마지막에 검토한 두 촬영 위치가 1.83m와 4.85m 떨어져 있어 첫 이동 한도 1.75m를 넘었다. 모서리 탐색 후보도 확정되지 않아 정지했다. A와 같은 방향으로 40cm 안쪽에 두고 다시 실행했을 때는 약 1.51m의 목표가 선택됐다.

영상은 전방 카메라 표본과 같은 시뮬레이션 시각의 오도메트리 궤적·LiDAR 윤곽을 실제 경과 시간 기준 4배속으로 보여준다. 좌표는 Isaac 월드 기준이고, A와 B는 차량 중심선의 양쪽에 있다. 추가 40cm 실험은 앱을 다시 연 뒤 실행했다. 이 표의 결과는 같은 차량과 번호판 한 종류에서 얻은 개별 주행 기록이다.

## 어떻게 다시 보는가

1. **먼저 현재 자리에서 읽는다.** 정지 상태에서 번호판 상자를 찾고 글자를 읽는다. 서로 다른 세 사진에서 같은 번호가 연속으로 나오고, 각 OCR 점수가 0.80 이상이며 촬영 시각과 상자 겹침 조건도 충족하면 성공이다.
2. **범퍼가 보이면 촬영 위치를 고른다.** 카메라 영상에서 CFV가 차량의 길이 방향을 추정한다. 단일 LiDAR로 측정한 차체 선분 중 이 방향에 가로놓인 면을 찾아, 그 면을 더 정면에서 볼 수 있는 자리를 계산한다.
3. **옆면만 보이면 모서리부터 탐색한다.** 이 옵션을 켜면 가까운 끝점을 지나 차량을 다시 바라본다. 거기서도 번호판이 읽히지 않으면 새 영상과 스캔으로 다음 자리를 고른다. 같은 끝면이 연속 세 번 확인되고 경로 간격을 확보했을 때 이동한다.
4. **도착한 자리의 사진으로 결과를 확정한다.** 출발 때와 같은 판독 기준을 쓴다. 적절한 후보가 없거나 주행 중 센서 조건이 깨지면 정지하고 그 이유를 기록한다.

C++ 노드가 LiDAR 윤곽과 조준점 후보를 만들고, 번호판 상자 검출과 연속 판독의 성공 여부를 판정한다. Python 통합 제어기가 CFV 방향 추정과 후보 선택, 이동·재관측 순서를 연결한다. 문자 인식은 Python PaddleOCR가 맡는다. [Create 3 주행 원리와 실측 그림](ros2_ws/src/amr_reobservation/docs/create3_motion_guide.md)

## 실행하기

실험 환경은 Ubuntu 22.04, ROS 2 Humble, Isaac Sim 5.1.0, C++17이다. Create 3에는 전방 Hawk RGB 카메라 한 대와 관측용 RPLidar S2E 한 대를 사용한다.

**처음 준비할 때** 프로젝트 루트에 장면이 참조하는 `sedan_car.usd`, 로컬 번호판 검출 모델 `assets/models/plate_kr_rtmdet_v4.onnx`, CFV 가중치와 PaddleOCR 환경을 준비한다. 차량 원본과 모델 가중치는 Git에서 제외되어 있다. 다른 PC에서는 `--config`로 CFV·OCR 환경 경로를, `--model-path`로 번호판 검출 모델 경로를 지정할 수 있다. [ROS 패키지 빌드 안내](ros2_ws/src/amr_reobservation/README.md) · [통합 실행과 의존성](ros2_ws/src/amr_reobservation/docs/single_candidate_ocr_recovery.md)

```bash
cd ~/peter_pjt
bash tools/setup_onnxruntime.sh
source /opt/ros/humble/setup.bash
(cd ros2_ws && colcon build --symlink-install --packages-select amr_reobservation)
./run_recovery_test.sh
```

**실험할 때** Isaac Sim에서 `amr_create3_camera_fail.usd`를 연다. Stop 상태에서 시작 자세를 정하고 Play를 누른 뒤, 터미널 한 곳에서 아래 명령을 실행한다. 차량의 월드 위치는 실행기가 현재 `odom`으로 변환하므로 로봇을 옮길 때마다 힌트를 다시 입력할 필요가 없다.

```bash
~/peter_pjt/run_recovery_test.sh --execute --corner-probe-once 1.50
```

`1.50`은 관측한 옆면의 끝점을 지나 추가로 갈 거리(m)다. `--corner-probe-once`만 주면 0.35m이고, 0.15·0.50·1.00m도 지정할 수 있다. 옵션을 빼면 처음부터 끝면을 확인할 수 있는 배치를 시험한다. 기본 속도 상한은 0.42m/s와 1.35rad/s다. 이전 iw.hub 배치와 비교하려면 `amr_camera_fail.usd`를 열고 `--robot iw_hub --speed fast`를 사용한다.

기본 번호판 상자 검출 문턱은 0.50이다. 비교용 기존 검출기는 `--model-path assets/models/semi_allimages_v5n.onnx`로 선택한다. 검출 점수는 번호판 상자에 대한 값이며, 글자를 읽은 뒤의 OCR 점수는 별도로 기록된다.

### 실험 기록 찾기

터미널 마지막 줄에 이번 실행의 폴더가 표시된다. 기본 경로는 `local_artifacts/recovery_runs/<실행 ID>/`다.

| 확인할 내용 | 실행 폴더 안의 파일 |
|---|---|
| 최종 상태, 번호, 중단 사유 | `result.json`의 `status`, `text`, `error` |
| 출발·모서리·최종 위치의 사진과 판독 | `controller/before_capture/`, `corner_capture/`, `after_capture/` 및 대응하는 `*_ocr/report.json` |
| 선택한 목표와 실제 이동·정지 시점 | `controller/selection.json`, `controller/events.jsonl` |
| LiDAR·오도메트리·시계·좌표 변환 | `sensors/` ROS 2 bag |

각 단계까지 도달한 경우에만 해당 사진과 OCR 기록이 생긴다. 차체 판정 반경 2.00m·2.10m와 경로 간격 1.20m는 이 시뮬레이션에서 검증한 값이다. [차량 좌표와 힌트](ros2_ws/src/amr_reobservation/docs/fixed_vehicle_hint.md) · [목표 선택과 정지 기준](ros2_ws/src/amr_reobservation/docs/create3_motion_guide.md)

## 단계별 개발 기록

- [2단계 실행·알고리즘 설명](ros2_ws/src/amr_reobservation/docs/stage2_vehicle_side.md): 측면 후보, 방향·길이·잔차, ROS 메시지와 RViz 표시.
- [3단계 실행·모델 준비·판정 기준](ros2_ws/src/amr_reobservation/docs/stage3_plate_observation.md): C++ 번호판 검출, 상태 메시지, 디버그 영상 및 입력 타임아웃.
- [CFV 관측 방향 모델 시험](ros2_ws/src/amr_reobservation/docs/cfv_orientation_test.md): 원본/crop 비교, 차량 축과 LiDAR를 이용한 후보 선택 및 현재 ROS 입력으로 제한 이동·복귀 비교. 11월 18일 한 후보 실행은 완료했으나 번호판 검출은 회복되지 않음.
- [4단계 평행 배치 검증](ros2_ws/src/amr_reobservation/docs/stage4_parallel_candidates.md): 관측 후보 기하, RViz 표시, 실측 결과와 남은 제약.
- [4단계 회전 비교](ros2_ws/src/amr_reobservation/docs/stage4_rotation_check.md): +10° 자세 → 평행 및 평행 → −10° 평균 비교 통과. 원본 후보의 순간 점프를 별도로 기록했다.
- [4단계 연속 관측 확인](ros2_ws/src/amr_reobservation/docs/stage4_candidate_stability.md): 원본 시각의 `odom` TF로 급변을 검사하고, 확인된 현재 후보만 출력한다. −10°·평행·+10° 정지 자세의 평균 비교와 급변 거부·재확인을 검증했다. 원본 측면 추정의 간헐적 무효화는 개선 과제로 남아 있다.
- `tools/`: Isaac Sim 장면에 번호판, 카메라 마스트, 관측 LiDAR를 설정하는 Python 도구.
- `amr_side_lidar_check.usd`: 차량 측면과 관측 LiDAR를 확인하는 장면.
- `assets/license_plate/`: 실험용 번호판과 배치 설정.

## 통합 주행 중 LiDAR 누락 기록

ROS bag과 함께 Isaac 내부의 GPU 데이터도 기록하려면 앱을 연 뒤 **Window → Script Editor**에서 아래 두 줄을 한 번 실행한다. 같은 앱에서 Stop → Play 하거나 실험을 반복할 때는 그대로 둔다.

```python
import runpy
runpy.run_path('/home/peter-msi/peter_pjt/tools/watch_recovery_lidar_in_gui.py')
```

Play 상태에서 평소처럼 `run_recovery_test.sh --execute`를 실행한다. ROS 스캔 또는 Isaac 내부의 완성 스캔에서 유효 점 수가 220개 미만이고 직전 최대 10개 스캔의 중앙값보다 25% 이상이자 30점 이상 적은 상태가 3회 이어지면 첫 감소 시각을 기록한다. 이 기준은 이번 배치의 약 400점에서 수십 점으로 줄던 구간을 찾기 위한 진단값이다. 시뮬레이션 시간이 0.5초 진행되는 동안 내부 완성 스캔 자체가 끊겨도 기록한다.

실험 폴더의 `lidar_internal/`에 감지 전 최대 3초와 감지 후 5초(실제 시간)의 GPU 원본·누적 배열을 저장한다. `frames.jsonl`의 GPU 오프셋으로 `gpu_payloads.bin`을 읽고, `scan_summary.jsonl`의 발행 시각과 `sensors/` bag의 LaserScan 헤더 시각을 맞춰 비교한다. `result.json`의 `lidar_internal.status`에서 `event_window_complete`는 전후 기록 완료, `run_ended_without_trigger`는 감지 없이 종료, `reader_unavailable`은 GUI 기록기 연결 실패를 뜻한다. GPU 복사 부하를 비교할 때는 `--no-lidar-internals`를 붙여 ROS bag만 기록할 수 있다.
