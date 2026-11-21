# AMR_NUMBER_PLATE_PJT

Isaac Sim과 ROS 2를 이용한 C++ AMR 번호판 재관측 프로젝트. 번호판 관측이 실패했을 때 센서 정보로 재접근 위치를 정하고, 이동 후 다시 관측하는 알고리즘을 개발한다.

준비된 실험 PC에서 Isaac Sim에 `amr_camera_fail.usd`를 열고 실패 배치에서 Play한 뒤,
터미널에서 다음 명령을 실행한다. 현재 위치에서 번호판을 읽을 수 없으면 CFV와 LiDAR로
재관측 후보를 선택하고, 이동 후 OCR을 다시 확인한다.

```bash
~/peter_pjt/run_recovery_test.sh --execute
```

실행할 때마다 `local_artifacts/recovery_runs/`에 전후 사진, 판독 결과, 이동·센서 기록을 저장한다.
기본 차량 힌트의 좌표계와 다른 배치에서의 실행 방법은 [통합 실험 안내](ros2_ws/src/amr_reobservation/docs/single_candidate_ocr_recovery.md)를 참고한다.

- 환경: Ubuntu 22.04, ROS 2 Humble, Isaac Sim 5.1, C++17.
- [`ros2_ws/src/amr_reobservation`](ros2_ws/src/amr_reobservation/README.md): C++ 코드, 빌드·실행 방법, 테스트, 파라미터 설명.
- [2단계 실행·알고리즘 설명](ros2_ws/src/amr_reobservation/docs/stage2_vehicle_side.md): 측면 후보, 방향·길이·잔차, ROS 메시지와 RViz 표시.
- [3단계 실행·모델 준비·판정 기준](ros2_ws/src/amr_reobservation/docs/stage3_plate_observation.md): C++ 번호판 검출, 상태 메시지, 디버그 영상 및 입력 타임아웃.
- [CFV 관측 방향 모델 시험](ros2_ws/src/amr_reobservation/docs/cfv_orientation_test.md): 원본/crop 비교, 차량 축과 LiDAR를 이용한 후보 선택 및 현재 ROS 입력으로 제한 이동·복귀 비교. 11월 18일 한 후보 실행은 완료했으나 번호판 검출은 회복되지 않음.
- [4단계 평행 배치 검증](ros2_ws/src/amr_reobservation/docs/stage4_parallel_candidates.md): 관측 후보 기하, RViz 표시, 실측 결과와 남은 제약.
- [4단계 회전 비교](ros2_ws/src/amr_reobservation/docs/stage4_rotation_check.md): +10° 자세 → 평행 및 평행 → −10° 평균 비교 통과. 원본 후보의 순간 점프를 별도로 기록했다.
- [4단계 연속 관측 확인](ros2_ws/src/amr_reobservation/docs/stage4_candidate_stability.md): 원본 시각의 `odom` TF로 급변을 검사하고, 확인된 현재 후보만 출력한다. −10°·평행·+10° 정지 자세의 평균 비교와 급변 거부·재확인을 검증했다. 원본 측면 추정의 간헐적 무효화는 개선 과제로 남아 있다.
- `tools/`: Isaac Sim 장면에 번호판, 카메라 마스트, 관측 LiDAR를 설정하는 Python 도구.
- `amr_side_lidar_check.usd`: 차량 측면과 관측 LiDAR를 확인하는 장면.
- `assets/license_plate/`: 실험용 번호판과 배치 설정.

3단계 빌드에는 프로젝트 로컬 ONNX Runtime C++ 배포판이 필요하다. `bash tools/setup_onnxruntime.sh`로 준비한다. 모델은 기존 `ALLEY_PARK_PATROL_PJT`의 사용자 보유 가중치를 재사용하며, 원본/ONNX 모델과 다운로드한 런타임은 Git에 포함하지 않는다. 준비 및 검증 방법은 3단계 문서를 참고한다.

차량 원본 및 변환 에셋은 저장소에 포함하지 않는다. 장면이 참조하는 `sedan_car.usd`는 로컬에 별도로 준비해야 한다. C++ 노드 자체는 `/observation_lidar/scan`을 제공하는 다른 장면이나 기록 데이터로도 실행할 수 있다.
