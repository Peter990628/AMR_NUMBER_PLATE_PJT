# AMR_NUMBER_PLATE_PJT

Isaac Sim과 ROS 2를 이용한 C++ AMR 번호판 재관측 프로젝트. 번호판 관측이 실패했을 때 센서 정보로 재접근 위치를 정하고, 이동 후 다시 관측하는 알고리즘을 개발한다.

현재 구현은 **1단계: 관측 LiDAR 전처리와 RViz 표시**까지다. 차량 측면 추정, 재관측 계획, 자동 이동 제어는 이후 단계다.

- 환경: Ubuntu 22.04, ROS 2 Humble, Isaac Sim 5.1, C++17.
- [`ros2_ws/src/amr_reobservation`](ros2_ws/src/amr_reobservation/README.md): C++ 코드, 빌드·실행 방법, 테스트, 파라미터 설명.
- `tools/`: Isaac Sim 장면에 번호판, 카메라 마스트, 관측 LiDAR를 설정하는 Python 도구.
- `amr_side_lidar_check.usd`: 차량 측면과 관측 LiDAR를 확인하는 장면.
- `assets/license_plate/`: 실험용 번호판과 배치 설정.

차량 원본 및 변환 에셋은 저장소에 포함하지 않는다. 장면이 참조하는 `sedan_car.usd`는 로컬에 별도로 준비해야 한다. C++ 노드 자체는 `/observation_lidar/scan`을 제공하는 다른 장면이나 기록 데이터로도 실행할 수 있다.
