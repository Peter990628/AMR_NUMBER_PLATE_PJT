# 측면 추정의 실패 원인·반복 횟수 기록

`vehicle_side_estimator_node`는 처리한 스캔마다 `/debug/vehicle_side/fit_diagnostics`를 발행한다. 빈 입력·잘못된 metadata·빈 frame도 진단 결과를 발행한다. 입력 자체가 중단되면 새 진단도 생성하지 않는다. **PCA 반복 상한은 시작 파라미터 `refinement_limit`이며 기본 20회, 허용 범위 1~100회**다. 품질 조건과 최종 후보 선택은 그대로 유지한다.

## 적용과 실시간 확인

새 메시지를 포함해 패키지를 빌드하고 기존 추정기 또는 후보 launch 터미널을 `Ctrl+C`로 종료한 후 다시 실행한다. 이미 실행 중인 노드에는 빌드한 코드가 자동 적용되지 않는다. 추정기를 중복 실행하지 않는다.

```bash
cd ~/peter_pjt/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amr_reobservation
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch amr_reobservation observation_candidates.launch.py
```

Isaac Sim을 Play한 상태에서 별도 터미널:

```bash
source /opt/ros/humble/setup.bash
source ~/peter_pjt/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic echo /debug/vehicle_side/fit_diagnostics --once
```

진단 토픽명은 `config/vehicle_side_estimator.yaml`의 `fit_diagnostics_topic`으로 변경할 수 있다. 기존 `/vehicle_side/estimate` 메시지의 구조와 무효 결과의 기하 초기화 규칙은 그대로다.

같은 YAML의 `refinement_limit`을 수정하고 노드를 재시작하면 반복 상한을 변경할 수 있다. 실행 중 `ros2 param set`은 허용하지 않는다. 7회에 수렴하는 입력은 상한이 20이어도 7회에 끝난다. RANSAC 시도 수와는 독립된 설정이다. 현재 값은 다음과 같이 확인한다.

```bash
ros2 param get /vehicle_side_estimator refinement_limit
```

후보 launch에서 다른 추정기 YAML을 지정하려면 `estimator_params_file:=/절대/경로/설정.yaml`을 사용한다. 개별 측면 추정 launch는 기존 `params_file` 인자를 사용한다. 두 launch를 동시에 실행하지 않는다.

## 기록 항목

상위 메시지의 `header`, `valid`, `status`, 점·클러스터 수는 같은 스캔의 추정 결과와 일치한다. `selected_cluster_index`는 선택된 클러스터의 번호이며 무효이면 `-1`이다. `processing_time_ms`는 steady wall clock으로 측정한 **전처리와 추정 계산 시간**이다. 전송 대기·마커 생성·ROS 발행 시간은 포함하지 않는다.

`clusters`에는 작은 클러스터를 포함해 각 점 묶음의 진단이 하나씩 들어간다.

| 필드 | 해석 |
|---|---|
| `reason` | 해당 클러스터의 최종 성공·탈락 이유 |
| `selected` | 전체 후보 중 이 클러스터가 선택됐는지 여부 |
| `ransac_iterations` | 실제 RANSAC 시도 수. 중복 점·퇴화 표본도 한 번의 시도로 계산 |
| `refinement_iterations` | 실제 PCA 호출 수. PCA 호출이 실패해도 한 번으로 계산 |
| `refinement_limit` | 이 클러스터에 적용한 PCA 반복 상한. 기본 20 |
| `converged` | PCA 재계산 전후의 inlier **점 집합**이 동일했는지 여부 |
| `fit_inlier_count` | 마지막 PCA 호출에 사용한 점 수. 호출 전 탈락했다면 0 |
| `inlier_count`, `inlier_ratio` | 가장 최근에 얻은 점 집합의 크기와 클러스터 내 비율 |
| `reclassification_available` | 마지막 PCA 호출 뒤 점 집합을 다시 골랐는지 여부 |
| `inlier_change_count` | 마지막 재분류에서 소속이 바뀐 점 수. 빠진 점과 새로 들어온 점을 모두 셈 |
| `metrics_available` | 수렴 후 유한한 길이·RMSE까지 계산했는지 여부 |
| `observed_length_m`, `rmse_m` | 해당 클러스터의 관측 길이·잔차. 위 플래그가 false이면 0은 측정값이 아닌 기본값 |

예를 들어 기존 100개의 inlier 중 2개가 빠지고 2개가 추가되면 개수는 그대로여도 `inlier_change_count=4`이고 수렴이 아니다. `reclassification_available=false`일 때 변화량 0을 수렴으로 해석하지 않는다. `converged=true`여도 길이·잔차 등 후속 품질 검사에서 탈락할 수 있다. `reason=ok`여도 다른 가까운 후보가 선택되면 `selected=false`다.

| reason | 의미 |
|---|---|
| `ok` | 클러스터 적합과 품질 검사 통과 |
| `cluster_too_small` | 최소 클러스터 크기 미달, RANSAC/PCA 미실행 |
| `insufficient_inliers` | PCA 시작에 필요한 점 수 미달 |
| `pca_failed` | PCA의 퇴화·수치 계산 실패 |
| `refinement_limit` | 설정한 반복 상한 안에 점 집합이 수렴하지 않음 |
| `low_inlier_ratio` | 수렴했지만 inlier 비율 미달 |
| `nonfinite_metrics` | 길이 또는 RMSE가 유한하지 않음 |
| `too_short` | 관측 길이 미달 |
| `rmse_exceeded` | RMSE 상한 초과 |

검사 순서상 가장 먼저 만난 탈락 이유를 남긴다. 예를 들어 비율 검사에서 탈락하면 길이·잔차는 계산하지 않아 `metrics_available=false`다. 빈 입력 등 클러스터 생성 이전의 실패는 상위 `status`와 빈 `clusters`로 표현한다.

## 파일 저장

기존 후보 기록 도구는 기본으로 진단 토픽도 구독한다. 최종 후보가 무효이거나 TF가 없어도 수신한 진단을 보관한다. 진단 자체에는 TF 변환이 필요하지 않다.

```bash
cd ~/peter_pjt
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/amr_fit_diagnostics_logs
python3 tools/record_candidate_stability.py \
  --duration 30 \
  --raw-topic /observation_candidates/raw \
  --stable-topic /observation_candidates \
  --output-dir local_artifacts/fit_diagnostics_run01
```

출력 폴더는 재수집할 때마다 새 이름을 쓴다. `local_artifacts/`는 Git 공유 대상에서 제외된다.

- `streams.jsonl`: 수신한 모든 스캔·추정·후보·진단. 진단 항목은 `stream=fit_diagnostics`이며 무효 프레임도 포함한다.
- `paired.jsonl`: 동일한 clock epoch·frame·원본 스캔 시각의 자료를 연결한다. 진단이 없어도 기존 측면·후보 대응은 유지한다.
- `report.json`: 진단의 상태·원인별 건수, 성공·실패별 반복 수, 처리 시간 요약. 클러스터의 탈락 건수와 스캔 전체의 실패 건수는 별개다.

모든 수신 메시지는 파일에 저장하지만 대응 분석은 기본 최대 3,000개씩의 보관 창을 사용한다. 전체 수집 통계와 보관 창 통계를 구분해서 읽는다. 새 진단 토픽이 없으면 진단 수집 결과는 없음으로 표시하며, 이를 실패 0건으로 해석하지 않는다. 과거 버전의 토픽만 수집하려면 `--fit-diagnostics-topic ''`로 진단 구독을 끌 수 있다.

노드 터미널에는 약 5 wall초마다 스캔 상태별·클러스터 이유별 건수, PCA 최대 반복 수, 계산 시간 평균·최댓값을 출력한다. 입력이 없을 때는 이 요약도 출력하지 않는다. 터미널 요약은 개별 메시지 파일을 대신하지 않는다. DDS에서 수신하지 못한 스캔의 계산 결과까지 기록할 수는 없으므로 메시지 수와 대응 수를 함께 확인한다.

## 진단 최초 구현 검증 (2026-11-12, 당시 상한 10회)

- 패키지 빌드와 C++ 단위 테스트 86개 통과. 측면 추정 테스트 26개 중 11개가 진단용이다. 미수렴, 점 교환으로 개수만 같은 상태, PCA 실패, 후속 품질 거부, 작은 클러스터 건너뛰기를 포함한다.
- 기록 도구 Python 테스트 15개 통과. 실제 콜백/파일 경로를 모의 ROS로 실행해 모든 수신 진단 저장, 제한된 보관 창, TF 없는 진단, 선택적 구독 해제를 검사했다.
- 격리 ROS 도메인 88에서 설치된 노드·launch에 합성 스캔을 넣어 성공/실패, 정확한 헤더 대응, 진단 초기화, 파라미터 계약 검사 통과. Isaac 장면의 실행 중 노드는 재시작하지 않았다.
- 기존 +10° 기록의 91개 스캔을 실제 C++ 코드로 재생했다. 상태·점 수·클러스터 수와 유효 결과의 inlier 수·비율·길이·RMSE·양 끝점 출력이 이전 결과 파일과 바이트 단위로 동일하다. 79개 유효, 12개 `no_valid_line`을 유지한다.
- 총 97개 클러스터 진단은 `ok=79`, `refinement_limit=12`, `cluster_too_small=6`이다. 계산한 91개 클러스터의 반복 수·마지막 PCA 입력 점 수·재분류 점 수·소속 변화량은 이전 임시 진단 trace와 모두 일치했다.

재생 감사 결과는 `local_artifacts/fit_diagnostics_implementation_2026-11-12/replay_audit.json`에 저장했다. 위 결과는 상한을 늘리기 전 진단 구현 시점의 기록이다. 이후 같은 91개 스캔의 10·20·30회 비교에서 20과 30의 추정 결과·실제 반복 수가 같고 최대 15회에 수렴하여, 기본 상한을 20회로 선택했다. 이 오프라인 시간 측정은 `local_artifacts/refinement_cap_benchmark_2026-11-12/report.json`에 있다.

## 기본 상한 20회 적용 검증 (2026-11-12)

`SideEstimatorConfig`, ROS 시작 파라미터, 기본 YAML에 `refinement_limit=20`을 적용했다. 허용 범위는 1~100이며 실행 중 변경은 거절한다. 진단은 실제 적용된 값을 작은 클러스터에도 기록한다. 후보 launch의 `estimator_params_file` 인자로 다른 추정기 설정을 선택할 수 있다.

C++ 테스트 총 89개와 격리 ROS 도메인 88의 기본값 20·별도 상한 1 테스트를 통과했다. 상한 0·101 거절, 마지막 허용 반복에서 수렴 성공, 조기 종료, 기존 품질 거부와 무효 결과 초기화가 유지됨을 확인했다.

### 같은 저장 입력의 회귀 비교

실제 수정한 추정기에 파라미터만 10·20으로 바꿔 저장된 원본 스캔을 재생했다.

| 장면 | 전체 스캔 수 | 유효: 상한 10 | 유효: 상한 20 | 상한 20에서 실제 최대 반복 |
|---|---:|---:|---:|---:|
| +10° | 91 | 79 | 91 | 15 |
| 평행 복귀 | 91 | 85 | 91 | 13 |
| −10° | 89 | 81 | 89 | 16 |

기존 유효 245개 결과는 중심·축·법선·양 끝점·길이·잔차·비율·선택 클러스터·inlier 인덱스까지 정확히 동일하다. 실패했던 26개가 추가로 수렴했다. 이는 전체 저장 스캔 기준이므로 과거 수집 보고서의 대응된 스캔 수와 경계 부분에서 차이가 있다. 재현 프로그램과 감사 결과는 `local_artifacts/refinement_limit_live_2026-11-12/offline_replay/`에 있다.

### 현재 Isaac Sim 장면의 30초 전후 수집

동일한 기본 설정에서 상한만 10·20으로 바꾸고 각각 30 wall초 동안 수집했다. 최초 확인 때 추정 노드가 꺼져 있어 LiDAR만 기록된 자료는 `input_only_preflight/`로 분리했으며 전후 비교에 사용하지 않았다. 실제 비교는 새 빌드의 상한 10을 실행한 `before_10/`, 기본값 20을 실행한 `after_20/`이다.

| 항목 | 상한 10 | 상한 20 |
|---|---:|---:|
| 대응된 스캔·측면·후보·진단 | 122개 | 122개 |
| 측면 추정 유효 | 103/122 (84.4%) | 122/122 (100%) |
| 미수렴으로 측면 추정 실패 | 19개 | 0개 |
| 후속 확인을 통과한 후보 | 60/122 (49.2%) | 91/122 (74.6%) |
| 전처리+추정 시간 평균 | 0.849ms | 0.892ms |
| 전처리+추정 시간 p95 | 1.052ms | 1.110ms |
| 전처리+추정 시간 최댓값 | 1.399ms | 2.684ms |
| 유효 후보 간 최대 source 시간 간격 | 1.0초 | 0.5초 |

시간 측정은 ROS 전송·TF 대기·후속 후보 확인을 제외한다. 전후 수집은 서로 다른 잡음·점 분포·시스템 부하에서 얻은 관측값이므로 시간 차이를 상한 변경만의 효과로 단정하지 않는다. 상한 20 측정 중 한 스캔은 정확히 20회에 수렴했다. 이 구간의 실패 0건이 모든 장면에서 수렴함을 보장하지 않는다.

AMR은 두 수집에서 odom yaw 약 9.692°를 유지했다. 평균 위치 차이는 약 1.22mm, 각 구간 평균 대비 위치 편차는 최대 약 0.22mm였다. 로봇 이동 명령이나 시뮬레이터 배치 변경은 하지 않았고 clock 역행과 중복 발행자도 없었다.

20회 측정의 남은 무효 후보 31개는 급변 판정 18개와 재확인 대기 13개다. 측면 직선이 유효해도 끝점 변화로 후보를 거부할 수 있으므로 후보 확인 조건(3회 연속, 위치 15cm, 각도 3°)은 그대로 유지했다. 채택한 후보는 현재 원본과 정확히 같은 기하였고, 무효 출력은 모두 빈 기하였다.

전후 모두 122개의 원본 header가 정확히 대응됐다. 20회 수집에서는 첫 한 스캔의 recorder TF가 없어 측면·원본·확인 결과의 좌표 변환 3건이 누락됐다. 위치 감사는 나머지 121개를 사용했으며 진단·실패율·처리 시간은 122개 모두를 포함한다. 또한 종료 경계의 스캔 1개만 추가로 수신되어 원본 스캔 수는 123개지만 downstream 비교 분모는 122개다.

상세 결과·재검증 스크립트는 `local_artifacts/refinement_limit_live_2026-11-12/live_comparison.json`, `audit_live.py`에 있다. 기본 20회 설정으로 실제 노드를 실행해 확인했다. 이후 작업은 남은 후보 급변과 끝점 변동을 이 진단과 함께 분석하는 것이다.
