# Google Vision OCR 비교 준비 및 접근 오류 기록

실행일: 2026-11-19 · 브랜치: `feature/reobservation-recovery`

**비교를 실행했으나 Google 서비스 접근이 거부되어 OCR 결과를 얻지 못했다.**
첫 이미지 요청이 HTTP 403으로 실패했고, 나머지 10건은 전송하지 않았다.
이를 판독 실패 0/11이나 Google OCR의 정확도 결과로 해석하면 안 된다.

## 확인한 오류와 현재 한계

```text
HTTP 403
status: PERMISSION_DENIED
reason: ACCESS_DENIED
service: vision.googleapis.com
message: Permission denied on the service.
```

응답에는 어느 프로젝트 설정 때문에 거부되었는지 구체적인 사유가 없었다.
키의 제한, API 사용 설정, 결제 연결 등의 실제 상태는 로컬 코드만으로 확인하지 못했다.
인증 문제가 해결되기 전에는 모델 비교 결과를 낼 수 없다.

- 기존 코드: `/home/peter-msi/DoosanPjt/doosan_intel/Alley_Park_Patrol/detect/plate_ocr_node.py`
- 해당 코드의 `GCP_API_KEY`를 AST로 읽어 메모리에서만 사용했다. ROS 노드를 실행하지 않았다.
- `test_ocr_only.py` 등 관련 테스트 파일 5개도 같은 키를 사용했다. 별도 대체 키는 발견하지 못했다.
- 해당 프로젝트 파일 검색에서 별도 인증 JSON·환경 파일은 발견하지 못했다.
- 현재 프로세스의 `GOOGLE_VISION_API_KEY`, `GOOGLE_APPLICATION_CREDENTIALS`,
  `GOOGLE_CLOUD_PROJECT`는 미설정이며, 기본 gcloud ADC 파일도 없었다.
- 인증값은 새 소스·manifest·실험 결과에 복사하지 않았다. 결과 저장 전 키 값도 마스킹한다.
- Google 프로젝트 설정·결제·기존 외부 소스는 변경하지 않았다.

Google Cloud 프로젝트 접근 권한이 있는 기존 OCR 담당자가 API 활성화, 결제 연결,
키의 허용 서비스 및 실행 환경 제한을 확인해야 한다. 현재 오류만으로 키 만료 등
특정 원인을 단정하지 않는다. 키를 채팅이나 Git에 전달할 필요는 없다.

## 고정한 비교 조건

[11월 18일 EasyOCR 시험](ocr_validation.md)의 이동 후 사진 5장과 원본 번호판
텍스처 1장을 사용한다. 정답은 `123가4567`이며 API 요청에는 포함하지 않는다.
기존 관측의 검출 상자를 재사용하며 YOLO 검출을 다시 실행하지 않는다.

| 조건 | 입력 준비 | 전송 시도 | OCR 채점 완료 |
|---|---:|---:|---:|
| 어제와 동일한 이동 후 crop PNG | 5장 | 1건 → 접근 거부 | 0건 |
| 동일한 원본 텍스처 PNG | 1장 | 0건 | 0건 |
| 기존 코드의 전처리를 적용한 이동 후 crop | 5장 | 0건 | 0건 |

1. **동일 crop 비교:** 어제 CRAFT+EasyOCR에 넣었던 PNG 파일을 그대로 복사했다.
   SHA-256 및 원본 영상에서 자른 픽셀과의 일치를 확인했다. 엔진별 입력 차이를 줄이는 조건이다.
2. **기존 전처리 비교:** 저장된 검출 상자의 xyxy 좌표를 정수로 절삭하고, 여백 30px를
   추가해 영상 경계로 제한한다. OpenCV `INTER_CUBIC`으로 8배 확대 후 기본 JPEG
   품질 95로 저장한다. 기존 코드의 전처리와 API 설정을 재현하지만, 기존 검출기를 포함한
   전체 파이프라인의 재현은 아니다.
3. **텍스처 대조:** 원본 2080×440 PNG만 시험한다. 이를 다시 8배 확대하는 조건은 제외했다.

Google에는 `DOCUMENT_TEXT_DETECTION`, `languageHints=["ko"]`를 사용했다.
REST `images:annotate`를 호출하며 API 키는 URL이 아닌 `x-goog-api-key` 헤더에 넣는다.
Google 서비스 기본 모델을 사용하므로 로컬 모델처럼 가중치 버전을 고정하지 못한다.

엄격한 채점은 어제와 동일하게 Unicode NFC·공백 제거만 적용한다.
별도로 기존 `normalize_plate`의 숫자/한글 이외 문자 제거 결과도 저장한다.
괄호 제거로 맞아진 결과를 엄격한 일치 결과와 섞지 않는다.
신뢰도는 기존 코드와 동일하게 반환된 문자 symbol confidence의 산술평균으로 계산한다.
이는 EasyOCR 신뢰도와 동일 척도로 보장되지 않는다.

## 보존한 파일과 검증

- 도구: `tools/probe_google_plate_ocr.py`
- 테스트: `tools/test_probe_google_plate_ocr.py`
- [입력 manifest](../../../../local_artifacts/google_ocr_validation_2026-11-19/manifest.json)
- [실행 결과 및 오류](../../../../local_artifacts/google_ocr_validation_2026-11-19/run/report.json)
- 전송 입력: `local_artifacts/google_ocr_validation_2026-11-19/inputs/`
- 원본 API 응답: 같은 폴더의 `run/*_response.json` — 키 값 제거 후 보존.

이미지·기준 보고서 해시, crop 범위, 크기, 전처리, 요청 시각, HTTP 상태,
네트워크 왕복 시간을 기록했다. 첫 요청은 약 283ms였으나 오류 응답이므로
OCR 처리 시간으로 사용할 수 없다.

```bash
python3 -m unittest discover -s tools -p 'test_probe*ocr.py'
```

총 12개 테스트 통과. crop/채점, 요청에 정답 미포함, 기존 정규화와 엄격한 채점의 분리,
인증 소스 비실행, 키 마스킹, 403 이후 추가 요청 중단 및 기존 실행 덮어쓰기 방지를 검증했다.
테스트에서는 실제 API를 호출하지 않았다.

## 인증 해결 후 재실행

동일 입력으로 다시 비교하되, 이전 실패 기록을 보존하도록 새 실행 폴더를 지정한다.
아래 명령은 실제 Google Vision 요청을 전송하므로 인증 설정을 해결한 뒤 실행한다.

```bash
python3 tools/probe_google_plate_ocr.py run \
  --manifest local_artifacts/google_ocr_validation_2026-11-19/manifest.json \
  --key-source-python /home/peter-msi/DoosanPjt/doosan_intel/Alley_Park_Patrol/detect/plate_ocr_node.py \
  --output local_artifacts/google_ocr_validation_2026-11-19/run_02
```

실행 프로세스에 `GOOGLE_VISION_API_KEY`가 설정되어 있으면 그 값을 우선 사용한다.
자동 재시도는 없으며, 첫 API/네트워크 오류에서 남은 요청을 중단한다.
원본 입력 준비가 필요하면 `prepare --baseline <어제 report.json> --output <새 폴더>`를 사용한다.

이번에도 한 차량·동일 번호판의 소수 저장 사진이며, 이동 후 사진에는 디버그 검출 상자가 있다.
인증이 해결되어 좋은 결과가 나오더라도 원본 카메라 영상·다른 번호판·연속 프레임 검증은 별도다.

공식 참고:
[Vision OCR](https://docs.cloud.google.com/vision/docs/ocr),
[REST images:annotate](https://docs.cloud.google.com/vision/docs/reference/rest/v1/images/annotate),
[API 키 헤더 사용](https://docs.cloud.google.com/docs/authentication/api-keys-use),
[Vision 설정](https://docs.cloud.google.com/vision/docs/setup),
[API 키 제한](https://docs.cloud.google.com/api-keys/docs/add-restrictions-api-keys).
