# 빠른 Monolith 로그 분석

원본 SD 카드의 `.log`를 CSV로 변환하지 않고 직접 읽습니다. Python 표준 라이브러리만 필요하며, 로그와 펌웨어를 수정하지 않습니다.

```powershell
python C:\HEVEN\HEVEN-vcu\tools\quick_monolith.py "C:\path\to\drive.log"
python C:\HEVEN\HEVEN-vcu\tools\quick_monolith.py "C:\path\to\*.log"
python C:\HEVEN\HEVEN-vcu\tools\quick_monolith.py "C:\path\to\drive.log" --json
python C:\HEVEN\HEVEN-vcu\tools\quick_monolith.py "C:\path\to\drive.log" --around 437.1 --json
```

PowerShell의 `*.log` 인수는 환경에 따라 자동 확장되지 않을 수 있습니다. 여러 파일은 경로를 공백으로 나열하세요.

요약: 최고속도, 좌우 동시시각 컨트롤러 버스전류 합·보고 DC 전력, 부하 중 최저 컨트롤러 전압, BMS 별도 측정값, TV·회생 적용 프레임 수, 핸드셰이크 프로브, 피드백 공백, 전압 급락, VCU 차단/fault 진단. `--json`에는 각 CAN ID의 마지막 관측 시각과 이벤트 원자료가 더 있습니다. `--around`는 지정 초 ±2초의 VCU 고속 로깅 프레임을 JSON에 포함합니다.

해석 주의:

- 버스전류 합은 좌우 컨트롤러 Part I 피드백을 150ms 이내로 맞춘 **보고값**입니다. BMS 전류는 별도 센서·갱신주기라 수치가 다를 수 있습니다.
- 보고 DC 전력은 `V_L × Ibus_L + V_R × Ibus_R`의 관측 최대값입니다. 대회 에너지미터 판정값이 아닙니다.
- 8V 급락은 각 컨트롤러의 연속 샘플 사이(최대 500ms)의 관측값입니다. BMS 차단·AIR 탈락·배선 전압강하 중 원인을 단독 확정하지 않습니다.
- 차단 `first`는 해당 구간 최초 **복합 비트**입니다. `last_block`의 역사적 `first`를 현재 원인으로 오해하지 마세요.
- CAN ID가 기록되지 않은 값은 `기록 없음`입니다. 업로드된 펌웨어 버전에 진단 프레임이 없거나 로거 필터가 막을 수 있습니다.
- 로그 레코드 시각(초)을 그대로 출력합니다. 파일 이름의 시각이나 실제 시계와 혼동하지 마세요.

기존 `decode_log_frames.py`의 VCU 고속 프레임 해독을 그대로 재사용합니다. CAN 필드가 바뀌면 먼저 해당 디코더 및 프로토콜 계약을 갱신해야 합니다.

2026-09-28 오류 복구 단순화 이후 `0x1C08C0D0` Byte7 bit0은 현재 오류/정상 2회 대기를 뜻합니다. 과거 펌웨어에서는 래치였으므로 반드시 업로드 버전과 대조합니다. 분석기는 `fault_blocked`를 제공하고 `latched`는 기존 분석 스크립트 호환 별칭으로 남깁니다. Byte0~6 최초 오류 기록이 남아 있어도 bit0=0이면 현재 오류 차단은 해제된 상태입니다. Byte7 bit1(옛 rearmReady)은 새 펌웨어에서 항상0입니다.
