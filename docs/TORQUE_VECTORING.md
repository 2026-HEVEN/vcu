# 토크벡터링 개발 가이드 (팀원용)

토크벡터링(TV)은 이 펌웨어에서 **개발할 게 가장 많은 부분**입니다. 그래서 한 함수에 몰아넣지 않고
**5개 stage(순수 함수)** 로 나눴습니다. 각 stage는 입력→출력이 명확해서 **한 명씩 맡아 병렬로**
개발하고, 하드웨어 없이 노트북에서 테스트할 수 있습니다.

## 파이프라인 한눈에

```
[1 reference]  조향각 + 차속            → 목표 yaw rate        (바이시클 모델)
[2 yaw]        목표 yaw − IMU 실측 yaw   → 요 모멘트 Mz         (PID, 이력 필요)
[3 load]       종/횡 가속도             → 바퀴별 수직하중 Fz    (하중이동, 모델 기반)
[4 traction]   Fz × μ (마찰원)          → 바퀴별 최대 종토크    (휠스핀 방지)
[5 allocation] 총토크 + Mz, 상한 제약    → torque_L / torque_R  (최종 배분)
```

`src/modules/torque_vectoring.cpp`(오케스트레이터)가 이 5개를 순서대로 호출합니다.
**오케스트레이터와 각 `.h`(입출력 계약)는 건드리지 말고**, 아래 `.cpp` 본문만 채우세요.

## 파일 ↔ 담당 ↔ 테스트

| Stage | 채울 파일 | 함수 | 테스트 |
|-------|-----------|------|--------|
| 1 레퍼런스 | `src/modules/tv/reference.cpp` | `tv_reference_compute` | `test/test_tv_reference/` |
| 2 yaw 제어 | `src/modules/tv/yaw_control.cpp` | `tv_yaw_compute` | `test/test_tv_yaw/` |
| 3 하중 추정 | `src/modules/tv/load.cpp` | `tv_load_compute` | `test/test_tv_load/` |
| 4 트랙션 | `src/modules/tv/traction.cpp` | `tv_traction_compute` | `test/test_tv_traction/` |
| 5 배분 | `src/modules/tv/allocation.cpp` | `tv_alloc_compute` | `test/test_tv_alloc/` |

> 각 `.cpp` 맨 위 `담당: ______` 칸에 이름을 적고, 그 stage의 owner가 되세요.

## 지금 상태 = 안전한 stub

5개 다 **안전한 pass-through** 로 채워져 있습니다: 레퍼런스=0, Mz=0, 하중=좌우 동일,
트랙션=무제한, 배분=50:50. → **조립 결과가 지금 차 거동과 100% 동일**하므로, 여러분이
한 stage씩 채워도 나머지는 안전하게 통과만 시킵니다. 하나씩 점진적으로 켜세요.

## 튜닝 상수는 한 곳에서

차량 제원·μ·PID 게인 등은 전부 **`src/modules/tv/tv_config.h`의 `TVParams`** 에 있습니다.
각 `.cpp`에 상수를 흩뿌리지 말고 여기서만 바꾸세요. (지금 값은 전부 임시 placeholder — 실측 필요)

## 중간신호는 눈에 보입니다

각 stage의 중간 출력(목표 yaw, Mz, 바퀴별 Fz, 최대토크)은 `VehicleState`로 복사되어
**시리얼 debug monitor(5Hz)와 Cluster에서 실시간 관측**됩니다. yaw 제어기 튜닝 시 필수예요.
`pio device monitor` 로 `TV: yaw*=… Mz=… Fz(L/R)=… Tmax(L/R)=…` 줄을 보세요.

## 작업 사이클

1. 자기 stage의 `.cpp` 본문 작성 (`.h`는 읽기만)
2. 같은 이름의 `test/` 에 불변식 테스트 추가 (파일 안 `TODO` 목록 참고)
3. `pio test -e native -f test_tv_<stage>` 로 통과 확인
4. `pio run -e esp32dev` 빌드 그린 확인 → PR

## 규칙 (코어와 동일)

- stage `.cpp` 에서 `state.h` / `<Arduino.h>` **include 금지** (native 빌드가 깨짐 = 방어장치)
- 이력(적분/필터)은 전역변수 대신 **상태 struct 인자**로 (yaw 제어기의 `TVYawState`)
- 입출력은 도메인 타입(`Percent`/`Unit`/`Rpm`)으로 — 범위 자동 보장

## 단계별 부호 규약 (먼저 합의할 것)

reference/yaw/load/allocation이 **같은 좌표계·부호 규약**을 써야 합니다(예: 좌회전 = yaw +).
개발 시작 전에 TV팀이 이 규약 하나를 못 박고 각 `.cpp` 주석에 적어두세요.

## 참고자료 (초심자용 학습 로드맵)

> 📌 이 섹션은 노션 자료 DB의 **[토크벡터링 입문 가이드 + 학습 자료 모음](https://app.notion.com/p/3a7913e532e6810995bbff35fee8912f)** 페이지와 **동일한 내용**입니다. (노션 = 팀 공용, 이 파일 = 코드 옆에서 바로 참고용) 링크 추가·수정은 편한 쪽에서 하고 다른 쪽에도 반영해 주세요.

제어(PID)·차체 거동 모델링·타이어 마찰원·회생 제어를 처음 접하는 팀원용. **추천 학습 순서: Stage 2(PID) → Stage 1(reference) → Stage 3/4 → Stage 5.** PID를 이해하면 나머지가 "입력을 만들어주는 모델"로 보입니다.

**0단계 — 전체 그림 (제일 먼저, 쉬운 것부터)**
- [한국어 기사] 오토카코리아 「매력적인 기술, 토크 벡터링의 세계」 — 가장 부담 없는 입문: https://www.iautocar.co.kr/news/articleView.html?idxno=31309
- [유튜브·한국어] 「코너링이 자동으로 되는 트윈클러치의 마법 (feat. 토크 벡터링)」: https://www.youtube.com/watch?v=YAlVv9FbRws
- [유튜브·영어, 애니메이션] Engineering Explained 「Torque Vectoring Differential - Explained」: https://www.youtube.com/watch?v=qwwFZAbYGW0
  - ⚠️ 위 영상은 **기계식(클러치 디퍼렌셜)** 예시입니다. 직관("좌우 바퀴 토크 차이로 코너링을 돕는다")은 그대로 우리 것이지만, 우리는 클러치가 아니라 **좌우 독립 모터 2개**로 구현합니다(Stage 5 allocation).
- [심화·준비되면] Politecnico di Torino 석사논문 「Yaw Control for 4WD FS EV」 — 우리와 거의 똑같은 제어 구조(목표 yaw rate → PID → 요모멘트 → 토크배분). 영어 논문이라 진입장벽은 있지만 **그림 위주로만 봐도** 우리 5-stage가 그대로 보입니다: https://webthesis.biblio.polito.it/28820/1/tesi.pdf

**Stage 2 (yaw PID) — 제어 초심자에게 가장 중요, 제일 먼저**
- Brian Douglas 유튜브 채널 (제어 입문 최고 명강, 애니메이션 직관적): https://www.youtube.com/channel/UCq0imsn84ShAe9PBOFnoIrg
- MATLAB 「Understanding PID Control」 1~4편 (Brian Douglas 제작, 한글자막) — P/I/D 역할 + anti-windup(`TVYawState.integral` 폭주 방지): https://www.mathworks.com/videos/understanding-pid-control-part-1-what-is-pid-control--1527089264373.html
- 한국어 개념: [위키백과 PID 제어기](https://ko.wikipedia.org/wiki/PID_%EC%A0%9C%EC%96%B4%EA%B8%B0) → [velog PID 정리](https://velog.io/@sms6536/PID%EC%A0%9C%EC%96%B4%EC%97%90-%EA%B4%80%ED%95%9C-%EC%9D%B4%ED%95%B4)

**Stage 1 (reference) & Stage 3 (load) — 차체 거동 모델링**
- Rajesh Rajamani, 「Vehicle Dynamics and Control」 — 이 분야 표준 교과서. **2.3장 Bicycle Model**(=reference stage), 2.6장 yaw rate & slip angle이 핵심. 대학원 교재라 해당 챕터만 발췌해서 볼 것: https://www.academia.edu/31492223/
- 바이시클 모델은 위 Torino 논문 2~3장에도 우리 코드 수준으로 그림+수식이 잘 정리돼 있음.

**Stage 4 (traction) — 타이어 마찰원**
- 마찰원(friction circle) + 하중이동은 Rajamani 타이어 챕터 + Torino 논문 트랙션 파트로 충분. 핵심 직관: **"수직하중 Fz × μ = 그 바퀴가 낼 수 있는 최대 힘"**.

**회생 제어 (`longitudinal.cpp`)**
- 개념 입문(한국어): [전기차 회생제동 원리(GM)](https://www.globalmotors.co.kr/view.php?ud=2021050311311245890d88486204_5) · [브런치: 회생제동의 제어원리와 에너지 회수율](https://brunch.co.kr/@1212ac31a500435/163)
- 우리 코드에서 회생 = "음수 토크"일 뿐. 모터 내부제어(FOC)까지 몰라도 됨. `longitudinal_compute`가 brake_pct/SOC 보고 −토크를 얼마나 줄지 결정하는 전략 함수라는 것만 이해하면 충분.

> 솔직한 한계: 토크벡터링을 처음부터 한국어로 다룬 영상 시리즈는 사실상 없습니다. 한국어 자료는 개념 잡기용(블로그/위키)이고, **실제 구현 감각은 Torino 논문 + Brian Douglas 영어 영상**에서 나옵니다. 영어 자막을 켜고 보는 걸 팀 기본으로 잡으세요.
