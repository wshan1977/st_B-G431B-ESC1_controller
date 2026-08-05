# st_B-G431B-ESC1_controller

ST **B-G431B-ESC1** 보드(STM32G431CB) 기반 BLDC 모터 컨트롤러 펌웨어입니다.
ST Motor Control SDK의 `ElectronicSpeedControl_ESC-G4` 예제를 바탕으로, RC 수신기
PWM 입력 대신 **전원 인가 후 자동 시작 + 포텐셔미터 속도 제어**로 동작하도록 수정했습니다.

## 하드웨어

| 항목 | 내용 |
|---|---|
| 보드 | ST B-G431B-ESC1 Discovery kit (ESC) |
| MCU | STM32G431CB (Cortex-M4, 170 MHz, HSE 8 MHz + PLL) |
| 모터 | PMSM/BLDC, 7극쌍 (`pmsm_motor_parameters.h`) |
| 전류 측정 | 3-션트 + 내장 OPAMP1/2/3, ADC1/ADC2 injected 변환 |
| 과전류 보호 | COMP1/2/4 + DAC3 기준전압 → TIM1 브레이크 입력 |
| 속도 명령 | PB12 (ADC1_IN11) 포텐셔미터 |
| 텔레메트리 | PB6 = USART1 TX, 115200 8N1 (Hall 커넥터 핀 재활용) |

## 제어 방식

- **센서리스 FOC** — B-EMF 관측기(STO-PLL) + 오픈루프 리브업(rev-up) 시동
- PWM 20 kHz, TIM1 센터얼라인, FOC 루프는 PWM 주기마다 실행
- 속도 루프(Medium Frequency Task) 1 kHz, 속도 PID 제어 (`MCM_SPEED_MODE`)
- USART2(1.84 Mbps, MCP/ASPEP 프로토콜)로 **ST Motor Pilot** 모니터링/제어 가능

## 동작 방법

전원만 넣으면 자동으로 구동됩니다. 커스텀 로직은 모두 `Src/mc_app_hooks.c`에 있습니다.

1. **전원 인가** → 모터 권선을 스피커로 사용한 "삐-삑" 비프음 2회 재생 (~300 ms)
2. **약 2초 후 자동 시작** — 5,000 rpm(`DEFAULT_TARGET_SPEED_RPM`)까지 2초 램프
3. **포텐셔미터 속도 제어** — 운전(RUN) 중 PB12 전압을 100 ms 주기로 읽어
   **1,200 ~ 9,500 rpm** 범위로 매핑 (IIR 필터, 100 rpm 데드밴드, 250 ms 램프)
4. **폴트 자동 복구** — 과전류 등 폴트 발생 시 자동 승인 후 3초 뒤 재시작
5. **Start/Stop 버튼** — 보드 사용자 버튼으로 수동 시작/정지 토글 가능

### UART 텔레메트리

PB6(USART1 TX, 115200 8N1)으로 100 ms마다 한 줄씩 ASCII 텔레메트리를 출력합니다.

```
RPM=5010 IPH=1836mA IBUS=250mA VBUS=12V
```

- `RPM` — 기계적 평균 속도
- `IPH` — 상전류 진폭 (0-to-peak)
- `IBUS` — 소비전류 추정치 (전기적 출력 전력 ÷ 버스전압, 전용 션트 없음)
- `VBUS` — DC 버스전압

센서리스 구동이라 Hall 커넥터의 PB6 핀이 비어 있어 이를 사용하며,
USART2는 Motor Pilot용으로 그대로 유지됩니다. 송신은 Medium Frequency Task에서
TX FIFO를 논블로킹으로 채우는 방식이라 모터 제어에 영향을 주지 않습니다.

원본 예제의 RC PWM 입력 처리(`esc_boot` / `esc_pwm_control`)는 주석 처리되어
사용하지 않습니다. 주요 튜닝 파라미터는 `mc_app_hooks.c` 상단의
`POT_SPEED_MIN_RPM` / `POT_SPEED_MAX_RPM`과 `Inc/drive_parameters.h`에 있습니다.

## 개발 환경

| 도구 | 버전 / 비고 |
|---|---|
| ST Motor Control Workbench | 6.4.2 (`ElectronicSpeedControl_ESC-G4.stwb6`) |
| ST MCSDK | v6.4.2-Full (`MCSDK_v6.4.2-Full/` 포함) |
| STM32CubeIDE | `ElectronicSpeedControl_ESC-G4/STM32CubeIDE/` 프로젝트 |
| HAL 드라이버 | STM32G4xx HAL (저장소에 포함) |
| 플래싱/디버깅 | 보드 내장 ST-LINK/V3 |

### 빌드 방법

1. STM32CubeIDE에서 **Import → Existing Projects into Workspace** 로
   `ElectronicSpeedControl_ESC-G4/STM32CubeIDE` 폴더를 임포트
2. Debug 구성으로 빌드 후 ST-LINK로 다운로드
3. 파라미터 재생성이 필요하면 `.stwb6` 파일을 MC Workbench 6.4.2로 열어 수정

## 프로젝트 구조

```
ElectronicSpeedControl_ESC-G4/
├── Src/, Inc/            # 애플리케이션 + MC 인터페이스 코드
│   └── mc_app_hooks.c    # 커스텀 로직 (비프, 자동 시작, 포텐셔미터)
├── Drivers/              # CMSIS, STM32G4xx HAL
├── MCSDK_v6.4.2-Full/    # ST Motor Control SDK 라이브러리
└── STM32CubeIDE/         # IDE 프로젝트 (빌드 산출물은 git 제외)
```

## 알려진 문제

- `beep_pwm_restore()`가 비프 종료 후 CCR2/CCR3를 0으로 리셋하지 않아,
  자동 시작 전까지(~1.7초) V/W상 하이사이드가 100% ON 되어 권선에 큰 전류가
  흐를 수 있습니다. 채널 재활성화 전에 `LL_TIM_OC_SetCompareCH1/2/3(TIM1, 0)`
  호출이 필요합니다 (원본 `esc.c`의 `SM_BEEP_4` 처리 참고).

## 안전 주의

모터 구동 테스트 시 프로펠러 등 부하를 제거하고, 전류 제한이 가능한
전원 공급 장치 사용을 권장합니다. 전원 인가 약 2초 후 모터가 **자동으로
회전을 시작**하므로 주의하세요.
