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
| 텔레메트리 | USART2 → ST-LINK 가상 COM 포트 (보드 USB), 115200 8N1 |

## 제어 방식

- **센서리스 FOC** — B-EMF 관측기(STO-PLL) + 오픈루프 리브업(rev-up) 시동
- PWM 20 kHz, TIM1 센터얼라인, FOC 루프는 PWM 주기마다 실행
- 속도 루프(Medium Frequency Task) 1 kHz, 속도 PID 제어 (`MCM_SPEED_MODE`)
- USART2는 ASCII 텔레메트리 전용 (Motor Pilot/ASPEP는 비활성화,
  `mc_tasks.c`의 `ASPEP_start` 주석 해제 + 보드레이트 1.84 Mbps 복원으로 되돌릴 수 있음)

## 동작 방법

전원만 넣으면 자동으로 구동됩니다. 커스텀 로직은 모두 `Src/mc_app_hooks.c`에 있습니다.

1. **전원 인가** → 모터 권선을 스피커로 사용한 "삐-삑" 비프음 2회 재생 (~300 ms)
2. **약 2초 후 자동 시작** — 5,000 rpm(`DEFAULT_TARGET_SPEED_RPM`)까지 2초 램프
3. **포텐셔미터 속도 제어** — 운전(RUN) 중 PB12 전압을 100 ms 주기로 읽어
   **1,200 ~ 9,500 rpm** 범위로 매핑 (IIR 필터, 100 rpm 데드밴드, 250 ms 램프)
4. **폴트 자동 복구** — 과전류 등 폴트 발생 시 자동 승인 후 3초 뒤 재시작
5. **Start/Stop 버튼** — 보드 사용자 버튼으로 수동 시작/정지 토글 가능

### UART 텔레메트리

보드 USB(ST-LINK 가상 COM 포트 = USART2, 115200 8N1)로 100 ms마다 한 줄씩
ASCII 텔레메트리를 출력합니다. USB 케이블만 꽂고 PC에서 시리얼 터미널을
열면 바로 보입니다.

```
RPM=5010 IPH=1836mA IQ=-1830mA IBUS=250mA VBUS=12V
```

- `RPM` — 기계적 평균 속도
- `IPH` — 상전류 진폭 (0-to-peak)
- `IQ` — 부호 있는 토크 전류 (음수 = 제동/발전, 양수 = 구동)
- `IBUS` — 소비전류 추정치 (전기적 출력 전력 ÷ 버스전압, 전용 션트 없음)
- `VBUS` — DC 버스전압

모니터 UI는 **토크**를 T = P / ω = (VBUS × IBUS) ÷ (RPM × 2π/60) 로 환산해
표시하고, 입력한 작용 반경(mm)으로 나눠 **브레이크력(gf)** 까지 계산합니다.
|RPM| < 100 에서는 나눗셈이 발산하므로 표시하지 않습니다.

USART2를 텔레메트리가 점유하므로 Motor Pilot(ASPEP)은 비활성화 상태입니다.
송신은 Medium Frequency Task에서 TX FIFO를 논블로킹으로 채우는 방식이라
모터 제어에 영향을 주지 않습니다.

### 모니터 UI (`tools/telemetry_monitor.py`)

RPM·상전류·버스전류·버스전압을 실시간 숫자 + 그래프(최근 60초)로 보여주는
PC용 시리얼 모니터입니다.

```
pip install pyserial
python tools/telemetry_monitor.py
```

실행 후 ST-LINK 가상 COM 포트가 자동 선택되며 [연결]을 누르면 됩니다.
다른 터미널 프로그램이 같은 포트를 열고 있으면 연결에 실패하므로 먼저
닫아야 합니다. (요구 사항: Python 3 + pyserial, tkinter는 기본 포함)

Python 없이 쓸 수 있는 단일 실행파일은 PyInstaller로 빌드합니다:

```
pip install pyinstaller
python -m PyInstaller tools/TelemetryMonitor.spec
```

결과물은 `tools/dist/TelemetryMonitor.exe` 로 생성됩니다 (git에는 미포함).

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

## 안전 주의

모터 구동 테스트 시 프로펠러 등 부하를 제거하고, 전류 제한이 가능한
전원 공급 장치 사용을 권장합니다. 전원 인가 약 2초 후 모터가 **자동으로
회전을 시작**하므로 주의하세요.
