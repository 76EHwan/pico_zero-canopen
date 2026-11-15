# pico_zero-canopen

> **⚠️ 프로토타입입니다.** 펌웨어 빌드와 PC 단위 테스트까지만 확인했고, **실제 AK40-10 모터에 연결해서 돌려 보지는 않았습니다.** 실기에서 확인할 항목은 [7. 실기에서 꼭 확인할 것](#7-실기에서-꼭-확인할-것)에 정리했습니다. 명령 형식과 동작은 시험 결과에 따라 바뀔 수 있습니다. 처음 돌릴 때는 모터를 고정하고, 모터 쪽 전원을 바로 끊을 수 있는 상태에서 시험하세요.

RP2350-Zero(또는 RP2040-Zero) + SN65HVD230 CAN 트랜시버로 **CubeMars AK40-10 액추에이터 3대**를 제어하는 USB-시리얼 → CAN 브리지 펌웨어입니다.

```
중앙 제어부 ──USB-C (CDC 시리얼, 텍스트 명령)──▶ RP2350-Zero ──CAN 1 Mbps──▶ AK40-10 ×3
            ◀──────── 피드백(FB)/이벤트(EVT) ───────                ◀── 0x29 피드백
```

> **CANopen 관련 참고:** AK40-10은 CANopen(CiA 301/402, SDO/PDO/NMT)을 지원하지 않고, CubeMars 자체 CAN 프로토콜(**Servo 모드**와 **MIT 모드**)을 씁니다. 이 펌웨어는 그 자체 프로토콜을 구현합니다(`src/ak_proto.c`). 프로젝트 이름은 처음 지은 그대로 두었습니다.

## 1. 하드웨어 연결

| RP2350-Zero / RP2040-Zero | SN65HVD230 모듈 | 비고 |
|---|---|---|
| 3V3 | VCC (3.3V) | SN65HVD230은 3.3V 전용 |
| GND | GND | 모터 전원 GND와도 공통 접지 |
| **GP4** | CRX (R, RXD) | MCU 수신 |
| **GP5** | CTX (D, TXD) | MCU 송신 |
| — | CANH / CANL | 모터 3대의 CAN_H / CAN_L에 데이지 체인으로 연결 |

- 핀은 [src/config.h](src/config.h)의 `CAN_GPIO_RX` / `CAN_GPIO_TX`에서 바꿀 수 있습니다. GP16은 보드의 WS2812 RGB LED이므로 피하세요.
- **종단 저항:** 버스 양 끝에 120Ω이 하나씩 있어야 합니다. 모든 장치의 전원을 끄고 CANH–CANL 사이 저항을 재서 약 **60Ω**이 나와야 정상입니다. SN65HVD230 모듈 대부분에 120Ω이 붙어 있습니다.
- **Rs 핀:** 1 Mbps에서는 Rs가 GND에 직결(고속 모드)되어 있어야 합니다. 10kΩ 등을 거쳐 GND에 연결된(슬로프 제어) 모듈은 1 Mbps에서 통신 에러가 날 수 있습니다.
- AK40-10 V3.0은 전원과 CAN이 한 커넥터(XT30PW(2+2))로 들어옵니다. 핀 배치는 CubeMars 드라이버 설치 문서를 확인하세요.

## 2. 모터 사전 설정 (CubeMars 상위 프로그램, R-Link)

모터마다 한 번씩 설정합니다.

1. **CAN ID**: 1, 2, 3 (펌웨어 기본값. 다르면 `config.h`의 `MOTOR_DEFAULT_ID_x`를 바꾸거나 `ID` 명령 사용)
2. **CAN 속도**: 1 Mbps
3. **CAN 피드백 모드**: 주기 피드백(예: 100–500 Hz) 또는 요청-응답 중 하나. 어느 쪽이든 0x29 피드백을 받아 처리합니다.

## 3. 빌드와 업로드

VS Code의 **Raspberry Pi Pico 확장**으로 만든 프로젝트 구조 그대로입니다(Pico SDK 2.3.1).

- VS Code: `Compile Project` → `build/pico_zero-canopen.uf2`
- 업로드: 보드의 BOOT 버튼을 누른 채 USB를 연결하면 USB 드라이브가 뜹니다. 여기에 `.uf2`를 복사하세요. 또는 `Run Project`(picotool)를 쓰면 됩니다.
- **RP2040-Zero로 빌드하려면:** `CMakeLists.txt`의 `PICO_BOARD`를 `waveshare_rp2040_zero`로 바꾸고(또는 확장의 *Switch Board*) `build` 폴더를 지운 뒤 다시 빌드합니다. 두 보드 모두 빌드를 확인했습니다.
- 명령줄로 빌드할 때는 PowerShell이나 cmd에서 `ninja -C build`를 실행하세요. Git Bash에서 직접 실행하면 링커 스크립트(`memmap_copy_to_ram.incl`)를 못 찾는 현상이 있었습니다.

`can2040`(PIO 소프트웨어 CAN)은 `lib/can2040`에 포함되어 있습니다. 출처는 [KevinOConnor/can2040](https://github.com/KevinOConnor/can2040), 커밋 `2988d4f`입니다.

## 4. 시리얼 프로토콜

- 아무 baud로 열면 됩니다(USB CDC). 단, **1200 baud로 열면 BOOTSEL로 재부팅**됩니다.
- 펌웨어는 DTR이 켜져 있어야 출력합니다. pyserial은 기본값이 DTR 켜짐입니다.
- 한 줄에 명령 하나를 보내고 끝에 `\n`을 붙입니다. `;`로 여러 명령을 한 줄에 묶을 수 있습니다. 토큰은 공백이나 `,`로 구분하고, 대소문자는 구분하지 않습니다.
- 모터 번호 `m`은 `1`~`3`이고, `*`(또는 `A`, `ALL`, `0`)는 전체입니다.

### 목표값 명령

성공하면 응답이 없고(고속 스트리밍용), 실패하면 `ERR ...`가 옵니다. `ACK 1`을 보내면 성공 시에도 `OK`를 받습니다.

| 명령 | 의미 | 단위 |
|---|---|---|
| `MIT m p v kp kd t` | MIT(임피던스) 제어 | p [rad], v [rad/s], kp 0–500, kd 0–5, t [Nm] |
| `POS m deg` | Servo 위치 | 출력축 deg (±36000) |
| `PV m deg erpm acc` | Servo 위치+속도+가속 | ERPM, ERPM/s |
| `VEL m erpm` | Servo 속도 | ERPM |
| `CUR m A` | 전류(토크) 제어 | A (±7.3 클램프) |
| `BRK m A` | 브레이크 전류 | A |
| `DUTY m d` | 듀티 | ±0.95 |

- 보낸 목표값은 보드가 **`RATE`(기본 500 Hz)마다 CAN으로 계속 재전송**합니다. 호스트는 원하는 주기로 갱신만 하면 됩니다.
- MIT 범위(AK40-10 V3): p ±12.56 rad, v ±45 rad/s, t ±5 Nm. 범위를 넘는 값은 잘라냅니다.
- **ERPM = 출력축 rpm × 140** (극쌍 14 × 감속비 10). 예: 출력 60 rpm = `VEL 1 8400`

예:
```
MIT 1 0.5 0 20 1 0;MIT 2 0 0 20 1 0;MIT 3 -0.5 0 20 1 0
POS * 90
STOP
```

### 제어 / 설정 / 진단

| 명령 | 응답 | 설명 |
|---|---|---|
| `STOP [m]` / `DIS m` | `OK` | 정지 + 스트리밍 중단. V3는 disable(ID 15) 프레임을 보내고, 모터가 `EVT DISABLED m`으로 확인 |
| `EN m` | `OK` | V2 프로토콜에서만 의미 있음 (MIT 진입 0xFC) |
| `ORIGIN m [0\|1]` | `OK` | 현재 위치를 원점으로 설정. 0=임시, 1=영구(플래시 저장) |
| `ID m canid` | `OK` | 모터 m의 CAN ID 변경 (재부팅하면 기본값으로 돌아감) |
| `PROTO m V3\|V2` | `OK` | 프로토콜 선택 (구형 AK 2.0 드라이버는 V2) |
| `RATE hz` | `OK` | CAN 재전송 주기 10–1000 Hz |
| `TIMEOUT ms` | `OK` | 호스트 워치독 (0=끔, 기본 500 ms) |
| `TELEM hz` | `OK` | FB 자동 출력 주기 (0=끔, 기본 100 Hz) |
| `GET [m]` | `FB ...` | 피드백 즉시 출력 |
| `INFO` / `STAT` | `INFO ...` / `STAT ...` | 설정 / CAN 통계 |
| `MON 0\|1` | `OK` | 버스의 모든 수신 프레임을 `RX E 00002901 8 ...` 형식으로 출력 (ID 확인용) |
| `RAW E\|S id_hex [data_hex]` | `OK` | 임의 프레임 1회 전송. 예: `RAW E F01`, `RAW S 1 FFFFFFFFFFFFFFFC` |
| `PING` / `K` | `OK PONG` / 없음 | 연결 확인 / 워치독용 keep-alive |
| `HELP` | `HELP ...` | 명령 목록 |

### 보드 → 호스트 출력

```
FB m pos_rad vel_rad_s torque_Nm current_A temp_C err age_ms
FB 1 0.5012 -0.013 0.120 0.25 34 0 3
EVT BOOT 2                        모터 2 전원 켜짐/리셋
EVT DISABLED 1                    disable 확인
EVT FAULT 3 1 motor_over_temp     에러 코드: 1 모터 과열, 2 과전류, 3 과전압, 4 저전압, 5 엔코더, 6 MOSFET 과열, 7 스톨
EVT TIMEOUT all_motors_stopped    호스트 명령이 TIMEOUT 동안 끊겨서 전체 정지
EVT CAN_STALL no_ack              송신 ACK 없음: 배선, 모터 전원, 종단, 속도 확인
READY pico_zero-canopen fw=1.0.0  포트를 열었을 때
```

- 위치와 속도는 출력축 기준입니다. torque는 `current × 0.4894 Nm/A`로 계산한 값입니다.
- `age_ms`는 마지막 피드백을 받은 뒤 지난 시간입니다. 값이 계속 커지면 해당 모터와 통신이 안 되는 것입니다.

### 파이썬 예제

```bash
pip install pyserial
python tools/ak_host.py COM5            # 연결 확인
python tools/ak_host.py COM5 --demo     # MIT 사인파 데모 (모터가 움직임)
```

## 5. 안전 동작

- **호스트 워치독:** `TIMEOUT`(기본 500 ms) 동안 USB 명령이 하나도 없으면 활성 모터를 모두 정지(V3 disable)하고 `EVT TIMEOUT`을 보냅니다. 새 목표값을 보내면 다시 동작합니다.
- **부팅 시 정지:** 전원이 켜지거나 리셋되면 모든 모터에 정지 명령부터 보냅니다.
- **하드웨어 워치독(1 s):** core1 제어 루프가 멈추면 보드가 리셋되고, 리셋 후 위 부팅 정지가 실행됩니다.
- **클램프:** 전류 ±7.3 A, 듀티 ±0.95, MIT 값은 프로토콜 범위로 잘라냅니다.
- **브리지 자체의 전원이 끊기면** 모터는 마지막 명령을 유지할 수 있습니다. 모터 쪽 비상 정지(전원 차단)를 따로 두세요.

## 6. 구조

| 파일 | 역할 |
|---|---|
| `pico_zero-canopen.c` | core0: USB 시리얼, 텔레메트리/이벤트 출력, 상태 LED, 워치독 |
| `src/motor_ctrl.c` | core1: 명령 적용, 고정 주기 CAN 전송, 피드백 해석, 호스트 타임아웃 |
| `src/can_bus.c` | can2040 래퍼 (PIO0, IRQ는 core1에서 최고 우선순위) |
| `src/ak_proto.c` | AK 프로토콜 인코딩/디코딩 (하드웨어 의존성 없음) |
| `src/serial_cmd.c` | 텍스트 명령 파서 |
| `src/usb_io.c` | 막히지 않는 USB 줄 단위 입출력 |
| `src/config.h` | 핀, 속도, 모터 ID/파라미터, 주기 |
| `tests/test_ak_proto.c` | PC용 프로토콜 단위 테스트 (매뉴얼 예제 바이트와 비교) |
| `tools/ak_host.py` | 중앙 제어부 파이썬 예제 |

- can2040은 인터럽트 지연에 민감합니다. 그래서 펌웨어 전체를 RAM에서 실행하고(`copy_to_ram`), CAN은 core1에만 두어 USB 인터럽트(core0)와 분리했습니다.
- 상태 LED 색: 파랑=USB 미연결, 노랑=연결됨 & 피드백 없음, 초록=피드백 수신 중, 빨강=모터 에러 또는 CAN ACK 없음.

단위 테스트 실행:
```bash
gcc -std=c99 -Wall -Isrc tests/test_ak_proto.c src/ak_proto.c -lm -o test_ak_proto && ./test_ak_proto
```

## 7. 실기에서 꼭 확인할 것

- **MIT 위치 범위:** V3 매뉴얼의 파라미터 표와 코드는 ±12.56 rad인데, 같은 매뉴얼의 예제 바이트는 ±12.5로 계산되어 있습니다. 펌웨어는 표(±12.56)를 따랐습니다. `MIT 1 6 0 5 0.5 0`을 보내고 FB의 pos가 6.00 rad에 오는지 확인하세요. 약 0.5% 어긋나면 `config.h`의 `AK_V3_P_MAX`를 `12.5f`로 바꾸면 됩니다.
- **STOP 후 재가동:** V3 매뉴얼은 disable(ID 15)만 정의하고 다시 enable하는 방법은 적어두지 않았습니다. 이 펌웨어는 부팅 직후와 STOP/TIMEOUT 때 disable을 보내므로, `STOP` 다음에 목표값을 보냈을 때 모터가 다시 움직이는지 먼저 확인하세요. 다시 움직이지 않으면 `config.h`에서 `AK_V3_STOP_MODE`를 `1`(전류 0 A로 정지)로 바꾸면 됩니다.
- **피드백 위치 범위:** 0x29 피드백의 위치는 ±3200°(0.1° 단위)라 약 ±8.9바퀴를 넘으면 표현할 수 없습니다. 상위 프로그램에서 0x2A(32비트 위치) 피드백을 켜면 자동으로 그 값을 씁니다.
- **버스 부하:** 3대 × 500 Hz 명령에 응답까지 더하면 1 Mbps의 약 40%입니다. CAN 장치를 더 붙이면 `RATE`를 낮추세요.

## 라이선스

이 프로젝트는 **GNU GPLv3**로 배포합니다. 전문은 [LICENSE](LICENSE)에 있습니다.

포함된 `lib/can2040`이 GPLv3이므로, 이를 링크한 펌웨어 전체도 GPLv3 조건을 따릅니다. 펌웨어 바이너리를 외부에 배포할 때는 소스도 함께 공개해야 합니다.
