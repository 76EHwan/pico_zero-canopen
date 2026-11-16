# pico_zero-canopen

> **⚠️ 프로토타입입니다.** 펌웨어 빌드와 PC 테스트(CANopen 스택, CiA 402 상태머신, slcan, AK 프로토콜)까지만 확인했고, **실제 AK40-10 모터와 실제 CANopen 마스터(ros2_canopen, python-canopen)에 연결해서 돌려 보지는 않았습니다.** 실기에서 확인할 항목은 [9. 실기에서 꼭 확인할 것](#9-실기에서-꼭-확인할-것)에 정리했습니다. 처음 돌릴 때는 모터를 고정하고, 모터 쪽 전원을 바로 끊을 수 있는 상태에서 시험하세요.

RP2350-Zero(또는 RP2040-Zero) + SN65HVD230으로 **CubeMars AK40-10 액추에이터 3대를 CANopen(CiA 301/402) 드라이브처럼 보이게 만드는 게이트웨이** 펌웨어입니다.

```
CANopen 마스터 (Jetson Orin: ros2_canopen / python-canopen)
     │  USB-C 시리얼 위의 slcan  (리눅스: slcand → SocketCAN 인터페이스 slcan0)
RP2350-Zero  ── CANopen 노드 1, 2, 3 = CiA 402 단일 축 서보 드라이브 3대
     │  CAN 1 Mbps, CubeMars 자체 프로토콜 (Servo / MIT 모드)
AK40-10 ×3
```

- AK40-10 자체는 CANopen을 지원하지 않습니다. 그래서 보드가 중간에서 **CANopen ↔ CubeMars 프로토콜**을 변환합니다.
- 마스터와 보드 사이는 CAN 배선 없이 USB로 연결합니다. 그 USB 위로 **CAN 프레임을 slcan 형식으로 그대로 주고받으므로**, 마스터 입장에서는 일반 CAN 버스에 CANopen 장치 3대가 붙어 있는 것과 같습니다. NMT, SDO, PDO, SYNC, EMCY, heartbeat가 모두 표준대로 동작합니다.
- CANopen 스택은 외부 라이브러리 없이 직접 구현했습니다(`src/co_node.c`, `src/cia402.c`).

## 1. 하드웨어 연결

**모터 쪽 (CAN)**

| RP2350-Zero / RP2040-Zero | SN65HVD230 모듈 | 비고 |
|---|---|---|
| 3V3 | VCC (3.3V) | SN65HVD230은 3.3V 전용 |
| GND | GND | 모터 전원 GND와도 공통 접지 |
| **GP4** | CRX (R, RXD) | MCU 수신 |
| **GP5** | CTX (D, TXD) | MCU 송신 |
| — | CANH / CANL | 모터 3대의 CAN_H / CAN_L에 데이지 체인으로 연결 |

- 핀은 [src/config.h](src/config.h)의 `CAN_GPIO_RX` / `CAN_GPIO_TX`에서 바꿀 수 있습니다. GP16은 보드의 WS2812 RGB LED이므로 피하세요.
- **종단 저항:** 버스 양 끝에 120Ω이 하나씩 있어야 합니다. 모든 장치의 전원을 끄고 CANH–CANL 사이 저항을 재서 약 **60Ω**이 나와야 정상입니다.
- **Rs 핀:** 1 Mbps에서는 Rs가 GND에 직결(고속 모드)되어 있어야 합니다. 10kΩ 등을 거쳐 GND에 연결된(슬로프 제어) 모듈은 1 Mbps에서 통신 에러가 날 수 있습니다.
- 꼬임선 두 가닥의 길이 차이(수십 cm)는 지연이 수 ns 수준이라 1 Mbps(비트당 1000 ns)에서는 문제가 되지 않습니다. 선이 길어질수록 종단, 공통 GND, Rs 설정이 더 중요해집니다.

**마스터 쪽 (USB)**

- 보드의 USB-C를 Jetson(또는 PC)에 연결합니다.
- USB 2.0 케이블 규격상 최대 길이는 **5 m**입니다. 그보다 멀면 액티브 연장 케이블이나 허브가 필요합니다.

## 2. 모터 사전 설정 (CubeMars 상위 프로그램, R-Link)

모터마다 한 번씩 설정합니다.

1. **CAN ID**: 1, 2, 3. 모터 CAN ID 기본값은 CANopen 노드 ID와 같고, 다르면 SDO `0x2000:01`로 바꿀 수 있습니다.
2. **CAN 속도**: 1 Mbps
3. **CAN 피드백 모드**: 주기 피드백(예: 100–500 Hz) 또는 요청-응답. 어느 쪽이든 0x29 피드백을 받아 처리합니다.

## 3. 빌드와 업로드

VS Code의 **Raspberry Pi Pico 확장**으로 만든 프로젝트 구조 그대로입니다(Pico SDK 2.3.1).

- VS Code: `Compile Project` → `build/pico_zero-canopen.uf2`
- 업로드: 보드의 BOOT 버튼을 누른 채 USB를 연결하면 USB 드라이브가 뜹니다. 여기에 `.uf2`를 복사하세요. 또는 `Run Project`(picotool)를 쓰면 됩니다.
- **RP2040-Zero로 빌드하려면:** `CMakeLists.txt`의 `PICO_BOARD`를 `waveshare_rp2040_zero`로 바꾸고(또는 확장의 *Switch Board*) `build` 폴더를 지운 뒤 다시 빌드합니다. 두 보드 모두 빌드를 확인했습니다.
- 명령줄로 빌드할 때는 PowerShell이나 cmd에서 `ninja -C build`를 실행하세요. Git Bash에서 직접 실행하면 링커 스크립트(`memmap_copy_to_ram.incl`)를 못 찾는 현상이 있었습니다.

`can2040`(PIO 소프트웨어 CAN)은 `lib/can2040`에 포함되어 있습니다. 출처는 [KevinOConnor/can2040](https://github.com/KevinOConnor/can2040), 커밋 `2988d4f`입니다.

**PC 테스트 / EDS 생성** (gcc 필요):
```bash
gcc -std=c99 -Wall -Isrc tests/test_canopen.c src/co_od.c src/co_node.c src/cia402.c src/slcan.c src/ak_proto.c -lm -o test_canopen && ./test_canopen
```
```bash
gcc -std=c99 -Wall -Isrc tests/test_ak_proto.c src/ak_proto.c -lm -o test_ak_proto && ./test_ak_proto
```
```bash
gcc -std=c99 -Isrc tools/gen_eds.c src/co_od.c -o gen_eds && ./gen_eds eds/pico_ak40.eds
```
오브젝트 딕셔너리(`src/co_od.c`)를 고치면 EDS를 다시 생성하세요. 펌웨어와 EDS가 같은 테이블에서 나오므로 서로 어긋나지 않습니다.

## 4. 마스터에서 연결하기 (slcan)

**Jetson Orin / 리눅스**
```bash
sudo apt install can-utils
```
```bash
sudo slcand -o -c -s8 /dev/ttyACM0 slcan0
```
```bash
sudo ip link set slcan0 up
```
```bash
candump slcan0
```
- 다른 터미널에서 `cansend slcan0 000#8200`(전체 노드 통신 리셋)을 보내면 `701#00`, `702#00`, `703#00`(boot-up)이 보여야 합니다.
- `/dev/ttyACM0` 권한이 없으면 사용자를 `dialout` 그룹에 넣으세요.
- Ubuntu의 ModemManager가 새 ttyACM 장치에 AT 명령을 보내 방해할 수 있습니다. 연결이 이상하면 `sudo systemctl disable --now ModemManager`로 끄세요.

**Windows**: python-can의 `slcan` 인터페이스가 COM 포트를 직접 엽니다(`interface='slcan', channel='COM5'`). slcand가 필요 없습니다.

주의: 포트를 **1200 baud로 열면 BOOTSEL로 재부팅**됩니다. slcan은 baud 값과 무관하므로 115200 등을 쓰세요.

## 5. CANopen 장치 사양

- 노드 ID **1, 2, 3** (`config.h`의 `CO_NODE_ID_x`) — 축마다 독립된 CiA 402 드라이브입니다.
- EDS: [eds/pico_ak40.eds](eds/pico_ak40.eds) (세 노드 공통, `$NODEID` 사용)
- Device type `0x1000` = `0x00020192` (CiA 402 서보 드라이브), 지원 모드 `0x6502` = `0x3A5`
- 지원 기능: NMT 슬레이브, heartbeat 생산자(`0x1017`, 기본 1000 ms)와 소비자 1개(`0x1016`), SDO expedited/segmented(block 전송 미지원), RPDO/TPDO 각 4개 + 동적 매핑, SYNC 전송 타입 0–240과 이벤트 타입 254/255(inhibit, event timer), EMCY

### 단위

| 오브젝트 | 1 단위 |
|---|---|
| 위치 `0x6064` `0x607A` `0x607C` `0x6067` | 0.0001 rad |
| 속도 `0x606C` `0x60FF` `0x60B1` `0x6081` `0x606D` | 0.001 rad/s |
| 가속 `0x6083` `0x6084` | 0.001 rad/s² |
| 토크 `0x6071` `0x6077` `0x6072` `0x60B2` | 정격 토크 `0x6076`(1.3 Nm)의 1/1000 |
| 전류 `0x6078` | 정격 전류 `0x6075`(2.7 A)의 1/1000 |

### 운전 모드 (`0x6060`) → AK 명령

| 모드 | 사용하는 오브젝트 | AK40-10 쪽 동작 |
|---|---|---|
| 1 Profile Position | `0x607A`, `0x6081`, `0x6083`, controlword bit4(새 목표) / bit6(상대) / bit8(halt) | Servo 위치-속도 모드 (모터가 궤적 생성) |
| 3 Profile Velocity | `0x60FF`, bit8(halt) | Servo 속도 모드 |
| 6 Homing | `0x6098` = 35 또는 37 (현재 위치를 원점으로) | 원점 설정 명령(임시) |
| 8 Cyclic Sync Position | `0x607A`, `0x60B1`(속도 FF), `0x60B2`(토크 FF), 게인 `0x2000:03/04` | MIT 모드 (범위 ±12.56 rad) |
| 9 Cyclic Sync Velocity | `0x60FF` + `0x60B1` | Servo 속도 모드 |
| 10 Cyclic Sync Torque | `0x6071` + `0x60B2`, `0x6072`로 제한 | MIT 모드 (토크만) |

- **Operation enabled**에서만 모터에 명령을 보내고, 그 밖의 상태에서는 모터를 정지시킵니다. Operation enabled로 들어가는 순간 `0x607A`를 현재 위치로 초기화해서 모터가 튀지 않게 합니다.
- Quick stop(`0x605A` = 0)은 즉시 정지 후 Switch on disabled로 갑니다.
- statusword bit4(voltage enabled)는 모터 피드백이 들어오고 있다는 뜻으로 씁니다.

### 기본 PDO 매핑

| PDO | COB-ID | 내용 | 전송 |
|---|---|---|---|
| RPDO1 | `0x200+n` | controlword `0x6040` + 모드 `0x6060` | 즉시(255) |
| RPDO2 | `0x300+n` | 목표 위치 `0x607A` + 목표 속도 `0x60FF` | 즉시(255) |
| RPDO3 | `0x400+n` | 목표 토크 `0x6071` + 토크 오프셋 `0x60B2` | 즉시(255) |
| TPDO1 | `0x180+n` | statusword `0x6041` + 모드 표시 `0x6061` | 값이 바뀔 때, 100 ms마다 |
| TPDO2 | `0x280+n` | 위치 `0x6064` + 속도 `0x606C` | 값이 바뀔 때(최소 10 ms 간격), 100 ms마다 |
| TPDO3 | `0x380+n` | 토크 `0x6077`, 에러 코드 `0x603F`, 모터 온도 `0x2000:05`, 모터 에러 `0x2000:06` | 값이 바뀔 때(최소 10 ms 간격), 100 ms마다 |

마스터가 부팅 때 SDO로 다시 매핑하는 것도 지원합니다(PDO 끄기 → sub0 = 0 → 항목 쓰기 → sub0 설정 → PDO 켜기). NMT reset communication을 받으면 위 기본값으로 돌아갑니다.

### 제조사 오브젝트 `0x2000` (AK motor)

| sub | 이름 | 설명 |
|---|---|---|
| 1 | Motor CAN ID | 기본값 = 노드 ID. Operation enabled 중에는 변경 불가 |
| 2 | Motor protocol | 0 = V3 (AK40-10 V3.0), 1 = V2 (구형 AK 2.0) |
| 3 / 4 | MIT Kp / Kd | CSP 게인 (REAL32, 기본 10 / 0.5) |
| 5 / 6 | Motor temperature / error code | 읽기 전용, TPDO 매핑 가능 |
| 7 | Feedback age | 마지막 모터 피드백 이후 ms |
| 8 | Feedback timeout | 운전 중 이 시간(기본 200 ms) 동안 피드백이 없으면 Fault. 0이면 끔 |

### EMCY / 에러 코드 (`0x603F`)

| 코드 | 원인 |
|---|---|
| `0x2310` | 모터 과전류 |
| `0x3210` / `0x3220` | 과전압 / 저전압 |
| `0x4310` / `0x4210` | 모터 과열 / MOSFET 과열 |
| `0x7305` | 엔코더 |
| `0x7121` | 모터 구속(스톨) |
| `0x8130` | 마스터 heartbeat 끊김 (`0x1016`) |
| `0x8100` | NMT stop, 또는 USB/slcan 연결 끊김 |
| `0xFF01` | 운전 중 모터 피드백 끊김 (`0x2000:08`) |

Fault는 원인이 사라진 뒤 controlword bit7(fault reset) 상승 에지로 해제합니다. 통신 끊김 때 어떻게 할지는 `0x6007`로 정합니다(0 무시, 1 Fault(기본), 2·3 정지).

## 6. 예제

### python-canopen (Windows/리눅스)

```bash
pip install canopen python-can pyserial
```
```bash
python tools/canopen_demo.py --interface slcan --channel COM5
```
```bash
python tools/canopen_demo.py --interface socketcan --channel slcan0
```
[tools/canopen_demo.py](tools/canopen_demo.py)는 다음 순서로 동작합니다. **모터가 실제로 1 rad 움직입니다.**
1. NMT 리셋 후 boot-up 대기
2. SDO로 장치 정보 읽기
3. Profile Position 설정
4. 상태머신 0x06 → 0x07 → 0x0F
5. set-point 핸드셰이크로 이동 후 복귀
6. 이동하는 동안 TPDO2(위치/속도) 수신

### candump / cansend로 직접 (리눅스)

```bash
cansend slcan0 601#4000100000000000
```
노드 1의 `0x1000` 읽기. 응답은 `581#4300100092010200`입니다.
```bash
cansend slcan0 000#0101
```
노드 1 NMT start. 이후 `601#2B40600006000000`, `...07000000`, `...0F000000`을 차례로 보내면 Operation enabled가 됩니다.

### ros2_canopen (Jetson Orin)

[ros2/bus.yml](ros2/bus.yml)이 버스 설정 예시입니다. 내용은 다음과 같습니다.
- 마스터: 노드 127, SYNC 100 Hz
- PDO 매핑: 위 표와 같음
- 단위 변환: `scale_pos_* = 10000 / 0.0001`, `scale_vel_* = 1000 / 0.001`
- heartbeat: 마스터 heartbeat를 슬레이브가 감시

EDS를 같은 config 폴더에 두고 ros2_canopen의 설정 생성 절차를 따르면 됩니다. 키 이름은 ros2_canopen 버전마다 다를 수 있으니 [공식 문서](https://ros-industrial.github.io/ros2_canopen/manual/rolling/)와 대조하세요.

## 7. 안전 동작

- **통신 끊김:** 다음 경우 `0x6007`(기본 1 = Fault)에 따라 모터를 멈춥니다.
  - 마스터 heartbeat가 끊김 (`0x1016` 설정 시)
  - NMT stop
  - USB 포트 닫힘(DTR off) 또는 slcan `C` 명령
- **운전 중 모터 무응답:** 모터 피드백이 `0x2000:08`(200 ms) 동안 없으면 Fault.
- **core0 ↔ core1 감시:** core1(모터 루프)에 명령이 500 ms 동안 오지 않으면 모터를 정지합니다. 운전 중에는 50 ms마다 명령을 갱신합니다.
- **부팅 시 정지:** 전원 투입, 리셋, slcan을 열 때 모든 노드를 초기화하고 모터를 정지시킵니다.
- **하드웨어 워치독(1 s):** core1 루프가 멈추면 보드가 리셋됩니다.
- **클램프:** 전류 ±7.3 A, 토크는 `0x6072`로 제한, MIT 값은 프로토콜 범위로 잘라냅니다.
- **보드 전원이 끊기면** 모터는 마지막 명령을 유지할 수 있습니다. 모터 쪽 비상 정지(전원 차단)를 따로 두세요.

## 8. 구조

| 파일 | 역할 |
|---|---|
| `pico_zero-canopen.c` | core0: slcan ↔ CANopen 노드 연결, 상태 LED, 워치독 |
| `src/slcan.c` | Lawicel(slcan) 텍스트 프로토콜 (하드웨어 의존성 없음) |
| `src/co_od.c` | 오브젝트 딕셔너리 테이블 — 펌웨어와 EDS의 유일한 정의 |
| `src/co_node.c` | CiA 301: NMT, heartbeat, SDO 서버, PDO, SYNC, EMCY |
| `src/cia402.c` | CiA 402 상태머신, 운전 모드 → AK 명령, 고장 처리 |
| `src/co_motor_pico.c` | CiA 402 → core1 motor_ctrl 명령 전달 |
| `src/motor_ctrl.c` | core1: 고정 주기 CAN 전송, 모터 피드백 해석 |
| `src/can_bus.c` | can2040 래퍼 (PIO0, IRQ는 core1에서 최고 우선순위) |
| `src/ak_proto.c` | AK 프로토콜 인코딩/디코딩 |
| `src/usb_io.c` | 막히지 않는 USB 입출력 |
| `src/config.h` | 핀, 노드 ID, 단위, 모터 파라미터, 주기 |
| `eds/pico_ak40.eds` | EDS (`tools/gen_eds.c`로 생성) |
| `tests/` | PC 테스트: CANopen/402/slcan(`test_canopen.c`), AK 프로토콜(`test_ak_proto.c`) |
| `tools/canopen_demo.py`, `ros2/bus.yml` | 마스터 예제 |

- can2040은 인터럽트 지연에 민감합니다. 그래서 펌웨어 전체를 RAM에서 실행하고(`copy_to_ram`), CAN은 core1에만 두어 USB 인터럽트(core0)와 분리했습니다.
- 상태 LED 색: 파랑 = slcan 닫힘, 노랑 = 열림 & 운전 중인 축 없음, 초록 = 운전 중, 빨강 = Fault 또는 모터 CAN ACK 없음.

## 9. 실기에서 꼭 확인할 것

- **실제 마스터와의 호환성:** PC 테스트는 CiA 301/402 규칙대로 작성했지만, Lely(ros2_canopen)나 python-canopen과 실제로 붙여 본 적은 없습니다. 특히 다음 두 가지는 먼저 확인하세요.
  - ros2_canopen 부팅 과정에서 SDO abort가 나오는지 (`candump slcan0`에서 `58x#80...`)
  - EDS를 마스터 도구가 문제없이 읽는지
- **MIT 위치 범위:** V3 매뉴얼의 파라미터 표와 코드는 ±12.56 rad인데, 예제 바이트는 ±12.5로 계산되어 있습니다. CSP로 6 rad를 보냈을 때 위치가 6.00 rad에 오는지 확인하세요. 약 0.5% 어긋나면 `config.h`의 `AK_V3_P_MAX`를 `12.5f`로 바꾸면 됩니다.
- **정지 후 재가동:** V3 매뉴얼은 disable(ID 15)만 정의하고 다시 enable하는 방법은 적어두지 않았습니다. Operation enabled → Switched on → Operation enabled로 다시 갔을 때 모터가 움직이는지 확인하세요. 안 움직이면 `config.h`의 `AK_V3_STOP_MODE`를 `1`(전류 0 A로 정지)로 바꾸면 됩니다.
- **모드 전환:** V3에서 Servo 모드(PP/PV/CSV)와 MIT 모드(CSP/CST)를 운전 중에 바꿔도 모터가 따라오는지 확인하세요. 구형 V2 드라이버는 상위 프로그램에서 고른 모드(Servo 또는 MIT) 쪽 운전 모드만 쓸 수 있습니다.
- **피드백 위치 범위:** 0x29 피드백의 위치는 ±3200°(0.1° 단위)라 약 ±8.9바퀴를 넘으면 표현할 수 없습니다. 상위 프로그램에서 0x2A(32비트 위치) 피드백을 켜면 자동으로 그 값을 씁니다.
- **버스 부하:** 모터 쪽은 3대 × 500 Hz 명령에 응답까지 더하면 1 Mbps의 약 40%입니다.

## 라이선스

이 프로젝트는 **GNU GPLv3**로 배포합니다. 전문은 [LICENSE](LICENSE)에 있습니다.

포함된 `lib/can2040`이 GPLv3이므로, 이를 링크한 펌웨어 전체도 GPLv3 조건을 따릅니다. 펌웨어 바이너리를 외부에 배포할 때는 소스도 함께 공개해야 합니다.
