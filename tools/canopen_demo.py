#!/usr/bin/env python3
"""
CANopen 마스터 예제 (python-canopen): pico_zero-canopen 게이트웨이의 한 축을
CiA 402 Profile Position 모드로 움직였다가 제자리로 돌려놓는다.

    pip install canopen python-can pyserial

    # Windows: python-can 의 slcan 인터페이스가 COM 포트를 직접 연다
    python tools/canopen_demo.py --interface slcan --channel COM5

    # Linux / Jetson: slcand 로 slcan0 을 만든 뒤 SocketCAN 으로
    sudo slcand -o -c -s8 /dev/ttyACM0 slcan0 && sudo ip link set slcan0 up
    python tools/canopen_demo.py --interface socketcan --channel slcan0

주의: 모터가 실제로 움직인다 (기본 1 rad 이동 후 복귀). 축을 고정하고 실행할 것.
"""
import argparse
import os
import time

import canopen

EDS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "eds", "pico_ak40.eds")
POS_UNIT = 1e-4  # 0x6064 / 0x607A 1 단위 = 0.0001 rad
VEL_UNIT = 1e-3  # 0x606C / 0x6081 1 단위 = 0.001 rad/s


def wait_status(node, mask, value, timeout=2.0):
    """statusword(0x6041) 의 mask 비트가 value 가 될 때까지 SDO 로 확인"""
    t0 = time.time()
    sw = 0
    while time.time() - t0 < timeout:
        sw = node.sdo["Statusword"].raw
        if sw & mask == value:
            return sw
        time.sleep(0.01)
    raise TimeoutError(f"statusword 0x{sw:04X}: mask 0x{mask:04X} != 0x{value:04X}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="slcan", help="slcan (Windows/COM) 또는 socketcan (Linux)")
    ap.add_argument("--channel", default="COM5", help="COM5 / slcan0 등")
    ap.add_argument("--node", type=int, default=1, help="CANopen 노드 ID (축 1~3 = 1~3)")
    ap.add_argument("--move", type=float, default=1.0, help="이동량 [rad]")
    args = ap.parse_args()

    net = canopen.Network()
    net.connect(interface=args.interface, channel=args.channel, bitrate=1000000)
    node = canopen.RemoteNode(args.node, EDS)
    net.add_node(node)

    latest = {}

    def on_tpdo2(pdo_map):
        for var in pdo_map:
            latest[var.name] = var.raw

    try:
        # 1) NMT: 통신 리셋 -> boot-up 대기
        node.nmt.state = "RESET COMMUNICATION"
        node.nmt.wait_for_bootup(5)

        # 2) SDO 로 장치 정보 읽기
        print("Device type :", hex(node.sdo["Device type"].raw))
        print("Device name :", node.sdo["Manufacturer device name"].raw)
        print("Motor CAN ID:", node.sdo["AK motor"]["Motor CAN ID"].raw)
        print("Supported   :", hex(node.sdo["Supported drive modes"].raw))

        # 3) PDO 설정을 장치에서 읽고, TPDO2(위치/속도) 수신 콜백 등록
        node.tpdo.read()
        node.tpdo[2].add_callback(on_tpdo2)
        node.nmt.state = "OPERATIONAL"

        # 4) Profile Position 설정
        node.sdo["Modes of operation"].raw = 1
        node.sdo["Profile velocity"].raw = int(2.0 / VEL_UNIT)       # 2 rad/s
        node.sdo["Profile acceleration"].raw = int(10.0 / VEL_UNIT)  # 10 rad/s^2

        # 5) CiA 402 상태머신: (고장 리셋) -> Shutdown -> Switch on -> Enable operation
        node.sdo["Controlword"].raw = 0x0080
        node.sdo["Controlword"].raw = 0x0006
        wait_status(node, 0x006F, 0x0021)
        node.sdo["Controlword"].raw = 0x0007
        wait_status(node, 0x006F, 0x0023)
        node.sdo["Controlword"].raw = 0x000F
        wait_status(node, 0x006F, 0x0027)
        print("Operation enabled")

        # 6) 이동: 목표 쓰기 -> bit4(new set-point) 올림 -> ack 확인 -> bit4 내림 -> target reached 대기
        start = node.sdo["Position actual value"].raw
        for target in (start + int(args.move / POS_UNIT), start):
            node.sdo["Target position"].raw = target
            node.sdo["Controlword"].raw = 0x001F
            wait_status(node, 0x1000, 0x1000)
            node.sdo["Controlword"].raw = 0x000F
            wait_status(node, 0x0400, 0x0400, timeout=10)
            pos = latest.get("Position actual value", 0) * POS_UNIT
            vel = latest.get("Velocity actual value", 0) * VEL_UNIT
            print(f"target {target * POS_UNIT:+.3f} rad reached | TPDO2 pos {pos:+.3f} rad, vel {vel:+.3f} rad/s")
    finally:
        try:
            node.sdo["Controlword"].raw = 0x0006  # Shutdown: 모터 정지
        except Exception as exc:  # noqa: BLE001 - 정리 단계라 실패해도 계속
            print("shutdown failed:", exc)
        net.disconnect()


if __name__ == "__main__":
    main()
