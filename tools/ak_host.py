#!/usr/bin/env python3
"""
중앙 제어부 예제: pico_zero-canopen 보드에 USB 시리얼로 명령을 보내고 피드백을 받는다.

    pip install pyserial
    python tools/ak_host.py COM5            # 연결 확인 (INFO / STAT / GET)
    python tools/ak_host.py COM5 --demo     # 3개 모터에 MIT 사인파 (모터가 움직임!)

주의: 포트를 1200 baud 로 열면 보드가 BOOTSEL(펌웨어 업로드) 모드로 재부팅된다.
      USB CDC 라 baud 값은 의미가 없으니 115200 으로 두면 된다.
"""
import argparse
import math
import threading
import time

import serial  # pyserial


class AkBridge:
    """pico_zero-canopen 텍스트 프로토콜 클라이언트"""

    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.02)
        self.ser.dtr = True  # 펌웨어는 DTR 이 켜져야 출력함
        self.fb = {}  # motor(1..3) -> dict(pos, vel, torque, cur, temp, err, age_ms, t)
        self.lock = threading.Lock()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # ---------------- 송신 ----------------
    def send(self, line: str) -> None:
        self.ser.write((line + "\n").encode("ascii"))

    def mit(self, m, p, v, kp, kd, t) -> str:
        """MIT 명령 문자열 (rad, rad/s, -, -, Nm)"""
        return f"MIT {m} {p:.4f} {v:.4f} {kp:.3f} {kd:.3f} {t:.4f}"

    def send_many(self, cmds) -> None:
        """여러 명령을 ';' 로 묶어 한 줄로 보냄 (USB 패킷 1개)"""
        self.send(";".join(cmds))

    def stop(self, m="*") -> None:
        self.send(f"STOP {m}" if m != "*" else "STOP")

    def close(self) -> None:
        try:
            self.stop()
            time.sleep(0.05)
        finally:
            self._running = False
            self.ser.close()

    # ---------------- 수신 ----------------
    def _read_loop(self) -> None:
        buf = b""
        while self._running:
            try:
                buf += self.ser.read(4096)
            except serial.SerialException:
                break
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle(raw.decode("utf-8", "replace").strip())

    def _handle(self, line: str) -> None:
        if not line:
            return
        parts = line.split()
        if parts[0] == "FB" and len(parts) == 9:
            m = int(parts[1])
            pos, vel, tq, cur = map(float, parts[2:6])
            with self.lock:
                self.fb[m] = dict(pos=pos, vel=vel, torque=tq, cur=cur, temp=int(parts[6]),
                                  err=int(parts[7]), age_ms=int(parts[8]), t=time.time())
        elif parts[0] == "FB":
            pass  # "FB m nodata"
        else:
            print("<<", line)  # OK / ERR / EVT / INFO / STAT ...

    def get_fb(self, m):
        with self.lock:
            return dict(self.fb.get(m, {}))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="예: COM5 또는 /dev/ttyACM0")
    ap.add_argument("--demo", action="store_true", help="MIT 사인파로 모터 3개 구동")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--rate", type=float, default=200.0, help="호스트 명령 주기 [Hz]")
    args = ap.parse_args()

    br = AkBridge(args.port)
    time.sleep(0.2)
    br.send("PING")
    br.send("INFO")
    br.send("TELEM 50")
    br.send("TIMEOUT 300")  # 300 ms 동안 명령 없으면 보드가 모터 정지
    time.sleep(0.3)
    br.send("STAT")
    br.send("GET")
    time.sleep(0.2)

    if not args.demo:
        br.close()
        return

    print("데모 시작: 3개 모터 MIT 위치 사인파 (진폭 0.5 rad, kp=8, kd=0.3)")
    period = 1.0 / args.rate
    t0 = time.time()
    next_t = t0
    last_print = 0.0
    try:
        while time.time() - t0 < args.seconds:
            t = time.time() - t0
            cmds = []
            for m in (1, 2, 3):
                p = 0.5 * math.sin(2 * math.pi * 0.5 * t + (m - 1) * 2.0944)
                cmds.append(br.mit(m, p, 0.0, 8.0, 0.3, 0.0))
            br.send_many(cmds)

            if t - last_print > 0.5:
                last_print = t
                for m in (1, 2, 3):
                    fb = br.get_fb(m)
                    if fb:
                        print(f"m{m} pos={fb['pos']:+.3f}rad vel={fb['vel']:+.2f}rad/s "
                              f"tq={fb['torque']:+.2f}Nm T={fb['temp']}C err={fb['err']}")
            next_t += period
            time.sleep(max(0.0, next_t - time.time()))
    except KeyboardInterrupt:
        pass
    finally:
        br.close()
        print("정지")


if __name__ == "__main__":
    main()
