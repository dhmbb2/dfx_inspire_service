#!/usr/bin/env python3
"""Fake Inspire RH56 hand on a pseudo-terminal, for testing the driver without hardware.

Prints the pty path to open (e.g. /dev/pts/7) on the first line, then serves the RH56 serial
protocol: register reads and writes, fingers moving toward ANGLE_SET at the SPEED_SET speed.
Optional faults reproduce what upsets a fragile driver:
  --reply-delay MS   delay before every reply
  --ack-delay MS     extra delay before write acknowledgements
  --latency MS       deliver outgoing bytes only every MS (like an FTDI latency timer)
  --drop P           probability of not answering a request
  --garbage P        probability of sending a random byte before a reply
Stdlib only:  python3 fake_rh56.py --id 1 --seconds 60 --latency 16 --ack-delay 8
"""
import argparse
import os
import random
import select
import struct
import sys
import threading
import time
import tty

ANGLE_SET, FORCE_SET, SPEED_SET, ANGLE_ACT = 1486, 1498, 1522, 1546
FORCE_ACT, CURRENT, ERROR, STATUS, TEMP = 1582, 1594, 1606, 1612, 1618
CLEAR_ERROR, FORCE_CLB = 1004, 1009
FULL_RANGE_S = 0.8          # full stroke time at speed 1000


class Hand:
    def __init__(self, hid, start):
        self.id = hid
        self.mem = bytearray(2048)
        self.lock = threading.Lock()
        self.q = [float(v) for v in start]
        self._set6(ANGLE_SET, [1000] * 6)
        self._set6(SPEED_SET, [1000] * 6)
        self._set6(FORCE_SET, [500, 500, 500, 500, 500, 259])
        self._set6(FORCE_ACT, [66, -18, 15, 592, 669, -195])
        self.mem[STATUS:STATUS + 6] = bytes([255] * 6)
        self.mem[TEMP:TEMP + 6] = bytes([30] * 6)
        self.n_read = self.n_write = self.n_angle_write = 0
        self._sync()

    def _set6(self, addr, vals):
        self.mem[addr:addr + 12] = struct.pack("<6h", *vals)

    def _get6(self, addr):
        return list(struct.unpack("<6h", bytes(self.mem[addr:addr + 12])))

    def _sync(self):
        self._set6(ANGLE_ACT, [int(round(v)) for v in self.q])

    def step(self, dt):
        with self.lock:
            tgt, spd = self._get6(ANGLE_SET), self._get6(SPEED_SET)
            st = []
            for i in range(6):
                d = tgt[i] - self.q[i]
                vmax = max(spd[i], 1) / 1000.0 * 1000.0 / FULL_RANGE_S * dt
                self.q[i] += max(-vmax, min(vmax, d))
                st.append(2 if abs(tgt[i] - self.q[i]) < 1 else (0 if d > 0 else 1))
            self.mem[STATUS:STATUS + 6] = bytes(st)
            self._sync()

    def read(self, addr, n):
        with self.lock:
            self.n_read += 1
            return bytes(self.mem[addr:addr + n])

    def write(self, addr, data):
        with self.lock:
            self.n_write += 1
            if addr == ANGLE_SET:
                self.n_angle_write += 1
                new = list(struct.unpack(f"<{len(data) // 2}h", data))
                old = self._get6(ANGLE_SET)
                self._set6(ANGLE_SET, [o if v == -1 else max(0, min(1000, v)) for o, v in zip(old, new)])
            elif addr == FORCE_CLB:
                self._set6(FORCE_ACT, [0] * 6)
            elif addr == CLEAR_ERROR:
                self.mem[ERROR:ERROR + 6] = bytes(6)
            else:
                self.mem[addr:addr + len(data)] = data
            return True


def frame(hid, cmd, addr, payload):
    body = bytes([hid, 3 + len(payload), cmd, addr & 0xFF, addr >> 8]) + payload
    return b"\x90\xeb" + body + bytes([sum(body) & 0xFF])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--start", default="0,6,0,232,321,196", help="initial ANGLE_ACT")
    ap.add_argument("--reply-delay", type=float, default=1.0)
    ap.add_argument("--ack-delay", type=float, default=0.0)
    ap.add_argument("--latency", type=float, default=0.0)
    ap.add_argument("--drop", type=float, default=0.0)
    ap.add_argument("--garbage", type=float, default=0.0)
    a = ap.parse_args()

    hand = Hand(a.id, [int(v) for v in a.start.split(",")])
    master, slave = os.openpty()
    tty.setraw(master)
    print(os.ttyname(slave), flush=True)
    out_lock, outq = threading.Lock(), []          # (release time, bytes)
    stop = time.monotonic() + a.seconds

    def sender():
        next_tick = time.monotonic()
        while time.monotonic() < stop:
            now = time.monotonic()
            with out_lock:
                if a.latency > 0:                   # FTDI-like: flush only on latency ticks
                    if now < next_tick:
                        ready = []
                    else:
                        next_tick = now + a.latency / 1000
                        ready = [b for t, b in outq if t <= now]
                else:
                    ready = [b for t, b in outq if t <= now]
                if ready:
                    outq[:] = [(t, b) for t, b in outq if t > now]
            for b in ready:
                os.write(master, b)
            time.sleep(0.0005)

    def mover():
        while time.monotonic() < stop:
            hand.step(0.005)
            time.sleep(0.005)

    threading.Thread(target=sender, daemon=True).start()
    threading.Thread(target=mover, daemon=True).start()
    buf = b""
    while time.monotonic() < stop:
        r, _, _ = select.select([master], [], [], 0.05)
        if not r:
            continue
        try:
            buf += os.read(master, 256)
        except OSError:
            break
        while True:
            i = buf.find(b"\xeb\x90")
            if i < 0:
                buf = buf[-1:]
                break
            buf = buf[i:]
            if len(buf) < 4 or len(buf) < 5 + buf[3]:
                break
            n = 5 + buf[3]
            f, buf = buf[:n], buf[n:]
            if sum(f[2:-1]) & 0xFF != f[-1] or f[2] != hand.id:
                continue
            cmd, addr, payload = f[4], f[5] | f[6] << 8, f[7:-1]
            if random.random() < a.drop:
                continue
            delay = a.reply_delay / 1000
            if cmd == 0x11:
                reply = frame(hand.id, 0x11, addr, hand.read(addr, payload[0]))
            elif cmd == 0x12:
                hand.write(addr, payload)
                reply = frame(hand.id, 0x12, addr, b"\x01")
                delay += a.ack_delay / 1000
            else:
                continue
            if random.random() < a.garbage:
                reply = bytes([random.randrange(256)]) + reply
            with out_lock:
                outq.append((time.monotonic() + delay, reply))
    print(f"[fake id {hand.id}] reads {hand.n_read}, writes {hand.n_write} (angle {hand.n_angle_write}), "
          f"final angles {[int(round(v)) for v in hand.q]}", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
