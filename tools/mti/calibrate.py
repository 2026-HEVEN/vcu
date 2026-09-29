"""MTi Xbus inclination reset (not factory reset), via ESP32 UART bridge."""
import argparse
import json
import math
import statistics
import struct
import time


def select_port(ports, explicit=None):
    """Select one USB serial port without opening/probing unrelated devices."""
    if explicit:
        return explicit
    candidates = []
    for p in ports:
        identity = " ".join(str(getattr(p, k, "") or "")
                            for k in ("description", "hwid", "manufacturer")).lower()
        if "bluetooth" in identity or "bthenum" in identity:
            continue
        if getattr(p, "vid", None) is not None and getattr(p, "pid", None) is not None:
            candidates.append(p)
    if len(candidates) == 1:
        p = candidates[0]
        print("Auto-selected USB serial:", p.device, p.description, flush=True)
        return p.device
    if not candidates:
        raise ValueError("No USB serial device found. Connect ESP32 or specify --port.")
    names = ", ".join(p.device + " (" + p.description + ")" for p in candidates)
    raise ValueError("Multiple USB serial devices: " + names +
                     ". Disconnect other boards or specify --port / upload_port; not guessing.")


def packet(mid, payload=b""):
    body = bytes((0xFF, mid, len(payload))) + payload
    return b"\xfa" + body + bytes((-sum(body) & 255,))


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buf.extend(data)
        while self.buf:
            if self.buf[0] != 0xFA:
                del self.buf[0]
                continue
            if len(self.buf) < 5:
                return
            header, size = 4, self.buf[3]
            if size == 255:
                if len(self.buf) < 7:
                    return
                header, size = 6, int.from_bytes(self.buf[4:6], "big")
            if size > 4096:
                del self.buf[0]
                self.errors += 1
                continue
            end = header + size + 1
            if len(self.buf) < end:
                return
            frame = self.buf[:end]
            if sum(frame[1:]) & 255:
                del self.buf[0]
                self.errors += 1
                continue
            del self.buf[:end]
            yield frame[2], bytes(frame[header:-1])


def decode(payload):
    result = {}
    names = {0x2030: "rpy_deg", 0x4020: "acc_mps2", 0x8020: "gyro_rads"}
    while payload:
        if len(payload) < 3:
            raise ValueError("Truncated MTData2 header")
        xid, size = int.from_bytes(payload[:2], "big"), payload[2]
        data, payload = payload[3:3+size], payload[3+size:]
        if len(data) != size:
            raise ValueError("Truncated MTData2 payload")
        if (xid & 0xFFF0) in names:
            if (xid & 0x000F) != 0 or size != 12:
                raise ValueError("Expected float32 ENU output; configure MTi first")
            values = struct.unpack(">fff", data)
            if not all(math.isfinite(x) for x in values):
                raise ValueError("Non-finite sensor data")
            result[names[xid & 0xFFF0]] = values
    return result


def validate(sample, zero=False):
    for key in ("rpy_deg", "acc_mps2", "gyro_rads"):
        if sample.get(key + "_count", 0) < 30:
            raise ValueError("Need at least 30 fresh samples of " + key)
    if max(abs(x) for x in sample["gyro_rads"]) > 0.03 or max(sample["acc_mps2_std"]) > 0.1:
        raise ValueError("Vehicle is moving/vibrating; not storing")
    gravity = math.sqrt(sum(x*x for x in sample["acc_mps2"]))
    if not 9.3 < gravity < 10.3:
        raise ValueError("Unexpected gravity magnitude; not storing")
    if zero and (max(abs(x) for x in sample["rpy_deg"][:2]) > 2 or
                 max(abs(x) for x in sample["acc_mps2"][:2]) > 0.2):
        raise ValueError("Zero verification failed; not storing")


class Connection:
    def __init__(self, ser):
        self.ser = ser
        self.parser = Parser()

    def collect(self, seconds):
        end = time.monotonic() + seconds
        frames = []
        while time.monotonic() < end:
            frames.extend(self.parser.feed(self.ser.read(self.ser.in_waiting or 1)))
        return frames

    def command(self, mid, payload=b""):
        print("TX", packet(mid, payload).hex(" "), flush=True)
        self.ser.write(packet(mid, payload))
        self.ser.flush()
        frames = self.collect(2)
        for rid, data in frames:
            if rid == 0x42:
                raise RuntimeError("MTi error: " + data.hex())
        replies = [data for rid, data in frames if rid == mid + 1]
        if not replies:
            raise RuntimeError("No ACK for 0x%02X; no automatic retry/store" % mid)
        print("ACK 0x%02X" % (mid + 1), replies[-1].hex(), flush=True)
        return replies[-1]

    def sample(self, label):
        # Discard settling data, then use a fresh fixed-duration window.
        self.collect(1)
        errors = self.parser.errors
        rows = [decode(data) for mid, data in self.collect(3) if mid == 0x36]
        result = {"label": label, "frames": len(rows)}
        for key in ("rpy_deg", "acc_mps2", "gyro_rads"):
            values = [v[key] for v in rows if key in v]
            result[key + "_count"] = len(values)
            if values:
                result[key] = [statistics.mean(x) for x in zip(*values)]
                result[key + "_std"] = [statistics.pstdev(x) for x in zip(*values)]
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if self.parser.errors != errors:
            raise RuntimeError("Checksum errors in measurement window")
        return result


def main():
    import serial
    from serial.tools import list_ports
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("mode", choices=("check", "zero"))
    ap.add_argument("--port", help="Explicit ESP32 port; otherwise auto-select the sole USB serial device")
    args = ap.parse_args()
    print("Requires mti_bridge firmware, RX19/TX21, 115200. Close all serial monitors/MT Manager.")
    args.port = select_port(list_ports.comports(), args.port)
    if args.mode == "zero":
        print("Vehicle level and stationary; sensor fixed; HV/motor power OFF.")
        print("This stores current roll/pitch alignment in the MTi, not a factory or heading reset.")
        if input("Type ZERO to confirm these conditions: ").strip() != "ZERO":
            raise ValueError("Cancelled; no serial commands sent")
    ser = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=1)
    ser.dtr = ser.rts = False
    ser.port = args.port
    conn = Connection(ser)
    entered_config = False
    try:
        ser.open()
        conn.collect(2)  # ESP32 startup; port open can reset the bridge.
        entered_config = True
        conn.command(0x30)
        did = conn.command(0x00)
        if len(did) != 4:
            raise RuntimeError("Unexpected device ID response")
        print("MTi device ID:", did.hex().upper())
        if args.mode == "zero" and input("Type this device ID to apply: ").strip().upper() != did.hex().upper():
            raise ValueError("Device confirmation cancelled")
        conn.command(0x10)
        entered_config = False
        before = conn.sample("before")
        validate(before)
        if args.mode == "zero":
            conn.command(0xA4, b"\x00\x03")
            validate(conn.sample("after_zero"), zero=True)
            entered_config = True
            conn.command(0x30)
            conn.command(0xA4, b"\x00\x00")
            print("STORE acknowledged; persistent alignment changed.")
            conn.command(0x10)
            entered_config = False
            validate(conn.sample("after_store"), zero=True)
            print("PASS: zero/store verified. Power-cycle MTi and run mti_check to verify persistence.")
    finally:
        if ser.is_open and entered_config:
            try:
                conn.command(0x10)
            except Exception as exc:
                print("Could not restore measurement mode:", exc)
        ser.close()
        print("Port closed. Restore esp32dev firmware before driving.")


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print("STOP:", exc)
        raise SystemExit(1)
