import serial
import time

PORT = "COM7"   # e.g., "COM5" on Windows
BAUD = 1_000_000
IDS  = [1,2,3,4,5,6]

speed_limit = 0
# SO-101 starting values (counts)
PROFILES = {
    1: {"speed":speed_limit, "accel":1000},
    2: {"speed":speed_limit, "accel":1000},
    3: {"speed":speed_limit, "accel":1000},
    4: {"speed":speed_limit, "accel":1000},
    5: {"speed":speed_limit, "accel":1000},
    6: {"speed":speed_limit, "accel":1000},
}

# Control table (Feetech STS3215 family, protocol 1.0-like)
ADDR_SPEED = 46      # uint16, steps/s
ADDR_ACCEL = 48      # uint16, unitless
INSTR_WRITE = 0x03
INSTR_SYNCW = 0x83

def checksum(bytes_):
    return (~(sum(bytes_) & 0xFF)) & 0xFF

def packet(id_, instr, params):
    length = len(params) + 2  # instr + checksum
    body = bytes([id_, length, instr]) + params
    return b"\xFF\xFF" + body + bytes([checksum(body)])

def write_u16(ser, id_, addr, value):
    params = bytes([addr, value & 0xFF, (value>>8) & 0xFF])
    ser.write(packet(id_, INSTR_WRITE, params))

def sync_write_profiles(ser, ids, speed_vals, accel_vals):
    # SYNC_WRITE: [0xFF 0xFF, ID=0xFE, LEN, 0x83, start_addr, bytes_per_servo, (id, data...), ...]
    start_addr = ADDR_SPEED
    bytes_per  = 4  # speed(2) + accel(2)
    header = bytes([start_addr, bytes_per])
    entries = b""
    for i in ids:
        sp = speed_vals[i]
        ac = accel_vals[i]
        entries += bytes([i, sp & 0xFF, (sp>>8)&0xFF, ac & 0xFF, (ac>>8)&0xFF])
    length = len(header) + len(entries) + 2
    body = bytes([0xFE, length, INSTR_SYNCW]) + header + entries
    pkt  = b"\xFF\xFF" + body + bytes([checksum(body)])
    ser.write(pkt)

def main():
    with serial.Serial(PORT, BAUD, timeout=0.02) as ser:
        # Option A: individual writes
        for i in IDS:
            p = PROFILES[i]
            write_u16(ser, i, ADDR_SPEED, p["speed"])
            time.sleep(0.002)
            write_u16(ser, i, ADDR_ACCEL, p["accel"])
            time.sleep(0.002)

        # Option B: one broadcast sync write (uncomment to use instead of individual writes)
        # speed_vals = {i: PROFILES[i]["speed"] for i in IDS}
        # accel_vals = {i: PROFILES[i]["accel"] for i in IDS}
        # sync_write_profiles(ser, IDS, speed_vals, accel_vals)

if __name__ == "__main__":
    main()
