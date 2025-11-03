import serial

PORT = "COM7"   # e.g., "COM5"
BAUD = 1_000_000
IDS  = [1,2,3,4,5,6]

ADDR_SPEED = 46
ADDR_ACCEL = 48
INSTR_READ  = 0x02

def checksum(bytes_):
    return (~(sum(bytes_) & 0xFF)) & 0xFF

def packet(id_, instr, params):
    length = len(params) + 2
    body = bytes([id_, length, instr]) + params
    return b"\xFF\xFF" + body + bytes([checksum(body)])

def read_n(ser, id_, addr, nbytes):
    ser.reset_input_buffer()
    ser.write(packet(id_, INSTR_READ, bytes([addr, nbytes])))
    # Expected status: 0xFF 0xFF, ID, LEN, ERR, PARAMS..., CHK
    hdr = ser.read(4)
    if len(hdr) < 4 or hdr[0:2] != b"\xFF\xFF":
        raise IOError("Bad header")
    sid, length = hdr[2], hdr[3]
    rest = ser.read(length)  # ERR + PARAMS + CHK
    if len(rest) != length:
        raise IOError("Truncated status")
    err  = rest[0]
    params = rest[1:-1]
    chk_expected = rest[-1]
    if checksum(bytes([sid, length]) + bytes([0]) + params) != chk_expected:
        # Many firmwares compute checksum on [ID, LEN, ERR, PARAMS]
        if checksum(bytes([sid, length, err]) + params) != chk_expected:
            raise IOError("Checksum mismatch")
    if err != 0:
        raise IOError(f"Servo {sid} error: 0x{err:02X}")
    return params

def main():
    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        print("ID  Speed  Accel")
        for i in IDS:
            # Read 4 bytes starting at ADDR_SPEED: speed(2), accel(2)
            params = read_n(ser, i, ADDR_SPEED, 4)
            speed = params[0] | (params[1] << 8)
            accel = params[2] | (params[3] << 8)
            print(f"{i:2d}  {speed:5d}  {accel:5d}")

if __name__ == "__main__":
    main()
