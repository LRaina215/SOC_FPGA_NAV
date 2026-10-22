"""RK3588 -> FPGA target chassis speed. No serial port is opened by importing this file."""
import math
import struct


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def encode_speed(speed_mps, yaw_radps=0.0):
    """Return a 10-byte atomic command; m/s and rad/s, positive forward/left."""
    if not math.isfinite(speed_mps) or not -0.6 <= speed_mps <= 0.6:
        raise ValueError('speed_mps must be finite and within [-0.6, 0.6]')
    if not math.isfinite(yaw_radps) or not -2.9 <= yaw_radps <= 2.9:
        raise ValueError('yaw_radps must be finite and within [-2.9, 2.9]')
    payload = struct.pack('<BBhh', 0x10, 4, round(speed_mps * 1000), round(yaw_radps * 1000))
    return b'\xa5\x5a' + payload + struct.pack('<H', crc16(payload))


def send_speed(serial_port, speed_mps=0.3, yaw_radps=0.0):
    """Send an atomic chassis command through an already-open 230400 8N1 Serial."""
    frame = encode_speed(speed_mps, yaw_radps)
    if serial_port.write(frame) != len(frame):
        raise IOError('Incomplete speed command write')
