"""Backward-compatible V1/V2 FPGA telemetry, little endian and CRC protected."""
import math
import struct

MAGIC = b'\xa5\x5a'
PAYLOAD = struct.Struct('<IH6B6h6i3i2i2h2h4HII2i')
assert PAYLOAD.size == 100
NAMES = ('timestamp_ms sequence status policy_fault sensor_error left_fault right_fault fusion_flags '
         'ax_raw ay_raw az_raw gx_raw gy_raw gz_raw '
         'wx_q12 wy_q12 wz_q12 gravity_x_q12 gravity_y_q12 gravity_z_q12 '
         'forward_q12 yaw_q12 height_q12 left_target_mmps right_target_mmps '
         'left_cms right_cms left_duty right_duty lh_us lk_us rh_us rk_us '
         'completed_frames inference_cycles left_delta right_delta').split()
EXTRA_NAMES = ('left_reference_mmps right_reference_mmps left_error_mmps right_error_mmps '
               'left_control_sequence right_control_sequence').split()
PAYLOAD_V2 = struct.Struct(PAYLOAD.format+'4i2H')
assert PAYLOAD_V2.size == 120
FORMATS = {(1, 100): (PAYLOAD, NAMES), (2, 120): (PAYLOAD_V2, NAMES+EXTRA_NAMES)}



def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.skipped = 0
        self.last_sequence = None

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 4:
            index = self.buffer.find(MAGIC)
            if index < 0:
                self.buffer[:] = self.buffer[-1:]
                break
            if index:
                del self.buffer[:index]
            if len(self.buffer) < 4:
                break
            header = (self.buffer[2], self.buffer[3])
            if header not in FORMATS:
                del self.buffer[0]
                continue
            length = header[1]
            packet_size = length+6
            if len(self.buffer) < packet_size:
                break
            packet = self.buffer[:packet_size]
            if crc16(packet[2:length+4]) != int.from_bytes(packet[length+4:packet_size], 'little'):
                self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:packet_size]
            payload, names = FORMATS[header]
            row = dict(zip(names, payload.unpack(packet[4:length+4])))
            row['protocol_version'] = header[0]
            for name in EXTRA_NAMES:
                row.setdefault(name, None)
            if self.last_sequence is not None:
                delta = (row['sequence']-self.last_sequence) & 0xffff
                if 1 < delta < 32768:
                    self.skipped += delta-1
            self.last_sequence = row['sequence']
            frames.append(physical_units(row))
        return frames


def physical_units(row):
    row = dict(row)
    row['time_s'] = row['timestamp_ms']/1000.
    for axis in 'xyz':
        row['gyro_'+axis] = row['w'+axis+'_q12']/4096.
        row['accel_'+axis+'_g'] = row['a'+axis+'_raw']/10920.
    gx, gy, gz = [row['gravity_'+axis+'_q12']/4096. for axis in 'xyz']
    calibrated = not (row['fusion_flags'] & 3) and bool(row['status'] & 1)
    row['roll_deg'] = math.degrees(math.atan2(-gy, -gz)) if calibrated else float('nan')
    row['pitch_deg'] = math.degrees(math.atan2(gx, math.hypot(gy, gz))) if calibrated else float('nan')
    for key in ['forward', 'yaw', 'height']:
        row[key+'_command'] = row[key+'_q12']/4096.
    for side in ['left', 'right']:
        row[side+'_target'] = row[side+'_target_mmps']/1000.
        row[side+'_speed'] = row[side+'_cms']/100.
        row[side+'_pwm_percent'] = row[side+'_duty']*100./2500.
        for quantity in ('reference', 'error'):
            value = row.get(side+'_'+quantity+'_mmps')
            row[side+'_'+quantity] = value/1000. if value is not None else float('nan')
    row['wheel_mean_speed'] = (row['left_speed']+row['right_speed'])/2
    row['active'] = bool(row['status'] & 128)
    return row
