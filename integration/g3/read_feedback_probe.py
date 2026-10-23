"""Decode B1 passive BMI/encoder diagnostics. UART receive only, never sends."""
import argparse,csv,json,struct,sys,time
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'control/tools/fpga_telemetry'))
from protocol import crc16


class Decoder:
    def __init__(self):self.buffer=bytearray();self.bad_crc=0
    def feed(self,data):
        self.buffer.extend(data);rows=[]
        while len(self.buffer)>=4:
            start=self.buffer.find(b'\xa5\x5a\xb1\x78')
            if start<0:self.buffer[:]=self.buffer[-3:];break
            del self.buffer[:start]
            if len(self.buffer)<126:break
            packet=self.buffer[:126]
            if crc16(packet[2:124])!=int.from_bytes(packet[124:126],'little'):
                self.bad_crc+=1;del self.buffer[0];continue
            del self.buffer[:126];p=packet[4:124]
            u16=lambda n:struct.unpack_from('<H',p,n)[0]
            u32=lambda n:struct.unpack_from('<I',p,n)[0]
            s32=lambda n:struct.unpack_from('<i',p,n)[0]
            if u16(116)!=0xbc02:continue
            row=dict(timestamp_ms=u32(0),sequence=u16(4),status=p[6],error_code=p[7],operation=p[8],
                accel_address=p[9],gyro_address=p[10],accel_id=p[11],gyro_id=p[12],last_read=p[13],
                bmi_sample_sequence=u32(14),bmi_age_ms=u16(18),encoder_sample_sequence=u32(32),encoder_age_ms=u16(36),
                left_delta=s32(38),right_delta=s32(42),left_position=s32(46),right_position=s32(50),
                left_invalid=u32(54),right_invalid=u32(58),id_seen=p[114],encoder_flags=p[115])
            row.update(zip(('ax_raw','ay_raw','az_raw','gx_raw','gy_raw','gz_raw'),struct.unpack_from('<6h',p,20)))
            for i,address in enumerate(('18','19','68','69')):
                row['ack_'+address]=u32(62+4*i);row['nack_'+address]=u32(78+4*i)
                row['id_reads_'+address]=u32(94+4*i);row['last_id_'+address]=p[110+i]
            # An initialized raw publication is separate from its age and calibration.
            row['bmi_initialized_sample']=bool(row['status']&1 and not row['status']&2 and row['status']&4 and row['accel_id']==0x1e and row['gyro_id']==0x0f)
            row['bmi_fresh_50ms']=row['bmi_initialized_sample'] and row['bmi_age_ms']<50
            rows.append(row)
        return rows


def summarize(rows,decoder):
    sequences={r['encoder_sample_sequence'] for r in rows if r['encoder_sample_sequence']}
    return dict(diagnostic_frames=len(rows),crc_errors=decoder.bad_crc,
        distinct_encoder_publications=len(sequences),distinct_bmi_publications=len({r['bmi_sample_sequence']for r in rows if r['bmi_initialized_sample']}),
        last_frame=rows[-1]if rows else None,
        scope='raw diagnostic snapshots; no fusion/calibration, exact gate-edge position, motion or physical-stop acceptance')


def main():
    p=argparse.ArgumentParser();s=p.add_mutually_exclusive_group(required=True);s.add_argument('--port');s.add_argument('--replay',type=Path)
    p.add_argument('--seconds',type=float,default=30);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    if not 0.1<=a.seconds<=3600:p.error('seconds out of range')
    a.out.mkdir(parents=True,exist_ok=False);d=Decoder();rows=[];start=time.monotonic();error=None
    with (a.out/'rx.bin').open('wb')as f:
        def consume(b):f.write(b);f.flush();rows.extend(d.feed(b))
        try:
            if a.replay:consume(a.replay.read_bytes())
            else:
                import serial
                device=serial.Serial(port=None,baudrate=230400,timeout=.1,rtscts=False,dsrdtr=False,xonxoff=False)
                device.dtr=False;device.rts=False;device.port=a.port
                with device:
                    while time.monotonic()-start<a.seconds:consume(device.read(4096))
        except(OSError,ImportError)as e:error=str(e)
    if rows:
        with(a.out/'frames.csv').open('w',newline='',encoding='utf-8')as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),source='offline_replay'if a.replay else 'hardware_uart',
        port=a.port,tx_bytes=0,error=error,**summarize(rows,d))
    (a.out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
    return 0 if rows and error is None else 2


if __name__=='__main__':raise SystemExit(main())
