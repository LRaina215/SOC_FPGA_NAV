"""Read explicitly selected board sensors; never arm or drive actuators."""
import argparse,json,struct,time
from pathlib import Path
from datetime import datetime,timezone
from transport import SerialExchange,ping
from protocol import Packet

def decode(payload):
    if len(payload)!=64: raise ValueError('telemetry must be exactly64bytes')
    ticks,seq,it=struct.unpack_from('<III',payload)
    acc=struct.unpack_from('<hhh',payload,12);gyro=struct.unpack_from('<hhh',payload,18)
    es,et,angle,status,error,flags=struct.unpack_from('<IIHBBB',payload,24)
    right=struct.unpack_from('<IIHBBB',payload,40)
    diagnostic=dict(operation=payload[56],expected=payload[57],actual=payload[58],accel_id=payload[59],gyro_id=payload[60],cs_swapped=bool(payload[63]&8),probe_complete=bool(payload[63]&16),gyro_probe_cs1=payload[61],gyro_probe_cs2=payload[62])
    return dict(right_encoder_sequence=right[0],right_encoder_ticks=right[1],right_encoder_angle=right[2],
        right_encoder_status=right[3],right_encoder_error=right[4],right_encoder_ready=bool(right[5]&1),
        right_encoder_fault=bool(right[5]&2),right_encoder_frame=int.from_bytes(payload[53:56],'little'),
        input_levels=dict(spi_miso=bool(payload[63]&1),left_do=bool(payload[63]&2),right_do=bool(payload[63]&4)),imu_diagnostic=diagnostic,ticks=ticks,imu_sequence=seq,imu_ticks=it,accel_raw=acc,gyro_raw=gyro,
        encoder_sequence=es,encoder_ticks=et,encoder_angle=angle,encoder_status=status,encoder_error=error,
        imu_ready=bool(flags&1),imu_error=bool(flags&2),encoder_ready=bool(flags&4),encoder_fault=bool(flags&8),
        encoder_frame=int.from_bytes(payload[37:40],'little'),
        imu_age_seconds=((ticks-it)&0xffffffff)/50000000,
        encoder_age_seconds=((ticks-et)&0xffffffff)/50000000)

def main():
    p=argparse.ArgumentParser();p.add_argument('--port',required=True);p.add_argument('--seconds',type=float,default=5);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not 0<a.seconds<=60:raise ValueError('seconds must be in(0,60]')
    r=dict(mode='physical_sensor_readout',started_utc=datetime.now(timezone.utc).isoformat(),port=a.port,samples=[],actuator_outputs='constant_zero',status='running')
    x=None
    try:
        x=SerialExchange(a.port,2);r['ping_seconds']=ping(x,200000);start=time.monotonic();frame=200001
        while time.monotonic()-start<a.seconds:
            packet,elapsed=x.exchange(Packet(3,frame,b''),0x83)
            row=decode(packet.payload);row.update(frame=frame,roundtrip_seconds=elapsed);r['samples'].append(row);frame+=1
            if len(r['samples'])==1:print(json.dumps(row),flush=True)
            time.sleep(.003)
        r['status']='captured'
        rows=r['samples'];r['imu_seen']=any(s['imu_sequence']>0 and s['imu_ready'] and not s['imu_error'] for s in rows)
        r['right_encoder_seen']=any(s['right_encoder_sequence']>0 and s['right_encoder_ready'] and not s['right_encoder_fault'] for s in rows)
        r['encoder_seen']=any(s['encoder_sequence']>0 and s['encoder_ready'] and not s['encoder_fault'] for s in rows)
        dt=((rows[-1]['ticks']-rows[0]['ticks'])&0xffffffff)/50000000
        if dt>0:
            r['imu_publish_hz']=((rows[-1]['imu_sequence']-rows[0]['imu_sequence'])&0xffffffff)/dt
            r['encoder_crc_publish_hz']=((rows[-1]['encoder_sequence']-rows[0]['encoder_sequence'])&0xffffffff)/dt
        print(json.dumps({k:v for k,v in r.items() if k!='samples'},indent=2),flush=True)
    except Exception as e:r.update(status='failed',error=repr(e));raise
    finally:
        if x:x.close()
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(r,indent=2)+'\n')
if __name__=='__main__':main()
