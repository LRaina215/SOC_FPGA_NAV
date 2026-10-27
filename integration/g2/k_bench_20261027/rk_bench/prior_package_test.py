"""Exercise NVMP v3 receive/ACK validation on the retained PNR61 image.

Only map transfer/clear/query and zero no-op heartbeats are sent. No motion,
pose, firmware load, CPU reset or actuator enable. Prior is cleared on exit.
"""
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import struct
import sys
import time
import zlib

sys.path.insert(0,str(Path(__file__).resolve().parent/'tools'))
import nav_host as nh


def package(slots=None,nx=2,ny=2):
    if slots is None:slots=[(0,0x3ffff),(16383,0x15555),(77,0x2aaaa)]
    body=b''.join(struct.pack('<H',s)+w.to_bytes(3,'little') for s,w in slots)+bytes([0]*(nx*ny))
    h=bytearray(48);h[:4]=b'NVMP'
    struct.pack_into('<HHiiIiiHHIIIhh',h,4,3,0x0107,-64,-64,len(slots),-1,-1,nx,ny,
        zlib.crc32(body),len(body),0x33580001,350,-300)
    return bytes(h)+body


def main():
    link=nh.open_link('uart:/dev/ttyACM0');rows=[];statuses=[];sent=Counter();started=time.monotonic()
    def send(b):
        assert b[:3]==b'NV\x01' and b[3] in (0x20,0x40,0x41,0x42,0x43,0x44)
        if b[3]==0x20:assert b==nh.target(0,0,found=0)
        if b[3]==0x44:assert b==nh.slf_config(set_=False)
        link.send(b);sent[f'0x{b[3]:02x}']+=1
    def receive(kind,timeout=1.5):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            raw=link.recv(0.025);m=nh.parse(raw) if raw else None
            if not m:continue
            if m.get('type')=='status':
                assert m['vx']==m['wz']==0,('Nonzero target; abort',m)
                statuses.append(m)
            if m.get('type')==kind:return m
        raise AssertionError(('Missing response',kind))
    retries=[]
    journal=open(Path(__file__).with_suffix('.journal.jsonl'),'w',buffering=1)
    def ack(b,state=None,nxt=None):
        # MBX TX retains only its newest reply, so periodic status can replace
        # an ACK before the host observes it. The map receiver explicitly
        # supports duplicate DATA/END/CLEAR; BEGIN is retried before any DATA.
        for attempt in range(4):
            send(b)
            journal.write(json.dumps(dict(event='request',type=b[3],bytes=len(b),attempt=attempt,
                expected_state=state,expected_next=nxt,host_s=time.monotonic()-started))+'\n')
            try:m=receive('map_ack',0.7)
            except AssertionError as e:
                if not e.args or e.args[0]!=('Missing response','map_ack'):raise
                retries.append(dict(type=b[3],attempt=attempt,reason='no ACK observed'))
                continue
            if state is not None and m['state_code']!=state:
                journal.write(json.dumps(dict(event='mismatched_state',reply=m))+'\n');continue
            if nxt is not None and m['next']!=nxt:
                journal.write(json.dumps(dict(event='mismatched_next',reply=m))+'\n');continue
            journal.write(json.dumps(dict(event='ACK',reply=m,host_s=time.monotonic()-started))+'\n')
            return m
        raise AssertionError(('No matching ACK after bounded retries',b[3],state,nxt))
    def transfer(name,p,expect=4,chunks=(48,1024),send_total=None):
        t=time.monotonic();total=len(p) if send_total is None else send_total
        ack(nh.map_begin(total),1,0);offset=0;i=0;last=None
        while offset<len(p):
            n=min(chunks[min(i,len(chunks)-1)],len(p)-offset)
            last=ack(nh.map_data(offset,p[offset:offset+n]))
            if last['state_code']!=1:break
            assert last['next']==offset+n,last
            offset+=n;i+=1
        last=ack(nh.map_end(),expect)
        rows.append(dict(name=name,bytes=len(p),state=last['state_code'],next=last['next'],
                         elapsed_s=round(time.monotonic()-t,6)))
        journal.write(json.dumps(dict(event='case_passed',case=rows[-1]))+'\n')
        ack(nh.map_clear(),0,0)

    send(nh.slf_config(set_=False));initial=receive('slf_ack')
    assert initial['window']=='前移 1000 mm' and initial['zhi_mm']==350
    touched=False
    try:
        touched=True;ack(nh.map_clear(),0,0)
        base=package()
        # Isolate each header/body failure, retaining the body CRC for all
        # geometry-record cases so a checksum failure cannot mask rejection.
        cases=[]
        for name,off,fmt,value in [
            ('legacy_version',4,'H',2),('wrong_geometry',6,'H',0x0108),
            ('wrong_profile',40,'I',0),('wrong_ZHI',44,'h',1500),('wrong_ZLO',46,'h',-299),
            ('grid_width65',28,'H',65),('grid_height65',30,'H',65),
            ('structure_count16385',16,'I',16385),('wrong_body_size',36,'I',999)]:
            p=bytearray(base);struct.pack_into('<'+fmt,p,off,value);cases.append((name,bytes(p)))
        p=bytearray(base);p[0]=ord('X');cases.append(('bad_magic',bytes(p)))
        for name,p in cases:transfer(name,p)
        for slot in (16384,65535):
            transfer('invalid_slot_'+str(slot),package([(0,1),(slot,2)]))
        transfer('invalid_word_19bits',package([(0,1),(7,0x40000)]))
        p=bytearray(base);p[32]^=1;transfer('bad_body_CRC',bytes(p),3)
        transfer('truncated_header',base[:17],4,send_total=len(base))
        transfer('truncated_body',base[:-1],4,send_total=len(base))
        transfer('valid_boundary_words_fragmented',base,2,chunks=(1,2,3,5,17,1))
        # Retry semantics: gap/duplicate data must not advance the accepted head.
        ack(nh.map_begin(len(base)),1,0);ack(nh.map_data(0,base[:48]),1,48)
        ack(nh.map_data(49,base[48:]),1,48)
        ack(nh.map_data(0,base[:48]),1,48)
        ack(nh.map_data(48,base[48:]),1,len(base));ack(nh.map_end(),2,len(base))
        rows.append(dict(name='gap_duplicate_then_retry',state=2,bytes=len(base)))
        ack(nh.map_clear(),0,0)
        # Actual maximum supported geometry and all16384 valid slot addresses.
        full=package([(i,(i*17)&0x3ffff) for i in range(16384)],64,64)
        assert len(full)==86064
        transfer('maximum_128_structure_64_grid',full,2,chunks=(48,1024))
        send(nh.slf_config(set_=False));final=receive('slf_ack')
        assert final==initial,(initial,final)
        result=dict(passed=True,ended_at=datetime.now().astimezone().isoformat(),cases=rows,
            case_count=len(rows),sent_counts=dict(sent),status_samples=len(statuses),
            all_observed_numeric_targets_zero=True,initial_SLF=initial,final_SLF=final,
            actual_CPU_mailbox_and_MAP_ACK=True,maximum_package_bytes=86064,
            no_motion_or_pose_or_firmware_changes=True,CPU_reset=False,
            final_map_state=0,IMU_required=False,elapsed_s=time.monotonic()-started,
            ACK_retry_events=retries,latest_reply_mailbox_requires_host_retry=True,
            caveat='Receive state/ACK validation; not numerical localization, physical stopping or full RAM readback equivalence')
        print(json.dumps(result,ensure_ascii=False))
    finally:
        if touched:ack(nh.map_clear(),0,0)


if __name__=='__main__':main()
