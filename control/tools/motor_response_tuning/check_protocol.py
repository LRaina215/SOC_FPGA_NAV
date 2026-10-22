import sys,struct,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/fpga_telemetry'))
from protocol import Decoder,crc16
raw=bytes(int(x,16) for x in Path('uart_v2_bytes.hex').read_text().split())
d=Decoder();frames=[]
for i in range(0,len(raw),7):frames+=d.feed(raw[i:i+7])
assert len(frames)==3 and d.bad_crc==0
times=[r['timestamp_ms'] for r in frames]
# Snapshot and the1ms tick can coincide; timestamp is the last completed ms.
assert times[0] in (9,10) and [b-a for a,b in zip(times,times[1:])]==[10,10]
assert [r['sequence'] for r in frames]==[0,1,2]
f=frames[0]
assert [f[k] for k in ['left_reference_mmps','right_reference_mmps','left_error_mmps','right_error_mmps','left_control_sequence','right_control_sequence']]==[-300,600,-50,25,12,34]
assert f['left_reference']==-.3 and f['right_reference']==.6
assert frames[1]['left_reference_mmps']==-1 and frames[1]['left_control_sequence']==65535
bad=bytearray(raw[:126]);bad[55]^=0x80
v1=bytearray(raw[:106]);v1[2:4]=bytes([1,100]);v1[104:106]=crc16(v1[2:104]).to_bytes(2,'little')
d=Decoder();mixed=d.feed(b'noise'+bad+v1+raw[:126])
assert len(mixed)==2 and d.bad_crc==1
assert [x['protocol_version'] for x in mixed]==[1,2]
assert set(mixed[0])==set(mixed[1]),'mixed versions must retain CSV schema'
assert mixed[0]['left_reference_mmps'] is None and math.isnan(mixed[0]['left_reference'])
assert crc16(b'123456789')==0x29b1
print('PASS V1/V2 decode: CRC, arbitrary fragments, corrupt-frame recovery, signed PID diagnostics, snapshot isolation, timestamps, stable mixed-version CSV schema')
