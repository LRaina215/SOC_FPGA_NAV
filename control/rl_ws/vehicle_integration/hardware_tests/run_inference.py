import sys,json,hashlib,time
from pathlib import Path
from datetime import datetime,timezone
DEPLOY=Path('/home/cyd/fpga/rl_ws2/vehicle_integration')
sys.path.insert(0,str(DEPLOY/'board/host'))
from transport import SerialExchange,ping,infer
from golden import load_vectors,compare_result
PORT='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5910012847-if00'
out=Path(__file__).resolve().parent/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
out.mkdir()
report={'mode':'physical_uart','port':PORT,'status':'running','results':[], 'vehicle_motion_tested':False}
class Capture:
 def __init__(self,serial): self.serial=serial; self.log=open(out/'wire.jsonl','w')
 def __getattr__(self,k): return getattr(self.serial,k)
 def write(self,data):
  n=self.serial.write(data);self.log.write(json.dumps({'direction':'tx','time':time.monotonic(),'hex':data[:n].hex()})+'\n');self.log.flush();return n
 def read(self,n):
  data=self.serial.read(n)
  if data:self.log.write(json.dumps({'direction':'rx','time':time.monotonic(),'hex':data.hex()})+'\n');self.log.flush()
  return data
 def close(self): self.serial.close();self.log.close()
x=None
try:
 report['build']=json.loads((DEPLOY/'board/reports/acceptance.json').read_text())['bitstream']
 vectors=[('synthetic',DEPLOY/'assets/vectors'),('stress',DEPLOY/'assets/vectors/stress')]
 report['vector_hashes']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for _,d in vectors for p in d.glob('*.mem')}
 x=SerialExchange(PORT,2.0);x._serial=Capture(x._serial)
 report['ping_seconds']=ping(x,9999);print('PASS hardware PING model digest',flush=True)
 sticky=None;frame=10000
 for name,directory in vectors:
  for i,sample in enumerate(load_vectors(directory)):
   result,elapsed=infer(x,frame,sample);sticky=compare_result(result,sample,sticky)
   report['results'].append({'dataset':name,'sample':i,'frame':frame,'comparison':'bit-exact','layers':425,'latent':3,'raw':6,'actions':6,'cycles':result.cycles,'flags':result.flags,'saturated':result.saturated,'seconds':elapsed})
   if i==0 or (i+1)%32==0 or name=='stress':print(f'PASS {name} {i+1} cycles={result.cycles}',flush=True)
   frame+=1
 report['status']='pass'
except Exception as exc:
 report['status']='failed';report['error']=repr(exc);print('FAIL',repr(exc),flush=True)
finally:
 if x:x.close()
 report['finished_utc']=datetime.now(timezone.utc).isoformat()
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 print('REPORT',out/'report.json',flush=True)
sys.exit(0 if report['status']=='pass' else 1)
