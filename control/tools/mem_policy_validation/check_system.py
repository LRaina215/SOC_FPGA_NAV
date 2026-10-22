"""Check real integrated RTL snapshots/layers/actuators against integer Python model."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,'/home/cyd/fpga/pt_to_fpga/parking_speed_4500/reference')
from fixed_policy import FixedPolicy,read_mem
model=FixedPolicy.from_assets('/home/cyd/fpga/model/mem_4500')
inputs=read_mem('/tmp/policy_inputs.mem',24).reshape(-1,168)
layers=read_mem('/tmp/policy_layers.mem',24)
outputs=np.loadtxt('/tmp/policy_outputs.txt',dtype=np.int64).reshape(-1,14)
rom=Path('/home/cyd/fpga/fpga/src/mem')
ik=[read_mem(rom/f'leg_ik_{x}.mem',12) for x in ('lh','lk','rh','rk')]
def scale(x,gain):
 p=int(x)*gain
 return (abs(p)+2048)//4096 * (-1 if p<0 else 1)
for n,row in enumerate(outputs):
 obs,history=inputs[n,:28],inputs[n,28:]
 result=model.infer_fixed(obs,history)
 np.testing.assert_array_equal(result.all_layers,layers[n*425:(n+1)*425])
 np.testing.assert_array_equal(result.clipped,row[:6])
 a=result.clipped
 np.testing.assert_array_equal([scale(a[2],-600),scale(a[5],600)],row[6:8])
 height=row[12];idx=int(np.clip(height,348,778))-348
 expected=[int(np.clip(1500+table[idx]+scale(a[j],gain),500,2500)) for table,j,gain in zip(ik,[0,1,3,4],[19,-19,19,19])]
 np.testing.assert_array_equal(expected,row[8:12])
 assert row[13]==83534
 if n==0:
  np.testing.assert_array_equal(history,np.tile(obs,5))
 else:
  np.testing.assert_array_equal(history[:112],inputs[n-1,56:])
  np.testing.assert_array_equal(history[112:],obs)
  np.testing.assert_array_equal(obs[22:],outputs[n-1,:6])
 assert obs[9]==0
 print(f'frame {n+1}: exact 425 layer words, 6 actions, targets={row[6:8].tolist()} mm/s, servos={row[8:12].tolist()} us, height_q12={height}')
assert len(outputs)==4
assert outputs[0,12]==535 and outputs[1,12]==615
print('PASS integrated ROM numerical oracle, actuator directions/units, updated height IK, history and last-action feedback')
