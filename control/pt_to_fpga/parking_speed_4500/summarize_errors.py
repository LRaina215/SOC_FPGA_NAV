"""Report full fixed-point error in normalized and physical output units."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
import torch
from export_policy import Policy
sys.path.insert(0,str(Path(__file__).parent/'reference'))
from fixed_policy import FixedPolicy,SCALE
torch.set_num_threads(2)
root=Path(__file__).resolve().parent;assets=root/'assets'
with np.load(assets/'inference_fp32.npz') as data:
    floating=Policy({k:torch.from_numpy(data[k].copy()) for k in data.files}).eval()
fixed=FixedPolicy.from_assets(assets)
report=json.loads((assets/'vectors/numerical_report.json').read_text())
names=['left_hip_residual','left_knee_residual','left_wheel_speed','right_hip_residual','right_knee_residual','right_wheel_speed']
def stats(delta):
    return {'max_abs':float(np.abs(delta).max()),'mean_abs':float(np.abs(delta).mean()),'rmse':float(np.sqrt(np.mean(delta**2)))}
datasets=[];all_errors=[]
for path in sorted((root/'vectors').glob('rollout_*.npz')):
    with np.load(path) as data:obs=data['obs'].reshape(-1,28);hist=data['history'].reshape(-1,140)
    with torch.no_grad():raw,clipped,latent=floating(torch.from_numpy(obs),torch.from_numpy(hist))
    result=fixed.infer_float(obs,hist)
    delta=result.clipped.astype(np.float64)/SCALE-clipped.numpy()
    all_errors.append(delta)
    datasets.append({'dataset':path.stem,'samples':len(obs),'clipped_error':stats(delta),'input_saturation':result.diagnostics['input_saturation'],'layer_saturation_count':sum(x['pre_elu_saturation_count'] for x in result.diagnostics['layers'].values())})
errors=np.concatenate(all_errors)
channels=[]
for i,name in enumerate(names):
    wheel=i in (2,5);scale=.6 if wheel else .03*180/np.pi
    channels.append({'channel':i,'name':name,'normalized_error':stats(errors[:,i]),'physical_unit':'m/s' if wheel else 'degree','physical_error':stats(errors[:,i]*scale)})
summary={'checkpoint_sha256':hashlib.sha256((root/'model_4500.pt').read_bytes()).hexdigest(),'trajectory_samples':len(errors),'trajectory_clipped_error':stats(errors),'synthetic_clipped_error':report['datasets'][0]['outputs']['clipped']['error_vs_original_float'],'channels':channels,'datasets':datasets,'scope':'Same-input open-loop comparison of original FP32 vs fully quantized integer inference, including input quantization, ELU lookup and intermediate rounding. This is not a new fixed-policy closed-loop validation.','stress_note':'Out-of-range numerical stress is checked for bit agreement and saturation separately; excluded from physical error statistics.'}
(root/'reports/error_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
