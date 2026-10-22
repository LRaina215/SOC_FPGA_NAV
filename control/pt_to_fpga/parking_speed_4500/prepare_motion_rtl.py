"""Choose balanced trajectory vectors for bit-exact RTL comparison."""
import json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'reference'))
from validate import emit_vectors
manifest=json.loads((root/'assets/vectors/manifest.json').read_text())
parts=[];sources=[]
for name,entry in manifest['datasets'].items():
    if not name.startswith('trajectory_'):continue
    path=root/'assets/vectors'/entry['directory']/'fixed_vectors.npz'
    with np.load(path) as data:
        indices=np.linspace(0,len(data['obs'])-1,8,dtype=int)
        parts.append({k:data[k][indices].copy() for k in data.files})
    sources.append({'dataset':name,'selected_indices':indices.tolist()})
assert len(parts)==7
arrays={k:np.concatenate([part[k] for part in parts]) for k in parts[0]}
destination=root/'assets/vectors/rtl_motion'
result=emit_vectors(destination,arrays)
(destination/'selection.json').write_text(json.dumps({'sources':sources,**result},indent=2))
print('MOTION_RTL_VECTORS',result['samples'])
