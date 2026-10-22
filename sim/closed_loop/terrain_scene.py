"""Independent continuous terrain profiles and their actual MuJoCo solids.

The surface functions do not call the navigation classifier. The trench has
separate banks and a recessed bottom; no plane spans its mouth.
"""
from dataclasses import dataclass, asdict
import math
import ctypes
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import world as W

S=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])


@dataclass
class TerrainSpec:
    kind: str = 'flat'
    value: float = 0.0
    depth: float = .15
    entry: float = .55
    run: float = .50
    plateau: float = .25
    yaw_deg: float = 0.
    speed: float = .20
    seconds: float = 20.
    friction: float = .6
    payload_kg: float = 0.
    wheel_torque_scale: float = 1.
    profile: str = 'native'
    mechanism: str = 'open'
    wheel_shape: str = 'training_capsule'
    policy_noise: bool = False
    start_x: float = 0.
    start_y: float = 0.
    align_start: bool = False
    lateral_test: bool = False

    @property
    def name(self):
        v=f'{self.value:g}'.replace('.','p')
        return f'{self.kind}_{v}_{self.mechanism}_{self.profile}_{self.wheel_shape}'


class TerrainScene(W.Scene):
    def __init__(self,spec):
        super().__init__(size=(5.,3.),origin=(-1.,-1.5),res=.01)
        self.spec=spec;self.prims=[]
        self.initial_height=float(self.height(spec.start_x,spec.start_y))
        x=self.w.o+(np.arange(self.w.n)+.5)*self.w.res
        X,Y=np.meshgrid(x,x);self.w.h[:]=self.height(X,Y)
        self.goal_x=self.end+.35 if spec.kind!='flat' else 1.1
        self.goal_y=.6
        self.bounds=(-1.,-1.5,4.,1.5)

    @property
    def rise(self):return math.tan(math.radians(self.spec.value))*self.spec.run

    @property
    def end(self):
        s=self.spec
        return s.entry+2*s.run+s.plateau if s.kind=='ramp_pair' else s.entry+s.run if s.kind.startswith('ramp') else s.entry+s.value if s.kind=='trench' else s.entry+.25

    def height(self,x,y):
        x,y=np.broadcast_arrays(np.asarray(x,dtype=float),np.asarray(y,dtype=float));s=self.spec
        h=np.zeros(x.shape)
        if s.kind in ('ramp_up','ramp_pair'):
            h=np.clip((x-s.entry)/s.run,0,1)*self.rise
            if s.kind=='ramp_pair':h=np.minimum(h,np.clip((self.end-x)/s.run,0,1)*self.rise)
        elif s.kind=='ramp_down':h=(1-np.clip((x-s.entry)/s.run,0,1))*self.rise
        elif s.kind=='step_up':h=np.where(x>=s.entry,s.value,0.)
        elif s.kind=='step_down':h=np.where(x<s.entry,s.value,0.)
        elif s.kind=='trench':h=np.where((x>=s.entry)&(x<s.entry+s.value)&(np.abs(y)<.65),-s.depth,0.)
        return h

    def geometric_labels(self,x,y,cell=.05,step=.020,slope_deg=12.):
        """Reference-envelope geometry, NOT learned-policy physical capability.

        Boundary = cells intersecting discontinuity or ramp transition; retained
        separately from interiors rather than omitted from all denominators.
        """
        s=self.spec;x,y=np.broadcast_arrays(x,y);danger=np.zeros(x.shape,bool);boundary=np.zeros(x.shape,bool)
        if s.kind.startswith('ramp'):
            ramp=(x>=s.entry)&(x<=s.entry+s.run)
            if s.kind=='ramp_pair':ramp |= (x>=self.end-s.run)&(x<=self.end)
            danger=ramp & (s.value>slope_deg)
            edges=[s.entry,s.entry+s.run]+([self.end-s.run,self.end] if s.kind=='ramp_pair' else [])
            for e in edges:boundary |= np.abs(x-e)<=cell/2
        elif s.kind.startswith('step'):
            boundary=np.abs(x-s.entry)<=cell/2
            danger=boundary & (s.value>step)
        elif s.kind=='trench':
            mouth=(x>=s.entry)&(x<s.entry+s.value)&(np.abs(y)<.65)
            lips=((np.abs(x-s.entry)<=cell/2)|(np.abs(x-s.entry-s.value)<=cell/2)) & (np.abs(y)<.65+cell/2)
            boundary=lips | (mouth & (np.abs(np.abs(y)-.65)<=cell/2))
            danger=mouth|lips
        return danger,boundary

    def gt_lookup(self,x,y,cell=.05):
        dangerous,boundary=self.geometric_labels(np.asarray(x),np.asarray(y),cell)
        return np.where(dangerous,1,np.where(boundary,2,0)).astype(np.int8)

    def metadata(self):
        return dict(spec=asdict(self.spec),initial_height_m=self.initial_height,goal_x=self.goal_x,
                    geometry='continuous profile, real convex wedges/boxes; no plane',
                    truth='independent profile;20mm/12deg labels are reference assumptions, not controller ability')


def add_terrain(xml,scene):
    root=xml.getroot();wb=root.find('worldbody');asset=root.find('asset');s=scene.spec
    for g in list(wb.findall('geom')):
        if g.get('name')=='floor' or g.get('name','').startswith(('scene_','terrain_')):wb.remove(g)
    def box(name,x0,x1,y0,y1,top,bottom=-.30):
        assert top>bottom
        center=S.T@np.array([(x0+x1)/2,(y0+y1)/2,(top+bottom)/2])
        ET.SubElement(wb,'geom',name='terrain_'+name,type='box',pos=' '.join(map(str,center)),
                      size=f'{(y1-y0)/2} {(x1-x0)/2} {(top-bottom)/2}',contype='0',conaffinity='1',group='0',
                      priority='1' if s.friction<.6 else '0',
                      friction=f'{s.friction} .005 .0001',rgba='.45 .51 .46 1')
    def wedge(name,x0,x1,z0,z1):
        v=np.array([[x,y,z] for zkind in [0,1] for y in [-1.5,1.5] for x in [x0,x1]
                    for z in [(-.1 if zkind==0 else z0 if x==x0 else z1)]]) @ S
        # Convex hull is exactly this wedge. Explicit closed triangulation.
        faces=[0,2,3,0,3,1,4,5,7,4,7,6,0,1,5,0,5,4,2,6,7,2,7,3,0,4,6,0,6,2,1,3,7,1,7,5]
        ET.SubElement(asset,'mesh',name='terrain_mesh_'+name,vertex=' '.join(map(str,v.reshape(-1))),face=' '.join(map(str,faces)))
        ET.SubElement(wb,'geom',name='terrain_'+name,type='mesh',mesh='terrain_mesh_'+name,
                      contype='0',conaffinity='1',group='0',friction=f'{s.friction} .005 .0001',rgba='.43 .57 .39 1')
        if s.friction<.6:wb.find(f'geom[@name="terrain_{name}"]').set('priority','1')
    if s.kind=='trench':
        box('left_bank',-2,s.entry,-2,2,0,bottom=-s.depth-.1)
        box('right_bank',s.entry+s.value,4,-2,2,0,bottom=-s.depth-.1)
        box('bottom',s.entry,s.entry+s.value,-.65,.65,-s.depth,bottom=-s.depth-.1)
        box('side_bank_l',s.entry,s.entry+s.value,-2,-.65,0,bottom=-s.depth-.1)
        box('side_bank_r',s.entry,s.entry+s.value,.65,2,0,bottom=-s.depth-.1)
    elif s.kind.startswith('step'):
        box('low',-2,4,-2,2,0)
        if s.kind=='step_up':box('upper',s.entry,4,-2,2,s.value,bottom=-.05)
        else:box('upper',-2,s.entry,-2,2,s.value,bottom=-.05)
    elif s.kind.startswith('ramp'):
        box('ground',-2,4,-2,2,0)
        if s.kind=='ramp_down':
            box('entry',-2,s.entry,-1.5,1.5,scene.rise,bottom=-.05)
            wedge('down',s.entry,s.entry+s.run,scene.rise,0)
        else:
            wedge('up',s.entry,s.entry+s.run,0,scene.rise)
            x1=scene.end-s.run if s.kind=='ramp_pair' else 4
            box('plateau',s.entry+s.run,x1,-1.5,1.5,scene.rise,bottom=-.05)
            if s.kind=='ramp_pair':wedge('down',x1,scene.end,scene.rise,0)
    else:box('ground',-2,4,-2,2,0)


class PhysicsRayWorld:
    """VirtualLidar ray source uses the actual MuJoCo terrain collision geometry."""
    raycast_version='mujoco_mj_ray_batch_v2'
    def __init__(self,scene,model,data,mj):
        self.scene=scene;self.model=model;self.data=data;self.mj=mj
        self.h=scene.w.h;self.res=scene.w.res;self.o=scene.w.o;self.n=scene.w.n
        self.lib=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'sim/build/terrain4500_20261022/terrain_ray_batch.dll'))
        self.mjlib=ctypes.CDLL(str(Path(mj.__file__).parent/'mujoco.dll'))
        self.batch=self.lib.terrain_rays
        self.batch.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p]
        self.batch.restype=None;self.groups=np.array([1,0,0,0,0,0],dtype=np.uint8)
    def height(self,x,y):return self.scene.height(x,y)
    def _idx(self,x,y):return np.floor((np.asarray(x)-self.o)/self.res).astype(int),np.floor((np.asarray(y)-self.o)/self.res).astype(int)
    def raycast(self,origin,dirs,tmax=8.,step=None):
        p=S.T@np.asarray(origin);v=np.ascontiguousarray(np.asarray(dirs)@S)
        hit=np.empty(len(v));geomid=np.empty(len(v),dtype=np.int32)
        p=np.ascontiguousarray(p,dtype=np.float64)
        self.batch(ctypes.cast(self.mjlib.mj_ray,ctypes.c_void_p),self.model._address,self.data._address,
                   p.ctypes.data,v.ctypes.data,len(v),self.groups.ctypes.data,hit.ctypes.data,geomid.ctypes.data)
        return np.where((hit>=0)&(hit<=tmax),hit,np.inf)
