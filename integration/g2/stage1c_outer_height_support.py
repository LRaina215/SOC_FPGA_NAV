"""Isolated stage1c scene, prior crop and live C-HAL height interception."""
from pathlib import Path
import ctypes
import json
import os
import numpy as np


def inside_xy(points, bounds_mm):
    x0,y0,x1,y1=bounds_mm
    return (points[:,0]>=x0)&(points[:,0]<=x1)&(points[:,1]>=y0)&(points[:,1]<=y1)


def make_scene(R, spec, with_outer=True):
    inner=spec['inner_scene']
    scene=R.W.Scene(size=tuple(inner['size_m']),origin=tuple(inner['origin_m']),res=inner['world_res_m'])
    scene.walls(t=inner['walls']['thickness_m'],h=inner['walls']['height_m'])
    for p in inner['primitives']:getattr(scene,p['kind'])(*p['args'],p['name'])
    old=scene.w
    class ExtendedWorld(R.sw.World):
        def height(self,x,y):
            x=np.asarray(x,dtype=float);y=np.asarray(y,dtype=float)
            value=super().height(x,y)
            # Preserve old coordinate lookup exactly throughout inner field + board.
            mask=(x>=-.05)&(x<=5.05)&(y>=-1.55)&(y<=1.55)
            if np.any(mask):value[mask]=old.height(x[mask],y[mask])
            return value
    world=ExtendedWorld(size_m=spec['expanded_world_size_m'],res=old.res)
    shift=int(round((old.o-world.o)/old.res));assert abs(old.o-world.o-shift*old.res)<1e-9
    world.h[shift:shift+old.n,shift:shift+old.n]=old.h
    scene.w=world
    if with_outer:
        for p in spec['outer_primitives']:getattr(scene,p['kind'])(*p['args'],p['name'])
    scene.original_inner_world=old
    scene.inner_embedding_shift_cells=shift
    return scene


def crop_prior(struct_xy,all_pts,bounds_mm):
    sm=inside_xy(struct_xy,bounds_mm);am=inside_xy(all_pts,bounds_mm)
    audit=dict(struct_before=len(struct_xy),struct_after=int(sm.sum()),struct_outside_removed=int((~sm).sum()),
               all_before=len(all_pts),all_after=int(am.sum()),all_outside_removed=int((~am).sum()),bounds_mm=list(bounds_mm))
    return struct_xy[sm],all_pts[am],audit


def install(R,M,spec,root):
    upper=int(os.environ['SLF_ZHI']);assert upper in(350,1500) and M.Z_HI==upper
    ledger=dict(scene=spec['id'],storage_SN=M.SN,LF_N=M.LF_N,window_mode=M.LF_WINDOW_MODE,ZHI=upper,
                phase='setup',mask_calls=[],zrel_writes=[],cold_configuration=[],prior_packages=[],builds=[],scores=[])
    models=[]
    original_init=R.fpga_model.FpgaModel.__init__
    def model_init(self,*args,**kwargs):
        original_init(self,*args,**kwargs);models.append(self)
    R.fpga_model.FpgaModel.__init__=model_init
    original_ctl=R.fpga_model.FpgaModel.ctl
    def ctl(self,op,a,b,c):
        requested=(a,b,c)
        if op==5:b=upper
        original_ctl(self,op,a,b,c)
        if op==5:
            assert self.zlo==a and self.zhi==upper
            ledger['zrel_writes'].append(dict(op=5,requested=list(requested),applied=[a,b,c],actual_model_ZREL=[self.zlo,self.zhi],phase=ledger['phase']))
    R.fpga_model.FpgaModel.ctl=ctl
    original_mask=M.structure_mask
    def mask(points,pv,z_ref,z_lo=None,z_hi=None):
        lo=M.Z_LO if z_lo is None else z_lo;hi=M.Z_HI if z_hi is None else z_hi
        value=original_mask(points,pv,z_ref,z_lo,z_hi)
        relative=points[value,2]-z_ref
        assert hi==upper and (not len(relative)or(relative.min()>=lo and relative.max()<=hi))
        ledger['mask_calls'].append(dict(phase=ledger['phase'],input_points=len(points),selected=int(value.sum()),
            applied_ZREL=[int(lo),int(hi)],selected_zrel_min=int(relative.min())if len(relative)else None,
            selected_zrel_max=int(relative.max())if len(relative)else None))
        return value
    M.structure_mask=mask
    original_build=M.StructMap.build_lf
    def build(self,cx,cy):
        original_build(self,cx,cy)
        ledger['builds'].append(dict(requested_EM_center=[int(cx),int(cy)],actual_origin=list(self.lf[:2]),
            actual_center=[int(self.lf[0]+self.lf[2].shape[0]//2),int(self.lf[1]+self.lf[2].shape[0]//2)],prior=bool(self.prior),MARGIN=M.MARGIN,n_struct=int(self.n_struct)))
    M.StructMap.build_lf=build
    original_score=M.StructMap.score
    def score(self,px,py,candidates,cx_mm,cy_mm):
        value=original_score(self,px,py,candidates,cx_mm,cy_mm)
        ledger['scores'].append(dict(points=len(px),n_valid=int(value[0,0])if len(value)else 0,
            vehicle_rotation_center_mm=[int(cx_mm),int(cy_mm)],LF_origin=list(self.lf[:2])if self.lf is not None else None,
            prior=bool(self.prior),MARGIN=0 if self.prior else M.MARGIN))
        return value
    M.StructMap.score=score
    OriginalFirmware=R.Firmware
    class ColdConfiguredFirmware(OriginalFirmware):
        def py_set_ctl_cb(self,callback):
            self._stage1c_callback=callback # Retain callable for the complete run.
            register=self.d.py_set_ctl_cb
            register.argtypes=[ctypes.c_void_p];register.restype=None
            register(ctypes.cast(callback,ctypes.c_void_p))
            assert len(models)==1
            model=models[0]
            state=(ctypes.c_int32*12)();self.d.py_get_state.argtypes=[ctypes.POINTER(ctypes.c_int32)];self.d.py_get_state.restype=None;self.d.py_get_state(state)
            before=list(state)
            assert model.xyz is None and len(model.spts)==0 and not model.smap.prior
            assert np.count_nonzero(model.smap.hit)==0 and np.all(model.cost==255)
            assert state[6]==0 and state[7]==0
            ledger['phase']='cold_configuration'
            clear=R.rk_host.map_clear();packet=(ctypes.c_uint8*len(clear)).from_buffer_copy(clear)
            self.d.py_set_time.argtypes=[ctypes.c_uint32];self.d.py_set_time.restype=None;self.d.py_set_time(ctypes.c_uint32(1000))
            self.d.py_push_host.argtypes=[ctypes.POINTER(ctypes.c_uint8),ctypes.c_int,ctypes.c_uint32];self.d.py_push_host.restype=None
            self.d.py_push_host(packet,len(clear),ctypes.c_uint32(1000))
            self.d.py_poll.argtypes=[];self.d.py_poll.restype=None;self.d.py_poll()
            self.d.py_get_state(state)
            command=(ctypes.c_int16*4)();self.d.py_get_cmd.argtypes=[ctypes.POINTER(ctypes.c_int16)];self.d.py_get_cmd.restype=None;self.d.py_get_cmd(command)
            commit=(ctypes.c_int32*16)();self.d.py_get_commit.argtypes=[ctypes.POINTER(ctypes.c_int32)];self.d.py_get_commit.restype=ctypes.c_int
            pending=int(self.d.py_get_commit(commit))
            assert model.xyz is None and len(model.spts)==0 and not model.smap.prior
            assert np.count_nonzero(model.smap.hit)==0 and np.all(model.cost==255)
            assert state[6]==0 and state[7]==0 and pending==0 and list(command)==[0,0,0,0]
            assert ledger['zrel_writes'] and ledger['zrel_writes'][-1]['requested'][:2]==[60,1500]
            ledger['cold_configuration'].append(dict(before_state=before,after_state=list(state),command=list(command),
                callback_pointer_bits=ctypes.sizeof(ctypes.c_void_p)*8,native_registration_argtype='void*; explicit cast retains64bit pointer',
                simulated_time_us=1000,polls=1,host_packets=['original MAP_CLEAR'],sensor_packets=0,lidar_frames=0,
                empty_structure_before_after=True,empty_cost_before_after=True,pending_commit=pending,
                first_poll_also_completes_original_DLL_LUT_generation=True))
            ledger['phase']='runtime'
    R.Firmware=ColdConfiguredFirmware
    R.W.SCENES[spec['id']]=lambda:make_scene(R,spec)
    bounds_mm=[0,-1500,5000,1500]
    def get_prior(scene_name,setting):
        assert scene_name==spec['id']and setting=='auto'
        key=dict(scene=scene_name,scene_version=1,SLF_SN=M.SN,LF_N=M.LF_N,window_mode='fixed_field',
            ZREL=[M.Z_LO_PRIOR,upper],crop_inner_mm=bounds_mm,survey='N1c_noise4_lidarSeed11_surveySeed3_spacing1_yaws6_legacy_march')
        directory=root/'prior_cache'/scene_name/f'SN{M.SN}'/f'LF{M.LF_N}_fixed_field'/f'Zlo{M.Z_LO_PRIOR}_hi{upper}_cropInner'
        directory.mkdir(parents=True,exist_ok=True);package=directory/'N1c_noise4_ls11_ss3_spacing1_yaws6.nvmp';meta=package.with_suffix('.json')
        if package.exists():
            saved=json.loads(meta.read_text(encoding='utf8'));assert saved['key']==key
            data=package.read_bytes();R.prior_map.parse_package(data)
            ledger['prior_packages'].append(dict(key=key,path=str(package),metadata=str(meta),cache_hit=True));return data
        ledger['phase']='prior_survey'
        scene=make_scene(R,spec);lidar=R.sw.VirtualLidar(noise_mm=4.0,seed=11,rng_max=8.0,step=.015)
        struct_xy,all_pts,poses=R.prior_map.survey_scan(scene,lidar,R.sw.mount_pose(),zcap=upper/1000)
        struct_xy,all_pts,crop=crop_prior(struct_xy,all_pts,bounds_mm)
        data,info=R.prior_map.build_package(struct_xy,all_pts,bounds_mm=bounds_mm)
        assert inside_xy(struct_xy,bounds_mm).all()and inside_xy(all_pts,bounds_mm).all()
        package.write_bytes(data)
        record=dict(key=key,crop=crop,poses=len(poses),package_bytes=len(data),package_info={k:info[k]for k in('ox','oy','n_struct','n_words','gox','goy','gnx','gny')},source_band_upper=upper,
            all_points_survey_zcap_mm=upper,remaining_outside_struct=0,remaining_outside_all=0,
            gate_halo_is_not_external_structure=True)
        meta.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        ledger['prior_packages'].append(dict(key=key,path=str(package),metadata=str(meta),cache_hit=False,crop=crop))
        ledger['phase']='runtime';return data
    R.get_prior=get_prior
    return ledger
