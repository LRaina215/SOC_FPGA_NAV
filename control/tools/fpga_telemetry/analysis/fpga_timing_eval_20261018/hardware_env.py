"""Isolated timing/interface/PID experiment. Not a real-motor digital twin."""
from pathlib import Path
import math
import torch
from isaacgym import gymtorch
from isaacgym.torch_utils import quat_rotate_inverse
from parking_env import ParkingRobot
from fpga_pid import make_state,step as pid_step,clear as pid_clear
MODES=['original','sampled_timing','fpga_pid_tau015','fpga_pid_tau030','fpga_pid_tau050','fpga_pid_tau030_gain18']
CASES=['stop','forward','reverse','left','right','low','high']
PER_CASE=10
class HardwareRobot(ParkingRobot):
 def __init__(self,*args,**kwargs):
  self.hw_ready=False
  super().__init__(*args,**kwargs)
  assert abs(self.sim_params.dt-.002)<1e-9 and self.cfg.control.decimation==4
  assert self.cfg.control.leg_integral_gain==0
  n=self.num_envs;dev=self.device
  self.mode=torch.arange(n,device=dev)//(len(CASES)*PER_CASE)
  self.phase=torch.arange(n,device=dev)%PER_CASE
  self.hw_age=torch.zeros(n,dtype=torch.int64,device=dev)
  self.pending=torch.zeros(n,6,device=dev);self.delivered=self.pending.clone()
  self.hw_imu=torch.zeros(n,6,device=dev);self.hw_imu[:,5]=-1
  self.hw_wheel_omega=torch.zeros(n,2,device=dev);self.hw_cms=torch.zeros(n,2,dtype=torch.int64,device=dev)
  self.hw_count_origin=self.dof_pos[:,self.wheel_joint_indices].clone()
  self.hw_last_counts=torch.zeros(n,2,dtype=torch.int64,device=dev)
  self.leg_ids=[self.left_indices[0],self.left_indices[1],self.right_indices[0],self.right_indices[1]]
  # Consistent explicit wheel order [left,right].
  self.wheel_ids=[self.left_indices[2],self.right_indices[2]]
  self.hw_count_origin=self.dof_pos[:,self.wheel_ids].clone()
  self.latched_legs=self.dof_pos[:,self.leg_ids].clone()
  self.pid=make_state((n,2),dev)
  self.motor_stop=torch.zeros(n,dtype=torch.bool,device=dev)
  self.motor_fault=torch.zeros(n,2,dtype=torch.int64,device=dev)
  self.ever_fault=self.motor_fault.clone()
  self.held_timing_torque=torch.zeros(n,2,device=dev)
  self.last_hw_target=torch.zeros(n,2,dtype=torch.int64,device=dev)
  self.stall_torque=torch.full((n,1),.3,device=dev)
  self.stall_torque[self.mode==2]=.15;self.stall_torque[self.mode==4]=.5
  # Unknown physical parameter. Declared assumption, not measured.
  self.no_load_mps=1.0
  rom=[]
  for name in ['lh','lk','rh','rk']:
   v=[int(x.strip(),16) for x in Path('/home/cyd/fpga/fpga/src/mem/leg_ik_'+name+'.mem').read_text().splitlines() if x.strip()]
   rom.append([x-4096 if x>=2048 else x for x in v])
  self.ik_rom=torch.tensor(rom,device=dev).T
  self.servo_sign=torch.tensor([1.,-1.,1.,1.],device=dev)
  self.hw_ready=True
 def reset_idx(self,ids):
  super().reset_idx(ids)
  if getattr(self,'hw_ready',False):
   for name in ['hw_age','pending','delivered','hw_wheel_omega','hw_cms','hw_last_counts','motor_stop','motor_fault','last_hw_target','held_timing_torque']:
    getattr(self,name)[ids]=0
   self.hw_imu[ids]=0;self.hw_imu[ids,5]=-1
   self.hw_count_origin[ids]=self.dof_pos[ids][:,self.wheel_ids]
   self.latched_legs[ids]=self.dof_pos[ids][:,self.leg_ids]
   for v in self.pid.values():v[ids]=0
 @staticmethod
 def signed_round(x):return (torch.sign(x)*torch.floor(torch.abs(x)+.5)).to(torch.int64)
 def compute_proprioception_observations(self):
  obs=super().compute_proprioception_observations()
  if not getattr(self,'hw_ready',False):return obs
  mask=self.mode>0
  hw=obs.clone()
  hw[:,:3]=self.hw_imu[:,:3]*self.obs_scales.ang_vel
  hw[:,3:6]=self.hw_imu[:,3:]*self.obs_scales.gravity
  qact=self.signed_round(self.delivered*4096).clamp(-4096,4096).float()/4096
  hw[:,10:16]=qact*self.cfg.control.pos_action_scale*self.obs_scales.dof_pos
  hw[:,10+self.wheel_ids[0]]=0;hw[:,10+self.wheel_ids[1]]=0
  hw[:,16:22]=0
  hw[:,16+self.wheel_ids[0]]=self.hw_wheel_omega[:,0]*self.obs_scales.dof_vel
  hw[:,16+self.wheel_ids[1]]=self.hw_wheel_omega[:,1]*self.obs_scales.dof_vel
  hw[:,22:28]=qact
  return torch.where(mask[:,None],hw,obs)
 def _compute_torques(self,actions):
  if not getattr(self,'hw_ready',False):return super()._compute_torques(actions)
  hw=self.mode>0;full=self.mode>=2
  phase=self.hw_age%4
  self.pending=torch.where((phase==0)[:,None],actions,self.pending)
  self.delivered=torch.where((phase==1)[:,None],self.pending,self.delivered)
  applied=torch.where(hw[:,None],self.delivered,actions)
  # Source uses 100 Hz acquisition. Capture exact state only; fusion/bias are not emulated.
  self.gym.refresh_actor_root_state_tensor(self.sim)
  q=self.root_states[:,3:7]
  w=quat_rotate_inverse(q,self.root_states[:,10:13])
  g=quat_rotate_inverse(q,self.gravity_vec)
  imu=torch.cat([w,g],1)
  imu=self.signed_round(imu*4096).float()/4096
  sample=(self.hw_age>=5)&((self.hw_age+self.phase)%5==0)
  self.hw_imu=torch.where(sample[:,None],imu,self.hw_imu)
  counts=torch.floor((self.dof_pos[:,self.wheel_ids]-self.hw_count_origin)*1320/(2*math.pi)).to(torch.int64)
  delta=counts-self.hw_last_counts
  self.hw_last_counts=torch.where(sample[:,None],counts,self.hw_last_counts)
  omega=delta.float()*(2*math.pi/1320/.01)
  self.hw_wheel_omega=torch.where(sample[:,None],omega,self.hw_wheel_omega)
  physical=omega*torch.tensor([-.0325,.0325],device=self.device)
  cms=self.signed_round(physical*100)
  self.hw_cms=torch.where(sample[:,None],cms,self.hw_cms)
  # Preserve the exact original torques for mode0. One IK update per physics substep.
  reference=self._leg_reference()
  pos_ref=actions*self.cfg.control.pos_action_scale
  pos_ref[:,self.wheel_joint_indices]=0
  vel_ref=actions*self.cfg.control.vel_action_scale
  vel_ref[:,self.leg_joint_indices]=0
  torques=self.p_gains*(pos_ref+reference-self.dof_pos)+self.d_gains*(vel_ref-self.dof_vel)
  # Mode1: original servo reference held at 50Hz, without changing its semantics.
  goal=reference[:,self.leg_ids]+applied[:,self.leg_ids]*self.cfg.control.pos_action_scale
  # Full interface: use actual signed IK ROM + integer pulse rounding and 50Hz latch.
  h=torch.floor(self.commands[:,2]*4096+.5).long().clamp(348,778)
  aq=self.signed_round(applied[:,self.leg_ids]*4096).clamp(-4096,4096)
  delta_us=self.signed_round(aq.float()*19/4096)*self.servo_sign.long()
  pulse=(1500+self.ik_rom[h-348]+delta_us).clamp(500,2500)
  full_goal=self.default_dof_pos[:,self.leg_ids]+(pulse.float()-1500)*self.servo_sign*(math.pi/2000)
  goal=torch.where(full[:,None],full_goal,goal)
  servo_tick=(self.hw_age+self.phase)%10==0
  latch=servo_tick&~self.motor_stop
  self.latched_legs=torch.where(latch[:,None],goal,self.latched_legs)
  leg_t=self.p_gains[:,self.leg_ids]*(self.latched_legs-self.dof_pos[:,self.leg_ids])-self.d_gains[:,self.leg_ids]*self.dof_vel[:,self.leg_ids]
  torques[:,self.leg_ids]=torch.where(hw[:,None],leg_t,torques[:,self.leg_ids])
  # Timing-only: same original P speed law, sampled at 100Hz and held.
  if not hasattr(self,'held_timing_torque'):self.held_timing_torque=torch.zeros_like(self.hw_wheel_omega)
  target_omega=applied[:,self.wheel_ids]*self.cfg.control.vel_action_scale
  timing_torque=self.d_gains[:,self.wheel_ids]*(target_omega-self.hw_wheel_omega)
  self.held_timing_torque=torch.where(sample[:,None],timing_torque,self.held_timing_torque)
  torques[:,self.wheel_ids]=torch.where((self.mode==1)[:,None],self.held_timing_torque,torques[:,self.wheel_ids])
  # Actual hardware model output conversion and optional current IR gain1.8.
  aqw=self.signed_round(applied[:,self.wheel_ids]*4096).clamp(-4096,4096)
  target=self.signed_round(aqw.float()*torch.tensor([-600.,600.],device=self.device)/4096)
  boost=(self.mode==5)&(self.commands[:,0].abs()>.001)&(self.commands[:,1].abs()<.001)
  gain_q12=round(1.8*4096)
  boosted=torch.sign(target)*((target.abs()*gain_q12+2048)>>12)
  target=torch.where(boost[:,None],boosted,target).clamp(-600,600)
  self.last_hw_target=target
  pid_clear(self.pid,target==0)
  active=full&~self.motor_stop
  update=sample&active
  if update.any():
   ids=update.nonzero(as_tuple=False).flatten()
   subset={k:v[ids].clone() for k,v in self.pid.items()}
   duty,fault=pid_step(subset,target[ids],self.hw_cms[ids])
   for k,v in subset.items():self.pid[k][ids]=v
   self.motor_fault[ids]=torch.maximum(self.motor_fault[ids],fault)
   self.ever_fault[ids]=torch.maximum(self.ever_fault[ids],fault)
   self.motor_stop[ids]|=(fault!=0).any(1)
  # Averaged DC motor proxy. PWM/carrier deadtime (50us) is below2ms resolution.
  duty=self.pid['duty'].float()/2500
  actual_physical=self.dof_vel[:,self.wheel_ids]*torch.tensor([-.0325,.0325],device=self.device)
  drive=self.stall_torque*(duty-actual_physical/self.no_load_mps)
  coast=(target==0)|self.motor_stop[:,None]
  drive=torch.where(coast,0.,drive)
  drive*=torch.tensor([-1.,1.],device=self.device)
  torques[:,self.wheel_ids]=torch.where(full[:,None],drive,torques[:,self.wheel_ids])
  self.hw_age+=1
  return torch.clamp(torques*self.torques_scale,-self.torque_limits,self.torque_limits)
