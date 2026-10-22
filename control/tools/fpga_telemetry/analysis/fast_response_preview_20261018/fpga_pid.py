"""Integer sample-level model of motor520_velocity_pid (50 MHz microsteps collapsed).
No timeout in this function: a caller provides a valid sample every 10 ms.
"""
import torch
KEYS=['reference','integral','derivative','previous','seen','stall','opposite','fault','duty']
def make_state(shape,device='cpu'):
 return {k:torch.zeros(shape,device=device,dtype=torch.int64) for k in KEYS}
def clear(state,mask):
 for k in KEYS:state[k][mask]=0
def step(state,target,cms,enable=None,fast=False):
 if enable is None:enable=torch.ones_like(target,dtype=torch.bool)
 reset=(target==0)|~enable
 clear(state,reset)
 run=~reset & (state['fault']==0)
 feedback=torch.round(cms*10).to(torch.int64)
 bounded=target.clamp(-2000,2000)
 reference=state['reference']+(bounded-state['reference']).clamp(-20 if fast else -100,20 if fast else 100)
 low=(cms>=-2)&(cms<=2);running=reference.abs()>=200
 opposite=((reference>=200)&(cms<=-5))|((reference<=-200)&(cms>=5))
 new_fault=torch.where(running&low&(state['duty'].abs()>=500)&(state['stall']>=(499 if fast else 99)),2,
             torch.where(opposite&(state['opposite']>=(49 if fast else 9)),3,0))
 trip=run&(new_fault!=0);state['fault']=torch.where(trip,new_fault,state['fault'])
 update=run&~trip
 error=reference-feedback
 derivative=torch.where(state['seen']!=0,state['derivative']+((((feedback-state['previous'])*256-state['derivative'])*229)>>12) if fast else state['derivative']+(((feedback-state['previous'])*256-state['derivative'])>>2),0)
 d=(-derivative*(40960 if fast else 8192))>>8;d=d.clamp(-250*4096,250*4096)
 base=error*10240+reference*3277+d
 candidate=(state['integral']+error*(287 if fast else 1434)).clamp(-2400*4096,2400*4096)
 trial=base+candidate
 hold=((trial>2400*4096)&(error>0))|((trial< -2400*4096)&(error<0))
 integral=torch.where(hold,state['integral'],candidate)
 output=(base+integral).clamp(-2400*4096,2400*4096)
 duty=torch.where(output<0,-((-output+2048)>>12),(output+2048)>>12)
 values=dict(reference=reference,integral=integral,derivative=derivative,previous=feedback,seen=torch.ones_like(target),
             stall=torch.where(running&low&(state['duty'].abs()>=500),state['stall']+1,0),
             opposite=torch.where(opposite,state['opposite']+1,0),duty=duty)
 for k,v in values.items():state[k]=torch.where(update,v,state[k])
 state['duty']=torch.where(state['fault']!=0,0,state['duty'])
 state['integral']=torch.where(state['fault']!=0,0,state['integral'])
 return state['duty'],state['fault']
