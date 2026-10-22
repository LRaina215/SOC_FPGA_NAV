"""Parking rewards on the original velocity-action environment."""
import torch
from isaacgym.torch_utils import quat_rotate
from wheel_legged_gym.envs.car_robot.car_robot import CarRobot

class ParkingRobot(CarRobot):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        # XY bounding-box center of the original CAD base mesh, in meters.
        self.parking_offset=torch.tensor([-.080000002,-.006656542,0.],device=self.device).repeat(self.num_envs,1)
        self.parking_anchor=self.parking_center().clone()
        self.parking_last_center=self.parking_anchor.clone()
        self.parking_velocity=torch.zeros(self.num_envs,2,device=self.device)
        self.parking_heading_anchor=self.parking_heading().clone()
        self.parking_was_stop=torch.zeros(self.num_envs,dtype=torch.bool,device=self.device)

    def parking_center(self):
        return (self.root_states[:,:3]+quat_rotate(self.root_states[:,3:7],self.parking_offset))[:,:2]

    def parking_heading(self):
        q=self.root_states[:,3:7]
        return torch.atan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2))

    def parking_mask(self):
        return (self.commands[:,:2].abs()<1e-4).all(dim=1)

    def _resample_commands(self,ids):
        super()._resample_commands(ids)
        extra=float(getattr(self.cfg.commands,'parking_extra_fraction',0.))
        if len(ids) and extra:
            selected=ids[torch.rand(len(ids),device=self.device)<extra]
            self.commands[selected,:2]=0.
        if hasattr(self,'parking_was_stop') and len(ids):
            stop=self.parking_mask()[ids]
            entering=ids[stop & ~self.parking_was_stop[ids]]
            self.parking_anchor[entering]=self.parking_center()[entering]
            self.parking_heading_anchor[entering]=self.parking_heading()[entering]
            # Repeated zero commands must not erase accumulated drift.
            self.parking_was_stop[ids]=stop

    def reset_idx(self,ids):
        if hasattr(self,'parking_was_stop'):self.parking_was_stop[ids]=False
        super().reset_idx(ids)
        if hasattr(self,'parking_anchor'):
            center=self.parking_center()
            self.parking_anchor[ids]=center[ids]
            self.parking_last_center[ids]=center[ids]
            self.parking_velocity[ids]=0.
            self.parking_heading_anchor[ids]=self.parking_heading()[ids]

    def _post_physics_step_callback(self):
        super()._post_physics_step_callback()
        if hasattr(self,'parking_last_center'):
            self.parking_velocity=(self.parking_center()-self.parking_last_center)/self.dt

    def post_physics_step(self):
        super().post_physics_step()
        self.parking_last_center=self.parking_center().clone()

    def _reward_parking_velocity(self):
        return self.parking_mask()*(self.parking_velocity**2).sum(1)

    def _reward_parking_yaw(self):
        return self.parking_mask()*self.base_ang_vel[:,2]**2

    def _reward_parking_position(self):
        return self.parking_mask()*((self.parking_center()-self.parking_anchor)**2).sum(1).clamp_max(.25)

    def _reward_parking_heading(self):
        error=self.parking_heading()-self.parking_heading_anchor
        error=torch.atan2(torch.sin(error),torch.cos(error))
        return self.parking_mask()*error**2
