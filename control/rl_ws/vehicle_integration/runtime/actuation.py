"""Geometry and physical calibration reference; no serial or motor outputs."""
import sys,math
import numpy as np
sys.path.insert(0,'/home/cyd/fpga/rl_ws2/_car_tools')
from car_kinematics import FiveBar

class LegMapper:
    def __init__(self):self.geometry=FiveBar();self.height=.1305
    def update(self,commands,actions,dt):
        if not 0<dt<=.02:raise ValueError('invalid policy period')
        action=np.asarray(actions,float)
        if action.shape!=(6,) or not np.isfinite(action).all() or np.max(np.abs(action))>1:raise ValueError('invalid clipped action')
        target=min(.19,max(.085,float(commands[2])))
        self.height+=float(np.clip(target-self.height,-.05*dt,.05*dt))
        result={}
        for side,indices in [('Left',(0,1)),('Right',(3,4))]:
            ref=self.geometry.solve(side,-.03,.0325-self.height)
            qh=ref['front']+action[indices[0]]*.03
            qk=ref['front_knee']+action[indices[1]]*.03
            y,z=self.geometry.front_fk(side,qh,qk)
            physical=self.geometry.solve(side,y,z)
            result[side]={'front_rad':physical['front'],'back_rad':physical['back'],'wheel_yz_m':[y,z]}
        return result

def servo_pulse(joint_rad,calibration):
    required=('zero_joint_rad','zero_us','us_per_rad','min_us','max_us','min_joint_rad','max_joint_rad')
    if any(k not in calibration or calibration[k] is None for k in required):raise ValueError('servo physical calibration missing')
    if not all(math.isfinite(calibration[k]) for k in required) or not math.isfinite(joint_rad):raise ValueError('invalid servo calibration')
    if not calibration['min_joint_rad']<=joint_rad<=calibration['max_joint_rad']:raise ValueError('joint target beyond calibrated mechanical travel')
    pulse=calibration['zero_us']+(joint_rad-calibration['zero_joint_rad'])*calibration['us_per_rad']
    if not calibration['min_us']<=pulse<=calibration['max_us']:raise ValueError('pulse beyond calibrated travel')
    return round(pulse)

def wheel_torques(actions):
    a=np.asarray(actions,float)
    if a.shape!=(6,) or not np.isfinite(a).all() or np.max(np.abs(a))>1:raise ValueError('invalid action')
    # These are desired joint torques, not PWM/rpm/current setpoints.
    return {'Left':float(a[2]*.1),'Right':float(a[5]*.1)}
