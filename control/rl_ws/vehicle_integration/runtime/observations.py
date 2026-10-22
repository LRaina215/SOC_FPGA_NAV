"""CPU reference for physical sensor adaptation; does not drive hardware."""
import math
import numpy as np

GYRO_RAD_PER_LSB=math.pi/(180*16.384)
class GravityEstimator:
    def __init__(self,sensor_to_body,bias_raw):
        self.rotation=np.asarray(sensor_to_body,dtype=float);self.bias=np.asarray(bias_raw,dtype=float)
        if self.rotation.shape!=(3,3) or not np.allclose(self.rotation@self.rotation.T,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(self.rotation),1):raise ValueError('installation transform must be a proper3D rotation')
        if self.bias.shape!=(3,) or not np.isfinite(self.bias).all():raise ValueError('gyro bias needs three finite calibrated values')
        self.gravity=None;self.last_ticks=None
    def update(self,accel_raw,gyro_raw,ticks):
        acc=self.rotation@np.asarray(accel_raw,dtype=float)/10920
        omega=self.rotation@(np.asarray(gyro_raw,dtype=float)-self.bias)*GYRO_RAD_PER_LSB
        norm=float(np.linalg.norm(acc))
        if not np.isfinite(acc).all() or not np.isfinite(omega).all():raise ValueError('nonfinite sensor')
        if self.gravity is None:
            if not .8<=norm<=1.2:raise ValueError('initial gravity requires approximately1g')
            self.gravity=-acc/norm
        else:
            dt=((ticks-self.last_ticks)&0xffffffff)/50000000
            if not 0<dt<=.03:raise ValueError('duplicate or stale sensor timestamp')
            self.gravity-=np.cross(omega,self.gravity)*dt
            self.gravity/=np.linalg.norm(self.gravity)
            # Accelerometer corrects slow drift only while magnitude is plausible.
            if .8<=norm<=1.2:
                alpha=1-math.exp(-dt/.5)
                self.gravity=(1-alpha)*self.gravity-alpha*acc/norm
                self.gravity/=np.linalg.norm(self.gravity)
        self.last_ticks=ticks
        return omega.copy(),self.gravity.copy()

class ObservationHistory:
    def __init__(self):self.history=None
    def update(self,omega,gravity,commands,wheel_velocities,last_executed_action):
        w=np.asarray(omega,float);g=np.asarray(gravity,float);c=np.asarray(commands,float);vel=np.asarray(wheel_velocities,float);a=np.asarray(last_executed_action,float)
        if w.shape!=(3,) or g.shape!=(3,) or c.shape!=(3,) or vel.shape!=(2,) or a.shape!=(6,):raise ValueError('wrong observation dimensions')
        if not all(np.isfinite(v).all() for v in (w,g,c,vel,a)):raise ValueError('nonfinite observation')
        if np.max(np.abs(a))>1 or abs(np.linalg.norm(g)-1)>1e-3:raise ValueError('invalid action/gravity')
        pos=a*.03;pos[[2,5]]=0
        dq=np.zeros(6);dq[[2,5]]=vel
        obs=np.concatenate((w*1.3,g*3,c*[2,1.3,5],pos,dq*.15,a))
        self.history=np.tile(obs,(5,1)) if self.history is None else np.vstack((self.history[1:],obs))
        # Match simulator ordering: append unclipped current frame to history.
        return np.clip(obs,-100,100),self.history.reshape(-1).copy()

def quantize(values):
    scaled=np.rint(np.asarray(values,float)*4096)
    if not np.isfinite(scaled).all():raise ValueError('nonfinite fixed-point input')
    if np.any(scaled<-(1<<23)) or np.any(scaled>(1<<23)-1):raise ValueError('sensor/observation signed24 overflow')
    return scaled.astype(np.int32)
