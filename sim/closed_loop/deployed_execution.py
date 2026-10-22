"""J3 digital reference of fixed control snapshot 6443f56 (no control edits).

PID is per 10ms encoder sample, with the 8-clock arithmetic pipeline collapsed.
Feedback deadline/PWM carrier timing are separate: this helper does not claim
cycle-accurate simulation or calibrated motor/servo dynamics.
"""
import math


def clip(value, low, high):
    return max(low,min(high,value))


def symmetric_round(value, denominator):
    result=(abs(value)+denominator//2)//denominator
    return -result if value<0 else result


class WheelPid:
    def __init__(self):
        self.reset()

    def reset(self):
        self.reference=self.error=self.integral=self.duty=0
        self.previous_feedback=self.derivative_q8=0
        self.measurement_seen=False
        self.stall_count=self.opposite_count=self.fault=0

    def sample(self, target_mmps, feedback_cms):
        # A single wheel target zero disables PWM and clears PID. This says
        # nothing about body (vx,wz)==(0,0), which still runs the balance policy.
        if target_mmps==0:
            self.reset();return self.duty
        if self.fault:
            self.duty=self.integral=0;return 0
        bounded=clip(int(target_mmps),-2000,2000)
        reference=self.reference+clip(bounded-self.reference,-100,100)
        running=abs(reference)>=200
        low=abs(feedback_cms)<=2
        opposite=(reference>=200 and feedback_cms<=-5) or (reference<=-200 and feedback_cms>=5)
        stalled=running and low and abs(self.duty)>=500
        if stalled and self.stall_count>=99:
            self.fault=2;self.duty=self.integral=0;return 0
        if opposite and self.opposite_count>=9:
            self.fault=3;self.duty=self.integral=0;return 0
        self.stall_count=self.stall_count+1 if stalled else 0
        self.opposite_count=self.opposite_count+1 if opposite else 0
        feedback=int(feedback_cms)*10
        delta=(feedback-self.previous_feedback)<<8
        self.derivative_q8=(self.derivative_q8+((delta-self.derivative_q8)>>2)) if self.measurement_seen else 0
        self.previous_feedback=feedback;self.measurement_seen=True
        self.reference=reference;self.error=reference-feedback
        d=clip(-(self.derivative_q8*8192)>>8,-250*4096,250*4096)
        base=self.error*12288+reference*3277+d
        candidate=clip(self.integral+self.error*1434,-2400*4096,2400*4096)
        trial=base+candidate
        hold=(trial>2400*4096 and self.error>0) or (trial<-2400*4096 and self.error<0)
        if hold:output=base+self.integral
        else:self.integral=candidate;output=trial
        output=clip(output,-2400*4096,2400*4096)
        self.duty=symmetric_round(output,4096)
        return self.duty


class WheelEncoder:
    """1320 CPR, 65mm diameter, 10ms gate; same rational cm/s conversion as RTL."""
    def __init__(self, angle=0., phase=0.):
        self.phase=phase
        self.count=math.floor(angle*1320/(2*math.pi)+phase)
        self.speed_cms=0

    def sample(self, angle):
        count=math.floor(angle*1320/(2*math.pi)+self.phase)
        delta=count-self.count;self.count=count
        self.speed_cms=clip(symmetric_round(delta*1547,1000),-32767,32767)
        return self.speed_cms
