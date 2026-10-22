"""1 ms transaction-level reference of rtl/plan/safety_guard.v.

RTL's sub-microsecond pipeline latency is omitted. Enable is an explicit
simulation startup setting; the register reset value itself is disabled.
"""
class SafeModel:
    def __init__(self, enabled=True, vmax=500, wmax=1200, dv=2, dw=3, watchdog_us=200000):
        self.enabled, self.vmax, self.wmax = enabled, vmax, wmax
        self.dv, self.dw, self.watchdog_us = dv, dw, watchdog_us
        self.target = (0, 0, 0)
        self.output = (0, 0, 0)
        self.last_us = None
        self.wdog = True

    def submit(self, cmd, time_us):
        self.target, self.last_us = tuple(map(int, cmd)), time_us & 0xffffffff

    def tick(self, time_us, estop=False):
        self.wdog = self.last_us is None or ((time_us - self.last_us) & 0xffffffff) > self.watchdog_us
        if not self.enabled or estop:
            self.output = (0, 0, 0)
        else:
            t = (0, 0, 0) if self.wdog else (max(-self.vmax, min(self.vmax, self.target[0])),
                                             0, max(-self.wmax, min(self.wmax, self.target[2])))
            self.output = tuple(c + max(-d, min(d, v-c)) for c, v, d in zip(self.output, t, (self.dv,self.dv,self.dw)))
        return self.output
