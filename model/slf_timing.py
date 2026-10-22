"""Evaluation cycles of rtl/match/slf_match.v, excluding CPU/bus/update time.

FSM: gate 9/sample; rejected score sample 3 cycles; accepted sample adds18;
9/candidate overhead,4 frame overhead. Counts use the <=1024 sampled points.
"""
def evaluation_cycles(samples, valid, candidates):
    if not (0<=valid<=samples<=1024 and 1<=candidates<=256):
        raise ValueError('SLF timing input exceeds the hardware contract')
    return 4+9*samples+candidates*(3*samples+18*valid+9)


def evaluation_us(samples,valid,candidates,clock_hz=50_000_000):
    cycles=evaluation_cycles(samples,valid,candidates)
    return (cycles*1_000_000+clock_hz-1)//clock_hz
