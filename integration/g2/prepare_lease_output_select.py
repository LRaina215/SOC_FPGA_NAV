"""Move stop selection after identical ramp arithmetic in a frozen build copy.

No register, priority, timeout, arithmetic width or observable cycle changes.
The kept data nets prevent mux factoring from recreating the long stop cone.
"""
from pathlib import Path


def generate(path):
    p=Path(path);s=p.read_text(encoding='utf-8')
    marker=' endfunction\n always @(posedge clk or negedge reset_n)begin'
    wires=''' endfunction
 wire signed [23:0] ramp_v_run /* synthesis syn_keep = 1 */ = approach(out_forward_q12,target_v,STEP_V_Q12);
 wire signed [23:0] ramp_w_run /* synthesis syn_keep = 1 */ = approach(out_yaw_q12,target_w,STEP_W_Q12);
 wire signed [23:0] ramp_v_stop /* synthesis syn_keep = 1 */ = approach(out_forward_q12,24'sd0,STEP_V_Q12);
 wire signed [23:0] ramp_w_stop /* synthesis syn_keep = 1 */ = approach(out_yaw_q12,24'sd0,STEP_W_Q12);
 always @(posedge clk or negedge reset_n)begin'''
    assert s.count(marker)==1;s=s.replace(marker,wires)
    for output,axis,step in [('out_forward_q12','v','STEP_V_Q12'),('out_yaw_q12','w','STEP_W_Q12')]:
        old=f"{output}<=approach({output},stop_mode?24'sd0:target_{axis},{step});"
        new=f'{output}<=stop_mode?ramp_{axis}_stop:ramp_{axis}_run;'
        assert s.count(old)==1;s=s.replace(old,new)
    p.write_text(s,encoding='utf-8',newline='\n')
