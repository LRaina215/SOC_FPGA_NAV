"""Cross-check sample-level integer PID against actual RTL, not a copied expected formula."""
import os,subprocess,json
from pathlib import Path
import torch
from fpga_pid import make_state,step
root=Path(__file__).parent
samples=[]
# Stable, reversals, zero resets, fractional D rounding, both latched faults.
for i in range(150):samples.append((300, min(30,i),1))
for i in range(40):samples.append((-300,max(-30,30-i*3),1))
samples += [(0,0,1)]*2+[(500,0,1)]*110+[(0,0,1)]*2+[(-400,10,1)]*25+[(0,0,1)]*2
for i in range(80):samples.append((((i*137)%1201)-600,((i*17)%111)-55,1))
samples += [(300,10,0)]*2+[(300,30,1)]*4
stim=root/'pid_stim.hex'
stim.write_text('\n'.join(f'{(t&0xffffffff):08x}{(v&0xffff):04x}{en:01x}' for t,v,en in samples)+'\n')
tb=root/'tb_pid_crosscheck.v'
tb.write_text('''`timescale 1ns/1ps
module tb_pid_crosscheck;
reg clk=0;always #10 clk=~clk;
reg reset_n=0,enable=1,valid=0;reg signed [31:0] target;reg signed [15:0] speed;
wire signed [15:0] duty;wire [2:0] fault;
reg [51:0] input_samples[0:%d];integer i;
motor520_velocity_pid dut(.clk(clk),.reset_n(reset_n),.enable(enable),.target_mmps(target),.feedback_cms(speed),.sample_valid(valid),.duty_ticks(duty),.fault_code(fault));
initial begin
$readmemh("%s",input_samples);target=0;speed=0;#100;reset_n=1;
for(i=0;i<%d;i=i+1)begin
@(negedge clk);{target,speed,enable}={input_samples[i][51:20],input_samples[i][19:4],input_samples[i][0]};
// Leave time for zero/disable handling before publishing the sample.
repeat(2)@(negedge clk);valid=1;@(negedge clk);valid=0;
repeat(20)@(negedge clk);$display("ROW %%d %%d %%d",i,$signed(duty),fault);
end
$finish;end
endmodule
'''%(len(samples)-1,stim,len(samples)))
# Stim layout contains a four-bit enable nibble; input assignment above picks bit0.
ivl='/home/cyd/.iverilog/usr'
subprocess.run([ivl+'/bin/iverilog','-B',ivl+'/lib/x86_64-linux-gnu/ivl','-g2012','-s','tb_pid_crosscheck','-o',str(root/'pid.vvp'),'/home/cyd/fpga/fpga/src/motor520_closed_loop.v',str(tb)],check=True)
result=subprocess.run([ivl+'/bin/vvp',str(root/'pid.vvp')],capture_output=True,text=True,check=True)
(root/'pid_rtl.log').write_text(result.stdout)
rows=[line.split() for line in result.stdout.splitlines() if line.startswith('ROW')]
assert len(rows)==len(samples)
s=make_state((1,))
for i,((t,v,en),row) in enumerate(zip(samples,rows)):
 d,f=step(s,torch.tensor([t]),torch.tensor([v]),torch.tensor([bool(en)]))
 assert (int(d[0]),int(f[0]))==(int(row[2]),int(row[3])),(i,t,v,en,int(d[0]),int(f[0]),row)
(root/'pid_verification.json').write_text(json.dumps({'result':'PASS','samples':len(samples),'checks':'actual RTL sample outputs vs integer torch model; reversal, zero, antiwindup, D filtering, stall, wrong direction, disable'},indent=2))
print('PASS PID vs RTL',len(samples),'samples')
