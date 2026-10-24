"""Update-only integration proof using the original extractor/point transform.

The generated probe deliberately retains the old scoring RAM. It MUST NOT be
used as a complete matcher or joint candidate: scoring/cache coherence and
other clients are separate work. Only UPDATE is exercised in this proof.
"""
from datetime import datetime
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess

from probe_row_update import HERE,ROOT


def derive(original):
    s=original
    def one(a,b):
        nonlocal s
        assert s.count(a)==1,a
        s=s.replace(a,b)
    one('module slf_match #(','module slf_match_row_probe #(')
    one('    output reg         eval_done\n);','''    output reg         eval_done,
    input wire ui_clk,calib_done,output wire row_fault,
    output wire[2:0]row_cmd,output wire row_cmd_en,output wire[27:0]row_addr,
    input wire row_cmd_ready,output wire[255:0]row_wr_data,
    output wire[31:0]row_wr_mask,output wire row_wr_en,row_wr_end,
    input wire row_wr_ready,input wire[255:0]row_rd_data,
    input wire row_rd_valid,row_rd_end
);''')
    one('    assign busy = (st != S_IDLE) || lf_busy || ex_busy;', '''    wire row_busy,row_done,row_ready,row_record_ready;
    assign busy = (st != S_IDLE) || lf_busy || ex_busy || row_busy;
    nav_slf_row_update u_rows(
        clk,ui_clk,rst,calib_done,
        cmd_update&&!cfg_prior&&st==S_IDLE&&row_ready,lf_cx,lf_cy,
        md==MD_UPD&&st==S_U0,u_slot,{u_tag,u_mx[3:1],u_my[3:1]},
        md==MD_UPD&&st==S_UC0,row_record_ready,row_ready,row_busy,row_done,row_fault,
        row_cmd,row_cmd_en,row_addr,row_cmd_ready,row_wr_data,row_wr_mask,
        row_wr_en,row_wr_end,row_wr_ready,row_rd_data,row_rd_valid,row_rd_end);
''')
    one('''                    sm_re_m <= 1'b1; sm_ra_m <= {wya[LOG2N-1:0], wxa[LOG2N-1:0]};
                    tb_re <= 1'b1; tb_ra <= {wya[LOG2N-1:0], wxa[LOG2N-1:0]};
                    st <= S_U0;''', '                    st <= S_U0;')
    one('            S_U0: st <= S_U1;', '''            S_U0: if(row_record_ready)begin
                j<=jn;jj<=jj+1'b1;st<=S_T0;
            end''')
    a=s.index('            S_UC0: begin\n');b=s.index('            S_UC1:',a)
    s=s[:a]+'''            S_UC0:begin
                // Hold end through the final three-cycle record acceptance.
                if(cli==2)begin
                    lfo_x<=lf_cx-N/2;lfo_y<=lf_cy-N/2;st<=S_LFW;
                end else cli<=cli+1'b1;
            end
'''+s[b:]
    one('            S_LFW: if (lf_done) st <= S_IDLE;',
        '            S_LFW: if (row_done) st <= S_IDLE;')
    return s


def instance(module,name,suffix,extra=''):
    ports=dict(clk='clk',rst='rst',s_fire='fire',s_x='x',s_y='y',s_z='z',
        s_pv='pv',s_sof='sof',s_eof='eof',frame_en="1'b1",cfg_zlo='zlo',cfg_zhi='zhi',
        cfg_tx='tx',cfg_ty='ty',cfg_inv_res="16'd1311",cmd_eval="1'b0",cmd_update='update',
        cmd_clear="1'b0",cfg_cx_mm='cxm',cfg_cy_mm='cym',cfg_ncand="9'd245",upd_dx='dx',
        upd_dy='dy',upd_dyaw='dyaw',lf_cx='lcx',lf_cy='lcy',cfg_prior="1'b0",cfg_nogate="1'b0",
        cfg_lfo_x="32'd0",cfg_lfo_y="32'd0",pw_we="1'b0",pw_addr="16'd0",pw_data="18'd0",
        cand_we="1'b0",cand_waddr="9'd0",cand_wdata="32'd0",res_re="1'b0",res_raddr="9'd0",
        res_q='',dbg_req="1'b0",dbg_addr="16'd0",dbg_ack='',dbg_q='',busy='busy'+suffix,
        pts_ready='pts'+suffix,n_pts='n'+suffix,eval_done='')
    return module+' '+name+'(\n'+',\n'.join(f'.{p}({v})' for p,v in ports.items())+extra+');\n'


def run(out,row_probe):
    assert not out.exists();out.mkdir(parents=True)
    assert json.loads((row_probe/'functional_result.json').read_text())['passed']
    for name in ('nav_ddr_native.v','nav_ddr_row_stream.v','nav_slf_row_update.v'):
        assert (row_probe/name).read_bytes()==(HERE/name).read_bytes()
        shutil.copyfile(HERE/name,out/name)
    shutil.copyfile(row_probe/'slf_lf_ce.v',out/'slf_lf_ce.v')
    frozen=Path('F:/nav_integration_g2_20261023/sfp_lane2_private_snapshot_syn11_retry1/nav/rtl/match/slf_match.v')
    assert frozen.read_bytes()==(ROOT/'rtl/match/slf_match.v').read_bytes()
    original=frozen.read_text(encoding='utf-8-sig')
    (out/'slf_match_row_probe.v').write_text(derive(original),encoding='utf-8')
    for rel in ('rtl/match/slf_match.v','rtl/match/slf_lf.v','rtl/match/slf_extract.v','rtl/common/nav_ram_sdp.v','rtl/plan/sin_lut.hex'):
        shutil.copyfile(ROOT/rel,out/Path(rel).name)
    for name in ('pts.hex','frm.hex'):shutil.copyfile(ROOT/'sim/build/tb_slf'/name,out/name)
    service=(HERE/'tb_slf_row_update.sv').read_text().split('module tb_slf_row_update;')[0]
    (out/'row_service_model.sv').write_text(service)
    tb='''`timescale 1ns/1ps
module tb_transform;
reg clk=0,ui_clk=0;always #10 clk=~clk;always #5 ui_clk=~ui_clk;
reg rst=1,fire=0,pv=0,sof=0,eof=0,update=0;
reg signed[31:0]x=0,y=0,z=0,zlo,zhi,tx,ty,cxm,cym,lcx,lcy,dx,dy;
reg[15:0]dyaw;
wire busya,busyb,ptsa,ptsb;wire[12:0]na,nb;
wire fault;wire[2:0]cmd;wire cmd_en,cmd_ready,wen,wend,wready,rv,re;
wire[27:0]addr;wire[255:0]wd,rd;wire[31:0]wm;
integer refresh_phase=389,ready_period=16,read_delay=20;
reg[31:0]points[0:9600*4*5-1],frames[0:12*5-1];
integer cycles=0,started,max_cycles=0,checks=0;
always@(posedge clk)begin cycles++;if(fault)$fatal(1,"row fault");end
'''
    tb+=instance('slf_match','reference_core','a')
    tb+=instance('slf_match_row_probe','candidate_core','b',''',.ui_clk(ui_clk),.calib_done(1'b1),.row_fault(fault),
.row_cmd(cmd),.row_cmd_en(cmd_en),.row_addr(addr),.row_cmd_ready(cmd_ready),
.row_wr_data(wd),.row_wr_mask(wm),.row_wr_en(wen),.row_wr_end(wend),.row_wr_ready(wready),
.row_rd_data(rd),.row_rd_valid(rv),.row_rd_end(re)''')
    tb+='''row_service_model dram(ui_clk,rst,1'b1,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re,
refresh_phase,ready_period,read_delay);
initial begin
 $readmemh("pts.hex",points);$readmemh("frm.hex",frames);
 for(integer a=0;a<8192;a++)dram.mem[a]=0;
 repeat(16)@(negedge clk);rst=0;
 wait(!busya&&!busyb);
 for(integer f=0;f<5;f++)begin
  {zlo,zhi,tx,ty,cxm,cym,lcx,lcy,dx,dy}={frames[f*12],frames[f*12+1],frames[f*12+2],frames[f*12+3],
   frames[f*12+4],frames[f*12+5],frames[f*12+6],frames[f*12+7],frames[f*12+8],frames[f*12+9]};
  dyaw=frames[f*12+10];
  for(integer p=0;p<9600;p++)begin
   @(negedge clk);fire=1;x=points[(f*9600+p)*4];y=points[(f*9600+p)*4+1];z=points[(f*9600+p)*4+2];
   {eof,sof,pv}=points[(f*9600+p)*4+3][2:0];
  end
  @(negedge clk);fire=0;
  wait(ptsa&&ptsb);#1;
  if(na!==nb||na!==frames[f*12+11][12:0])$fatal(1,"extraction changed");
  @(negedge clk);started=cycles;update=1;
  @(negedge clk);update=0;
  repeat(5)@(negedge clk);wait(!busya&&!busyb);#1;
  $display("TRANSFORM_TIMING frame=%0d points=%0d cycles=%0d us_at_50MHz=%0f",f,na,cycles-started,(cycles-started)/50.0);
  if(cycles-started>250000)$fatal(1,"model5ms failed");
  if(cycles-started>max_cycles)max_cycles=cycles-started;
  for(integer a=0;a<65536;a++)begin
   if(dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]!==reference_core.u_sm.mem[a])
    $fatal(1,"original update mismatch frame=%0d addr=%h got=%h gold=%h",f,a,
     dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18],reference_core.u_sm.mem[a]);
   checks++;
  end
 end
 $display("TEST PASSED frames=5 full_words=%0d maximum_cycles=%0d",checks,max_cycles);$finish;
end
initial begin #100000000;$fatal(1,"bounded simulation budget");end
endmodule
'''
    (out/'tb_transform.sv').write_text(tb)
    viv=Path('E:/2026.1/Vivado/bin')
    commands=[('xvlog',['nav_ddr_native.v','nav_ddr_row_stream.v','nav_slf_row_update.v',
        'slf_lf_ce.v','slf_match.v','slf_match_row_probe.v','slf_lf.v','slf_extract.v','nav_ram_sdp.v']),
        ('xvlog',['-sv','row_service_model.sv','tb_transform.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_transform','-s','snap']),('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,stdout=log,
                stderr=subprocess.STDOUT,timeout=180,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'FATAL|Fatal|ERROR:',s),s[-4000:]
    proof=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        timing=[{k:float(v) if '.' in v else int(v) for k,v in re.findall(r'(\w+)=([\d.]+)',line)}
            for line in s.splitlines() if line.startswith('TRANSFORM_TIMING')],
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        scope='original point stream/extractor/transform plus streaming UPDATE vs unchanged original matcher; native service model',
        complete_matcher=False,scoring_coherence_verified=False,vendor_service_bound_verified=False,
        full_joint_implementation=False,programmed=False)
    (out/'result.json').write_text(json.dumps(proof,indent=2))
    print(json.dumps(proof,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--row-probe',type=Path,required=True);a=p.parse_args()
    run(a.out.resolve(),a.row_probe.resolve())
