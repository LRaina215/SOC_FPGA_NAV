"""Freeze the one selected repair for merged packed-high ROM enables."""
import argparse
import json
from pathlib import Path
import re
from review_stage3_slf128_memories import meta

TB='''`timescale 1ns/1ps
module tb_weight_common_en;
reg clk=0;always #10 clk=~clk;
reg en=0;reg[15:0]addr=0;wire[15:0]oldq,newq;
old_weight_rom a(clk,en,addr,oldq);new_weight_rom b(clk,en,addr,newq);
reg[15:0]gold[0:40703];reg[15:0]held;
integer checks=0,valid_reads=0,hold_checks=0,fourstate_checks=0;
task step(input logic e,input logic[15:0]x);
begin
 @(negedge clk);#1;held=newq;en=e;addr=x;
 @(posedge clk);#2;checks++;
 if(oldq!==newq)$fatal(1,"COMMON_EN_DIFF check%0d en=%b addr=%h old=%h new=%h",checks,e,x,oldq,newq);
 if(e===1'b0)begin hold_checks++;if(newq!==held)$fatal(1,"COMMON_EN_HOLD check%0d",checks);end
 if(e===1'b1&&!$isunknown(x)&&x<40704)begin
  if($isunknown({oldq,newq})||newq!==gold[x])$fatal(1,"COMMON_EN_GOLD addr%0d want%h new%h old%h",x,gold[x],newq,oldq);
  valid_reads++;
 end
 if($isunknown({e,x}))fourstate_checks++;
end endtask
initial begin
 $readmemh("weights.mem",gold);repeat(3)@(negedge clk);
 //Every16-bit address, changing-bank consecutive accepts, and address changes
 //during two idle edges. Domain>40703 is checked for exact four-state behavior.
 for(integer k=0;k<65536;k++)begin step(1,k);step(0,k^65535);step(0,(k+12345)&65535);end
 for(integer k=0;k<256;k++)begin step(1,8192+k);step(1,16384+k);step(1,32768+k);step(1,k);end
 for(integer base=0;base<5;base++)begin
  logic[15:0]x;logic e;
  for(integer bitn=0;bitn<16;bitn++)for(integer z=0;z<2;z++)for(integer ev=0;ev<4;ev++)begin
   case(base)0:x=0;1:x=8192;2:x=16385;3:x=40703;default:x=65535;endcase
   if(z)x[bitn]=1'bz;else x[bitn]=1'bx;
   case(ev)0:e=0;1:e=1;2:e=1'bx;default:e=1'bz;endcase
   step(e,x);step(1,16385);step(0,0);
  end
 end
 $display("TEST PASSED WEIGHT_COMMON_EN all65536=1 valid_reads=%0d hold_checks=%0d fourstate=%0d checks=%0d",valid_reads,hold_checks,fourstate_checks,checks);$finish;
end
initial begin #6000000;$fatal(1,"finite common-en weight watchdog");end
endmodule
'''


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
    source=a.source/'fixed4500_weight_rom.v';old=source.read_text()
    pattern=r'always@\(posedge clk\)if\(en && addr\[15:13\]==3\x27d([0-4])\)(high_q[0-2]\[[0-4]\]<=high_bank[0-2]_[0-4]\[addr\[12:2\]\];)'
    hits=re.findall(pattern,old);assert len(hits)==15 and sorted(int(i)for i,_ in hits)==sorted(list(range(5))*3)
    new,n=re.subn(pattern,r'always@(posedge clk)if(en)\2',old);assert n==15
    (a.out/'fixed4500_weight_rom.v').write_text(new)
    (a.out/'old.v').write_text(old.replace('module fixed4500_weight_rom(','module old_weight_rom(',1))
    (a.out/'new.v').write_text(new.replace('module fixed4500_weight_rom(','module new_weight_rom(',1));(a.out/'tb.sv').write_text(TB)
    r=dict(prepared=True,ancestor=meta(source),repaired_source=meta(a.out/'fixed4500_weight_rom.v'),changed_statements=15,
           exact_change='Only high_q load enables change from en&&bank_select to common en; all15 private high payloads prefetch same addr[12:2] on each accepted read',
           preserved=['20low9-bit ROMs','35INIT payloads/all40704 original weights','selected_low/high/lane and threebit14 correction addresses','one public read clock','en0 public hold','all source address widths/ports'],
           budget=dict(predicted_BSRAM=35,low_pROMX9=20,high_pROM=15,added_storage_bits=0,added_ports=0,added_public_cycles=0,actual_FF_LUT_unknown=True),
           validation_inputs=[meta(a.out/f)for f in('old.v','new.v','tb.sv')],
           proof_basis='At every accepted edge the public selector chooses the same bank whose current indexed data was read. Extra unselected bank loads cannot be observed before another accepted read reloads the selected bank; idle keeps all private payloads/selectors. X/Z cases require exact four-state comparison.',
           official_attribute_review='SUG550-2.4E54-55 syn_keep preserves nets/duplicate cells;62 syn_preserve preserves registers. No verified local no-ROM-cross-enable-merge switch found; no global option changed.',
           tools_started=False,automatic_joint_SYN=False,automatic_PNR=False)
    (a.out/'preparation.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');print(json.dumps(r,ensure_ascii=False))


if __name__=='__main__':main()
