"""Only change the two reset ternaries rejected by actual C synthesis."""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,shutil
import psutil
from prepare_baseline_fusion_retime import metadata,same_bytes,save
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'stage3_SLF128_storage_RTL_prepared5';OUT=BASE/'stage3_FIFO_async_reset_syntax_candidate1';FIFO='nav/rtl/common/nav_async_fifo.v'
def prepare():
 assert not OUT.exists();assert not any((p.info['name']or'').lower()in{'xsim.exe','xsimk.exe','xelab.exe','xvlog.exe','vsim.exe','vsimk.exe'}for p in psutil.process_iter(['name']))
 OUT.mkdir();old=(SRC/FIFO).read_text(encoding='utf8');new=old
 for clk,rst,q,gray in [('wclk','wrst','rp_b_w','rp_g_w2'),('rclk','rrst','wp_b_r','wp_g_r2')]:
  a=f"    always @(posedge {clk} or posedge {rst}) {q} <= {rst} ? {{AW+1{{1'b0}}}} : g2b({gray});"
  b=f"""    always @(posedge {clk} or posedge {rst}) begin
        if ({rst}) {q} <= {{AW+1{{1'b0}}}};
        else {q} <= {rst} ? {{AW+1{{1'b0}}}} : g2b({gray});
    end"""
  assert new.count(a)==1,a;new=new.replace(a,b,1)
 (OUT/'nav_async_fifo.v').write_text(new,encoding='utf8',newline='\n')
 before=old.replace('module nav_async_fifo #(','module nav_async_fifo_before #(',1).replace('nav_fifo_ob_async_assert','nav_fifo_ob_async_assert_before')
 (OUT/'nav_async_fifo_before.v').write_text(before,encoding='utf8',newline='\n')
 for rel in ['nav/rtl/common/nav_cdc.v','nav/rtl/common/nav_fifo_ob.v']:shutil.copyfile(SRC/rel,OUT/Path(rel).name)
 shutil.copyfile(Path(__file__).with_name('tb_fifo_reset_syntax.sv'),OUT/'tb_fifo_reset_syntax.sv')
 (OUT/'syntax_only.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='prepared5/'+FIFO,tofile='canonical/'+FIFO)),encoding='utf8')
 r=dict(prepared=True,at=datetime.now().astimezone().isoformat(),candidate=str(OUT),source=str(SRC),modified_source_inputs=[FIFO],two_always_only=True,full_four_state_semantics='if reset=1 clear; if0/X/Z execute the original ternary. Thus for all four reset values and arbitrary vector D the original conditional merge is identical bitwise, including X/Z. Same sensitivity and nonblocking assignment.',RAM_processes_capacity_pointer_math_private_FWFT_public_helper_unchanged=True,old_Gowin_failure=str(BASE/'SLF128_storage_safeFW_syn41'),known_error='EX3534 single ternary under posedge clock or posedge async reset not recognized as asynchronous reset',test_profiles=[dict(DW=10,AW=11),dict(DW=10,AW=12),dict(DW=9,AW=11)],source_binding=metadata(SRC/FIFO),bindings=[metadata(p)for p in sorted(OUT.iterdir())if p.is_file()],SYN_started=False,RTL_simulation_started=False,PNR_started=False,programmed=False)
 save(OUT/'prepared_review.json',r);print(json.dumps(dict(candidate=str(OUT),two_always_only=True,files=len(r['bindings'])),ensure_ascii=False));return r
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');a=p.parse_args();assert a.prepare;prepare()
