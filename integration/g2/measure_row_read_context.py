"""One-shot, small, low-priority context validation beside an existing joint PNR.

The complete frozen extractor is identical in both variants. Only its row RAM
implementation changes. The measurement shell supplies clocked input launches;
it is not a product change. Each result is local, never joint acceptance.
"""
import argparse
from datetime import datetime
import html
import json
from pathlib import Path
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import psutil
from measure_slf_row_banks import TOOL, VIV, inventory
from prepare_row_read_share import transform
from retry_frozen_synthesis import memory_status

GIB=1024**3
HERE=Path(__file__).resolve().parent
TOP='''module g2_top(input clk,rst,en,s_fire,s_pv,s_sof,s_eof,
 input load,input[2:0]select,input[31:0]data,
 output pb_we,output[11:0]pb_waddr,output[35:0]pb_wdata,
 output done,output[12:0]n_pts,output busy);
 // Fixture-only launches reduce physical IO count; no change inside slf_extract.
 reg[31:0]zlo,zhi,tx,ty,x,y,z;
 reg rst_q,en_q,fire_q,pv_q,sof_q,eof_q;
 always @(posedge clk)begin
  rst_q<=rst;en_q<=en;fire_q<=s_fire;pv_q<=s_pv;sof_q<=s_sof;eof_q<=s_eof;
  if(load)case(select)
   0:zlo<=data;1:zhi<=data;2:tx<=data;3:ty<=data;
   4:x<=data;5:y<=data;6:z<=data;
  endcase
 end
 slf_extract u_extract(clk,rst_q,en_q,zlo,zhi,tx,ty,fire_q,x,y,z,pv_q,sof_q,eof_q,
  pb_we,pb_waddr,pb_wdata,done,n_pts,busy);
endmodule
'''


def now():return datetime.now().astimezone().isoformat()
def write(path,value):path.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')


def checked_options(options):
    actual={}
    for key,expected in [('global_freq',50.0),('convert_sdp32_36_to_sdp16_18',1),('timing_driven',1)]:
        match=re.search(r'^set_option -'+key+r'\s+(\S+)\s*$',options,re.M)
        assert match, 'missing effective option '+key
        # Gowin serializes50MHz as50.000. Compare numeric value, not spelling.
        actual[key]=float(match[1]) if key=='global_freq' else int(match[1])
        assert actual[key]==expected,(key,actual[key],expected)
    return actual


def reference_snapshot(build):
    runtime=json.loads((build/'tool_runtime.json').read_text())
    result=dict(build=str(build),pid=runtime['pid'],started_at=runtime['started_at'],
                total_limit_s=runtime['max_runtime_s'],idle_limit_s=runtime['max_no_observable_progress_s'])
    assert result['total_limit_s']==result['idle_limit_s']==0
    result['input_metadata']={str(p.relative_to(build)):[p.stat().st_size,p.stat().st_mtime_ns]
                             for p in [build/'g2_top.vg',build/'g2.sdc',build/'g2.cst',build/'run.tcl',*sorted((build/'ip').rglob('*.csr'))]}
    try:
        q=psutil.Process(runtime['pid']);result.update(created_epoch=q.create_time(),command=q.cmdline())
    except psutil.NoSuchProcess:result['process_exited']=True
    return result


def prepare(source,out,proof,reference):
    assert not out.exists(), 'new evidence directory required'
    old=(source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8')
    new=transform(old)
    previous=json.loads((proof/'result.json').read_text())
    assert previous['phase']=='isolated_proof_complete' and len(previous['checks'])==3
    assert new==(proof/'candidate.v').read_text(encoding='utf-8'), 'reuse the already proven candidate'
    extractor=(source/'nav/rtl/match/slf_extract.v').read_bytes()
    out.mkdir(parents=True)
    state=dict(created_at=now(),phase='prepared',source=str(source),prior_memory_proof=str(proof),
               protected_joint_build=reference_snapshot(reference),joint_build_modified=False,
               full_joint_acceptance=False,programmed=False,results={},
               scope='complete original extractor, local timing context; not joint routing/CDC acceptance',
               execution_policy=dict(priority='BELOW_NORMAL',maximum_cpu_cores=1,maximum_owned_rss_bytes=768*1024**2,
                  maximum_local_PNR_owned_rss_bytes=1536*1024**2,
                  local_PNR_runtime_minimum_physical_bytes=GIB,local_PNR_runtime_minimum_commit_bytes=2*GIB,
                  small_start_physical_bytes=GIB,small_start_commit_bytes=3*GIB,
                  physical_start_physical_bytes=2*GIB,physical_start_commit_bytes=4*GIB,
                  own_PNR_total_s=0,own_PNR_idle_s=0,changes_to_existing_PNR=False))
    for name,ram in [('baseline',old),('shared',new)]:
        folder=out/name;folder.mkdir()
        (folder/'ram.v').write_text(ram,encoding='utf-8',newline='\n')
        (folder/'extract.v').write_bytes(extractor)
        (folder/'top.v').write_text(TOP,encoding='utf-8',newline='\n')
        (folder/'clock.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk}]\n',encoding='utf-8')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
        settings += [f'add_file -type verilog {{{(folder/n).as_posix()}}}' for n in ('ram.v','extract.v','top.v')]
        settings += [f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}',
                     'set_option -top_module g2_top','set_option -output_base_name g2_top',
                     'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 1',
                     'set_option -looplimit 100000','set_option -place_option 1','set_option -route_option 2',
                     'set_option -timing_driven 1','set_option -gen_verilog_sim_netlist 1']
        (folder/'preflight.tcl').write_text('\n'.join(settings+[f'saveto -all_options {{{(folder/"effective_options.tcl").as_posix()}}}','exit'])+'\n',encoding='utf-8')
        (folder/'synthesis.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n',encoding='utf-8')
        # Explicitly use the measured netlist in a fresh PNR session, never resynthesize.
        physical_settings=[x for x in settings if not x.startswith('add_file -type verilog')]
        physical_settings.insert(1,f'add_file -type netlist {{{(folder/"impl/gwsynthesis/g2_top.vg").as_posix()}}}')
        (folder/'physical.tcl').write_text('\n'.join(physical_settings+['run pnr','exit'])+'\n',encoding='utf-8')
    sim=out/'context_rtl';sim.mkdir()
    for name,suffix in [('baseline','old'),('shared','new')]:
        text=(out/name/'ram.v').read_text(encoding='utf-8').replace('module nav_ram_sdp #(','module ram_'+suffix+' #(',1)
        (sim/(suffix+'_ram.v')).write_text(text,encoding='utf-8')
        text=extractor.decode('utf-8').replace('module slf_extract #(','module extract_'+suffix+' #(',1).replace('nav_ram_sdp #(','ram_'+suffix+' #(',1)
        (sim/(suffix+'_extract.v')).write_text(text,encoding='utf-8')
    (sim/'tb.sv').write_bytes((HERE/'tb_row_read_context.sv').read_bytes())
    assert (out/'baseline/extract.v').read_bytes()==(out/'shared/extract.v').read_bytes()==extractor
    write(out/'result.json',state)


class Runner:
    def __init__(self,out,resume_option_review=False,resume_physical_review=False):
        self.out=out;self.state=json.loads((out/'result.json').read_text())
        self.resume_option_review=resume_option_review
        self.resume_physical_review=resume_physical_review
        if resume_option_review:
            assert self.state['phase']=='local_analysis_failed_requires_review'
            assert self.state.get('failure','').startswith("('global_freq',"),'only the reviewed numeric-format rejection'
            assert not self.state.get('active_child') and not self.state['results']
            try:
                old=psutil.Process(self.state['controller_pid'])
                assert abs(old.create_time()-self.state['controller_created_epoch'])>1,'old controller is still active'
            except psutil.NoSuchProcess:pass
            source=Path(self.state['source'])
            baseline_ram=(source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8')
            extractor=(source/'nav/rtl/match/slf_extract.v').read_bytes()
            for name,ram,suffix in [('baseline',baseline_ram,'old'),('shared',transform(baseline_ram),'new')]:
                folder=out/name
                assert (folder/'ram.v').read_text(encoding='utf-8')==ram
                assert (folder/'extract.v').read_bytes()==extractor
                assert (folder/'top.v').read_text(encoding='utf-8')==TOP
                assert not (folder/'synthesis_runtime.json').exists() and not (folder/'impl/gwsynthesis/g2_top.vg').exists(), 'SYN must not already have run'
                assert (out/'context_rtl'/(suffix+'_ram.v')).read_text(encoding='utf-8')==ram.replace('module nav_ram_sdp #(','module ram_'+suffix+' #(',1)
                generated=extractor.decode('utf-8').replace('module slf_extract #(','module extract_'+suffix+' #(',1).replace('nav_ram_sdp #(','ram_'+suffix+' #(',1)
                # prepare() used Windows write_text newline translation, including
                # existing CRLF. Match those exact saved bytes, not stripped RTL.
                assert (out/'context_rtl'/(suffix+'_extract.v')).read_bytes()==generated.replace('\n','\r\n').encode('utf-8')
            assert (out/'context_rtl/tb.sv').read_bytes()==(HERE/'tb_row_read_context.sv').read_bytes()
            for relative in ['context_rtl/'+label+'_runtime.json' for label in ('rtl_compile','bench_compile','elaborate','simulate')]+['baseline/preflight_runtime.json']:
                runtime=json.loads((out/relative).read_text())
                assert runtime['tool_exit']==0 and not runtime['cancelled'],relative
            log=(out/'context_rtl/xsim.log').read_text(errors='replace')
            assert self.state['context_behavior'] in log and 'Fatal:' not in log
            checked_options((out/'baseline/effective_options.tcl').read_text(encoding='utf-8'))
            saved=out/'result_before_option_format_fix.json';assert not saved.exists()
            saved.write_bytes((out/'result.json').read_bytes())
            self.state.update(resumed_after='numeric50 versus50.000 option formatting; completed RTL/preflight reused',previous_result=str(saved))
            del self.state['failure']
        elif resume_physical_review:
            assert self.state['phase']=='local_analysis_failed_requires_review' and not self.state.get('active_child')
            try:
                old=psutil.Process(self.state['controller_pid'])
                assert abs(old.create_time()-self.state['controller_created_epoch'])>1,'old controller is still active'
            except psutil.NoSuchProcess:pass
            stopped=json.loads((out/'baseline/physical_runtime.json').read_text())
            assert stopped['cancelled'] and stopped['reason']=='owned probe or host memory protection'
            assert stopped['peak_owned_rss_bytes']>768*1024**2 and stopped['maximum_runtime_s']==0
            assert not (out/'baseline/impl/pnr/g2_top.tr').exists() and not (out/'shared/physical_runtime.json').exists()
            try:
                old=psutil.Process(stopped['pid'])
                assert abs(old.create_time()-stopped['created_epoch'])>1,'previous local router still active'
            except psutil.NoSuchProcess:pass
            source=Path(self.state['source'])
            old_ram=(source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8')
            for name,ram in [('baseline',old_ram),('shared',transform(old_ram))]:
                folder=out/name;row=self.state['results'][name]
                assert row['synthesis']['tool_exit']==0 and not row['synthesis']['cancelled']
                assert not row['no_timing_path'] and row['path_summaries']
                assert (folder/'ram.v').read_text(encoding='utf-8')==ram
                assert (folder/'extract.v').read_bytes()==(source/'nav/rtl/match/slf_extract.v').read_bytes()
                assert (folder/'top.v').read_text(encoding='utf-8')==TOP
                assert (folder/'clock.sdc').read_text()=='create_clock -name clk50 -period 20 [get_ports {clk}]\n'
                checked_options((folder/'effective_options.tcl').read_text(encoding='utf-8'))
                assert (folder/'impl/gwsynthesis/g2_top.vg').is_file()
            log=(out/'context_rtl/xsim.log').read_text(errors='replace')
            assert self.state['context_behavior'] in log and 'Fatal:' not in log
            saved=out/'result_before_local_memory_fix.json';assert not saved.exists()
            saved.write_bytes((out/'result.json').read_bytes())
            self.state.update(physical_budget_review=dict(previous_result=str(saved),previous_local_route=stopped,
                  reason='768MiB synthesis limit was incorrectly reused for device routing; preserve2/4GiB startup and stronger host reserve, permit1.5GiB for this isolated route only'),
                  reused_completed_synthesis=['baseline','shared'])
            self.state['execution_policy'].update(maximum_local_PNR_owned_rss_bytes=1536*1024**2,
                  local_PNR_runtime_minimum_physical_bytes=GIB,local_PNR_runtime_minimum_commit_bytes=2*GIB)
            del self.state['failure']
        else:
            assert self.state['phase']=='prepared','one-shot only; inspect existing result instead of rerunning'
        self.state.update(controller_pid=psutil.Process().pid,controller_created_epoch=psutil.Process().create_time())
        me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
        me.cpu_affinity([max(me.cpu_affinity())]);self.state['assigned_cpu']=me.cpu_affinity()
        self.save('waiting_for_small_probe_memory')

    def save(self,phase=None):
        if phase:self.state['phase']=phase
        self.state['updated_at']=now();write(self.out/'result.json',self.state)

    def wait_memory(self,physical=False):
        while True:
            m=memory_status()
            self.state['host_memory']=dict(free_bytes=m.avail_phys,commit_reserve_bytes=m.avail_page)
            self.save('waiting_for_local_PNR_memory' if physical else 'waiting_for_small_probe_memory')
            if m.avail_phys>=(2 if physical else 1)*GIB and m.avail_page>=(4 if physical else 3)*GIB:return m
            time.sleep(10)

    def run(self,folder,label,command,limit=180,physical=False):
        m=self.wait_memory(physical)
        self.save(label+'_running')
        runtime=dict(command=command,started_at=now(),maximum_runtime_s=limit,maximum_silent_s=0,
                     startup_free_bytes=m.avail_phys,startup_commit_reserve_bytes=m.avail_page,
                     priority='BELOW_NORMAL',cpu_affinity=self.state['assigned_cpu'],cancelled=False)
        owned_limit=(1536 if physical else 768)*1024**2
        runtime['maximum_owned_rss_bytes']=owned_limit
        with (folder/(label+'.log')).open('w',encoding='utf-8') as stream:
            p=subprocess.Popen(command,cwd=folder,stdout=stream,stderr=subprocess.STDOUT,
                 creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
            runtime['pid']=p.pid
            try:runtime['created_epoch']=psutil.Process(p.pid).create_time()
            except psutil.NoSuchProcess:pass
            self.state['active_child']=runtime.copy();self.save()
            start=time.monotonic();peak=0
            while p.poll() is None:
                try:children=[psutil.Process(p.pid),*psutil.Process(p.pid).children(recursive=True)]
                except psutil.NoSuchProcess:children=[]
                used=0
                for q in children:
                    try:used+=q.memory_info().rss
                    except psutil.NoSuchProcess:pass
                peak=max(peak,used);m=memory_status()
                reason=''
                if limit and time.monotonic()-start>limit:reason='finite local analysis runtime budget'
                if used>owned_limit:reason='owned probe memory budget'
                if m.avail_page<(2 if physical else 1)*GIB or (physical and m.avail_phys<GIB):reason='host memory protection'
                if reason:
                    runtime.update(cancelled=True,reason=reason,stop_owned_rss_bytes=used,
                                   stop_free_bytes=m.avail_phys,stop_commit_reserve_bytes=m.avail_page)
                    # Only this probe's child tree, never the protected joint PNR.
                    for q in reversed(children):
                        try:q.kill()
                        except psutil.NoSuchProcess:pass
                    p.wait();break
                time.sleep(.5)
            p.wait();runtime.update(tool_exit=p.returncode,elapsed_s=time.monotonic()-start,peak_owned_rss_bytes=peak,ended_at=now())
        write(folder/(label+'_runtime.json'),runtime)
        self.state['active_child']=None;self.save(label+'_finished')
        assert p.returncode==0 and not runtime['cancelled'],runtime
        return runtime

    def execute(self):
        folder=self.out/'context_rtl'
        files=['old_ram.v','old_extract.v','new_ram.v','new_extract.v']
        if not (self.resume_option_review or self.resume_physical_review):
            for label,exe,args in [('rtl_compile','xvlog',files),('bench_compile','xvlog',['-sv','tb.sv']),
                                  ('elaborate','xelab',['-debug','off','-timescale','1ns/1ps','tb','-s','snap']),
                                  ('simulate','xsim',['snap','-runall'])]:
                self.run(folder,label,['cmd','/c',str(VIV/(exe+'.bat')),*args],limit=120)
        log=(folder/'xsim.log').read_text(errors='replace')
        assert 'TEST PASSED' in log and 'Fatal:' not in log,log[-3000:]
        self.state['context_behavior']=re.search(r'TEST PASSED[^\n]*',log)[0];self.save('context_behavior_passed')
        for name in ('baseline','shared'):
            if self.resume_physical_review:continue
            folder=self.out/name
            if not (self.resume_option_review and name=='baseline'):
                self.run(folder,'preflight',[str(TOOL),'preflight.tcl'],limit=60)
            options=(folder/'effective_options.tcl').read_text(encoding='utf-8')
            actual_options=checked_options(options)
            runtime=self.run(folder,'synthesis',[str(TOOL),'synthesis.tcl'])
            log=(folder/'synthesis.log').read_text(errors='replace');assert not re.search(r'\bERROR\s*\(',log),log[-3000:]
            syn=folder/'impl/gwsynthesis'
            rpt=(syn/'g2_top_syn.rpt.html').read_text(encoding='utf-8')
            def cells(s):return [html.unescape(re.sub('<[^>]*>','',x)).strip() for x in re.findall(r'<td[^>]*>(.*?)</td>',s,re.S)]
            perf=cells(rpt.split('<a name="performance">',1)[1].split('</table>',1)[0])
            summaries=[]
            for item in rpt.split('Path Summary:')[1:]:
                fields=cells(item.split('</table>',1)[0]);summaries.append(dict(zip(fields[::2],fields[1::2])))
            self.state['results'][name]=dict(synthesis=runtime,resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
                inventory=inventory(syn/'g2_top.vg'),frequency_table=perf,path_summaries=summaries,
                no_timing_path='No timing path found' in rpt,synthesis_is_not_physical_timing=True,effective_options=actual_options)
            assert summaries and 'No timing path found' not in rpt,'context must create real timed launches and captures'
            self.save(name+'_context_synthesis_complete')
        # Physical checks are small and sequential; reserve at least the full2/4GiB before each.
        for name in ('baseline','shared'):
            folder=self.out/name
            if self.resume_physical_review and name=='baseline':
                retry=self.out/'baseline_physical_retry1';retry.mkdir(exist_ok=False)
                (retry/'physical.tcl').write_bytes((folder/'physical.tcl').read_bytes())
                write(retry/'source.json',dict(source_netlist=str(folder/'impl/gwsynthesis/g2_top.vg'),
                      incomplete_previous_run=str(folder/'physical_runtime.json'),synthesis_repeated=False,
                      purpose='complete the local physical comparison after correcting the too-small local routing memory cap'))
                folder=retry
            runtime=self.run(folder,'physical',[str(TOOL),'physical.tcl'],limit=0,physical=True)
            log=(folder/'physical.log').read_text(errors='replace')
            pnr=folder/'impl/pnr';timing=pnr/'g2_top.tr'
            row=dict(runtime=runtime,errors=re.findall(r'^.*ERROR\s*\(.*$',log,re.M),timing_report=str(timing),timing_report_available=timing.exists())
            if timing.exists():
                text=timing.read_text(errors='replace')
                for key,title in [('setup_violated_endpoints','Numbers of Setup Violated Endpoints'),('hold_violated_endpoints','Numbers of Hold Violated Endpoints'),('endpoints_analyzed','Numbers of Endpoints Analyzed'),('paths_analyzed','Numbers of Paths Analyzed')]:
                    match=re.search('<'+title+r'>:\s*(\d+)',text);row[key]=int(match[1]) if match else None
                row['read_path_names_in_report']=[line.strip() for line in text.splitlines() if 'u_rb/' in line or 'rbq1' in line]
            self.state['results'][name]['physical']=row;self.save(name+'_local_physical_complete')
            assert not row['errors'] and timing.exists(),row
        original=self.state['protected_joint_build'];latest=reference_snapshot(Path(original['build']))
        self.state['protected_joint_build_input_metadata_unchanged']=original['input_metadata']==latest['input_metadata']
        self.state['protected_joint_build_latest']=latest
        assert self.state['protected_joint_build_input_metadata_unchanged']
        self.save('local_context_analysis_complete_requires_path_review')


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--prior-proof',type=Path);p.add_argument('--protected-pnr',type=Path)
    p.add_argument('--run-prepared',action='store_true');p.add_argument('--resume-after-option-review',action='store_true')
    p.add_argument('--resume-local-physical-after-budget-review',action='store_true')
    a=p.parse_args();out=a.out.resolve()
    if (a.resume_after_option_review or a.resume_local_physical_after_budget_review) and not a.run_prepared:p.error('resume requires --run-prepared')
    if a.resume_after_option_review and a.resume_local_physical_after_budget_review:p.error('choose one reviewed recovery point')
    if not a.run_prepared:
        assert a.source and a.prior_proof and a.protected_pnr
        prepare(a.source.resolve(),out,a.prior_proof.resolve(),a.protected_pnr.resolve());return
    runner=Runner(out,a.resume_after_option_review,a.resume_local_physical_after_budget_review)
    try:runner.execute()
    except BaseException as exc:
        runner.state['failure']=str(exc);runner.save('local_analysis_failed_requires_review');raise


if __name__=='__main__':main()
