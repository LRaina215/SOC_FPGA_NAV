"""Bounded observation implementation comparison, not joint acceptance."""
import argparse,json,re,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
from measure_slf_row_banks import TOOL,LIBRARY,run_syn,simulate,namespaced,inventory
from prepare_observation_reuse import transform
from preflight import validate
from retry_frozen_synthesis import memory_status

WRAPPER='''module g2_top(input clk,rst_n,capture,input[71:0]gyro,gravity,input signed[15:0]ld,rd,
 input signed[23:0]linear,yaw,height,input[143:0]actions,output valid,initialized,
 output[671:0]current_flat,output[3359:0]history_flat);
rl_observation_v5 #(.LEFT_ENCODER_SIGN(-1),.RIGHT_ENCODER_SIGN(-1)) dut(
 clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,valid,initialized,current_flat,history_flat);
endmodule
'''


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--resume',action='store_true',help='reuse verified RTL results after a host-memory pause; never overwrite completed SYNs')
    p.add_argument('--prefetch',action='store_true',help='prepare operands one existing state edge earlier without moving external outputs')
    p.add_argument('--reuse-baseline',type=Path,help='reuse the exact original module SYN from an earlier measurement; verify source and wrapper bytes')
    p.add_argument('--retry-preflight',action='store_true',help='one fresh settings-only attempt after a preserved timeout; no repeated SYN')
    a=p.parse_args();source=a.source.resolve();out=a.out.resolve()
    if not a.resume:out.mkdir(parents=True,exist_ok=False)
    old=(source/'control/rl_observation_v5.v').read_text(encoding='utf-8');new=transform(old,a.prefetch)
    assert '.LEFT_ENCODER_SIGN(-1),.RIGHT_ENCODER_SIGN(-1)' in (source/'control/mem_policy.v').read_text()
    bench=(Path(__file__).parent/'tb_observation_reuse.sv').read_text()
    bounds=[]
    for gain in (1,2,3,5,13):
        lo,hi=-(2**23)*gain,(2**23-1)*gain
        assert -(2**27)<=lo<=hi<2**27
        bounds.append(dict(gain=gain,min=lo,max=hi,signed28_sufficient=True))
    result=dict(source=str(source),build=str(out),normal_scaling_bound_proof=bounds,operand_prefetch=a.prefetch,
        actual_encoder_signs=[-1,-1],input_capture_timing_unchanged=True,shared_divider64_and_rounding_unchanged=True,
        policy_algorithm_weights125hz_unchanged=True,all_observation_lanes_and_history_retained=True,
        programmed=False,deployment_accepted=False,runs=[],checks=[],
        scope='isolated RTL and vendor primitive equivalence/resource comparison; full SYN/PNR still required')
    if a.resume:
        result=json.loads((out/'result.json').read_text())
        assert result['source']==str(source) and result['checks'][0]['variant']=='rtl'
        assert (out/'rtl/tb.sv').read_text()==bench, 'test bench changed since verified RTL run'
    def save():(out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8',newline='\n')
    for label,text in [('old',old),('new',new)]:
        name='obs_original' if label=='old' else 'obs_candidate'
        code=text.replace('RL_OBSERVATION_V5_V_INCLUDED','OBS_REUSE_'+label.upper()+'_INCLUDED').replace('module rl_observation_v5 #(',f'module {name} #(')
        target=out/(label+'_rtl.v')
        if a.resume:assert target.read_text()==code, 'frozen RTL changed since verified comparison'
        else:target.write_text(code,encoding='utf-8')
    if not a.resume:
        result['checks'].append(simulate(out,'rtl',[out/'old_rtl.v',out/'new_rtl.v'],bench));save()
    for label,text in [('original',old),('reused',new)]:
        if any(r['variant']==label for r in result['runs']):continue
        if label=='original' and a.reuse_baseline:
            cached=a.reuse_baseline.resolve();prior=json.loads((cached/'result.json').read_text())
            assert (cached/'original/observation.v').read_text()==old and (cached/'original/top.v').read_text()==WRAPPER
            row=next(r for r in prior['runs'] if r['variant']=='original')
            row['reused_measurement_from']=str(cached)
            result['runs'].append(row);shutil.copyfile(cached/'original_vendor.v',out/'original_vendor.v');save();continue
        m=memory_status()
        if m.avail_phys<1024**3 or m.avail_page<3*1024**3:
            result.update(phase='rtl_passed_small_syn_not_started_host_memory',pending_variant=label,
                host_memory=dict(free_bytes=m.avail_phys,available_commit_bytes=m.avail_page,
                                 minimum_free_bytes=1024**3,minimum_commit_bytes=3*1024**3),
                module_resource_result_available=False)
            save();print(json.dumps(dict(phase=result['phase'],host_memory=result['host_memory'])),flush=True);return 2
        folder=out/label;folder.mkdir(exist_ok=True)
        assert not (folder/'impl/gwsynthesis/g2_top.vg').exists(), 'unexpected prior SYN artifact; inspect instead of rerunning'
        (folder/'observation.v').write_text(text,encoding='utf-8');(folder/'top.v').write_text(WRAPPER)
        (folder/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
        settings += [f'add_file -type verilog {{{(folder/n).as_posix()}}}' for n in ('observation.v','top.v')]
        settings += [f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}',
            'set_option -top_module g2_top','set_option -output_base_name g2_top',
            'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0','set_option -looplimit 100000']
        settings_folder=folder
        if a.retry_preflight:
            assert a.resume and (folder/'effective_options.tcl').exists(), 'only a saved settings-stage timeout may be retried'
            settings_folder=folder/'settings_retry1';settings_folder.mkdir(exist_ok=False)
            result['settings_only_retry']=dict(first_log=str(folder/'preflight.log'),first_readback=str(folder/'effective_options.tcl'),
                new_settings_directory=str(settings_folder),maximum_retry_count=1,source_and_synthesis_inputs_unchanged=True)
            save()
        effective=validate(TOOL,settings_folder,settings,50,0)
        (folder/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
        try:runtime=run_syn(folder,'isolated observation exact shared scaling only')
        except AssertionError as error:
            if str(error)!='small SYN host reserve':raise
            m=memory_status();result.update(phase='rtl_passed_small_syn_not_started_host_memory',pending_variant=label,
                host_memory=dict(free_bytes=m.avail_phys,available_commit_bytes=m.avail_page,
                                 minimum_free_bytes=1024**3,minimum_commit_bytes=3*1024**3),module_resource_result_available=False)
            save();print(json.dumps(dict(phase=result['phase'],host_memory=result['host_memory'])),flush=True);return 2
        syn=folder/'impl/gwsynthesis'
        report=(syn/'g2_top_syn.rpt.html').read_text()
        clock=re.search(r'<td>clk</td>\s*<td>([\d.]+)\(MHz\)</td>\s*<td>([\d.]+)\(MHz\)</td>',report)
        row=dict(variant=label,effective_options=effective,runtime=runtime,
                 module_synthesis_estimated_fmax_mhz=float(clock[2]) if clock else None,
                 resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,inventory=inventory(syn/'g2_top.vg'))
        result['runs'].append(row);save();print(json.dumps(dict(variant=label,resources=row['resources'])),flush=True)
        (out/(label+'_vendor.v')).write_text(namespaced((syn/'g2_top.vg').read_text(encoding='utf-8'),'old' if label=='original' else 'new'),encoding='utf-8')
    before,after=[r['inventory'] for r in result['runs']]
    result.update(bsram_unchanged=before['bsram']==after['bsram']==0,
                  module_resource_result_available=True,register_delta=after['ff']-before['ff'],full_joint_result_available=False)
    for row in result['runs']:
        measurement=Path(row.get('reused_measurement_from',str(out)))/row['variant']
        report=(measurement/'impl/gwsynthesis/g2_top_syn.rpt.html').read_text()
        clock=re.search(r'<td>clk</td>\s*<td>([\d.]+)\(MHz\)</td>\s*<td>([\d.]+)\(MHz\)</td>',report)
        row['module_synthesis_estimated_fmax_mhz']=float(clock[2]) if clock else None
    # The full pair of mapped implementations exceeds xsim BASIC's instance
    # limit because each vendor DSP model expands internally. Keep any failed
    # attempt and compare the mapped candidate against the unchanged RTL oracle
    # instead. If that also exceeds the installed license, retain the failure
    # and report the verification gap rather than claiming a simulation pass.
    failed=out/'vendor/xsim.log'
    if failed.exists() and 'exceed maximum allowed 50000' in failed.read_text(errors='replace'):
        result['paired_vendor_attempt']=dict(passed=False,reason='simulator instance license limit',
            log=str(failed),does_not_indicate_functional_mismatch=True)
    vendor_bench=('`define OBS_REUSE_VENDOR\n'+bench).replace(
        ' g2_old a(', ' obs_original #(.LEFT_ENCODER_SIGN(-1),.RIGHT_ENCODER_SIGN(-1)) a(')
    variant='vendor_candidate_against_rtl'
    if not any(c['variant']==variant for c in result['checks']):
        log=out/variant/'xsim.log'
        if not log.exists():
            try:result['checks'].append(simulate(out,variant,[LIBRARY,out/'old_rtl.v',out/'reused_vendor.v'],vendor_bench))
            except AssertionError:
                if not log.exists() or 'exceed maximum allowed 50000' not in log.read_text(errors='replace'):raise
        if not any(c['variant']==variant for c in result['checks']):
            content=log.read_text(errors='replace')
            assert 'exceed maximum allowed 50000' in content, 'inspect existing simulator failure; do not rerun or mask it'
            count=re.search(r'Number of instances in this design, (\d+)',content)
            result.update(phase='rtl_and_synthesis_complete_vendor_sim_unavailable',vendor_verification_available=False,
                vendor_simulator_block=dict(reason='installed simulator instance license limit',maximum_instances=50000,
                    reported_instances=int(count[1]) if count else None,log=str(log),functional_mismatch_not_observed=True),
                ready_for_joint_resource_measurement_only=True)
            save();print(json.dumps(dict(phase=result['phase'],candidate=result['runs'][-1]['resources'],
                synthesis_estimate=result['runs'][-1]['module_synthesis_estimated_fmax_mhz'])),flush=True);return 3
    result['vendor_reference']='unchanged frozen original observation RTL, actual encoder signs -1/-1'
    result['vendor_candidate']='actual synthesized candidate vendor primitives'
    save()
    result.update(phase='module_comparison_complete',vendor_verification_available=True)
    save();print(json.dumps({k:v for k,v in result.items() if k!='runs'}),flush=True)


if __name__=='__main__':raise SystemExit(main())
