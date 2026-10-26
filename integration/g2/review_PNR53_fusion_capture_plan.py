"""Read the existing fusion paths and propose one private-capture direction.

No RTL generation option, mathematical test, EDA, or queued build mutation.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json
from prepare_baseline_fusion_retime import metadata,save
from analyze_pnr53_failed_endpoints import parse_path
from plan_PNR53_PID_unity_bypass import path_cells
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
HW=BASE/'SLF128_storage_safeFW_SYN43_prepared1';PNR=BASE/'SLF128_storage_place1_pnr53'

def review(out):
    assert not out.exists();out.mkdir()
    source=HW/'control/imu_fusion.v';text=source.read_text()
    ancestors=[BASE/'baseline_fusion50_ring_MAX_prepared1/control/imu_fusion.v',BASE/'baseline_fusion_only_prepared1/control/imu_fusion.v']
    for p in ancestors:assert source.read_bytes()==p.read_bytes(),str(p)
    norm_anchors=['reg [63:0] norm_square_x=0,norm_square_y=0,norm_square_z=0;',
        'wire [63:0] sqrt_value=norm_square_x+norm_square_y+norm_square_z;',
        'norm_square_x<={32\'d0, abs16(accel_x)} * abs16(accel_x);',
        'norm_square_x<=abs24(egx) * abs24(egx);']
    for a in norm_anchors:assert a in text,a
    cls=BASE/'PNR53_all_failed_endpoints_classification1'
    r=json.loads((cls/'result.json').read_text());before=metadata(PNR/'impl/pnr/g2_top.tr')
    assert before in r['input_bindings'],'original TR changed'
    group=next(x for x in r['classes']if x['category']=='control_fusion')
    assert group['negative_unique_endpoints']==107 and group['worst_path_number']==387
    raw=(cls/'worst_class_path387.txt').read_text();path=parse_path(raw)
    previous=json.loads((cls/'representative_path_details.json').read_text())['387'];assert path==previous
    failures=[x for x in json.loads((cls/'all_failed_endpoint_classes.json').read_text())if x['category']=='control_fusion']
    wanted={x['node'].rsplit('/',1)[0]for x in path['actual_data_rows']}
    vo=PNR/'impl/pnr/g2_top.vo';cells=path_cells(vo,wanted)
    dsp=cells['u_control/model_thread/fusion/mult_3469_s1']
    assert dsp['ports']['CE'].endswith('egx_23_12 }')
    for p in('AREG_CLK="CLK0"','AREG_CE="CE0"','OREG_CLK="BYPASS"'):
        assert p in dsp['parameters'],p
    refs=[dict(line=n,text=line.strip())for n,line in enumerate(text.splitlines(),1)if any(x in line for x in('egx','egy','egz','5\'d6:','5\'d7:','5\'d8:','5\'d9:','5\'d12:','5\'d13:','integration_ticks<=','valid <= 1;'))]
    (out/'actual_TR_path387.txt').write_text(raw,encoding='utf8');save(out/'actual_path_primitives.json',cells)
    pin_counts=dict(Counter(x['to_node'].rsplit('/',1)[1]for x in failures));kind_counts=dict(
        CE=sum(x['to_node'].endswith('/CE')for x in failures),DSP_CE=sum('/CE['in x['to_node']for x in failures),
        DSP_A=sum('/A['in x['to_node']for x in failures),D=sum(x['to_node'].endswith('/D')for x in failures))
    assert kind_counts==dict(CE=65,DSP_CE=2,DSP_A=27,D=13),kind_counts
    result=dict(at=datetime.now().astimezone().isoformat(),phase='read-only fusion review, one next-direction plan',
        source_bindings=[metadata(source),*[metadata(p)for p in ancestors],metadata(cls/'result.json'),metadata(cls/'representative_path_details.json'),before,metadata(vo)],
        current_norm_pipeline=dict(already_inherited_from_SYN23_SYN40=True,source_byte_identical_to_both_ancestors=True,
            current_registers='three64bit norm_square registers feed the already retimed sqrt start path; state4/state9 write them',
            unchanged_literals=norm_anchors,not_readding_old_square2_or_other_failed_stack=True),
        all_fusion_failures=dict(negative_unique_endpoints=107,pin_counts=kind_counts,raw_pin_counts=pin_counts,
            worst_slack_ns=-5.936,worst_path_number=387,not_all107_the_same_issue=True),
        actual_worst=dict(source='sensor gyro_y bit3 FF',endpoint='fusion MULT27X36 mult_3469_s1 CE[0]',
            logic_levels=24,data_delay_ns=25.610,gate_ns=10.169,route_ns=15.111,C2Q_ns=0.331,
            single_final_route_ns=8.058,final_driver_fanout=74,
            final_driver_location='R42C162[2][A]',DSP_location='DSP_R55[23]',
            source_to_enable_chain='raw gyro abs16 -> two gyro magnitude sum carry chains -> state6 trust/rejected selection -> eg* capture enable -> DSP AREG CE',
            note='Endpoint is an operand-register enable, not the old square-sum datapath or an inference multiplication delay',
            AREG_actual='CLK0/CE0, operand capture was absorbed into hardDSP; BREG has its separate existing CE. No source stage change is inferred from a name alone',
            actual_DSP_ports=dsp['ports'],actual_DSP_parameters=dsp['parameters']),
        original_capture_and_use=dict(source_reference_lines=refs,
            state6='accepted accel leaves eg* holding and starts recip; rejected accel writes eg*=n1/n2/n3 then goes state9',
            state7='accepted path waits recip_done, writes nx/ny/nz; no source consumer of eg*',
            state8='accepted path unconditionally writes blended eg* then goes state9',
            consumers='eg* reads only at state9 norm_square capture and state12 gravity/gx/gy/gz publish',
            final_valid='state13 pulses valid and accel_rejected, unchanged',
            measured_dt='sample_clock/integration_ticks captures raw sample_valid independently of FSM; cannot silently resample or retime this',
            earlier_precompute_not_selected='moving gyro trust sampling earlier would change behavior if raw gyro updates during a busy interval; retain original state6 decision exactly'),
        only_next_structure_direction=dict(name='unconditional provisional eg* capture in state6',
            change='in state6 write egx=n1,egy=n2,egz=n3 for both accepted and rejected paths; retain original trust predicate, recip_d/start, rejected_sample and st transitions verbatim',
            accepted_lifetime='new provisional eg* differs privately only during state7 wait; original state8 blend overwrites it before every state9/state12 consumer',
            rejected_lifetime='same original n* values captured on the same state6 edge',
            expected_CE='state6 OR state8 capture, independent of raw gyro abs/sum trust predicate',
            expected_resources='same three existing24bit eg* registers, no new clock/state/public latency; actual FF/DSP mapping not measured',
            no_RTL_written=True),
        exact_proof_obligations=['list all actual source consumers and prove no eg* use in accepted state7 interval; explicitly permit private-state difference only there',
            'public outputs/valid/accel_rejected/calibrating/bias_suspect and all sqrt/recip/div start/done/return cycles must compare exactly',
            'accepted/rejected transition, raw publication during each busy stage, actual state6 gyro decision unchanged, reset in state6/7/8/9 and reciprocal waits',
            'retain full signed24 arithmetic and all norms/trust thresholds; existing arithmetic proofs can bind unchanged expressions, not replace new capture-lifetime comparison',
            'no forced calibration, unknown output rejection, production50MHz and BIAS_SAMPLES200/100Hz cases if a full dynamic run is selected'],
        required_actual_mapping_checks=['actual DSP/FF captureCE must lose dependency on raw gyro trust sum; if optimizer reconstructs it, the mapping objective is not met',
            'check whether the27 negative DSP_A inputs and13 D/state/divider endpoints remain; this CE change does not promise they disappear',
            'measure resulting CE fanout/route/arrival and all107 negative fusion endpoints, plus complete new design timing categories',
            'unchanged clocks/INIT/4500/125Hz/control/reset/stop and no new timing exception'],
        priority='reasonable small isolated backup after53/54 comparison; not a claim to close overall50MHz or justification to alter the running layout',
        mathematical_or_RTL_test_started=False,RTL_generated_or_modified=False,EDA_started=False,current54_or_prepared_queue_modified=False,deployment_ready=False)
    save(out/'result.json',result);save(REPO/'docs/4_design/evidence/g2_20261023/PNR53_fusion_capture_readonly_plan_20261026.json',result)
    print(json.dumps(dict(output=str(out),fusion_failed=107,worst_slack=-5.936,only_direction='state6 provisional capture',RTL=False,EDA=False)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'PNR53_fusion_capture_readonly_plan1');review(p.parse_args().out)
