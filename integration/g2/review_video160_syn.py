"""Review exact frozen memories, original26 and new video synchronizer geometry.

No full-G2 equivalence or physical acceptance is inferred. New video crossings
remain fully timed for the initial PNR; no blanket timing exceptions are emitted.
"""
from collections import Counter,defaultdict
from datetime import datetime
from pathlib import Path
import argparse,json,re
from audit_netlist_memories import inventory,MODULE,PARAM
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions,revalidate
from audit_routing_topology import primitive_directions
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023/K_timing_syn50')
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')

def load(p):return json.loads(p.read_text(encoding='utf-8'))
def memories(b):
    v=inventory(b);params={}
    for mod,body in MODULE.findall((b/'impl/gwsynthesis/g2_top.vg').read_text(encoding='utf-8')):
        for inst,key,value in PARAM.findall(body):params.setdefault((mod,inst.strip()),{})[key]=value.strip()
    for x in v['leaves']:x['all_parameters']=params.get((x['module_type'],x['path'].rsplit('/',1)[-1]),{})
    return v

def review(build,out):
    assert not out.exists()
    j=load(build/'job.json');assert j['phase']=='synthesis_complete_actual_netlist_review_pending'and j['tool_exit']==0
    log=(build/'gw_sh.log').read_text(encoding='utf-8')
    assert not re.search(r'ERROR\s*\(|WARN\s*\(EX3988\)',log)
    manifest=load(build/'manifest.json');assert manifest['missing_literal_memory_files']==[]
    assets=manifest['original_asset_byte_comparisons_passed']
    assert len(assets)==100 and sum(a.startswith('src/mem/weight9_')for a in assets)==35
    assert all((build/a).read_bytes()==(BASE/a).read_bytes()for a in assets)
    old,new=memories(BASE),memories(build);rows=[]
    for prefix in ('u_soc','u_nav','u_control'):
        def c(v):return Counter((x['primitive'],tuple(sorted(x['all_parameters'].items())))
            for x in v['leaves']if '/'+prefix+'/'in x['path'])
        a,b=c(old),c(new);assert a==b,('RAM parameters/INIT',prefix)
        rows.append(dict(module=prefix,original=sum(a.values()),new=sum(b.values()),all_parameters_and_INIT_multiset_equal=True))
    image=[x for x in new['leaves']if '/u_video/'in x['path']]
    assert len(image)==38 and all(x['primitive']=='SDPB'and x['parameters']['BIT_WIDTH_0']==x['parameters']['BIT_WIDTH_1']=='16'for x in image)
    assert len(new['leaves'])==307 and new['wide32_36_count']==11
    m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3
    directions=primitive_directions(LIB)
    a=read(BASE/'impl/gwsynthesis/g2_top.vg');c=read(build/'impl/gwsynthesis/g2_top.vg')
    loads=defaultdict(list);drivers=defaultdict(list)
    for n,v in c.items():
        for pin,nets in v['ports'].items():
            for net in nets:
                if directions[v['kind']][pin]=='input':loads[net].append((n,pin))
                elif directions[v['kind']][pin]=='output':drivers[net].append((n,pin))
    sdc=(build/'g2.sdc').read_text(encoding='utf-8');cdc=[]
    for src,first,budget in re.findall(r'set_max_delay -from \[get_pins \{(.+)/Q\}\] -to \[get_pins \{(.+)/D\}\] ([\d.]+)',sdc):
        s,f=c[src],c[first];assert s['kind'].startswith('DFF')and f['kind'].startswith('DFF')
        assert s['ports']['Q']==f['ports']['D']
        fan=loads[f['ports']['Q'][0]];assert len(fan)==1 and fan[0][1]=='D'
        second=c[fan[0][0]];assert second['kind'].startswith('DFF')
        cdc.append(dict(source_register=src,first_register=first,second_register=second['name'],
            source_clock=s['ports']['CLK'][0],target_clock=f['ports']['CLK'][0],route_max_delay_ns=float(budget)))
    assert len(cdc)==26
    original=revalidate(a,c,dict(passed=True,paths=cdc),directions,
        definitions(BASE/'impl/gwsynthesis/g2_top.vg'),definitions(build/'impl/gwsynthesis/g2_top.vg'),
        reset_pins=('RESET','CLEAR','PRESET'))
    video=[]
    for prefix,fname,sname in [('','request_meta','request_sync'),('','stale_meta','stale_sync'),
        ('capture/','ready_sync_0','ready_sync_1'),('capture/','ack_sync_0','ack_sync_1'),
        ('store/','req_meta','req_sync'),('hud/','req_meta','req_sync'),('hud/','ack_meta','ack_sync')]:
        fs=[v for n,v in c.items()if n.startswith('u_video/'+prefix+fname+'_s')and v['kind'].startswith('DFF')]
        assert len(fs)==1;f=fs[0];ds=drivers[f['ports']['D'][0]];assert len(ds)==1 and ds[0][1]=='Q'
        src=c[ds[0][0]];assert src['kind'].startswith('DFF')
        fan=loads[f['ports']['Q'][0]];assert len(fan)==1 and fan[0][1]=='D';sec=c[fan[0][0]]
        assert sec['name'].startswith('u_video/'+prefix+sname+'_s')
        assert f['ports']['CLK']==sec['ports']['CLK']and f['ports'].get('CE')==sec['ports'].get('CE')==['VCC']
        video.append(dict(source=src['name'],first=f['name'],second=sec['name'],
            source_clock=src['ports']['CLK'][0],target_clock=f['ports']['CLK'][0],first_only_drives_second_D=True))
    pll=c['u_video_pll/u_pll/PLL_inst'];assert pll['kind']=='PLL'
    assert pll['ports']['CLKIN']==['clk50_d']and pll['ports']['CLKOUT1']==['video_clock']
    clocks=Counter(v['ports']['CLK'][0]for v in c.values()if v['kind'].startswith('DFF'))
    roots={'clk50_d','rgmii_rxc_d','CAMERA_PCLK_d','video_clock'};inverted={}
    for net,count in clocks.items():
        if net in roots:continue
        ds=drivers[net];assert len(ds)==1 and ds[0][1]=='O'
        inv=c[ds[0][0]];assert inv['kind']=='INV'and inv['ports']['I'][0]in roots
        old_inv=a.get(inv['name']);assert old_inv==inv,'additional inverted clock was not in the original design'
        inverted[net]=dict(source=inv['ports']['I'][0],driver=inv['name'],loads=count,unchanged_from_baseline=True)
    assert roots<=set(clocks)
    result=dict(passed_for_initial_exploratory_PNR=True,recorded_at=datetime.now().astimezone().isoformat(),
        build=str(build),asset_count=100,all35_packed_weights_and_CPU_assets_byte_equal=True,
        memories=rows,image_RAMS=38,image_width=16,source_BSRAM=307,physical_split_prediction=318,
        original26=original,video7=video,clock_FF_loads=dict(clocks),original_inverted_clock_branches=inverted,new_video_exceptions_added=False,
        scope='Actual storage/INIT and synchronizer connectivity only. New video crossings stay fully timed; complete mapped datapath equivalence, routed STA/CDC/reset/pulse/IO and board signoff pending.',programmed=False)
    out.write_text(json.dumps(result,indent=2),encoding='utf-8');return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();r=review(a.build,a.out);print(json.dumps({k:r[k]for k in ('passed_for_initial_exploratory_PNR','source_BSRAM','physical_split_prediction','clock_FF_loads')}))
