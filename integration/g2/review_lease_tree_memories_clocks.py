"""Read actual explicitMAX SYN37 versus SYN34; writes only this new evidence directory."""
from pathlib import Path
import sys,json,re,traceback,collections,datetime,argparse
sys.dont_write_bytecode=True
sys.path.insert(0,'D:/Desktop/SOC FPGA/nav/integration/g2')
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions,revalidate
from review_small_block_stores import parameter_reader
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from review_model25_joint_syn import same,meta
from review_baseline_single_factor_syn import cpu_image

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'baseline_fusion_model25_NAV50_syn34'
_p=argparse.ArgumentParser();_p.add_argument('--new',type=Path,required=True);_p.add_argument('--out',type=Path,required=True);_a=_p.parse_args()
NEW=_a.new;OUT=_a.out;OUT.mkdir(parents=True,exist_ok=False)
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
REF=BASE/'model25_SYN34_actual_review1/actual_review4/actual_original_radar26.json'
def load(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(n,v):
    p=OUT/('v2_'+n)
    assert not p.exists(),('evidence already exists',str(p))
    p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def progress(s):
    print(s,flush=True)
state=dict(passed=False,scope='Actual complete SYN37 versus SYN34 memory/INIT/three-clock/original radar26 structural inspection; no synthesis, PNR, simulation, physical timing or deployment acceptance',recorded_at=datetime.datetime.now().astimezone().isoformat(),old=str(OLD),new=str(NEW))
try:
    job=load(NEW/'job.json')
    assert job['phase']=='synthesis_complete' and job['tool_exit']==0
    watched=[OLD/'impl/gwsynthesis/g2_top.vg',NEW/'impl/gwsynthesis/g2_top.vg',OLD/'g2.sdc',NEW/'g2.sdc',OLD/'g2.cst',NEW/'g2.cst',REF,NEW/'job.json']
    binding=[meta(p) for p in watched]
    def hardware(root):
        rows=[Path(p).relative_to(root).as_posix() for p in load(root/'manifest.json')['source_files']]
        assert len(rows)==len(set(rows))
        return set(rows)
    names=hardware(OLD)
    assert names==hardware(NEW)
    changed=[n for n in sorted(names) if not same(OLD/n,NEW/n)]
    assert changed==['command_lease_guard.v'],changed
    source_binding=dict(hardware_inputs=len(names),changed_inputs=changed,identical_input_count=len(names)-1,CST_identical=same(OLD/'g2.cst',NEW/'g2.cst'),SDC_identical=same(OLD/'g2.sdc',NEW/'g2.sdc'))
    assert source_binding['CST_identical'] and source_binding['SDC_identical']
    progress('parse_actual_modules_and_connectivity')
    oldvg=OLD/'impl/gwsynthesis/g2_top.vg';newvg=NEW/'impl/gwsynthesis/g2_top.vg'
    oldmods,newmods=definitions(oldvg),definitions(newvg)
    module_delta=dict(old_count=len(oldmods),new_count=len(newmods),removed=sorted(oldmods.keys()-newmods.keys()),added=sorted(newmods.keys()-oldmods.keys()),changed=sorted(k for k in oldmods.keys()&newmods.keys() if oldmods[k]!=newmods[k]))
    save('actual_module_text_delta.json',module_delta)
    before=read(oldvg)
    try:after=read(newvg)
    except Exception as e:
        save('initial_new_connectivity_reader_failure.json',dict(error=str(e),traceback=traceback.format_exc()));raise
    oldparams,newparams=parameter_reader(oldmods),parameter_reader(newmods)
    directions=primitive_directions(LIB)
    def index(cells):
        drivers=collections.defaultdict(list);loads=collections.defaultdict(list)
        for n,c in cells.items():
            assert c['kind'] in directions,('unknown kind',n,c['kind'])
            for pin,nets in c['ports'].items():
                assert pin in directions[c['kind']],('unknown pin',n,pin)
                for i,net in enumerate(nets):
                    if net in ('GND','VCC'):continue
                    if directions[c['kind']][pin]=='output':drivers[net].append((n,pin,i))
                    if directions[c['kind']][pin] in ('input','inout'):loads[net].append((n,pin,i))
        return drivers,loads
    olddrivers,oldloads=index(before);drivers,loads=index(after)
    def clock_reader(cells,drivers,params):
        cache={}
        def clock(net,seen=()):
            if net in ('GND','VCC'):return net,0
            if net in cache:return cache[net]
            assert net not in seen and len(drivers[net])==1,('clock driver',net,drivers[net],seen)
            n,p,i=drivers[net][0];c=cells[n];k=c['kind']
            if k=='IBUF':
                assert p=='O' and i==0 and c['ports']['I'] in (['clk50'],['rgmii_rxc'])
                result=(c['ports']['I'][0],0)
            elif k=='PLL':
                assert n=='u_model_pll/PLL_inst' and p=='CLKOUT0' and i==0,(net,n,p)
                assert clock(c['ports']['CLKIN'][0],seen+(net,))==('clk50',0)
                result=('model25',0)
            else:
                assert k in ('INV','BUF','BUFG'),('unexpected clock driver',net,c)
                root,parity=clock(c['ports']['I'][0],seen+(net,))
                result=(root,parity^int(k=='INV'))
            cache[net]=result
            return result
        return clock
    oldclock=clock_reader(before,olddrivers,oldparams);clock=clock_reader(after,drivers,newparams)
    def clock_inventory(cells,drivers,clock,params):
        pins={};nets={};counts=collections.Counter()
        for n,c in cells.items():
            for p,values in c['ports'].items():
                if re.fullmatch(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK',p):
                    for i,net in enumerate(values):
                        if net in ('GND','VCC'):continue
                        root=clock(net);pins[(n,p,i)]=root;counts[root]+=1
                        nets[net]=dict(root=root[0],inverted=bool(root[1]),actual_unique_driver=drivers[net],driver_cell=cells[drivers[net][0][0]])
        return pins,nets,[dict(root=r,inverted=bool(p),pins=n)for(r,p),n in sorted(counts.items())]
    opins,onets,ocounts=clock_inventory(before,olddrivers,oldclock,oldparams)
    npins,nnets,ncounts=clock_inventory(after,drivers,clock,newparams)
    common_clock_pins=opins.keys()&npins.keys()
    assert all(opins[k]==npins[k]for k in common_clock_pins),('common actual instance clock-root/phase changed',[(k,opins[k],npins[k])for k in common_clock_pins if opins[k]!=npins[k]])
    clock_mapping=dict(common_instance_pin_bindings=len(common_clock_pins),all_common_instance_pin_roots_and_phase_equal=True,removed_instance_pins=[dict(instance=k[0],pin=k[1],index=k[2],root=opins[k])for k in sorted(opins.keys()-npins.keys())],added_instance_pins=[dict(instance=k[0],pin=k[1],index=k[2],root=npins[k])for k in sorted(npins.keys()-opins.keys())],all_old_and_new_clock_pins_individually_traced_to_real_driver=True,instance_name_changes_do_not_imply_clock_changes=True)
    save('actual_clock_instance_name_changes.json',clock_mapping)
    assert {x['root']for x in ncounts}=={'clk50','model25','rgmii_rxc'}
    oplls={n:c for n,c in before.items()if c['kind']=='PLL'};nplls={n:c for n,c in after.items()if c['kind']=='PLL'}
    assert set(oplls)==set(nplls)=={'u_model_pll/PLL_inst'}
    plls=[]
    for n,c in nplls.items():
        a=oplls[n];pa,pb=oldparams(a),newparams(c)
        assert a==c and pa==pb,('PLL actual ports/parameters changed',n)
        expected=dict(FCLKIN='"50"',IDIV_SEL='1',FBDIV_SEL='1',MDIV_SEL='16',ODIV0_SEL='32',MDIV_FRAC_SEL='0',ODIV0_FRAC_SEL='0',CLKOUT0_EN='"TRUE"',CLKFB_SEL='"INTERNAL"')
        assert all(pb.get(k)==v for k,v in expected.items()),pb
        assert all(pb[f'CLKOUT{i}_EN']=='"FALSE"' for i in range(1,7))
        assert all(c['ports'][p]==['GND']for p in ('PLLPWD','RESET_I','RESET_O','CLKFB'))
        assert c['ports']['ENCLK0']==['VCC'] and oldclock(a['ports']['CLKIN'][0])==clock(c['ports']['CLKIN'][0])==('clk50',0)
        plls.append(dict(actual_cell=c,all_parameters=pb,all_ports_and_parameters_identical=True,input_mhz=50,VCO_mhz=800,CLKOUT0_mhz=25))
    save('actual_three_clocks_PLL.json',dict(passed=True,old_clock_nets=onets,new_clock_nets=nnets,old_clock_pin_counts=ocounts,new_clock_pin_counts=ncounts,all_common_actual_clock_pin_instance_root_and_phase_equal=True,all_old_and_new_clock_pins_individually_traced=True,clock_instance_mapping=clock_mapping,clock_pin_count=len(npins),PLLs=plls,CST_and_SDC_identical=True,clock_route_and_timing_signoff=False))
    progress('audit_every_actual_memory_and_CPU_INIT')
    def memories(cells,params):
        return {n:dict(name=n,kind=c['kind'],ports=c['ports'],parameters=params(c))for n,c in cells.items()if c['kind']in BSRAM or c['kind'].startswith(('RAM16','ROM16'))}
    oldmem,newmem=memories(before,oldparams),memories(after,newparams)
    assert oldmem.keys()==newmem.keys(),('memory instance set',sorted(oldmem.keys()-newmem.keys()),sorted(newmem.keys()-oldmem.keys()))
    memdiff=[]
    for n,a in oldmem.items():
        b=newmem[n]
        assert a['kind']==b['kind'] and a['parameters']==b['parameters'],('memory all parameters/INIT',n)
        assert a['ports'].keys()==b['ports'].keys()
        for pin in a['ports']:
            assert len(a['ports'][pin])==len(b['ports'][pin])
            if 'CLK'in pin:assert [oldclock(v)for v in a['ports'][pin]]==[clock(v)for v in b['ports'][pin]],('memory clock',n,pin)
        if a['ports']!=b['ports']:
            memdiff.append(dict(name=n,changed_ports={p:dict(old=a['ports'][p],new=b['ports'][p])for p in a['ports']if a['ports'][p]!=b['ports'][p]}))
    bsram={n:r for n,r in newmem.items()if r['kind']in BSRAM}
    wide=[n for n,r in bsram.items()if any(r['parameters'].get(k)in('32','36')for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1'))]
    assert len(bsram)==314 and len(wide)==18
    save('actual_all_memory_binding.json',dict(passed=True,old_memory=oldmem,new_memory=newmem,all_primitive_kinds_all_explicit_parameters_and_INIT_identical=True,effective_vendor_defaults_identical_by_same_primitive_and_identical_overrides=True,memory_cells=len(newmem),BSRAM=len(bsram),wide_leaves=len(wide),physical_split_prediction_only=332,port_name_differences=memdiff,all_port_logic_equivalence_claimed=not bool(memdiff)))
    libtext=LIB.read_text(encoding='utf8')
    spbody=re.search(r'\bmodule\s+SP\b(.*?)\bendmodule',libtext,re.S)[1]
    defaults=dict(re.findall(r'\bparameter\s+(INIT_RAM_\w+)\s*=\s*([^;]+);',spbody))
    assert len(defaults)==64 and all(int(re.fullmatch(r"256'h([0-9a-fA-F]+)",v.strip())[1],16)==0 for v in defaults.values())
    oi,ni=cpu_image(oldmem,defaults),cpu_image(newmem,defaults)
    binary=(NEW/'firmware/nav_fw.bin').read_bytes()
    oldbin=(OLD/'firmware/nav_fw.bin').read_bytes()
    words=b''.join(int(x,16).to_bytes(4,'little')for x in(NEW/'ram_w.hex').read_text().split())
    assert len(binary)==51712 and oldbin==binary
    assert ni==oi==words==binary+bytes(131072-len(binary))
    for lane in range(4):
        assert same(OLD/f'ram_b{lane}.hex',NEW/f'ram_b{lane}.hex')
        assert bytes(int(x,16)for x in(NEW/f'ram_b{lane}.hex').read_text().split())==ni[lane::4]
    assert same(OLD/'ram_w.hex',NEW/'ram_w.hex')
    save('actual_CPU_INIT_binding.json',dict(passed=True,CPU_SP_bitplanes=64,INIT_pages_examined=4096,reconstructed_bytes=len(ni),original_FW_BIN_bytes=len(binary),old_and_new_images_byte_identical=True,actual_BIN_wordHEX_and_four_laneHEX_bound=True,zero_padding_bytes=131072-len(binary),vendor_SP_default_INIT_pages=64,changed_byte_offsets=[]))
    progress('revalidate_original_radar26_using_existing_strict_checker')
    reference=load(REF)
    assert reference['passed'] and len(reference['paths'])==26
    strict=None;strict_failure=None
    try:
        strict=revalidate(before,after,reference,directions,oldmods,newmods)
        save('initial_strict_radar26_revalidation.json',strict)
    except Exception as e:
        strict_failure=dict(passed=False,error=str(e),traceback=traceback.format_exc())
        save('initial_strict_radar26_failure.json',strict_failure)
    cdc_differences=[]
    for row in reference['paths']:
        for role in ('source_register','first_register','second_register'):
            name=row[role];a,b=before.get(name),after.get(name)
            assert a and b,('reference endpoint absent',name)
            if a!=b or oldparams(a)!=newparams(b):
                cdc_differences.append(dict(role=role,name=name,old=a,new=b,parameters_equal=oldparams(a)==newparams(b),old_parameters=oldparams(a),new_parameters=newparams(b)))
    save('actual_radar26_endpoint_differences.json',dict(endpoint_differences=cdc_differences,strict_checker_passed=strict is not None))
    changed_ff=[]
    for n in before.keys()&after.keys():
        a,b=before[n],after[n]
        if a['kind'].startswith('DFF') and b['kind'].startswith('DFF') and a['ports'].get('Q')!=b['ports'].get('Q'):
            changed_ff.append(dict(name=n,old=a,new=b,kind_and_INIT_equal=a['kind']==b['kind']and oldparams(a)==newparams(b),all_non_Q_pins_identical={p:v for p,v in a['ports'].items()if p!='Q'}=={p:v for p,v in b['ports'].items()if p!='Q'}))
    save('actual_changed_register_Q_names.json',dict(changed_register_Q_ports=changed_ff,no_suffix_substitution_used=True))
    unchanged=all(meta(Path(row['path']))==row for row in binding)
    assert unchanged
    state.update(passed=strict is not None and not memdiff,phase='initial_inspection_complete',source_binding=source_binding,module_text_changes=len(module_delta['changed']),actual_cell_count_old=len(before),actual_cell_count_new=len(after),memory_cells=len(newmem),BSRAM=len(bsram),wide_leaves=len(wide),predicted_physical_split_BSRAM=332,all_memory_parameters_INIT_equal=True,memory_port_name_changed_cells=len(memdiff),CPU_reconstructed_bytes=131072,original_FW_BIN_bytes=51712,true_clock_roots=['clk50','model25','rgmii_rxc'],clock_pin_count=len(npins),all_common_clock_pin_roots_phase_equal=True,all_old_and_new_clock_pins_individually_traced=True,PLL_all_ports_parameters_identical=True,original_radar26_strict_checker_passed=strict is not None,original_radar26_changed_endpoint_cells=len(cdc_differences),changed_Q_port_cells=len(changed_ff),source_netlist_reference_metadata_unchanged_at_end=unchanged,metadata_bindings=binding)
    save('initial_result.json',state)
    print(json.dumps(state,ensure_ascii=False),flush=True)
except Exception as e:
    state.update(error=str(e),traceback=traceback.format_exc())
    save('inspection_failed.json',state)
    raise

from prepare_no_sfp_clock_plan import construct_shared_logic
from prove_soc_console_remap import z3
def load(n):return json.loads((OUT/n).read_text(encoding='utf8'))
def save(n,v):
    p=OUT/n
    assert not p.exists(),str(p)
    p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
try:
    initial=load('v2_initial_result.json')
    assert initial['original_radar26_strict_checker_passed'] and initial['original_radar26_changed_endpoint_cells']==0,('radar26 strict or endpoint review did not pass',initial)
    assert initial['all_memory_parameters_INIT_equal'] and initial['PLL_all_ports_parameters_identical'] and initial['all_common_clock_pin_roots_phase_equal']
    frozen=initial['metadata_bindings']
    assert all(meta(Path(r['path']))==r for r in frozen)
    memory=load('v2_actual_all_memory_binding.json')
    before=read(OLD/'impl/gwsynthesis/g2_top.vg');after=read(NEW/'impl/gwsynthesis/g2_top.vg')
    oldparams=parameter_reader(definitions(OLD/'impl/gwsynthesis/g2_top.vg'))
    newparams=parameter_reader(definitions(NEW/'impl/gwsynthesis/g2_top.vg'))
    directions=primitive_directions(LIB)
    pending=collections.defaultdict(list)
    for row in memory['port_name_differences']:
        n=row['name'];a,b=before[n],after[n]
        for pin,change in row['changed_ports'].items():
            assert directions[a['kind']][pin]=='input',('changed non-input memory pin',n,pin)
            for i,(an,bn)in enumerate(zip(change['old'],change['new'])):
                if an!=bn:
                    pending[(an,bn)].append(dict(memory_instance=n,pin=pin,vector_index=i))
    print('unique changed memory input pairs '+str(len(pending)),flush=True)
    oldlogic,newlogic,boundaries=construct_shared_logic(before,after,oldparams,newparams,directions)
    def actual_driver(logic,params,net):
        rows=logic.drivers.get(net,[])
        assert len(rows)==1,('nonunique changed memory input driver',net,rows)
        n,pin,index=rows[0]
        return dict(instance=n,output_pin=pin,vector_index=index,cell=logic.cells[n],parameters=params(logic.cells[n]))
    checks=[]
    for i,((an,bn),pins)in enumerate(sorted(pending.items())):
        ad,bd=actual_driver(oldlogic,oldparams,an),actual_driver(newlogic,newparams,bn)
        delta=z3.simplify(z3.Xor(oldlogic.expr(an),newlogic.expr(bn)))
        method='canonical_Boolean_identity'
        if not z3.is_false(delta):
            solver=z3.Solver();solver.set(timeout=10000);solver.add(delta);status=solver.check()
            if status!=z3.unsat:
                save('memory_input_binding_miter_failure.json',dict(passed=False,old_net=an,new_net=bn,old_actual_driver=ad,new_actual_driver=bd,affected_memory_pins=pins,status=str(status),detail=str(solver.model())if status==z3.sat else solver.reason_unknown(),already_passed_checks=checks))
                raise AssertionError(('actual changed memory input not proved',an,bn,str(status)))
            method='SAT_unsatisfiable_miter'
        driver_same_primitive=ad['instance']==bd['instance'] and ad['cell']['kind']==bd['cell']['kind'] and ad['parameters']==bd['parameters']
        driver_non_Q_equal=driver_same_primitive and {p:v for p,v in ad['cell']['ports'].items()if p!='Q'}=={p:v for p,v in bd['cell']['ports'].items()if p!='Q'}
        checks.append(dict(passed=True,old_net=an,new_net=bn,old_actual_driver=ad,new_actual_driver=bd,same_actual_driver_primitive_kind_INIT=driver_same_primitive,all_driver_non_Q_pins_identical=driver_non_Q_equal,method=method,affected_memory_pins=pins))
        print('proved input pair '+str(i+1)+'/'+str(len(pending))+' '+an+' -> '+bn,flush=True)
    proof=dict(passed=True,recorded_at=datetime.datetime.now().astimezone().isoformat(),changed_memory_leaves=len(memory['port_name_differences']),changed_memory_scalar_input_bindings=sum(len(v)for v in pending.values()),unique_input_pairs=len(checks),every_changed_memory_input_Boolean_equivalent_at_actual_state_boundaries=True,actual_state_boundary_contract=boundaries,no_suffix_substitution_used=True,checks=checks,full_sequential_or_physical_equivalence_claimed=False)
    save('actual_memory_input_binding_proof.json',proof)
    assert all(meta(Path(r['path']))==r for r in frozen)
    final=dict(initial)
    final.update(passed=True,phase='actual_memory_clock_original_radar26_review_complete',memory_input_binding_proof='actual_memory_input_binding_proof.json',every_changed_memory_input_Boolean_equivalent_at_actual_state_boundaries=True,unique_changed_memory_input_pairs=len(checks),changed_scalar_memory_input_bindings=proof['changed_memory_scalar_input_bindings'],all_memory_parameters_INIT_CPU_and_clock_checks_passed=True,original_radar26_strict_revalidation_passed=True,original_radar26_all_78_endpoint_cells_ports_identical=True,source_netlist_reference_metadata_unchanged_at_end=True,scope='Actual complete SYN37 versus SYN34 all memory primitive parameters/INIT/CPU image, three-clock/PLL driver tracing, original radar26 strict structural revalidation, and changed memory-input combinational proof at corresponding actual primitive state boundaries. No full sequential equivalence, PNR, timing, electrical CDC or deployment signoff.',full_sequential_or_physical_equivalence_claimed=False)
    save('result.json',final)
    print(json.dumps({k:v for k,v in final.items()if k!='metadata_bindings'},ensure_ascii=False),flush=True)
except Exception as e:
    save('memory_binding_review_failed.json',dict(passed=False,error=str(e),traceback=traceback.format_exc()))
    raise
