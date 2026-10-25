"""Prove the actual live TX RAM migration after independent shadow removal.

Only the small mapped mailbox partition is expanded. Original live RAM leaves
are selected by the actual tq register D cones, never by an assumed suffix.
The shared-memory induction checks every write/async-read coordinate, then
proves tq/pb next-state. Downstream transport/state remains the caller's proof.
"""
import argparse
from collections import defaultdict
from datetime import datetime
import json,re,time
from pathlib import Path

from prove_soc_console_remap import Logic,literal,z3

PREFIX='u_nav/u_mbx/'
LIVE='u_nav/u_mbx/u_txb/'
OLD_STORAGE='u_nav/u_mbx/u_txs/'

def prove(before,after,old_params,new_params,directions,qsymbols_old=None,qsymbols_new=None,timeout_ms=10000):
    start=time.monotonic()
    from revalidate_observation_routing import LIBRARY
    body=re.search(r'module RAM16SDP4\b(.*?)endmodule',Path(LIBRARY).read_text(encoding='utf-8'),re.S)[1]
    assert all(re.search(r'parameter INIT_'+str(i)+r"\s*=\s*16'h0000\s*;",body) for i in range(4))
    a={n:c for n,c in before.items() if n.startswith(PREFIX)}
    b={n:c for n,c in after.items() if n.startswith(PREFIX)}
    assert a and b,'mailbox partition unavailable'
    def params_a(c):return old_params(c)
    def params_b(c):return new_params(c)
    def driver_index(cells):
        d=defaultdict(list)
        for n,c in cells.items():
            for pin,nets in c['ports'].items():
                if directions[c['kind']].get(pin)=='output':
                    for i,net in enumerate(nets):
                        if net not in ('GND','VCC'):d[net].append((n,pin,i))
        return d
    da,db=driver_index(a),driver_index(b)
    def read_registers(cells):
        found={}
        for n,c in cells.items():
            m=re.fullmatch(re.escape(LIVE)+r'g_distributed\.qr_(\d+)_s0',n)
            if m:
                bit=int(m[1]);assert c['kind']=='DFFRE'
                assert c['ports']['Q']==[f'u_nav/tq[{bit}]'];found[bit]=c
        assert set(found)==set(range(32)),'live tq register boundary changed'
        return found
    ra,rb=read_registers(a),read_registers(b)
    def leaves(cells,drivers,roots):
        seen=set();ram=set();other=set()
        def walk(net):
            if net in seen or net in ('GND','VCC'):return
            seen.add(net);ds=drivers.get(net,[])
            assert len(ds)<=1,('nonunique live read driver',net,ds)
            if not ds:return
            name,pin,index=ds[0];c=cells[name]
            if c['kind']=='RAM16SDP4':ram.add(name);return
            if c['kind'].startswith('DFF'):other.add(name);return
            assert c['kind'].startswith(('LUT','MUX2_LUT')) or c['kind'] in ('INV','GND','VCC'),('unexpected live-read operator',name,c['kind'])
            for p,nets in c['ports'].items():
                if directions[c['kind']].get(p)=='input':
                    for n in nets:walk(n)
        for n in roots:walk(n)
        return ram,seen,other
    old_live,old_cone,old_state=leaves(a,da,[ra[i]['ports']['D'][0] for i in range(32)])
    new_live,new_cone,new_state=leaves(b,db,[rb[i]['ports']['D'][0] for i in range(32)])
    assert len(old_live)==len(new_live)==256
    assert all(n.startswith(OLD_STORAGE) for n in old_live)
    assert all(n.startswith(LIVE) for n in new_live)
    old_all={n for n,c in a.items() if n.startswith(OLD_STORAGE) and c['kind']=='RAM16SDP4'}
    discarded=old_all-old_live;assert len(discarded)==256
    # The live copy is distinguished by a proven backward read cone AND its
    # actual low read address. The shadow copy reads the CPU debug address.
    live_RAD=[f'u_nav/mb_pl_addr[{i}]' for i in (5,4,3,2)]
    shadow_RAD=[f'u_nav/r_MBX_TXB_ADDR_Z[{i}]' for i in (3,2,1,0)]
    assert all(a[n]['ports']['RAD']==live_RAD for n in old_live)
    assert all(b[n]['ports']['RAD']==live_RAD for n in new_live)
    assert all(a[n]['ports']['RAD']==shadow_RAD for n in discarded)
    def coordinate(name,c):
        m=re.search(r'\.mem_(\d+)_(\d+)_s$',name)
        assert m,('unexpected live-memory coordinate suffix',name)
        bank,nibble=map(int,m.groups());assert 0<=bank<32 and 0<=nibble<8
        # Naming supplies only a candidate coordinate. These precise data pins
        # and all write-enable functions are independently checked below.
        assert c['ports']['DI']==[f'u_nav/wd_q[{nibble*4+i}]' for i in (3,2,1,0)]
        assert c['ports']['WAD']==[f'u_nav/mb_twords[{i}]' for i in (3,2,1,0)]
        return bank,nibble
    ma={coordinate(n,a[n]):n for n in old_live};mb={coordinate(n,b[n]):n for n in new_live}
    assert len(ma)==len(mb)==256 and ma.keys()==mb.keys()=={(i,j) for i in range(32) for j in range(8)}
    qsa=dict(qsymbols_old or {});qsb=dict(qsymbols_new or {})
    own_state_pairs=[]
    for name in sorted(before.keys()&after.keys()):
        x,y=before[name],after[name]
        if not x['kind'].startswith('DFF') or not y['kind'].startswith('DFF'):continue
        if qsymbols_old is not None and qsymbols_new is not None and not name.startswith(PREFIX):continue
        assert x['kind']==y['kind'] and x['ports']['CLK']==y['ports']['CLK']
        assert params_a(x).get('INIT')==params_b(y).get('INIT')
        qx,=x['ports']['Q'];qy,=y['ports']['Q']
        if qx in qsa and qy in qsb:assert z3.eq(qsa[qx],qsb[qy])
        else:
            sym=z3.Bool('TX_partition_state:'+name);qsa[qx]=sym;qsb[qy]=sym
        own_state_pairs.append(dict(cell=name,old_Q=qx,new_Q=qy))
    osa={};osb={};pairs=[]
    for coord in sorted(ma):
        old,new=ma[coord],mb[coord];x,y=a[old],b[new]
        assert x['kind']==y['kind']=='RAM16SDP4'
        assert set(x['ports'])==set(y['ports'])=={'DO','DI','WAD','RAD','WRE','CLK'}
        assert all(len(x['ports'][p])==len(y['ports'][p])==size for p,size in [('DO',4),('DI',4),('WAD',4),('RAD',4),('WRE',1),('CLK',1)])
        assert x['ports']['CLK']==y['ports']['CLK']==['clk50_d']
        def init(params):
            assert set(params)<={f'INIT_{i}' for i in range(4)}
            return [literal(params.get(f'INIT_{i}',"16'h0000")) for i in range(4)]
        ix,iy=init(params_a(x)),init(params_b(y));assert ix==iy==[0]*4
        for i in range(4):
            symbol=z3.Bool(f'TX_memory:{coord[0]}:{coord[1]}:DO:{i}')
            osa[(old,'DO',i)]=symbol;osb[(new,'DO',i)]=symbol
        pairs.append(dict(bank=coord[0],nibble=coord[1],old_live_leaf=old,new_live_leaf=new,primitive='RAM16SDP4',geometry='16x4',INIT=ix,clock='clk50_d'))
    # Caller may supply the full already-parsed shell. Expanding actual parent
    # decode logic avoids unsoundly treating renamed common-decode nets as
    # independent mailbox inputs. Register Qs remain explicit induction cuts.
    la,lb=Logic(before,params_a,directions,qsa,osa),Logic(after,params_b,directions,qsb,osb)
    checks=[];sat=0
    def check(label,x,y):
        nonlocal sat
        delta=z3.simplify(z3.Xor(x,y))
        if z3.is_false(delta):checks.append(dict(endpoint=label,method='canonical_logic_identity'));return
        solver=z3.Solver();solver.set(timeout=timeout_ms);solver.add(delta);sat+=1;status=solver.check()
        assert status==z3.unsat,('TX mapped boundary mismatch or unproved',label,str(status),str(solver.model())[:1600] if status==z3.sat else solver.reason_unknown())
        checks.append(dict(endpoint=label,method='SAT_unsatisfiable_miter'))
    for record in pairs:
        x,y=a[record['old_live_leaf']],b[record['new_live_leaf']]
        for pin in ('WAD','DI','RAD','WRE'):
            for i,(ox,ny) in enumerate(zip(x['ports'][pin],y['ports'][pin])):check(f'RAM[{record["bank"]},{record["nibble"]}].{pin}[{i}]',la.expr(ox),lb.expr(ny))
    # Different bank enables must be distinct one-hot slices; this rejects an
    # erroneous duplicate-bank pairing that happens to have matching DI.
    wr_a=[la.expr(a[ma[(bank,0)]]['ports']['WRE'][0]) for bank in range(32)]
    for bank in range(32):
        for nibble in range(1,8):check(f'bank{bank} nibble{nibble} same write slice',wr_a[bank],la.expr(a[ma[(bank,nibble)]]['ports']['WRE'][0]))
        for other in range(bank):
            solver=z3.Solver();solver.set(timeout=timeout_ms);solver.add(z3.And(wr_a[bank],wr_a[other]));sat+=1
            assert solver.check()==z3.unsat,('TX write bank overlap',bank,other)
    for bit in range(32):
        x,y=ra[bit],rb[bit]
        assert x['ports']['CLK']==y['ports']['CLK']==['clk50_d']
        assert params_a(x)==params_b(y) and x['ports']['CE']==y['ports']['CE']==['u_nav/mb_pl_re']
        assert x['ports']['RESET']==y['ports']['RESET']==['GND']
        check(f'tq[{bit}] full next state',la.next_state(x),lb.next_state(y))
    pb=[]
    for bit in range(2):
        name=PREFIX+f'pb_{bit}_s0';x,y=a[name],b[name]
        assert x==y and params_a(x)==params_b(y)
        assert x['ports']==dict(Q=[f'u_nav/pb[{bit}]'],D=[f'u_nav/mb_pl_addr[{bit}]'],CLK=['clk50_d'],RESET=['GND'],CE=['u_nav/mb_pl_re'])
        check(f'pb[{bit}] full next state',la.next_state(x),lb.next_state(y));pb.append(name)
    # Wrong-reader negative control: the other original 256 leaves cannot be
    # substituted because their RAD pins are the independent debug address.
    negative=z3.Solver();negative.add(z3.Xor(la.expr(live_RAD[-1]),la.expr(shadow_RAD[-1])))
    assert negative.check()==z3.sat,'negative control failed to distinguish live and shadow readers'
    return dict(passed=True,old_TX_RAM_total=len(old_all),old_live_RAM_selected_by_actual_tq_D_cone=len(old_live),old_shadow_RAM_not_selected=len(discarded),new_live_RAM=len(new_live),bijective_live_storage_mapping=True,geometry_INIT_clock_all256_equal=True,all_write_and_live_read_coordinates_proved=True,tq32_and_pb2_complete_next_state_proved=True,shared_memory_induction='Each 16x4 leaf has equal zero INIT and CLK/WRE/WAD/DI for all assignments; corresponding state stays equal. Equal RAD then justifies DO cuts, followed by exact tq/pb next-state proof.',wrong_shadow_reader_negative_control_passed=True,wrong_shadow_reader_witness=str(negative.model()),memory_leaf_mapping=[dict(old=r['old_live_leaf'],new=r['new_live_leaf']) for r in pairs],mapped_pairs=pairs,checked_endpoints=checks,endpoint_count=len(checks),SAT_calls=sat,mailbox_state_correspondences=own_state_pairs,live_boundary_outputs=[f'u_nav/tq[{i}]' for i in range(32)]+[f'u_nav/pb[{i}]' for i in range(2)],primitive_RAM16SDP4_default_zero_INIT_checked=True,downstream_byte_select_or_transport_requires_callers_retained_shell_proof=True,scope='Actual live TX storage/read partition only; no full mailbox/safety/timing or deployment acceptance.',elapsed_s=time.monotonic()-start)

def main():
    import psutil
    from retry_frozen_synthesis import memory_status
    process=psutil.Process();process.nice(psutil.IDLE_PRIORITY_CLASS);process.cpu_affinity([16])
    host=memory_status()
    assert host.avail_phys>=1536*1024**2 and host.avail_page>=3072*1024**2,'host reserve for small TX proof unavailable'
    p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,required=True);a=p.parse_args()
    before=json.loads((a.cache/'tx_partition_old.json').read_text());after=json.loads((a.cache/'tx_partition_new.json').read_text())
    from audit_routing_topology import primitive_directions
    from revalidate_observation_routing import LIBRARY
    directions=primitive_directions(LIBRARY)
    text=Path(LIBRARY).read_text(encoding='utf-8')
    body=re.search(r'module RAM16SDP4\b(.*?)endmodule',text,re.S)[1]
    assert all(re.search(r'parameter INIT_'+str(i)+r"\s*=\s*16'h0000\s*;",body) for i in range(4))
    result=dict(passed=False)
    try:result=prove(before,after,lambda c:c['parameters'],lambda c:c['parameters'],directions)
    except Exception as exc:result=dict(passed=False,error=repr(exc));raise
    finally:
        memory=process.memory_info()
        result.update(recorded_at=datetime.now().astimezone().isoformat(),primitive_library=str(LIBRARY),primitive_RAM16SDP4_default_zero_INIT_checked=True,process_priority='IDLE',cpu_core=16,minimum_start_host_physical_bytes=host.avail_phys,minimum_start_host_commit_reserve_bytes=host.avail_page,process_peak_working_set_bytes=getattr(memory,'peak_wset',memory.rss),partition_source_netlists=dict(old='F:/nav_integration_g2_20261023/sfp_lane2_private_snapshot_syn11_retry1/impl/gwsynthesis/g2_top.vg',new='F:/nav_integration_g2_20261023/sfp_lane2_nav_strong_syn21/impl/gwsynthesis/g2_top.vg'),PNR_started=False,SYN_started=False,programmed=False)
        (a.cache/'actual_TX_memory_mapping_proof.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('mapped_pairs','memory_leaf_mapping','checked_endpoints','mailbox_state_correspondences')}))

if __name__=='__main__':main()
