"""Stream the three attributed DFFREs omitted by the old flat VO regexp.

No whole-G2 graph, EDA, simulator or source edit. Source hierarchy is expanded
only along the three requested paths. Physical VO is scanned once per build;
all attribute lines are enumerated, not just a name whitelist.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re

from synthesis_connectivity import CELL, PORT, DECL, TOKEN
from audit_netlist_memories import PARAM
from prepare_baseline_fusion_retime import metadata
# Exact pre-fix expression, retained locally so this historical supplement is
# reproducible after the shared physical reader is repaired.
OLD_VO_CELL=re.compile(r'^(\w+)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)\)\s*;',re.M|re.S)


BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'SLF128_storage_safeFW_syn43'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
TARGETS=('u_lmac/mdio_oe_s1',
         'u_control/display/axes_display.thread/driver/bus/scl_low_s1',
         'u_control/display/axes_display.thread/driver/bus/sda_low_s1')
HEAD=re.compile(r'^\s*(?:(\(\*.*?\*\))\s*)?(\w+)\s+(\\\S+|[^\s(]+)\s*\(\s*$')
EXTRA=('clk50_ibuf','eth_mdio_s5','eth_mdio_iobuf','sck_iobuf','sda_iobuf',
       'u_control/display/axes_display.thread/driver/bus/IIC_SCL_s3',
       'u_control/display/axes_display.thread/driver/bus/IIC_SDA_s3')
CLOCK=re.compile(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK')


def module_text(path,name):
    active=False;rows=[];first=None
    with path.open(encoding='utf8') as f:
        for line,s in enumerate(f,1):
            if re.match(r'^module\s+'+re.escape(name)+r'\s*\(',s):
                assert not active;active=True;first=line
            if active:
                rows.append(s)
                if s.startswith('endmodule'):
                    return ''.join(rows),first
    raise AssertionError(('module not found',name,path))


class LocalSource:
    def __init__(self,path):self.path=path;self.cache={};self.cells={};self.contexts=[]
    def definition(self,kind):
        if kind not in self.cache:
            text,line=module_text(self.path,kind)
            assert not re.search(r'^\s*assign\b',text,re.M)
            decl={n.lstrip('\\'):(d,int(hi)if hi else None,int(lo)if lo else None)for d,hi,lo,n in DECL.findall(text)}
            cells={n.lstrip('\\'):(t,{p.lstrip('\\'):x for p,x in PORT.findall(b)})for t,n,b in CELL.findall(text)}
            params={}
            for n,k,v in PARAM.findall(text):params.setdefault(n.strip().lstrip('\\'),{})[k]=v.strip()
            self.cache[kind]=(text,line,decl,cells,params)
        return self.cache[kind]
    @staticmethod
    def indices(h,l):return range(h,l+(1 if l>=h else-1),1 if l>=h else-1)
    def expand(self,expr,decl,prefix,bindings):
        out=[];pos=0
        def local(n):return n if n in ('GND','VCC')else bindings.get(n,((prefix+'/')if prefix else'')+n)
        while pos<len(expr):
            if not expr[pos:].strip():break
            m=TOKEN.match(expr,pos);assert m,('unsupported scalar/vector alias',expr[pos:]);pos=m.end()
            w,b,v,n,h,l,punct=m.groups()
            if punct:continue
            if w:
                assert not re.search('[xXzZ]',v);value=int(v,{'b':2,'h':16,'d':10}[b]);out+=['VCC'if value>>i&1 else'GND'for i in range(int(w)-1,-1,-1)]
            else:
                n=n.lstrip('\\')
                if h is not None:out +=[local(n+'['+str(i)+']')for i in self.indices(int(h),int(l or h))]
                elif n in decl and decl[n][1]is not None:out +=[local(n+'['+str(i)+']')for i in self.indices(decl[n][1],decl[n][2])]
                else:out.append(local(n))
        return out
    def target(self,path):
        kind='g2_top';prefix='';bindings={};route=[]
        parts=path.split('/')
        for i,inst in enumerate(parts):
            text,line,decl,cells,params=self.definition(kind)
            assert inst in cells,(kind,inst)
            child,raw=cells[inst]
            ports={p:self.expand(x,decl,prefix,bindings)for p,x in raw.items()}
            full=(prefix+'/'+inst).strip('/')
            route.append(dict(module=kind,module_start_line=line,instance=inst,kind=child,resolved_ports=ports))
            if i==len(parts)-1:
                result=dict(name=full,kind=child,ports=ports,parameters=params.get(inst,{}),source_route=route)
                self.cells[full]=result;return result
            _,_,childdecl,_,_=self.definition(child)
            new={}
            for pin,nets in ports.items():
                _,h,l=childdecl[pin]
                names=[pin]if h is None else[pin+'['+str(j)+']'for j in self.indices(h,l)]
                assert len(names)==len(nets),(full,pin)
                new.update(zip(names,nets))
            bindings=new;prefix=full;kind=child
        raise AssertionError(path)


def physical_stream(path,wanted):
    cells={};attributes=[];active=None;rows=[];clock_count=0;attr_clock_count=0;heads=0
    with path.open(encoding='utf8')as f:
        for line,s in enumerate(f,1):
            if active is None:
                m=HEAD.match(s)
                if m and m[2]!='module':
                    attr,kind,name=m.groups();name=name.lstrip('\\');heads+=1
                    active=dict(name=name,kind=kind,line=line,attribute=attr)
                    rows=[s]
                    if attr:attributes.append(dict(active))
                elif '(*'in s or '*)'in s:
                    raise AssertionError(('unsupported/standalone attribute must not be silently omitted',line,s))
                else:
                    m=PARAM.match(s)
                    if m:
                        n,k,v=m.groups();n=n.strip().lstrip('\\')
                        if n in cells:cells[n]['parameters'][k]=v.strip()
            else:
                rows.append(s)
                if ');'in s:
                    text=''.join(rows)
                    ports={p.lstrip('\\'):[v.strip().lstrip('\\').strip()]for p,v in PORT.findall(text)}
                    clocks=[p for p in ports if CLOCK.fullmatch(p)]
                    clock_count+=sum(len(ports[p])for p in clocks if ports[p]not in(['GND'],['VCC']))
                    if active['attribute']:
                        attr_clock_count+=len(clocks)
                        attributes[-1].update(clock_pins={p:ports[p]for p in clocks},raw_head=rows[0].strip())
                    if active['name']in wanted or active['attribute']:
                        active.update(ports=ports,parameters={},raw=text)
                        assert active['name']not in cells
                        cells[active['name']]=active
                    active=None;rows=[]
    assert active is None
    return cells,dict(all_attributed_primitives=attributes,all_attributed_primitives_count=len(attributes),
        attributed_clock_pins=attr_clock_count,primitive_heads_scanned=heads,
        clock_pin_token_groups_scanned=clock_count,whole_graph_retained=False)


def comparable(cell):return {k:cell[k]for k in('kind','ports','parameters')}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--source',type=Path,default=SOURCE)
    ap.add_argument('--base-review',type=Path,default=BASE/'PNR53_actual_clock_stop_review1')
    a=ap.parse_args();assert not a.out.exists()
    vg=a.source/'impl/gwsynthesis/g2_top.vg'
    src=LocalSource(vg)
    selected={n:src.target(n)for n in TARGETS+EXTRA}
    assert all(selected[n]['kind']=='DFFRE'for n in TARGETS)
    assert all(set(selected[n]['ports'])=={'D','Q','CLK','CE','RESET'}for n in TARGETS)
    assert all(selected[n]['parameters']=={'INIT':"1'b0"}for n in TARGETS)
    assert selected['clk50_ibuf']['ports']=={'O':['clk50_d'],'I':['clk50']}
    ff_macro,ff_line=module_text(LIB,'DFFRE')
    io_macro,io_line=module_text(LIB,'IOBUF')
    assert 'always @(posedge CLK)'in ff_macro
    assert re.search(r'if\s*\(RESET\)\s*Q_reg\s*<=\s*1\x27b0\s*;\s*else if\s*\(CE\)\s*Q_reg\s*<=\s*D\s*;',ff_macro)
    assert 'assign Q = Q_reg;'in ff_macro and 'initial Q_reg = INIT;'in ff_macro
    assert 'bufif0 IB (IO,I,OEN);'in io_macro
    old_path=a.base_review/'all_original_clock_pin_role_comparison.json'
    old=json.loads(old_path.read_text(encoding='utf8'))
    missing={v['instance']for v in old['missing']}
    assert missing==set(TARGETS)and not old['different_root_or_inversion']
    bindings=[metadata(vg),metadata(old_path),metadata(a.base_review/'review_failed.json'),metadata(LIB),metadata(Path(__file__).resolve())]
    result=[]
    for name in('SLF128_storage_place1_pnr53','SLF128_storage_place2_pnr54'):
        build=BASE/name;vo=build/'impl/pnr/g2_top.vo'
        bindings.append(metadata(vo))
        physical,inventory=physical_stream(vo,set(TARGETS+EXTRA))
        assert {x['name']for x in inventory['all_attributed_primitives']}==set(TARGETS)
        assert inventory['all_attributed_primitives_count']==inventory['attributed_clock_pins']==3
        checks=[]
        for n in TARGETS+EXTRA:
            s=selected[n];p=physical[n]
            assert comparable(s)==comparable(p),(name,n,comparable(s),comparable(p))
            checks.append(dict(name=n,source=comparable(s),physical=comparable(p),physical_line=p['line'],
                physical_attribute=p['attribute'],strict_kind_all_ports_explicit_parameters_identical=True))
            if n in TARGETS:
                assert p['ports']['CLK']==['clk50_d']
                assert p['attribute']=='(*gowin_io_reg = "FALSE" *)'
                assert not OLD_VO_CELL.search(p['raw']),'old parser unexpectedly recognizes attributed head'
                assert p['parameters']=={'INIT':"1'b0"}
        # Verify the complete output-enable chain, not just a missing name.
        chains=[]
        for ff,inv,io in ((TARGETS[0],'eth_mdio_s5','eth_mdio_iobuf'),
                           (TARGETS[1],EXTRA[-2],'sck_iobuf'),
                           (TARGETS[2],EXTRA[-1],'sda_iobuf')):
            assert physical[inv]['kind']=='INV'and physical[io]['kind']=='IOBUF'
            assert physical[ff]['ports']['Q']==physical[inv]['ports']['I']
            assert physical[inv]['ports']['O']==physical[io]['ports']['OEN']
            chains.append(dict(FF=ff,registered_Q=physical[ff]['ports']['Q'],inverter=inv,
                IOBUF=io,OEN=physical[io]['ports']['OEN'],Q_to_OEN_inversion=True,
                source_and_physical_identical=True,active_high_FF_drives_active_low_IO_enable=True,
                IO_data=physical[io]['ports']['I'],IO_pad=physical[io]['ports']['IO']))
        result.append(dict(build=name,attributed_inventory=inventory,checks=checks,output_enable_chains=chains,
            no_IOB_absorption_or_FF_replacement=True,all3_still_explicit_DFFRE=True,
            all3_original_clk50_noninverted=True,source_to_pad_inversion_unchanged=True))
    assert bindings==[metadata(Path(x['path']))for x in bindings]
    r=dict(passed=True,scope='Lightweight stream supplement for exactly3 parser-omitted clock pins and their complete OE/IOBUF chains',
        recorded_at=datetime.now().astimezone().isoformat(),whole_G2_graph_loaded=False,EDA_started=False,
        root_cause='Old physical CELL regular expression starts at primitive kind and misses an inline Verilog attribute prefix',
        original_failed_review_preserved=True,all_other_original_clock_checks_reused_from=metadata(old_path),
        original_nonconstant_clock_pins=old['original_source_nonconstant_clock_pin_count'],
        old_matched_original_clock_pins=old['matched_original_pin_count'],additional_strictly_matched_original_clock_pins=3,
        supplemented_original_matched_clock_pins=old['matched_original_pin_count']+3,
        all_original_source_clock_pin_roles_accounted_for=old['matched_original_pin_count']+3==old['original_source_nonconstant_clock_pin_count'],
        reused_base_scope='Full source-clock role accounting is reused from PNR53 only. PNR54 here proves these3 attributed sites, not its remaining all-clock inventory.',
        findings=result,source_local_hierarchy_routes={n:selected[n]['source_route']for n in TARGETS},
        source_modules_streamed=sorted(src.cache),source_module_count=len(src.cache),input_bindings=bindings,
        exact_pin_scope='D/CLK/CE/synchronous RESET/Q nets and explicit INIT unchanged, through resolved source port aliases; local upstream combinational functionality not re-proved by this clock supplement',
        DFFRE_semantics='Same primitive on both sides, positive-edge CLK, active-high synchronous RESET and CE, Q noninverting, explicit INIT0',
        official_primitive_semantic_binding=dict(DFFRE_start_line=ff_line,IOBUF_start_line=io_line,
            DFFRE_synchronous_RESET_over_CE_and_noninverting_Q_checked=True,IOBUF_bufif0_active_low_OEN_checked=True),
        limitations=['Does not prove whole OLED/MDIO state-machine functionality or timing','Does not accept external IO timing/reset pulse/CDC MTBF/physical stop'],
        deployment_accepted=False)
    assert r['all_original_source_clock_pin_roles_accounted_for']
    a.out.mkdir(parents=True)
    write=lambda p,v:p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    write(a.out/'result.json',r)
    print(json.dumps(dict(passed=True,result=str(a.out/'result.json'),matched_original_clock_pins=r['supplemented_original_matched_clock_pins'],attributed_FF_per_build=3),ensure_ascii=False))


if __name__=='__main__':main()
