"""Prepare auditable region-member inventories from unchanged PNR35 VG/VO/TR.
No synthesis, PNR, GUI automation, protected database access or coordinate inference.
Run with --out pointing to a new directory; generated CST templates are deliberately invalid.
"""
import argparse
import collections
import csv
import datetime
import json
import re
from pathlib import Path

BSRAM = {'SP','SPX9','SDPB','SDPX9B','DPB','DPX9B','pROM','pROMX9','SDP36KE'}
MODULE = re.compile(r'^module\s+(\S+)\s*\((.*?)^endmodule', re.M|re.S)
INSTANCE = re.compile(r'^\s{2}(\w+)\s+(\\\S+|[^\s(]+)\s*\(', re.M)
PHYSICAL = re.compile(r'^(?!module\b)(\w+)\s+(\\\S+|\w+)\s*\(', re.M)
PROFILES = {'action': 'u_control/model_thread/actuator_map/',
            'fusion': 'u_control/model_thread/fusion/', 'all_control': 'u_control/'}


def normalize(name):
    return name.strip().lstrip('\\')


def category(kind):
    if kind is None:
        return None
    if kind in BSRAM:
        return 'BSRAM_hard_resource'
    if kind.startswith('MULT'):
        return 'DSP_hard_resource'
    if kind in ('GND','VCC'):
        return 'constant_not_placeable_member'
    if kind in ('INV','BUF','BUFG','IBUF','OBUF','IOBUF','TBUF'):
        return 'INV_or_buffer_separate_binding'
    if kind.startswith(('LUT','DFF','DL','MUX2_LUT','RAM16')) or kind in ('ALU','ROM16'):
        return 'CFU_implementation_primitive'
    if kind.startswith(('IO','DQS','PLL','CLK','DCS','DCE','DQCE')):
        return 'IO_clock_or_other_hard_resource'
    return 'unclassified_requires_review'


def metadata(path):
    st = path.stat()
    return {'path':str(path), 'bytes':st.st_size, 'mtime_ns':st.st_mtime_ns}


def source_leaves(path):
    modules = dict(MODULE.findall(path.read_text(encoding='utf-8')))
    assert 'g2_top' in modules
    children = {kind:INSTANCE.findall(body) for kind,body in modules.items()}
    leaves = {}
    def walk(kind, prefix='', ancestors=()):
        assert kind not in ancestors, ('recursive module',kind)
        for child,local_name in children[kind]:
            local_name=normalize(local_name)
            full=prefix+'/'+local_name if prefix else local_name
            if not prefix and full!='u_control':
                continue
            if child in modules:
                walk(child,full,ancestors+(kind,))
            else:
                assert full not in leaves,('duplicate VG path',full)
                leaves[full]=child
    walk('g2_top')
    assert leaves and all(n.startswith('u_control/')for n in leaves)
    return leaves


def physical_leaves(path):
    rows = [(normalize(name),kind) for kind,name in PHYSICAL.findall(path.read_text(encoding='utf-8'))
            if normalize(name).startswith('u_control/')]
    leaves=dict(rows)
    assert len(rows)==len(leaves), 'duplicate VO paths'
    return leaves


def reconciliation(vg, vo, prefix):
    names=sorted(n for n in set(vg)|set(vo) if n.startswith(prefix))
    result=[]
    for name in names:
        a,b=vg.get(name),vo.get(name)
        match=('exact_name_and_kind' if a==b else 'physical_only' if a is None else
               'source_only' if b is None else 'same_name_kind_changed')
        result.append(dict(name=name,VG_kind=a,VO_kind=b,VG_category=category(a),
                           VO_category=category(b),match=match))
    return result


def write_csv(path, rows):
    keys=['name','VG_kind','VO_kind','VG_category','VO_category','match']
    with path.open('w',encoding='utf-8-sig',newline='')as file:
        writer=csv.DictWriter(file,fieldnames=keys)
        writer.writeheader();writer.writerows(rows)


def action_path(tr):
    text=tr.read_text(encoding='utf-8')
    setup=text.split('3.3.1 Setup Analysis Report',1)[1]
    match=re.search(r'^\s*Path1\s*$\n(.*?)^\s*Path2\s*$',setup,re.M|re.S)
    assert match
    block=match[1]
    source=re.search(r'^From\s*:\s*(\S+)',block,re.M)[1]
    target=re.search(r'^To\s*:\s*(\S+)',block,re.M)[1]
    arrival=block.split('Data Arrival Path:',1)[1].split('Data Required Path:',1)[0]
    arc_regex=re.compile(r'^\s*([-\d.]+)\s+([-\d.]+)\s+(t\w+)\s+([RF]+)\s+(\d+)\s+(\S+)\s+(.+?)\s*$',re.M)
    arcs=[]
    for at,delay,kind,edge,fanout,site,node in arc_regex.findall(arrival):
        row=dict(arrival_ns=float(at),delay_ns=float(delay),type=kind,transition=edge,
                 fanout=int(fanout),reported_site=site,node=node)
        if kind=='tNET'and arcs:
            row['from_reported_node']=arcs[-1]['node']
            row['from_reported_site']=arcs[-1]['reported_site']
        arcs.append(row)
    long_nets=[x for x in arcs if x['type']=='tNET'and x['delay_ns']>=5]
    assert source=='u_control/model_thread/policy/actions_61_s1'
    assert target=='u_control/model_thread/actuator_map/left_mmps_18_s1'
    assert [x['delay_ns']for x in long_nets]==[11.042,6.234]
    return dict(from_register=source,to_register=target,slack_ns=float(re.search(r'^Slack\s*:\s*([-\d.]+)',block,re.M)[1]),
                full_actual_data_arrival_arcs=arcs,actual_long_nets=long_nets,
                total_route_ns=21.960,total_data_delay_ns=30.066,
                DSP_GRID_column_and_adjacency_inferred=False,unreported_capacity=None,
                original_report_block=block)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    vg_path=args.build/'g2_top.vg';vo_path=args.build/'impl/pnr/g2_top.vo';tr=args.build/'impl/pnr/g2_top.tr'
    before=[metadata(p)for p in (vg_path,vo_path,tr)]
    vg=source_leaves(vg_path);vo=physical_leaves(vo_path)
    summary={}
    for profile,prefix in PROFILES.items():
        directory=args.out/profile;directory.mkdir()
        rows=reconciliation(vg,vo,prefix)
        exact_cfu=[r for r in rows if r['match']=='exact_name_and_kind'and r['VO_category']=='CFU_implementation_primitive']
        write_csv(directory/'all_leaf_reconciliation.csv',rows)
        write_csv(directory/'physical_CFU_inventory.csv',[r for r in rows if r['VO_category']=='CFU_implementation_primitive'])
        write_csv(directory/'exact_source_physical_CFU_members.csv',exact_cfu)
        write_csv(directory/'BSRAM_separate_members.csv',[r for r in rows if 'BSRAM_hard_resource'in(r['VG_category'],r['VO_category'])])
        write_csv(directory/'DSP_separate_members.csv',[r for r in rows if 'DSP_hard_resource'in(r['VG_category'],r['VO_category'])])
        write_csv(directory/'INV_IO_constant_other_excluded.csv',[r for r in rows if not(r['match']=='exact_name_and_kind'and r['VO_category']=='CFU_implementation_primitive') and r['VG_category']not in('CFU_implementation_primitive','BSRAM_hard_resource','DSP_hard_resource')and r['VO_category']not in('CFU_implementation_primitive','BSRAM_hard_resource','DSP_hard_resource')])
        write_csv(directory/'all_name_or_kind_differences.csv',[r for r in rows if r['match']!='exact_name_and_kind'])
        (directory/'exact_source_physical_CFU_names.lst').write_text(''.join(r['name']+'\n'for r in exact_cfu),encoding='utf-8')
        template=['REGION_TEMPLATE_NOT_READY_COORDINATES_CAPACITY_AND_BINDING_MISSING;',
                  '// Deliberately invalid: not a runnable CST. No coordinates have been selected.',
                  '// Members below exactly match PNR35 VG and VO names and primitive types.',
                  '// ALU/ROM16/RAM16/MUX/DFF binding and compatibility must be checked in official FloorPlanner.',
                  '// Hard DSP/BSRAM, INV/buffers/constants and unmatched PNR-added leaves are separate.',
                  'GROUP PNR35_'+profile.upper()+'_CFU = {']
        template+=[' "'+r['name']+'"'for r in exact_cfu]
        template+=['};','GRP_LOC PNR35_'+profile.upper()+'_CFU R[ROW_BEGIN:ROW_END]C[COL_BEGIN:COL_END];',
                   '// No exclusive reservation. Placeability, legal sites and capacity remain unknown.']
        (directory/'region_not_ready.cst.disabled.template').write_text('\n'.join(template)+'\n',encoding='utf-8')
        summary[profile]=dict(prefix=prefix,source_leaves=sum(r['VG_kind']is not None for r in rows),
            physical_leaves=sum(r['VO_kind']is not None for r in rows),reconciliation_counts=dict(collections.Counter(r['match']for r in rows)),
            exact_source_physical_CFU_members=len(exact_cfu),exact_CFU_by_kind=dict(collections.Counter(r['VO_kind']for r in exact_cfu)),
            physical_CFU_members=sum(r['VO_category']=='CFU_implementation_primitive'for r in rows),
            source_CFU_members=sum(r['VG_category']=='CFU_implementation_primitive'for r in rows),
            physical_BSRAM=sum(r['VO_category']=='BSRAM_hard_resource'for r in rows),
            physical_DSP=sum(r['VO_category']=='DSP_hard_resource'for r in rows),
            physical_only_CFU_unusable_as_original_VG_names=[r for r in rows if r['match']=='physical_only'and r['VO_category']=='CFU_implementation_primitive'],
            geometry_coordinates=None,capacity_LUT_DFF_ALU_RAM_DSP_free_sites=None,region_ready=False)
    path=action_path(tr)
    (args.out/'PNR35_actual_action_path.json').write_text(json.dumps(path,ensure_ascii=False,indent=2),encoding='utf-8')
    (args.out/'PNR35_actual_action_path.txt').write_text(path['original_report_block'],encoding='utf-8')
    assert before==[metadata(p)for p in(vg_path,vo_path,tr)],'source changed while reading'
    manifest=dict(recorded_at=datetime.datetime.now().astimezone().isoformat(),scope='Read-only public VG/VO/TR primitive inventory and disabled region templates',
        build=str(args.build),source_bindings=before,source_VG_bytes_not_modified=True,
        original_PNR35_CST_SDC_options_not_modified=True,profiles=summary,
        full_name_and_kind_only_not_ports_or_logic_equivalence=True,
        differences_not_automatically_aliased=True,coordinates_selected=False,
        available_region_capacity_proven=False,FloorPlanner_object_binding_executed=False,
        previously_prepared_two_DSP_reported_site_lock_only=True,
        official_document='D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/doc/EN/SUG1541-1.0E_Gowin Software Physical Constraints User Guide.pdf',
        manual_printed_pages=dict(CFU='7-9',hard_RAM_DSP='10-11',GROUP='14-15',successful_PNR_back_annotation='44-45',Chip_Array_actual_placement_and_density='53-57'),
        missing=['Actual C-version DSP-to-GRID column mapping and legal nearby CFU sites.',
                 'Actual occupied and free per-site LUT/DFF/ALU/SSRAM capacity and hard-resource capacity for a proposed region.',
                 'Full 49 BSRAM / 70 DSP physical site map for whole-control proposal.',
                 'Official FloorPlanner binding of the exact selected low-level primitive kinds and proposed region.',
                 'Resource pressure and cut-net cost at control-to-NAV/system boundary after the region is selected.'],
        GUI_runtime_unavailable_reported_by_parent=True,no_repeated_GUI_recovery_attempted=True,
        current_PNR_started=False,current_SYN_started=False,all_templates_deliberately_invalid=True,
        no_protected_database_access=True,no_coordinate_inference=True,
        next_step='Use official successful-PNR35 Place View/back-annotation when computer runtime is available; choose and validate sites/capacity before preparing any runnable CST or PNR.')
    (args.out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:{x:v[x]for x in('source_leaves','physical_leaves','exact_source_physical_CFU_members','physical_CFU_members','source_CFU_members','physical_BSRAM','physical_DSP','reconciliation_counts')}for k,v in summary.items()},ensure_ascii=False))

if __name__=='__main__':
    main()
