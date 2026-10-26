"""Freeze small actual-SLF/XT fixtures. No simulator or EDA invocation."""
from pathlib import Path
import argparse, importlib.util, json, re, shutil
from prepare_baseline_fusion_retime import hardware_inputs, metadata

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion50_ring_MAX_syn40'
HERE=Path(__file__).parent


def prepare(candidate: Path):
    review=json.loads((candidate/'prepared_source_review.json').read_text(encoding='utf-8'))
    assert review['prepared'] and review['validation_not_run']
    for row in review['bindings']:
        assert metadata(Path(row['path']))==row, row
    val=candidate/'validation'
    assert not (val/'validation_request.json').exists(), 'Validation inputs already frozen'
    val.mkdir(exist_ok=True)
    def put(name,text):
        p=val/name
        if p.exists():assert p.read_text(encoding='utf-8')==text,('incomplete preparation content differs',p)
        else:p.write_text(text,encoding='utf-8')
    def copy(name,source):
        p=val/name
        if p.exists():assert p.read_bytes()==source.read_bytes(),('incomplete preparation bytes differ',p)
        else:shutil.copyfile(source,p)
    files=[]
    for role,root in [('ref',SOURCE),('new',candidate)]:
        for module,rel in (
            ('nav_ram_sdp','nav/rtl/common/nav_ram_sdp.v'),
            ('slf_extract','nav/rtl/match/slf_extract.v'),
            ('slf_lf','nav/rtl/match/slf_lf.v'),
            ('slf_match','nav/rtl/match/slf_match.v'),
            ('xt_frame_rx','nav/rtl/lidar/xt_frame_rx.v')):
            text=(root/rel).read_text(encoding='utf-8')
            if role=='ref':
                for ident in ('nav_ram_sdp','slf_extract','slf_lf','slf_match','xt_frame_rx'):
                    text=re.sub(r'\b'+ident+r'\b',ident+'_ref',text)
            name=f'{module}_{role}.v';put(name,text);files.append(name)
    for module in ('nav_ram_banked','nav_fifo_ob'):
        name=module+'.v';copy(name,SOURCE/'nav/rtl/common'/name);files.append(name)
    copy('tb.sv',HERE/'tb_stage2b_point_ownership.sv');files.append('tb.sv')
    # Frozen lookup asset from the actual source input path.
    manifest=json.loads((SOURCE/'manifest.json').read_text(encoding='utf-8-sig'))
    inputs=hardware_inputs(SOURCE,manifest)
    # hardware_inputs enumerates RTL only; the actual default filename is resolved
    # in the build working directory, not inside that RTL inventory.
    assert (SOURCE/'sin_lut.hex').read_bytes()==(candidate/'sin_lut.hex').read_bytes()
    copy('sin_lut.hex',SOURCE/'sin_lut.hex')
    spec=importlib.util.spec_from_file_location('xt_protocol_fixture',HERE.parents[1]/'model/xt_proto.py')
    model=importlib.util.module_from_spec(spec);spec.loader.exec_module(model)
    depth=[[1000+r*32+c for c in range(32)]for r in range(32)]
    frame=model.build_frame(depth,sn=9)
    words=[]
    for pkt in model.packetize(frame,sn=9):
        words += [byte|(int(i==len(pkt)-1)<<8)for i,byte in enumerate(pkt)]
    assert 1000<len(frame)<5000 and len(words)<5001
    put('xt_stim.hex','\n'.join(f'{w:03x}'for w in words)+'\n')
    put('xt_count.txt',f'{len(words)}\n')
    request=dict(profile='point_ownership',working_directory=str(val),files=files,top='tb_stage2b_point_ownership',
        minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3,
        RTL_simulation_started=False,synthesis_started=False,PNR_started=False,
        normal_miter='Actual original and candidate SLF plus XT modules; normal command/status/result/PB enable/address/data cycle identity. New frame-generation ready clearing is separately covered.',
        targeted_cases=['raw reservation before project/transform SOF','full4096 points','zero-point complete frame','online/prior UPDATE','unused NORMAL extraction','early command before SOF','command on final pb_we/ex_done boundary','HOLD cancel','match disable finishes accepted capture','IDLE soft-clear','no accepted frame empty EVAL','disabled noncapture pass','raw commit while owner blocks admission','retained replay request before EOF/read_wait','commit before a complete frame waits'],
        nonclaims=['No silicon collision behavior','No actual SDP CE/mode proof','No full nav_subsystem bus timing simulation','No Pico CPU or350ms/physical stop proof','No multiple configuration-descriptor queue'],
        run_entry=['D:/Anaconda/python.exe','-B','-X','utf8',str(HERE/'validate_stage2b_weight_point.py'),'--candidate',str(candidate),'--out','<new independent F directory>','--run'])
    (val/'validation_request.json').write_text(json.dumps(request,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    review['validation_bindings']=[metadata(p)for p in sorted(val.rglob('*'))if p.is_file()]
    assets=[]
    for p in sorted(candidate.rglob('*')):
        if p.is_file() and p.suffix.lower() in {'.hex','.mem','.bin','.mif'} and 'validation' not in p.relative_to(candidate).parts:
            old=SOURCE/p.relative_to(candidate)
            assert old.exists() and old.read_bytes()==p.read_bytes(),('asset differs',p)
            assets.append(metadata(p))
    review['bindings']+=assets
    review['asset_bindings_count']=len(assets)
    review['fixture_only_input_generator']=metadata(HERE/'tb_stage2b_point_ownership.sv')
    (candidate/'prepared_source_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(candidate=str(candidate),validation=str(val),files=len(files),xt_bytes=len(words),simulation_started=False),ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True);prepare(p.parse_args().candidate)
