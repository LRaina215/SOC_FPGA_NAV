"""Re-map an unchanged frozen netlist with an explicit wide-BSRAM option.

Preserves old evidence, IO/clock constraints and PNR choices. Never programs.
"""
import argparse,json,re,shutil,subprocess
from pathlib import Path


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--wide-split',type=int,choices=[0,1],required=True)
    ap.add_argument('--replicate-resources',type=int,choices=[0,1],help='otherwise preserve source setting; no RTL or policy change')
    ap.add_argument('--place-option',type=int,choices=[0,1,2,3,4],default=1,help='Gowin placement mode; 1 routability, 2 timing priority')
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();src=a.source.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    for old,new in [('impl/gwsynthesis/g2_top.vg','g2_top.vg'),('g2.cst','g2.cst'),('g2.sdc','g2.sdc')]:
        shutil.copyfile(src/old,out/new)
    command=src/'impl/pnr/cmd.do';cmd=command.read_text(encoding='utf-8') if command.exists() else None
    if cmd:
        frequency=float(re.search(r'-global_freq\s+(\S+)',cmd)[1]);replicate=int(bool(re.search(r'(?m)^-replicate(?:_resources)?(?:\s|$)',cmd)))
    else:
        # A SYN-only candidate has not generated a PNR command. Read its actual
        # synthesis project and effective options, never an unverified hint.
        import xml.etree.ElementTree as ET
        options={x.get('type'):x.get('value') for x in ET.parse(src/'impl/gwsynthesis/g2_top.prj').getroot().findall('./OptionList/Option')}
        frequency=float(options['global_freq'])
        effective=(src/'effective_options.tcl').read_text(encoding='utf-8')
        replicate=int(re.search(r'^set_option\s+-replicate_resources\s+(\d+)',effective,re.M)[1])
    if a.replicate_resources is not None:replicate=a.replicate_resources
    tcl=['set_device -device_version C GW5AT-LV138PG484AC1/I0',
        f'add_file -type netlist {{{(out/"g2_top.vg").as_posix()}}}',
        f'add_file -type cst {{{(out/"g2.cst").as_posix()}}}',
        f'add_file -type sdc {{{(out/"g2.sdc").as_posix()}}}',
        'set_option -top_module g2_top','set_option -output_base_name g2_top',
        'set_option -global_freq '+str(frequency),
        'set_option -replicate_resources '+str(replicate),
        'set_option -convert_sdp32_36_to_sdp16_18 '+str(a.wide_split),
        'set_option -gen_verilog_sim_netlist 1',
        'set_option -place_option '+str(a.place_option),'set_option -route_option 0']
    tcl += ['set_option -'+x+' 1' for x in ['use_sspi_as_gpio','use_mspi_as_gpio','use_ready_as_gpio','use_done_as_gpio','use_i2c_as_gpio','use_cpu_as_gpio']]
    from preflight import validate
    tool=Path(r'D:\Gowin2\Gowin_V1.9.12.04_x64\IDE\bin\gw_sh.exe')
    effective=validate(tool,out,tcl,frequency,a.wide_split,{'replicate_resources':replicate,'place_option':a.place_option})
    tcl += ['run pnr'];(out/'run.tcl').write_text('\n'.join(tcl)+'\n',encoding='utf-8',newline='\n')
    (out/'manifest.json').write_text(json.dumps(dict(source=str(src),frozen_netlist=True,wide_split=a.wide_split,
        original_pnr_command=cmd,replicate_resources=replicate,place_option=a.place_option,effective_options=effective,
        source_manifest=json.loads((src/'manifest.json').read_text()),programmed=False),indent=2))
    with (out/'gw_sh.log').open('w',encoding='utf-8') as log:
        r=subprocess.run([str(tool),'run.tcl'],cwd=out,stdout=log,stderr=subprocess.STDOUT)
    s=(out/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
    print(s[-4000:])
    from collect_pnr_result import finalize_gate
    gate=finalize_gate(out)
    return r.returncode or ('ERROR' in s) or not gate


if __name__=='__main__':raise SystemExit(main())
