"""Export the selected, frozen SLF128 map profile to a standalone offline tool folder."""
import argparse
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
TOOL_HELP = '''Selected SLF128 prior-map converter for the current FPGA profile.

Input: map-frame XYZ/PCD/PLY/NPY, metres, z up. Explicit --bounds is required.
Output: NVMPv3, 128x128 at5cm, 18bit structure words; cost grid at10cm.
The enclosure must fit the6.4m structure window. ZHI350 is required.
--step20 and --slope12 are provisional vehicle assumptions, not measured ability.
Build: map_tool.py build field.xyz -o field.nvmp --bounds=0,-1.5,5,1.5
Show: map_tool.py show field.nvmp -o field.png
This tool neither records lidar nor performs SLAM or FPGA upload.
'''


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Selected profile layout changed: ' + old[:70])
    return text.replace(old, new, 1)


def selected_tool(t):
    end = t.index('"""', 3) + 3
    t = '"""' + TOOL_HELP + '"""' + t[end:]
    t = replace_once(t, 'import prior_map as pm  # noqa: E402', '''import prior_map as pm  # noqa: E402
if (pm.SN, pm.PKG_VER, pm.GM_MAX, pm.DS, pm.GEOMETRY_FLAGS) != (128, 3, 64, 2, 0x0107):
    raise ValueError("This kit requires the selected SLF128/18bit/NVMPv3/10cm profile")''')
    t = replace_once(t, 'gx, gy = d["gox"] * 0.2, d["goy"] * 0.2', 'gc = pm.DS * 0.05\n    gx, gy = d["gox"] * gc, d["goy"] * gc')
    t = replace_once(t, 'gx + d["gnx"] * 0.2, gy, gy + d["gny"] * 0.2', 'gx + d["gnx"] * gc, gy, gy + d["gny"] * gc')
    t = replace_once(t, '全局代价图（20 cm；黑=致命，白=未知）', '全局代价图（10 cm；高代价为障碍，空白为未知）')
    t = replace_once(t, 'default=100, help="可跨越台阶 mm"', 'default=20, help="可跨越台阶 mm（暂定，待实测）"')
    t = replace_once(t, 'default=20.0, help="可通行坡度 °"', 'default=12.0, help="可通行坡度 °（暂定，待实测）"')
    return replace_once(t, 'if a.cmd == "show":', 'if a.zhi != 350: raise ValueError("The current deployed Web/FPGA profile requires ZHI350")\n    if a.cmd == "show":')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profile', type=Path, default=Path('F:/nav_integration_g2_20261023/K_timing_syn49/host_profile'))
    ap.add_argument('--out', type=Path, default=Path('D:/tools/nav_slf128_mapkit_20261027'))
    a = ap.parse_args()
    files = ['model/slf_match.py', 'model/prior_map.py', 'model/nav_fixed.py', 'model/nav_params.py', 'tools/map_tool.py']
    for rel in files:
        if not (a.profile / rel).is_file():
            raise FileNotFoundError(a.profile / rel)
    profile = (a.profile / 'model/prior_map.py').read_text(encoding='utf-8-sig')
    if 'PROFILE_ID = 0x33580001' not in profile or 'PKG_VER = 3' not in profile:
        raise ValueError('The source is not the selected NVMPv3 profile')
    tool = selected_tool((a.profile / 'tools/map_tool.py').read_text(encoding='utf-8-sig'))
    a.out.mkdir(parents=True, exist_ok=False)
    for rel in files:
        dest = a.out / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(a.profile / rel, dest)
    p = a.out / 'tools/map_tool.py'
    p.write_text(tool, encoding='utf8')
    shutil.copyfile(ROOT / 'tools/field_to_cloud.py', a.out / 'tools/field_to_cloud.py')
    shutil.copyfile(ROOT / 'config/field3x5_survey.example.json', a.out / 'field3x5_survey.example.json')
    record = {'profile_source': str(a.profile.resolve()), 'output': str(a.out.resolve()),
              'map_version': 3, 'header_bytes': 48, 'flags': '0x0107', 'profile': '0x33580001',
              'SLF_cells': [128, 128], 'SLF_resolution_m': .05, 'cost_resolution_m': .1, 'ZHI_mm': 350,
              'model_files_changed': False, 'display_change': 'Only fix coarse-map plotted scale from20cm to actual10cm',
              'tool_defaults': {'step_mm': 20, 'slope_deg': 12, 'vehicle_capability_measured': False},
              'example_is_real_survey': False, 'FPGA_access': False}
    (a.out / 'provenance.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    print(json.dumps(record, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
