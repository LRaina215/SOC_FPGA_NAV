"""Build the integration adapter from the maintained in-repo control source."""
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]
CONTROL=ROOT/'control/fpga/src'


def prepare(out):
    out=Path(out)
    shutil.copytree(CONTROL,out/'control')
    main=out/'control/main.v'
    main.write_bytes(main.read_text(encoding='utf-8').encode('utf-8'))
    cmd=['git','apply','--unsafe-paths','--directory='+(out/'control').as_posix()]
    patch=ROOT/'integration/g2/control_nav_adapter.patch'
    subprocess.run(cmd+['--check',str(patch)],cwd=ROOT,check=True)
    subprocess.run(cmd+[str(patch)],cwd=ROOT,check=True)
    shutil.copyfile(main,out/'control4500_core.v')
