"""A short migration check: old snapshot and in-repo fork use the same dynamics.

This checks paths/reference resolution, not terrain acceptance or calibration.
"""
import argparse,json
from pathlib import Path
import numpy as np
from control4500 import load_runtime,DEFAULT_REPO,FROZEN_REPO,reference_path


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    result=[]
    for label,repo in [('frozen',FROZEN_REPO),('fork',DEFAULT_REPO)]:
        rt,policy=load_runtime(a.out/label,repo=repo);bot=rt.Robot(forward_bias=0.)
        bot.reset((0.,0.,.1305));rows=[]
        for _ in range(16):
            bot.step(policy);rows.append(np.concatenate((bot.data.qpos.copy(),bot.data.qvel.copy(),bot.actions.copy())))
        result.append(np.array(rows))
    assert np.array_equal(result[0],result[1]),np.max(abs(result[0]-result[1]))
    report=dict(pass_=True,policy_frames=16,physics_and_actions_equal=True,
        frozen=str(FROZEN_REPO),fork=str(DEFAULT_REPO),checkpoint=str(reference_path(DEFAULT_REPO/'model/wheel_velocity_original/model_4500.pt')),
        scope='source/asset path migration, 128ms native zero-command trajectory; not new terrain, drift or real-vehicle evidence')
    (a.out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))


if __name__=='__main__':main()
