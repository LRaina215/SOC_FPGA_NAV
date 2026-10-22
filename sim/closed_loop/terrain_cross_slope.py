"""Lateral traversal from a declared initially terrain-aligned stance."""
from terrain_study import TerrainSpec,run_trial,OUT


def main():
    for angle in [4,8,12]:
        out=OUT/'cross_slope'/f'cross{angle}'
        out.parent.mkdir(exist_ok=True)
        spec=TerrainSpec('ramp_up',angle,yaw_deg=90.,start_x=.8,start_y=-.4,
                         align_start=True,lateral_test=True,seconds=18.)
        run_trial(spec,out)


if __name__=='__main__':main()
