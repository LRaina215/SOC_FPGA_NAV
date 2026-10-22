"""Per-sample digital contract vectors, fixed original4500 main parameters."""
from pathlib import Path
import random
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'closed_loop'))
from deployed_execution import WheelPid


def main(out):
    out=Path(out);p=WheelPid();rng=random.Random(19);v=[]
    pairs=[(600,0)]*120+[(0,0)]+[(600,-10)]*15+[(0,0)]
    pairs += [(rng.randrange(-2500,2501),rng.randrange(-150,151)) for _ in range(4000)]
    # Zero crossing must clear derivative/integral, and clamp/anti-windup must
    # remain symmetric around the Q12 rounding boundaries.
    for i in range(100):pairs += [(0,0),(1,0),(-1,0),(600,70),(-600,-70)]
    for target,feedback in pairs:
        duty=p.sample(target,feedback)
        v.extend((target,feedback,duty,p.reference,p.error,p.fault))
    (out/'vectors.hex').write_text('\n'.join(f'{x&0xffffffff:08x}' for x in v)+'\n')
    (out/'counts.txt').write_text(str(len(pairs)))


if __name__=='__main__':main(sys.argv[1])
