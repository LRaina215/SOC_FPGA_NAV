"""Same-input SAFE vectors from the transaction-level model, including wrap."""
from pathlib import Path
import random
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'closed_loop'))
from safe_model import SafeModel


def main(out):
    out=Path(out);rng=random.Random(37);m=SafeModel()
    rows=[]; t=0xffff0000
    for i in range(1200):
        t=(t+1000)&0xffffffff
        # A long intentional gap covers watchdog, with later equal-value writes.
        submit=i<30 or i%11==0 and not 100<=i<=400
        v=rng.randrange(-2000,2001);w=rng.randrange(-3000,3001)
        if 401<=i<410:v=w=0;submit=True
        m.enabled=i%47 not in (4,5)
        estop=i%53 in (10,11)
        if submit:m.submit((v,777,w),t)
        ov=m.tick(t,estop)
        rows.extend([t,int(submit),v,777,w,int(m.enabled),int(estop),ov[0],ov[2],int(m.wdog)])
    (out/'vectors.hex').write_text('\n'.join(f'{v&0xffffffff:08x}' for v in rows)+'\n')
    (out/'counts.txt').write_text(str(len(rows)//10))


if __name__=='__main__':main(sys.argv[1])
