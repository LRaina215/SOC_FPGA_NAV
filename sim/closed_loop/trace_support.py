"""Read-only raw support-query diagnostics for a frozen runner.

Keeps the first 24 missing/wide queries per 100ms; never alters callback data.
The shortened replay is diagnostic and is not an acceptance case.
"""
import argparse
import ctypes
import json
from pathlib import Path
import sys


def main():
    ap=argparse.ArgumentParser(add_help=False)
    ap.add_argument('--runner-dir',type=Path,required=True)
    ap.add_argument('--support-out',type=Path,required=True)
    a,rest=ap.parse_known_args()
    sys.path.insert(0,str(a.runner_dir.resolve()))
    import run_cl
    original=ctypes.CDLL
    a.support_out.parent.mkdir(parents=True,exist_ok=True)
    callbacks=[];now=0;bucket=-1;seen=set()
    cbtype=ctypes.CFUNCTYPE(ctypes.c_int,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32))
    with a.support_out.open('w',encoding='utf-8') as f:
        def loader(*args,**kw):
            d=original(*args,**kw)
            if not hasattr(d,'py_set_support_cb'):return d
            set_cb=d.py_set_support_cb;set_time=d.py_set_time
            def tick(t):
                nonlocal now,bucket,seen
                now=t.value if hasattr(t,'value') else t
                if now//100000!=bucket:bucket=now//100000;seen=set()
                return set_time(t)
            def wrap(cb):
                @cbtype
                def probe(x,y,out):
                    ok=cb(x,y,out);raw=list(out[:6]) if ok else None
                    key=(x,y)
                    if (not ok or raw[1]-raw[0]>18) and key not in seen and len(seen)<24:
                        seen.add(key);f.write(json.dumps(dict(t_us=now,x=x,y=y,raw=raw))+'\n');f.flush()
                    return ok
                callbacks.append(probe);return set_cb(probe)
            d.py_set_time=tick;d.py_set_support_cb=wrap
            return d
        ctypes.CDLL=loader;sys.argv=[run_cl.__file__,*rest]
        try:return run_cl.main()
        finally:ctypes.CDLL=original


if __name__=='__main__':raise SystemExit(main())
