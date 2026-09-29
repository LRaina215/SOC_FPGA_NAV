"""有图模式固件单元测试（真实固件动态库）：先验地图包下发（分块、乱序重发、CRC 错）、状态应答、SET_POSE。

用法: python sim/closed_loop/test_prior.py
"""
import ctypes
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(ROOT / "model"), str(ROOT / "tools")]

import nav_host as nh  # noqa: E402
import run_cl  # noqa: E402

errs = 0


def check(cond, what):
    global errs
    if not cond:
        errs += 1
        print("失败:", what)


def main():
    run_cl.build_dll()
    fw = run_cl.Firmware()
    writes = []
    CTL = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32)

    @CTL
    def ctl(op, a, b, c):
        writes.append((op, a & 0xFFFFFFFF, b & 0xFFFFFFFF, c))
    fw.py_set_ctl_cb(ctl)
    t = [1000]

    def push(b):
        buf = (ctypes.c_uint8 * len(b)).from_buffer_copy(b)
        fw.py_push_host(buf, len(b), ctypes.c_uint32(t[0]))
        t[0] += 1000
        fw.py_set_time(ctypes.c_uint32(t[0]))
        fw.py_poll()

    def ack():
        out = (ctypes.c_uint8 * 300)()
        n = fw.py_get_host_out(out)
        return nh.parse(bytes(out[:n]))

    loc = (ctypes.c_int32 * 12)()
    pkg = run_cl.get_prior("terrain", "auto")
    import prior_map
    info = prior_map.parse_package(pkg)

    # 1. 正常下发，第 3 块故意先发第 4 块（应被忽略，ACK 仍指向第 3 块）
    msgs = nh.map_messages(pkg)
    push(msgs[0])
    a = ack()
    check(a and a["type"] == "map_ack" and a["next"] == 0 and a["state_code"] == 1, f"BEGIN 应答 {a}")
    for i, m in enumerate(msgs[1:-1]):
        if i == 2:
            push(msgs[1 + 3])
            a = ack()
            check(a["next"] == 2 * nh.CHUNK, f"乱序块应被忽略，期望 next={2 * nh.CHUNK} 实得 {a}")
        push(m)
    a = ack()
    check(a["next"] == len(pkg), f"数据收完 next={a['next']} / {len(pkg)}")
    push(msgs[-1])
    a = ack()
    check(a["state_code"] == 2, f"END 后应已加载：{a}")
    ws = [w for w in writes if w[0] == 2]
    check(len(ws) == len(info["slot"]), f"槽字写入 {len(ws)} 条，期望 {len(info['slot'])}")
    check(all(w[1] == int(s) and w[2] == int(v) for w, s, v in zip(ws, info["slot"], info["word"])), "槽字内容")
    en = [w for w in writes if w[0] == 3]
    check(en and en[-1][1] == 1 and en[-1][2] == info["ox"] & 0xFFFFFFFF, f"有图开关 {en[-1:]}")
    fw.py_get_loc(loc)
    check(loc[8] == 2 and loc[0] == 0, f"加载后等待初始位姿 prior_state={loc[8]} loc={loc[0]}")

    # 2. SET_POSE -> 对准中
    push(nh.set_pose(-1000, -300, 0))
    fw.py_get_loc(loc)
    check(loc[0] == 1, f"SET_POSE 后应进入对准 loc={loc[0]}")
    st = (ctypes.c_int32 * 12)()
    fw.py_get_state(st)
    check(st[0] == -1000 and st[1] == -300, f"位姿跳到 SET_POSE：({st[0]}, {st[1]})")

    # 3. CRC 错
    bad = bytearray(pkg)
    bad[-5] ^= 0xFF
    for m in nh.map_messages(bytes(bad)):
        push(m)
    a = ack()
    check(a["state_code"] == 3, f"CRC 错应报 3：{a}")
    fw.py_get_loc(loc)
    check(loc[8] == 3 and loc[0] == 0, "CRC 错后回到无图")

    # 4. MAP_CLEAR
    push(nh.map_clear())
    a = ack()
    check(a["state_code"] == 0, f"MAP_CLEAR 应答 {a}")
    print("PRIOR TEST", "PASSED" if errs == 0 else f"FAILED errs={errs}")
    return 1 if errs else 0


if __name__ == "__main__":
    sys.exit(main())
