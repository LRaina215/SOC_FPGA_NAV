"""RK3588 侧协议参考实现（兼容入口）：报文构造与解析已并入 tools/nav_host.py（PC 与 RK3588 通用）。
RK3588 上的 C++ 节点可直接照 nav_host.py / fw/nav/proto.c 移植。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nav_host import *  # noqa: E402,F401,F403
from nav_host import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
