"""上板后的运行期初始化：把固件目前不会自己写、但实机必须设置的寄存器一次写好（掉电或按复位键后要重新运行）。

用法:
  python tools/board_init.py COM11                         缺省：50 MHz 核心时钟、IMU 115200、雷达离地 200 mm
  python tools/board_init.py COM11 --clk-mhz 100           板级打开 NAV_USE_PLL（100 MHz）时
  python tools/board_init.py COM11 --imu-baud 460800 --mount-z 165
  python tools/board_init.py COM11 --no-safe               只做感知测试：速度输出保持为 0
  python tools/board_init.py COM11 --dry-run               只打印要写的寄存器，不连板子

写入内容:
  LIDAR_CTRL     复位值（厂家 SDK 格式：小端、帧尾信息自动用）| WAIT_POSE：雷达帧等固件按当前位姿放行
                 （导航固件上电也会自己置 WAIT_POSE；这里再写一次，方便自检版或排查）
  IMU_BAUD_DIV   核心时钟 / 波特率（复位值 868 是按 100 MHz、115200 算的）
  MATCH_MOUNT_Z  雷达光心离地高度 mm，须与 fw/nav/nav_task.c 中 pose_init 的安装高度一致
  SAFE_LIM_V/W   速度 / 角速度上限（首次上车用低值）
  SAFE_CTRL      ENABLE：允许速度指令输出到底盘接口（复位值 0 = 输出恒为 0）
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import nav_regs_map as rm  # noqa: E402


def plan(a):
    w = [
        ("LIDAR_CTRL", a.lidar_ctrl if a.lidar_ctrl is not None else (rm.RESET["LIDAR_CTRL"] | 0x8)),
        ("IMU_BAUD_DIV", round(a.clk_mhz * 1e6 / a.imu_baud)),
        ("MATCH_MOUNT_Z", a.mount_z),
        ("SAFE_LIM_V", (a.vy_max << 16) | a.v_max),
        ("SAFE_LIM_W", a.w_max),
        ("SAFE_CTRL", 0 if a.no_safe else 1),
    ]
    for n, _ in w:
        if n not in rm.REGS:
            raise KeyError(f"寄存器表里没有 {n}（重新运行 python tools/gen_config.py？）")
    return w


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="")
    ap.add_argument("--clk-mhz", type=float, default=50.0, help="核心时钟 MHz（无 PLL = 50）")
    ap.add_argument("--imu-baud", type=int, default=115200)
    ap.add_argument("--mount-z", type=int, default=155, help="雷达光心离地高度 mm（= 腿长 L0 + 25，tools/robot_calib.py）")
    ap.add_argument("--v-max", type=int, default=300, help="前进速度上限 mm/s（首次上车建议 150–300）")
    ap.add_argument("--vy-max", type=int, default=0, help="横移速度上限 mm/s（两轮车为 0）")
    ap.add_argument("--w-max", type=int, default=800, help="角速度上限 mrad/s")
    ap.add_argument("--lidar-ctrl", type=lambda s: int(s, 0), default=None,
                    help="直接指定 LIDAR_CTRL（缺省 = 复位值 | WAIT_POSE；位：ENABLE=1 HDR_LE=2 PIX_LE=4 WAIT_POSE=8 AUTO_INFO=16）")
    ap.add_argument("--no-safe", action="store_true", help="不打开速度输出")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    writes = plan(a)
    if a.dry_run or not a.port:
        for n, v in writes:
            print(f"{n:14s} 0x{rm.REGS[n]:04X} <- 0x{v:08X} ({v})")
        return 0
    import nav_uart
    u = nav_uart.NavUart(a.port)
    ident = u.rd(rm.REGS["SYS_ID"])
    if ident != 0x4E415630:
        print(f"SYS_ID = 0x{ident:08X}，不是导航子系统（应为 0x4E415630），检查串口号 / 比特流")
        return 1
    bad = 0
    for n, v in writes:
        u.wr(rm.REGS[n], v)
        r = u.rd(rm.REGS[n])
        ok = r == v
        bad += not ok
        print(f"{n:14s} <- 0x{v:08X}  回读 0x{r:08X}  {'OK' if ok else '不一致'}")
    print("初始化完成" if not bad else f"{bad} 个寄存器回读不一致")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
