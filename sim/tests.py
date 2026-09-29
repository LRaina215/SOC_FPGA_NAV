"""测试清单：名字 -> 源文件、顶层、可选 pre/post 脚本（路径相对 nav/）。
pre/post 脚本最后一个参数自动附加为该测试的构建目录。
"""

COMMON = [
    "rtl/common/nav_ram_sdp.v",
    "rtl/common/nav_ram_banked.v",
    "rtl/common/nav_fifo_ob.v",
    "rtl/common/nav_fifo.v",
    "rtl/common/nav_async_fifo.v",
    "rtl/common/nav_cdc.v",
    "rtl/common/nav_skid.v",
    "rtl/common/nav_timebase.v",
]

TESTS = {
    "tb_common": {
        "src": COMMON + ["rtl/common/tb/tb_common.sv"],
        "top": "tb_common",
    },
}

LIDAR = ["rtl/lidar/xt_frame_rx.v", "rtl/lidar/pc_project.v", "rtl/lidar/pc_transform.v", "rtl/lidar/pc_filter.v"]
MAP = ["rtl/map/elev_map.v", "rtl/map/trav_v3.v", "rtl/map/map_subsys.v"]

TESTS["tb_lidar_map"] = {
    "pre": ["sim/gen/gen_map.py"],
    "src": COMMON + LIDAR + MAP + ["rtl/map/tb/tb_lidar_map.sv"],
    "top": "tb_lidar_map",
}

MATCH = ["rtl/match/slf_extract.v", "rtl/match/slf_lf.v", "rtl/match/slf_match.v"]

TESTS["tb_slf"] = {
    "pre": ["sim/gen/gen_slf.py"],
    "copy": ["rtl/plan/sin_lut.hex"],
    "src": COMMON + MATCH + ["rtl/match/tb/tb_slf.sv"],
    "top": "tb_slf",
}

PLAN = ["rtl/plan/traj_eval.v", "rtl/plan/safety_guard.v"]

TESTS["tb_traj_eval"] = {
    "pre": ["sim/gen/gen_traj.py"],
    "copy": ["rtl/plan/sin_lut.hex"],
    "src": COMMON + PLAN + ["rtl/plan/tb/tb_traj_eval.sv"],
    "top": "tb_traj_eval",
}
TESTS["tb_safety_guard"] = {
    "src": COMMON + PLAN + ["rtl/plan/tb/tb_safety_guard.sv"],
    "top": "tb_safety_guard",
    "copy": ["rtl/plan/sin_lut.hex"],
}

NET = ["rtl/net/udp_rx.v", "rtl/net/udp_tx.v", "rtl/net/net_port.v", "rtl/net/host_mbx.v"]

TESTS["tb_net"] = {
    "pre": ["sim/gen/gen_net.py"],
    "post": ["sim/gen/gen_net.py", "post"],
    "src": COMMON + NET + ["rtl/net/tb/tb_net.sv"],
    "top": "tb_net",
}

IO = ["rtl/io/imu_uart.v", "rtl/io/uart_bus_bridge.v", "rtl/io/nav_bus_arb.v"]
TOP = ["rtl/regs/nav_regfile.v", "rtl/top/nav_subsystem.v"]
ALL_RTL = COMMON + LIDAR + MAP + MATCH + PLAN + NET + IO + TOP

TESTS["tb_nav_top"] = {
    "pre": ["sim/gen/gen_top.py"],
    "copy": ["rtl/plan/sin_lut.hex"],
    "src": ALL_RTL + ["rtl/top/tb/tb_nav_top.sv"],
    "top": "tb_nav_top",
}

SOC = ["rtl/common/nav_ram_sp.v", "rtl/soc/third_party/picorv32.v", "rtl/soc/nav_soc.v"]

TESTS["tb_soc"] = {
    "pre": ["fw/bsp/build_rv.py", "--app", "selftest", "--ram-kb", "16", "--copy-to"],
    "copy": ["rtl/plan/sin_lut.hex", "rtl/lidar/lut_default.hex"],
    "src": ALL_RTL + SOC + ["rtl/soc/tb/tb_soc.sv"],
    "top": "tb_soc",
}

TESTS["tb_uart_bridge"] = {
    "src": COMMON + ["rtl/io/uart_bus_bridge.v", "rtl/io/tb/tb_uart_bridge.sv"],
    "top": "tb_uart_bridge",
}

for scn in ("be_depth", "le_inter", "planar"):
    TESTS[f"tb_xt_rx_{scn}"] = {
        "pre": ["sim/gen/gen_xt_rx.py", scn],
        "src": COMMON + ["rtl/lidar/xt_frame_rx.v", "rtl/lidar/tb/tb_xt_frame_rx.sv"],
        "top": "tb_xt_frame_rx",
    }
