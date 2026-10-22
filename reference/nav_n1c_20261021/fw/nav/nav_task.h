/* 导航任务：三种运行模式（PDF 选题 3 架构）共用同一张代价图与局部规划器。
 *   GOTO   定点/路径点导航：A*（粗栅格）给出全局路径 -> 前视点 -> 局部规划
 *   TRACK  目标跟踪：RK3588 目标 -> 里程计系 -> 站位点 -> 同 GOTO；目标丢失转 SEARCH
 *   SEARCH 自主探索：方向打分选探索点 -> 同 GOTO；发现目标转 TRACK
 * nav_task_step() 以固定周期调用（默认 10 Hz），只通过 hal_* 访问硬件。
 */
#ifndef NAV_TASK_H
#define NAV_TASK_H

#include "nav_types.h"
#include "hal.h"
#include "pose.h"
#include "target.h"
#include "astar.h"
#include "gmap.h"
#include "local_planner.h"
#include "explore.h"
#include "proto.h"

typedef struct {
    int32_t  lookahead_mm;       /* 路径前视距离 */
    uint32_t replan_us;          /* A* 最短重规划间隔 */
    uint32_t blocked_us;         /* 持续无可行候选多久判为受阻 */
    uint32_t search_repick_us;   /* 探索点最长保持时间 */
    int32_t  track_hold_mm;      /* 跟踪：到达站位点后原地保持的容差 */
    int32_t  wp_tol_mm;          /* 中间路径点的到达容差（比终点宽：只是途经） */
    uint32_t stall_us;           /* 这么久位移不到 stall_mm 视为卡住，执行脱困动作 */
    int32_t  stall_mm;
    int32_t  near_goal_mm;       /* 此范围内卡住时保持停止/受阻，不冒充到达或盲目恢复 */
    uint32_t settle_us;          /* 最终目标内，实际速度持续稳定多久才声明到达 */
    uint32_t recover_us;         /* 脱困动作时长 */
} nav_task_cfg_t;

typedef struct {
    nav_task_cfg_t cfg;
    lp_cfg_t       lp_cfg;
    astar_cfg_t    as_cfg;       /* 局部图上的 A*（use_gmap=false 时） */
    astar_cfg_t    as_cfg_g;     /* 全局记忆栅格上的 A* */
    bool           use_gmap;
    explore_cfg_t  ex_cfg;

    nav_mode_t mode;
    uint8_t    state;
    int32_t    goal_x, goal_y;
    int32_t    wp_x[PROTO_MAX_WP], wp_y[PROTO_MAX_WP];
    int        wp_n, wp_i;
    bool       wp_loop;

    pose_est_t pose;
    target_t   tgt;
    explore_t  exp;

    uint8_t        map[NP_MAP_N * NP_MAP_N];
    hal_map_info_t mi;
    bool           have_map;
    gmap_t         gm;
    astar_path_t   path;             /* use_gmap：gmap 栅格坐标；否则局部图窗口坐标 */
    uint32_t       path_shift;       /* 规划时 gmap 的平移计数（平移后路径坐标失效） */
    uint16_t       path_seq;
    uint32_t       path_t_us;
    uint32_t       settle_t_us;
    bool           path_valid;
    bool           settling;
    lp_result_t    lp;
    nav_cmd_t      cmd;
    uint32_t       last_ok_us;       /* 最近一次找到可行候选 */
    uint32_t       search_t_us;
    bool           search_goal_valid;
    int32_t        look_x, look_y;   /* 当前前视点（调试/显示） */
    bool           turning;          /* 原地转向对准前视点中 */
    int32_t        prog_x, prog_y;   /* 卡住检测参考点 */
    uint32_t       prog_t_us;
    uint32_t       recover_until_us;
    bool           recovering;
    int8_t         recover_dir;      /* 1 = 倒车，0 = 原地转 */
    uint32_t       n_recover;
    int32_t        stuck_x, stuck_y; /* 上一次卡住的位置 */
    bool           hold;             /* 定位未就绪：原地保持（速度为零、保持平衡），模式与目标不变 */
    bool           slow;             /* 定位退化：限速一半 */
    int16_t        hold_wz;          /* 保持期间的原地转向角速度（对准换视角），0 = 静止 */
    bool           stale;            /* 位姿跳变后，seq 仍为 stale_seq 的代价图是旧坐标系下的，不用 */
    uint16_t       stale_seq;
} nav_task_t;

void nav_task_init(nav_task_t *nt);
/* 处理一条主机报文（rx_us 为到达时间） */
void nav_task_on_msg(nav_task_t *nt, const proto_msg_t *m, uint32_t rx_us);
/* 设置模式 / 目标（本地调用接口，报文处理也走这里） */
void nav_task_set_mode(nav_task_t *nt, nav_mode_t mode);
void nav_task_set_goal(nav_task_t *nt, int32_t x, int32_t y);
/* 周期调用：规划并下发速度指令 */
void nav_task_step(nav_task_t *nt, uint32_t now_us);
/* 生成状态报文内容 */
void nav_task_status(const nav_task_t *nt, proto_status_t *s, uint32_t now_us);

/* 坐标换算工具 */
void nav_mm_to_win(const hal_map_info_t *mi, int32_t x, int32_t y, int *wx, int *wy);
void nav_win_to_mm(const hal_map_info_t *mi, int wx, int wy, int32_t *x, int32_t *y);

#endif
