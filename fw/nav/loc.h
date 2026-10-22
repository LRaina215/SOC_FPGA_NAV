/* 有图模式定位管理：初始对准 / 连续跟踪质量 / 丢失与重定位。
 *
 * 匹配硬件与无图模式相同（SLF 打分，每批至多 256 个候选、约 40 ms），区别只在候选怎么生成、结果怎么用：
 *   对准（SET_POSE 之后、丢失之后）：机器人原地保持，同一帧结构点上连续打多批候选——
 *     粗搜索：±range 平移（50 mm 步长）× ±range_ang 航向（1° 步长）全枚举，不做门控；
 *       同时保留若干个互相分开（> 150 mm 或 > 3°）的最优峰，用于判断歧义；
 *     细化两轮：最优峰周围 7×7×5 网格，步长 25 mm / 0.7°，再 10 mm / 0.35°。
 *     接受条件：内点率（残差 <= T_IN 的点 / 结构点）够高，且第二峰的平均代价明显更高
 *     （仿真统计：错误对准的第二峰代价差 < 0.15、正确对准多在 0.3 以上，门限 0.3）；
 *     否则原地转 ~35° 换个视角再试（单帧视野里只有平行结构时沿结构方向不可观）。
 *   跟踪：每帧 245 候选 + Hessian 退化判定（pose_match），先验图固定不并图；
 *     质量 = 最优候选的近邻命中点 / 本帧结构点（门控打开：先验图里没有的物体不算命中）。
 *     质量的指数平均低于门限 -> 退化（限速）；长时间很低或最优候选连续落在搜索边界 -> 丢失，
 *     以当前估计为中心自动重定位。
 */
#ifndef NAV_LOC_H
#define NAV_LOC_H

#include "nav_types.h"
#include "hal.h"
#include "pose_match.h"

typedef enum { LOC_ODOM = 0, LOC_ALIGN = 1, LOC_OK = 2, LOC_DEGRADED = 3, LOC_LOST = 4 } loc_state_t;

#define LOC_TOPN 6

typedef struct {
    int32_t  step_mm, step_ang;          /* 粗搜索步长 */
    int32_t  range_mm, range_ang;        /* SET_POSE 未给范围时的缺省半宽 */
    int32_t  reloc_mm, reloc_ang;        /* 丢失后重定位的搜索半宽 */
    int      accept_q;                   /* 对准接受：内点率 % 下限 */
    int      amb_d100;                   /* 歧义：第二峰与最优的平均代价差（1/16 格 × 100 / 点）低于它视为歧义 */
    int      q_ok, q_lost;               /* 跟踪：质量 % 门限（指数平均） */
    uint32_t lost_us;                    /* 质量低于 q_lost 持续这么久 -> 丢失 */
    uint32_t blind_us;                   /* 这么久没有有效帧（看不到结构）-> 退化 */
    uint32_t min_pts;                    /* 有效帧最少结构点 */
    int      edge_n;                     /* 最优候选连续落在搜索边界这么多帧 -> 丢失 */
    int      max_retry;
} loc_cfg_t;

typedef struct { int32_t dx, dy, da; uint32_t cost, n_in; } loc_cand_t;

typedef struct {
    loc_cfg_t   cfg;
    loc_state_t st;
    /* 对准 */
    int32_t  rng_mm, rng_ang;
    int      phase;                      /* 0 粗搜索，1、2 细化，3 结束 */
    int      nxy, na;
    uint32_t k_total, k_next;
    loc_cand_t top[LOC_TOPN];
    int      ntop;
    loc_cand_t fine;                     /* 细化轮的最优 */
    uint32_t n_pts;
    int      retry;
    int32_t  res_dx, res_dy, res_da;     /* 对准结果（相对该帧提交位姿） */
    int      res_q, res_q2;              /* 最优内点率 %、第二峰代价差（× 100 / 点） */
    bool     want_turn;                  /* 对准失败：请求原地转向换视角 */
    /* 跟踪 */
    int32_t  q_ema;                      /* % × 256 */
    int      q_last;
    uint32_t low_since, t_info, degr_since;
    bool     low;
    int      edge_cnt;
    uint32_t n_align, n_align_fail, n_lost;
} loc_t;

void loc_default_cfg(loc_cfg_t *c);
void loc_init(loc_t *l);
/* 开始对准（range 为 0 用缺省）；之后每帧由 nav_app 驱动 next/feed/finish */
void loc_start_align(loc_t *l, int32_t range_mm, int32_t range_ang);
/* 生成下一批候选，返回个数；0 表示本帧搜索完毕，调用 loc_align_finish */
int  loc_align_next(loc_t *l, hal_match_cand_t *cand, int kmax);
void loc_align_feed(loc_t *l, const hal_match_cand_t *cand, const hal_match_res_t *res, int k);
void loc_align_feed_generated(loc_t *l,const hal_match_res_t *res,int k);
/* 判定：接受返回 true（res_* 有效，状态 -> OK）；否则准备下一帧重试（重试用尽 -> LOST） */
bool loc_align_finish(loc_t *l, uint32_t now);
/* 跟踪一帧；返回 true 表示刚判为丢失（调用方应开始重定位） */
bool loc_track(loc_t *l, const pm_stat_t *st, const hal_match_res_t *res, bool solved, uint32_t now);
/* 需要原地保持（对准中 / 丢失） */
bool loc_hold(const loc_t *l);
int  loc_quality(const loc_t *l);

#endif
