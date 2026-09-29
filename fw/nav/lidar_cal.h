/* 雷达方向查找表：按内参在板上生成并写入 pc_project（每个像素的视线单位向量，12 位 Q0.11）。
 *
 * 内参来源：雷达每帧帧尾的 FrameInfo（xt_frame_rx 锁存到 LIDAR.LENS0..8）；上电时先用 M60 缺省值
 * （厂家 SDK cartesianTransform.cpp camParamsM60default）生成一张，收到雷达自带内参且不同时重新生成。
 * 算法与 model/lidar_lut.py 相同：列号翻转（数据第 q 列 = 标定第 W-1-q 列）、Brown-Conrady 不动点迭代
 * 去畸变、四角倒角切除（置零 = 无效）、归一化后换到本体系（前、左、上）。
 * 定点实现（Q24，除法只在每列/每行各一次），与 Python 模型相差不超过 1 LSB（sim/closed_loop/test_lidar_cal.py）。
 * 生成期间 lidar_cal_ready() 为 false，这段时间的雷达帧应丢弃（表只写了一部分）。
 */
#ifndef NAV_LIDAR_CAL_H
#define NAV_LIDAR_CAL_H

#include "nav_types.h"

#define LCAL_W 160
#define LCAL_H 60
#define LCAL_NPIX (LCAL_W * LCAL_H)
#define LCAL_CUT_R 30               /* 厂家上位机 M60 缺省 cut_corner=120，M60 取 1/4 */

typedef struct {
    uint32_t lens[9];               /* 当前表（或正在生成的表）所用内参：fx fy cx cy k1 k2 k3 p1 p2 的 float32 原始位 */
    uint32_t info_seq;              /* 上次处理过的 LIDAR.INFO_SEQ */
    uint16_t next;                  /* 下一个要生成的像素；LCAL_NPIX = 完成 */
    uint8_t  cut;                   /* 倒角半径（像素），0 = 不切 */
    bool     flip_u;
    bool     from_dev;              /* 当前内参来自雷达（否则为缺省值） */
    uint32_t n_gen;                 /* 完成生成的次数 */
    uint32_t t_start, gen_us;       /* 本次生成开始时刻、上次生成用时 */
    /* 生成用的定点参数 */
    int32_t  k[5];                  /* k1 k2 k3 p1 p2，Q24 */
    int32_t  xd[LCAL_W];            /* 数据第 q 列的畸变归一化横坐标，Q24 */
    int32_t  yd[LCAL_H];
} lidar_cal_t;

extern const uint32_t LCAL_M60_DEFAULT[9];

void lidar_cal_init(lidar_cal_t *lc);
/* 查看雷达是否带来了新内参；生成最多 budget 个像素。返回本次写入的像素数 */
int  lidar_cal_poll(lidar_cal_t *lc, int budget);
static inline bool lidar_cal_ready(const lidar_cal_t *lc) { return lc->next >= LCAL_NPIX; }
/* 立即按给定内参开始重新生成（lens 全零 = 缺省值） */
void lidar_cal_start(lidar_cal_t *lc, const uint32_t lens[9], bool from_dev);
/* 算一个像素（idx = 行 * W + 列），结果写入 out[3]（rx, ry, rz） */
void lidar_cal_pixel(const lidar_cal_t *lc, int idx, int16_t out[3]);

#endif
