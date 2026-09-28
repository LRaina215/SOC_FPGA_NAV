#include "pose_match.h"
#include "fixmath.h"

void pm_default_cfg(pm_cfg_t *c)
{
    /* SLF：7×7 平移（25 mm）× 5 角度（0.7°）= 245 个候选，搜索范围 ±75 mm、±1.4° */
    c->enable = true;
    c->step_mm = 25;
    c->step_ang = 128;          /* 0.70°：正弦表（1024 项）的 2 格，候选角正好落在表格点上 */
    c->half = 3;
    c->half_ang = 2;
    c->use_kf = false;
    c->out_pen = 0;
    c->gain_q8 = 160;
    c->gain_z_q8 = 0;
    c->min_valid = 60;
    c->min_curv = 600;          /* 每千点；单面墙法向约 10000，沿墙方向（法向距离代价）噪声 < 400（离线标定） */
    c->min_curv_ang = 1500;
    c->kf_dist_mm = 800;
    c->kf_ang = 4551;           /* 25° */
    c->kf_min_valid = 400;
}

void pose_match_after_commit(const pm_cfg_t *c, const nav_pose_t *at, pm_stat_t *st)
{
    if (!c->enable || !c->use_kf) return;
    bool need = !st->have_kf ||
                fx_dist(at->x - st->kf_x, at->y - st->kf_y) > c->kf_dist_mm ||
                nav_abs32(fx_angle_diff(st->kf_yaw, at->yaw)) > c->kf_ang ||
                (st->n_runs > 0 && st->n_valid < c->kf_min_valid);
    if (!need) return;
    hal_match_keyframe(at->z);
    st->have_kf = true;
    st->kf_x = at->x; st->kf_y = at->y; st->kf_yaw = at->yaw;
    st->n_kf++;
}

/* 代价：截断残差和 + 落到未观测格的点（按半截断计），使"把点推到未知区"的候选不占便宜 */
static int32_t s_out_pen;
static int64_t score(const hal_match_res_t *r, uint32_t nmax)
{
    return (int64_t)r->cost + (int64_t)(nmax - r->n_valid) * s_out_pen;
}

bool pose_match_run(const pm_cfg_t *c, pose_est_t *pe, const nav_pose_t *at,
                    const int32_t r_q14[3][3], const int32_t t_mm[3], pm_stat_t *st)
{
    st->applied = false;
    if (!c->enable || (c->use_kf && !st->have_kf)) return false;
    static hal_match_cand_t cand[PM_MAX_CAND];
    static hal_match_res_t res[PM_MAX_CAND];
    const int h = c->half, w = 2 * h + 1, ha = c->half_ang, wa = 2 * ha + 1;
    if (w * w * wa > PM_MAX_CAND) return false;
    s_out_pen = c->out_pen;
    int k = 0;
    for (int ia = -ha; ia <= ha; ia++)
        for (int iy = -h; iy <= h; iy++)
            for (int ix = -h; ix <= h; ix++) {
                cand[k].dx = ix * c->step_mm;
                cand[k].dy = iy * c->step_mm;
                cand[k].dyaw = ia * c->step_ang;
                k++;
            }
    if (!hal_match_eval(r_q14, t_mm, at->x, at->y, cand, k, res)) return false;
    st->n_runs++;

    uint32_t nmax = 0;
    for (int i = 0; i < k; i++) if (res[i].n_valid > nmax) nmax = res[i].n_valid;
    st->n_valid = nmax;
    if (nmax < c->min_valid) return false;

    /* 最优候选：从零增量出发，只有严格更优才替换（代价曲面平坦/并列时不乱跳） */
    const int ic = (w * w * wa) / 2;                    /* 零增量候选 */
    int best = ic;
    int64_t bs = score(&res[ic], nmax);
    for (int i = 0; i < k; i++) {
        int64_t s = score(&res[i], nmax);
        if (s < bs) { bs = s; best = i; }
    }
    st->cost0 = (uint32_t)score(&res[ic], nmax);
    st->cost_best = (uint32_t)bs;

    /* 亚步长与退化判定。
     * 平移：在最优角度切片上取最优点的 3×3 邻域拟合二次曲面，Hessian 做 2×2 特征分解，
     *       只沿"曲率足够"的特征方向做牛顿步——沿墙方向（特征值小）不修正。逐轴判定在墙斜着时失效，
     *       且沿墙方向代价的起伏是量化噪声，逐轴曲率偶尔越过门限就会沿运动方向持续误修（闭环中 25 cm/s 的漂移）。
     * 角度：沿角度轴抛物线拟合（逐轴）。 */
    int bx = best % w - h, by = (best / w) % w - h, ba = best / (w * w) - ha;
    int32_t off[3] = {bx * 256, by * 256, 0};
    bool degen[3] = {false, false, false};
#define RES(ix, iy, ia) score(&res[((ia) + ha) * w * w + ((iy) + h) * w + ((ix) + h)], nmax)
    const int64_t thr = (int64_t)c->min_curv * nmax / 1000;
    if (bx > -h && bx < h && by > -h && by < h) {
        int64_t c0 = bs;
        int64_t cxp = RES(bx + 1, by, ba), cxm = RES(bx - 1, by, ba);
        int64_t cyp = RES(bx, by + 1, ba), cym = RES(bx, by - 1, ba);
        int64_t hxx = cxp + cxm - 2 * c0, hyy = cyp + cym - 2 * c0;
        int64_t hxy4 = RES(bx + 1, by + 1, ba) + RES(bx - 1, by - 1, ba) - RES(bx + 1, by - 1, ba) - RES(bx - 1, by + 1, ba);
        int64_t gx2 = cxp - cxm, gy2 = cyp - cym;             /* 2·梯度 */
        /* λ = (hxx+hyy)/2 ± sqrt(((hxx-hyy)/2)^2 + hxy^2)，hxy = hxy4/4 */
        int64_t dd = hxx - hyy;
        int64_t disc = (int64_t)fx_isqrt((uint64_t)(dd * dd + hxy4 * hxy4 / 4));   /* = 2·sqrt(...) */
        int64_t lam[2] = {(hxx + hyy + disc) / 2, (hxx + hyy - disc) / 2};
        int64_t ox = 0, oy = 0;                               /* 1/256 步 */
        int nstrong = 0;
        for (int e = 0; e < 2; e++) {
            if (lam[e] <= thr) continue;
            nstrong++;
            /* 特征向量 (hxy, λ - hxx)，退化时换 (λ - hyy, hxy)；按 4 倍放大避免 hxy4/4 截断 */
            int64_t vx = hxy4, vy = 4 * (lam[e] - hxx);
            if (vx * vx + vy * vy < 16) { vx = 4 * (lam[e] - hyy); vy = hxy4; }
            if (vx * vx + vy * vy < 16) { vx = e == 0 ? 4 : 0; vy = e == 0 ? 0 : 4; }
            int64_t n2 = vx * vx + vy * vy;
            int64_t pg = vx * gx2 + vy * gy2;                 /* v·(2g) */
            /* δ = -(v·g)/λ · v/|v|²，单位 1/256 步 */
            ox -= 128 * pg / lam[e] * vx / n2;
            oy -= 128 * pg / lam[e] * vy / n2;
        }
        off[0] = bx * 256 + nav_clamp32((int32_t)ox, -192, 192);
        off[1] = by * 256 + nav_clamp32((int32_t)oy, -192, 192);
        degen[0] = degen[1] = (nstrong == 0);
        if (nstrong == 0) { off[0] = 0; off[1] = 0; }
        else if (nstrong == 1) {
            /* 只有一个方向可信：最优格点本身沿弱方向的分量也不可信，只保留沿强方向的投影 */
            int e = lam[0] > thr ? 0 : 1;
            int64_t vx = hxy4, vy = 4 * (lam[e] - hxx);
            if (vx * vx + vy * vy < 16) { vx = 4 * (lam[e] - hyy); vy = hxy4; }
            if (vx * vx + vy * vy < 16) { vx = 4; vy = 0; }
            int64_t n2 = vx * vx + vy * vy, pr = (int64_t)off[0] * vx + (int64_t)off[1] * vy;
            off[0] = (int32_t)(pr * vx / n2);
            off[1] = (int32_t)(pr * vy / n2);
            degen[0] = nav_abs32((int32_t)vx) < nav_abs32((int32_t)vy);    /* 记录哪个轴是弱的（调试用） */
            degen[1] = !degen[0];
        }
    } else {
        /* 最优在平移搜索边界：不外推；向内一侧代价上升不明显的轴视为退化 */
        int q[2] = {bx, by};
        for (int ax = 0; ax < 2; ax++) {
            int cur = q[ax];
            if (cur != -h && cur != h) continue;
            int64_t in = ax == 0 ? RES(bx - (cur > 0 ? 1 : -1), by, ba) : RES(bx, by - (cur > 0 ? 1 : -1), ba);
            degen[ax] = (in - bs) * 2 < thr;
            if (degen[ax]) off[ax] = 0;
        }
    }
    {   /* 角度轴 */
        int64_t sm = ba > -ha ? RES(bx, by, ba - 1) : INT64_MAX;
        int64_t sp = ba < ha ? RES(bx, by, ba + 1) : INT64_MAX;
        const int64_t thr_a = (int64_t)c->min_curv_ang * nmax / 1000;
        if (ba == -ha || ba == ha) {
            int64_t in = (ba == -ha) ? sp : sm;
            degen[2] = (in - bs) * 2 < thr_a;
            off[2] = degen[2] ? 0 : ba * 256;
        } else {
            int64_t curv = sm + sp - 2 * bs;
            degen[2] = curv < thr_a;
            int32_t sub = (!degen[2] && curv > 0) ? (int32_t)((sm - sp) * 128 / curv) : 0;
            off[2] = degen[2] ? 0 : ba * 256 + nav_clamp32(sub, -128, 128);
        }
    }
#undef RES
    st->degen_x = degen[0];
    st->degen_y = degen[1];
    st->degen_yaw = degen[2];

    int32_t dx = (int32_t)((int64_t)off[0] * c->step_mm / 256);
    int32_t dy = (int32_t)((int64_t)off[1] * c->step_mm / 256);
    int32_t da = (int32_t)((int64_t)off[2] * c->step_ang / 256);
    /* 高度：最优候选（取最近格点）的内点平均有符号残差。点比地图高 -> 位姿 z 偏高 -> 向下修 */
    int32_t dz = 0;
    if (res[best].n_in > c->min_valid / 2) dz = -(int32_t)(res[best].sum_dz / (int32_t)res[best].n_in);

    st->dx = dx * c->gain_q8 / 256;
    st->dy = dy * c->gain_q8 / 256;
    st->dyaw = da * c->gain_q8 / 256;
    st->dz = dz * c->gain_z_q8 / 256;
    /* 候选平移是绕机器人位置旋转后的平移，直接作用在位姿上 */
    pose_correct_delta(pe, st->dx, st->dy, st->dz, st->dyaw);
    st->applied = true;
    st->n_applied++;
    return true;
}
