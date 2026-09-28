#include "pose_match.h"
#include "fixmath.h"

void pm_default_cfg(pm_cfg_t *c)
{
    c->enable = true;
    c->step_mm = 40;
    c->step_ang = 182;          /* 1° */
    c->half = 2;                /* 5×5×5 = 125 个候选 */
    c->gain_q8 = 180;
    c->gain_z_q8 = 128;
    c->min_valid = 200;
    c->min_curv = 40;
    c->kf_dist_mm = 800;
    c->kf_ang = 4551;           /* 25° */
    c->kf_min_valid = 400;
}

void pose_match_after_commit(const pm_cfg_t *c, const nav_pose_t *at, pm_stat_t *st)
{
    if (!c->enable) return;
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
static int64_t score(const hal_match_res_t *r, uint32_t nmax)
{
    return (int64_t)r->cost + (int64_t)(nmax - r->n_valid) * 75;
}

bool pose_match_run(const pm_cfg_t *c, pose_est_t *pe, const nav_pose_t *at,
                    const int32_t r_q14[3][3], const int32_t t_mm[3], pm_stat_t *st)
{
    st->applied = false;
    if (!c->enable || !st->have_kf) return false;
    static hal_match_cand_t cand[PM_MAX_CAND];
    static hal_match_res_t res[PM_MAX_CAND];
    const int h = c->half, w = 2 * h + 1;
    int k = 0;
    for (int ia = -h; ia <= h; ia++)
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

    int best = 0;
    int64_t bs = INT64_MAX;
    for (int i = 0; i < k; i++) {
        int64_t s = score(&res[i], nmax);
        if (s < bs) { bs = s; best = i; }
    }
    const int ic = (w * w * w) / 2;                     /* 零增量候选 */
    st->cost0 = (uint32_t)score(&res[ic], nmax);
    st->cost_best = (uint32_t)bs;

    /* 在最优点附近各轴做抛物线拟合：亚步长精度 + 退化判定 */
    int bx = best % w - h, by = (best / w) % w - h, ba = best / (w * w) - h;
    int32_t off[3] = {0, 0, 0};
    bool degen[3];
    for (int ax = 0; ax < 3; ax++) {
        int q[3] = {bx, by, ba};
        int cur = q[ax];
        if (cur == -h || cur == h) {                     /* 最优在边界：不外推，按边界值修正 */
            degen[ax] = false;
            off[ax] = cur * 256;
            continue;
        }
        q[ax] = cur - 1;
        int64_t sm = score(&res[(q[2] + h) * w * w + (q[1] + h) * w + (q[0] + h)], nmax);
        q[ax] = cur + 1;
        int64_t sp = score(&res[(q[2] + h) * w * w + (q[1] + h) * w + (q[0] + h)], nmax);
        int64_t curv = sm + sp - 2 * bs;
        /* 曲率按有效点数归一：每千点 */
        degen[ax] = curv * 1000 < (int64_t)c->min_curv * nmax;
        int32_t sub = 0;
        if (!degen[ax] && curv > 0) sub = (int32_t)((sm - sp) * 128 / curv);   /* 抛物线顶点，单位 1/256 步 */
        off[ax] = cur * 256 + nav_clamp32(sub, -128, 128);
        if (degen[ax]) off[ax] = 0;
    }
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
