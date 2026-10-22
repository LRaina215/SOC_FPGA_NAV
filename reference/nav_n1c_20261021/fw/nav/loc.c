#include "loc.h"
#include "fixmath.h"

#define ANG_DEG 182             /* 1° ≈ 182 angle */

void loc_default_cfg(loc_cfg_t *c)
{
    c->step_mm = 50;
    c->step_ang = ANG_DEG;
    c->range_mm = 500;
    c->range_ang = 10 * ANG_DEG;
    c->reloc_mm = 500;
    c->reloc_ang = 10 * ANG_DEG;
    c->accept_q = 40;
    c->amb_d100 = 30;
    c->q_ok = 45;
    c->q_lost = 35;
    c->lost_us = 2000000;
    c->blind_us = 5000000;
    c->min_pts = 60;
    c->edge_n = 6;
    c->max_retry = 6;
}

void loc_init(loc_t *l)
{
    loc_cfg_t c = l->cfg;
    *l = (loc_t){0};
    l->cfg = c;
    l->st = LOC_ODOM;
}

static void align_reset(loc_t *l)
{
    l->phase = 0;
    l->nxy = 2 * (l->rng_mm / l->cfg.step_mm) + 1;
    l->na = 2 * (l->rng_ang / l->cfg.step_ang) + 1;
    l->k_total = (uint32_t)(l->nxy * l->nxy * l->na);
    l->k_next = 0;
    l->ntop = 0;
    l->n_pts = 0;
}

void loc_start_align(loc_t *l, int32_t range_mm, int32_t range_ang)
{
    l->rng_mm = range_mm > 0 ? range_mm : l->cfg.range_mm;
    l->rng_ang = range_ang > 0 ? range_ang : l->cfg.range_ang;
    l->retry = 0;
    l->st = LOC_ALIGN;
    align_reset(l);
}

bool loc_hold(const loc_t *l) { return l->st == LOC_ALIGN || l->st == LOC_LOST; }

int loc_quality(const loc_t *l)
{
    if (l->st == LOC_ODOM) return 0;
    if (l->st == LOC_ALIGN) return l->res_q;
    return l->q_ema / 256;
}

int loc_align_next(loc_t *l, hal_match_cand_t *cand, int kmax)
{
    int k = 0;
    if (l->phase == 0) {
        const int hxy = l->nxy / 2, ha = l->na / 2;
        while (k < kmax && l->k_next < l->k_total) {
            uint32_t i = l->k_next++;
            int ix = (int)(i % (uint32_t)l->nxy), iy = (int)((i / (uint32_t)l->nxy) % (uint32_t)l->nxy);
            int ia = (int)(i / (uint32_t)(l->nxy * l->nxy));
            cand[k].dx = (ix - hxy) * l->cfg.step_mm;
            cand[k].dy = (iy - hxy) * l->cfg.step_mm;
            cand[k].dyaw = (ia - ha) * l->cfg.step_ang;
            k++;
        }
        if (k > 0) return k;
        if (l->ntop == 0) { l->phase = 3; return 0; }
        l->phase = 1;
        l->fine = l->top[0];
    }
    if (l->phase == 1 || l->phase == 2) {
        const int32_t sm = l->phase == 1 ? 25 : 10, sa = l->phase == 1 ? 128 : 64;
        const loc_cand_t c0 = l->fine;
        for (int ia = -2; ia <= 2; ia++)
            for (int iy = -3; iy <= 3; iy++)
                for (int ix = -3; ix <= 3; ix++) {
                    cand[k].dx = c0.dx + ix * sm;
                    cand[k].dy = c0.dy + iy * sm;
                    cand[k].dyaw = c0.da + ia * sa;
                    k++;
                }
        l->phase++;
        return k;
    }
    return 0;
}

static bool near_c(const loc_cand_t *a, int32_t dx, int32_t dy, int32_t da)
{
    return nav_abs32(a->dx - dx) <= 150 && nav_abs32(a->dy - dy) <= 150 && nav_abs32(a->da - da) <= 3 * ANG_DEG;
}

static void top_insert(loc_t *l, loc_cand_t c)
{
    for (int i = 0; i < l->ntop; i++)
        if (near_c(&l->top[i], c.dx, c.dy, c.da)) {
            if (c.cost >= l->top[i].cost) return;
            /* 替换并重新排序（向前冒泡） */
            l->top[i] = c;
            for (int j = i; j > 0 && l->top[j].cost < l->top[j - 1].cost; j--) {
                loc_cand_t t = l->top[j]; l->top[j] = l->top[j - 1]; l->top[j - 1] = t;
            }
            return;
        }
    if (l->ntop == LOC_TOPN && c.cost >= l->top[LOC_TOPN - 1].cost) return;
    int j = l->ntop < LOC_TOPN ? l->ntop++ : LOC_TOPN - 1;
    l->top[j] = c;
    for (; j > 0 && l->top[j].cost < l->top[j - 1].cost; j--) {
        loc_cand_t t = l->top[j]; l->top[j] = l->top[j - 1]; l->top[j - 1] = t;
    }
}

void loc_align_feed(loc_t *l, const hal_match_cand_t *cand, const hal_match_res_t *res, int k)
{
    /* feed 在 next 之后调用：phase 已前进一步，按"刚生成的那一轮"处理 */
    for (int i = 0; i < k; i++) {
        loc_cand_t c = {cand[i].dx, cand[i].dy, cand[i].dyaw, res[i].cost, res[i].n_in};
        if (res[i].n_pts > l->n_pts) l->n_pts = res[i].n_pts;
        if (l->phase == 0) top_insert(l, c);
        else if (c.cost < l->fine.cost) l->fine = c;
    }
}

bool loc_align_finish(loc_t *l, uint32_t now)
{
    bool ok = false;
    if (l->ntop > 0 && l->n_pts >= l->cfg.min_pts) {
        l->res_q = (int)(l->fine.n_in * 100 / l->n_pts);
        l->res_q2 = l->ntop > 1 ? (int)(((int64_t)l->top[1].cost - l->top[0].cost) * 100 / l->n_pts) : 9999;
        ok = l->res_q >= l->cfg.accept_q && l->res_q2 >= l->cfg.amb_d100;
    } else {
        l->res_q = l->res_q2 = 0;
    }
    if (ok) {
        l->res_dx = l->fine.dx; l->res_dy = l->fine.dy; l->res_da = l->fine.da;
        l->st = LOC_OK;
        l->q_ema = l->res_q * 256;
        l->low = false;
        l->edge_cnt = 0;
        l->t_info = now;
        l->n_align++;
        return true;
    }
    l->n_align_fail++;
    l->want_turn = true;
    if (++l->retry >= l->cfg.max_retry) {
        l->st = LOC_LOST;               /* 等主机重新 SET_POSE；nav_app 过一段时间也会再试 */
        l->retry = 0;
    }
    align_reset(l);
    return false;
}

bool loc_track(loc_t *l, const pm_stat_t *st, const hal_match_res_t *res, bool solved, uint32_t now)
{
    if (l->st != LOC_OK && l->st != LOC_DEGRADED) return false;
    uint32_t np = res ? res[0].n_pts : 0;
    if (np >= l->cfg.min_pts) {
        l->t_info = now;
        int q = solved ? (int)(st->n_in_best * 100 / np) : 0;
        l->q_last = q;
        l->q_ema += (q * 256 - l->q_ema) / 4;
        l->edge_cnt = (solved && st->at_edge) ? l->edge_cnt + 1 : 0;
        if (l->q_ema < l->cfg.q_lost * 256) {
            if (!l->low) { l->low = true; l->low_since = now; }
        } else {
            l->low = false;
        }
    }
    bool blind = (uint32_t)(now - l->t_info) > l->cfg.blind_us;
    if (l->st == LOC_OK && (l->q_ema < l->cfg.q_ok * 256 || blind)) { l->st = LOC_DEGRADED; l->degr_since = now; }
    else if (l->st == LOC_DEGRADED && l->q_ema >= (l->cfg.q_ok + 5) * 256 && !blind) l->st = LOC_OK;
    if ((l->low && (uint32_t)(now - l->low_since) > l->cfg.lost_us) || l->edge_cnt >= l->cfg.edge_n) {
        l->st = LOC_LOST;
        l->n_lost++;
        l->low = false;
        l->edge_cnt = 0;
        return true;
    }
    return false;
}
