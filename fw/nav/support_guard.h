#ifndef NAV_SUPPORT_GUARD_H
#define NAV_SUPPORT_GUARD_H
#include "local_planner.h"
enum { SUPPORT_PASS, SUPPORT_UNKNOWN, SUPPORT_BLOCK, SUPPORT_FRAME };
/* Experimental static-ground contract: cap20mm, track270mm, old CAD projection
 * half-length34/half-width14mm; current actual tread width is not verified.
 * bootstrap=350 requires an independently cleared flat start disk >=400mm.
 * Remembered intervals remain explicitly remembered, not fresh observations. */
void support_reset(int bootstrap_mm);
void support_enable(bool enabled);
void support_frame_jump(void);
void support_note_cmd(const nav_cmd_t *cmd);
bool support_scan_needed(void);
void support_scan_finish(void);
int support_check(const nav_pose_t *p,const lp_cfg_t *cfg,const hal_map_info_t *mi,const nav_cmd_t *cmd);
int support_transition(int alo,int ahi,int blo,int bhi,int cap,int quant);
void support_diagnostics(uint32_t out[4]);
#endif
