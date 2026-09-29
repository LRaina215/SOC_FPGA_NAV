/* 软核入口（PicoRV32 SoC，启动代码 / 链接脚本 / 控制台在 fw/bsp）。 */
#include "nav_app.h"
#include "hal.h"
#include "soc.h"

int main(void)
{
    con_printf("\r\nnav fw " __DATE__ " " __TIME__ "  RAM %u KB\r\n", SOC_RAM_SIZE / 1024);
    nav_app_init();
    uint32_t t_log = hal_time_us(), loops = 0, max_us = 0, t_prev = t_log, n_gen = 0;
    for (;;) {
        nav_app_poll();
        uint32_t now = hal_time_us(), d = now - t_prev;
        t_prev = now;
        if (d > max_us) max_us = d;
        loops++;
        SOC_STATUS = loops;
        if (g_app.lcal.n_gen != n_gen) {
            n_gen = g_app.lcal.n_gen;
            con_printf("lidar LUT ready (%s lens), %u ms\r\n", g_app.lcal.from_dev ? "device" : "M60 default",
                       g_app.lcal.gen_us / 1000u);
        }
        if ((uint32_t)(now - t_log) >= 1000000u) {
            t_log = now;
            const nav_task_t *nt = &g_app.nt;
            con_printf("t=%u s  loop %u/s max %u us  frames %u msgs %u  mode %d state %d  loc %d  pose %d,%d mm  lut %d\r\n",
                       now / 1000000u, loops, max_us, g_app.n_frames, g_app.n_msgs, nt->mode, nt->state,
                       nav_app_have_prior() ? (int)g_app.loc.st : 0, nt->pose.cur.x, nt->pose.cur.y,
                       (int)lidar_cal_ready(&g_app.lcal));
            SOC_LED = (SOC_LED + 1u) & 0xFu;
            loops = 0;
            max_us = 0;
        }
    }
    return 0;
}
