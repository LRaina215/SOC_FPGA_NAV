/* 软核自检程序：控制台、导航寄存器读写、乘除法、计时。仿真（tb_soc）与第一次上板用。 */
#include "soc.h"
#include "nav_regs_map.h"

#define NAV(o) (*(volatile uint32_t *)(0x40000000u + (o)))

int main(void)
{
    uint32_t id = NAV(NAV_SYS_ID), ver = NAV(NAV_SYS_VERSION);
    con_printf("selftest: SYS_ID=%08x VERSION=%08x RAM=%u KB\r\n", id, ver, SOC_RAM_SIZE / 1024);
    NAV(NAV_MATCH_LFO_X) = 0x12345678u;
    uint32_t r = NAV(NAV_MATCH_LFO_X);
    NAV(NAV_MATCH_LFO_X) = 0;
    volatile int32_t a = -123456, b = 789;
    int32_t p = a * b, q = a / b, m = a % b;
    uint32_t t0 = NAV(NAV_SYS_TIME_US);
    uint32_t c0;
    __asm__ volatile("rdcycle %0" : "=r"(c0));
    int ok = (r == 0x12345678u) && (p == -97406784) && (q == -156) && (m == -372);
    con_printf("rw %s  mul %d div %d rem %d  time %u us  cycle %u\r\n", r == 0x12345678u ? "ok" : "BAD", p, q, m, t0, c0);
    SOC_STATUS = ok ? 0x600DC0DEu : 0xBADu;
    con_printf("SELFTEST %s\r\n", ok ? "PASSED" : "FAILED");
    uint32_t t_last = t0, n = 0;
    for (;;) {
        uint32_t t = NAV(NAV_SYS_TIME_US);
        if ((uint32_t)(t - t_last) >= 500000u) {
            t_last = t;
            SOC_LED = ++n & 0xFu;
            con_printf("alive %u  t=%u ms\r\n", n, t / 1000u);
        }
    }
    return 0;
}
