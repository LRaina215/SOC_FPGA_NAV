/* 软核 SoC（rtl/soc/nav_soc.v）地址映射。
 *   0x0000_0000  片上 RAM（代码 + 数据，大小见 link.ld）
 *   0x4000_0000  导航子系统寄存器（fw/include/nav_regs_map.h 的偏移，按字访问）
 *   0x8000_0000  SoC 外设：控制台、LED、状态字
 */
#ifndef NAV_SOC_H
#define NAV_SOC_H

#include <stdint.h>

#define SOC_CON_TX   (*(volatile uint32_t *)0x80000000u)   /* 写 1 字节到控制台环形缓冲（PC 经调试串口读出） */
#define SOC_LED      (*(volatile uint32_t *)0x80000004u)   /* [3:0] 板上 LED4..7 */
#define SOC_STATUS   (*(volatile uint32_t *)0x80000008u)   /* 状态字（PC 可直接读，主循环计数等） */
#define SOC_RAM_SIZE (*(volatile uint32_t *)0x8000000Cu)   /* 只读：RAM 字节数 */

void con_putc(char c);
void con_puts(const char *s);
/* 精简 printf：%d %u %x %s %c，宽度与 0 填充（如 %08x、%5d） */
void con_printf(const char *fmt, ...);

#endif
