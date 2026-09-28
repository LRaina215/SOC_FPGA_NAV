/* 软核入口。启动代码 / 链接脚本 / 中断在 fw/bsp（随软核型号提供）。 */
#include "nav_app.h"

int main(void)
{
    nav_app_init();
    for (;;) nav_app_poll();
    return 0;
}
