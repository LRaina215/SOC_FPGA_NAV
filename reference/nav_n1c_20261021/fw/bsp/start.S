/* PicoRV32 启动：设栈、清 .bss、进 main。程序与数据都在片上 RAM（由 PC 经调试串口装载或随比特流初始化），无需搬移 .data。 */
    .section .text.start
    .global _start
_start:
    la   sp, _stack_top
    la   a0, _bss_start
    la   a1, _bss_end
1:  bgeu a0, a1, 2f
    sw   zero, 0(a0)
    addi a0, a0, 4
    j    1b
2:  call main
3:  j    3b
