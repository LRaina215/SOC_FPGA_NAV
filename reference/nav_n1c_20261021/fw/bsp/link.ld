/* 片上 RAM 从 0 开始；大小与 rtl/soc/nav_soc.v 的 RAM_KB 一致（构建脚本传入 RAM_SIZE） */
RAM_SIZE = DEFINED(RAM_SIZE) ? RAM_SIZE : 128K;
STACK_SIZE = 8K;
MEMORY { RAM (rwx) : ORIGIN = 0, LENGTH = RAM_SIZE }
ENTRY(_start)
SECTIONS
{
    .text : { KEEP(*(.text.start)) *(.text .text.*) } > RAM
    .rodata : { *(.rodata .rodata.* .srodata .srodata.*) } > RAM
    .data : { *(.data .data.* .sdata .sdata.*) } > RAM
    .bss (NOLOAD) : { . = ALIGN(4); _bss_start = .; *(.sbss .sbss.* .bss .bss.* COMMON) . = ALIGN(4); _bss_end = .; } > RAM
    _stack_top = ORIGIN(RAM) + LENGTH(RAM);
    ASSERT(_bss_end + STACK_SIZE <= _stack_top, "RAM 不够：代码 + 数据 + 8 KB 栈超出 RAM_SIZE")
}
