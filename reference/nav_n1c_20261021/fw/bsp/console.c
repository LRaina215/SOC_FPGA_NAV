#include <stdarg.h>
#include "soc.h"

void con_putc(char c) { SOC_CON_TX = (uint8_t)c; }

void con_puts(const char *s)
{
    while (*s) con_putc(*s++);
}

static void put_num(uint32_t v, int base, int neg, int width, char pad)
{
    char buf[12];
    int n = 0;
    do { uint32_t d = v % (uint32_t)base; buf[n++] = (char)(d < 10 ? '0' + d : 'a' + d - 10); v /= (uint32_t)base; } while (v);
    if (neg) buf[n++] = '-';
    while (n < width) { con_putc(pad); width--; }
    while (n) con_putc(buf[--n]);
}

void con_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    for (; *fmt; fmt++) {
        if (*fmt != '%') { con_putc(*fmt); continue; }
        fmt++;
        char pad = ' ';
        int width = 0;
        if (*fmt == '0') { pad = '0'; fmt++; }
        while (*fmt >= '0' && *fmt <= '9') width = width * 10 + (*fmt++ - '0');
        switch (*fmt) {
        case 'd': { int32_t v = va_arg(ap, int32_t); put_num(v < 0 ? (uint32_t)-v : (uint32_t)v, 10, v < 0, width, pad); break; }
        case 'u': put_num(va_arg(ap, uint32_t), 10, 0, width, pad); break;
        case 'x': put_num(va_arg(ap, uint32_t), 16, 0, width, pad); break;
        case 's': con_puts(va_arg(ap, const char *)); break;
        case 'c': con_putc((char)va_arg(ap, int)); break;
        case '%': con_putc('%'); break;
        default: con_putc('?'); break;
        }
    }
    va_end(ap);
}
