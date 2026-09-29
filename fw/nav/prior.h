/* 先验地图包接收（有图模式）：MAP_BEGIN / MAP_DATA / MAP_END 报文按偏移顺序流式解析 .nvmp 包
 * （格式见 model/prior_map.py），不在软核内存里存整包：
 *   结构图条目 {u16 槽地址, u24 槽字} 直接写进 FPGA 结构图 RAM（hal_prior_write）；
 *   代价图字节写进记忆栅格的先验层（gmap_prior_put）。
 * 体的 CRC32 在接收时累计，MAP_END 时校验；通过后 hal_prior_enable 进入有图模式。
 * 丢包/乱序：只接受 offset == 下一个期望偏移的数据，其余忽略；每包回 MAP_ACK（下一个期望偏移），主机据此重发。
 */
#ifndef NAV_PRIOR_H
#define NAV_PRIOR_H

#include "nav_types.h"
#include "gmap.h"

#define PRIOR_HDR_LEN 40

enum { PRIOR_NONE = 0, PRIOR_RECV = 1, PRIOR_READY = 2, PRIOR_ECRC = 3, PRIOR_EFMT = 4 };

typedef struct {
    uint8_t  state;
    uint32_t total, next;
    uint8_t  hdr[PRIOR_HDR_LEN];
    bool     hdr_ok;
    int32_t  ox, oy, gox, goy;          /* 结构窗口原点（5 cm 格）、代价图原点（20 cm 格） */
    uint16_t gnx, gny;
    uint32_t n_struct, body_len, crc_exp, crc;
    uint8_t  ent[5];
    int      ent_n;
    uint32_t body_i;                    /* 已处理的体字节数 */
    uint32_t n_words;
} prior_rx_t;

void prior_rx_init(prior_rx_t *r);
void prior_rx_begin(prior_rx_t *r, gmap_t *g, uint32_t total);
/* 返回是否接受（offset 正确） */
bool prior_rx_data(prior_rx_t *r, gmap_t *g, uint32_t offset, const uint8_t *d, int n);
/* 校验并生效；返回最终状态 */
uint8_t prior_rx_end(prior_rx_t *r, gmap_t *g);
void prior_rx_clear(prior_rx_t *r, gmap_t *g);
uint32_t prior_crc32_update(uint32_t crc, const uint8_t *d, int n);

#endif
