/* 雷达口最小 TCP 客户端：启动 XT-M60 出流。
 * 实测（固件 V2.34.2，docs/2_interface/icd.md 第 2 节）：雷达上电不自动出流；UDP 只发给最近一次连它 TCP 的主机；TCP 断开后继续出流。
 * 所以收不到雷达帧时：ARP 找雷达 -> 连 7787 -> 发命令 1（开始测量，只出深度、连续）-> 等应答 -> 关闭（FIN），
 * 之后雷达把 UDP 发给本机；帧再次中断就整套重来。只实现这一条路径：不重排、不做窗口，丢包靠超时重发或整套重来。
 * 帧的收发走雷达口的原始帧通道（hal_lraw_*，NETL.RAW_*）。NETL.CTRL.LIDAR_TCP = 0 时不动作。 */
#ifndef XT_LINK_H
#define XT_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "hal.h"

#define XL_PORT          7787u
#define XL_BOOT_WAIT_US  2000000u   /* 上电后先等这么久（别人已经把雷达启动了就不用连） */
#define XL_STREAM_TMO_US 2000000u   /* 这么久没有新帧：重新启动雷达 */
#define XL_GRACE_US      5000000u   /* 发完开始命令后给雷达出流的时间（实测刚上电的雷达应答 OK 后约 3 s 才出第一帧） */
#define XL_ARP_RETRY_US  500000u
#define XL_ARP_TRIES     10          /* 雷达重启后约 6 s 才应答：5 s 没回就退避后重来 */
#define XL_SYN_RETRY_US  1000000u
#define XL_SYN_TRIES     3
#define XL_CMD_TMO_US    1500000u    /* 等开始命令的应答 */
#define XL_CMD_RETX_US   600000u     /* 数据段没被确认时重发 */
#define XL_FIN_TMO_US    1000000u
#define XL_BACKOFF_US    2000000u
#define XL_WATCH_EVERY_US 10000u     /* 正常出流时每 10 ms 看一次帧计数 */

enum { XL_WATCH = 0, XL_ARP = 1, XL_SYN = 2, XL_CMD = 3, XL_FIN = 4, XL_BACKOFF = 5 };

typedef struct {
    uint8_t  st;
    uint32_t t_st, t_try, t_poll;
    uint8_t  tries;
    uint32_t frames_last, t_frame;
    hal_netl_cfg_t net;
    uint8_t  peer_mac[6];
    uint16_t lport, ip_id;
    uint32_t iss, snd_nxt, rcv_nxt;
    bool     data_acked;
    uint8_t  rx[160];               /* 收到的 TCP 数据（找命令应答帧） */
    int      rx_n;
    bool     close_rst;             /* true：用 RST 关闭（缺省 FIN） */
    /* 统计 */
    uint32_t n_start, n_ok, n_fail, n_rx, n_tx;
    int      last_resp;             /* 最近一次开始命令的返回码（-1 = 没收到） */
} xt_link_t;

void xt_link_init(xt_link_t *x, uint32_t now_us);
void xt_link_poll(xt_link_t *x, uint32_t now_us);

#endif
