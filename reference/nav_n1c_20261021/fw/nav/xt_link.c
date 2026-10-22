#include "xt_link.h"

#include <string.h>

#define TCP_FIN 0x01u
#define TCP_SYN 0x02u
#define TCP_RST 0x04u
#define TCP_PSH 0x08u
#define TCP_ACK 0x10u

static const uint8_t XT_START[4] = {0x7E, 0xFF, 0xAA, 0x55};
static const uint8_t XT_END[4] = {0xFF, 0x7E, 0x55, 0xAA};
/* 命令 1（开始测量）：数据 {1 = 只出深度, 1 = 连续}；len = 1 + 数据 2 + 固定尾 00 01（tools/xt_config.py 同格式） */
static const uint8_t XT_CMD_START[17] = {0x7E, 0xFF, 0xAA, 0x55, 5, 0, 0, 0, 1, 1, 1, 0x00, 0x01, 0xFF, 0x7E, 0x55, 0xAA};
#define CMD_LEN ((uint32_t)sizeof XT_CMD_START)

static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static uint32_t get16(const uint8_t *p) { return (uint32_t)p[0] << 8 | p[1]; }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[3] << 24 | (uint32_t)p[2] << 16 | (uint32_t)p[1] << 8 | p[0]; }

static uint32_t csum_add(uint32_t s, const uint8_t *p, int n)
{
    for (int i = 0; i + 1 < n; i += 2) s += get16(p + i);
    if (n & 1) s += (uint32_t)p[n - 1] << 8;
    return s;
}
static uint32_t csum_fold(uint32_t s)
{
    while (s >> 16) s = (s & 0xFFFFu) + (s >> 16);
    return ~s & 0xFFFFu;
}

static void go(xt_link_t *x, uint8_t st, uint32_t now)
{
    x->st = st;
    x->t_st = x->t_try = now;
    x->tries = 0;
}

/* 回到监视；grace 微秒内没有新帧才算中断（给雷达出流的时间） */
static void watch(xt_link_t *x, uint32_t now, uint32_t grace)
{
    go(x, XL_WATCH, now);
    x->t_frame = now + grace - XL_STREAM_TMO_US;
    x->t_poll = now - XL_WATCH_EVERY_US;
}

static void eth_hdr(const xt_link_t *x, uint8_t *f, const uint8_t *dst, uint32_t etype)
{
    memcpy(f, dst, 6);
    memcpy(f + 6, x->net.mac, 6);
    put16(f + 12, etype);
}

static void send_arp(xt_link_t *x)
{
    static const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t f[42];
    eth_hdr(x, f, bcast, 0x0806);
    put16(f + 14, 1); put16(f + 16, 0x0800); f[18] = 6; f[19] = 4; put16(f + 20, 1);
    memcpy(f + 22, x->net.mac, 6); put32(f + 28, x->net.ip);
    memset(f + 32, 0, 6); put32(f + 38, x->net.lidar_ip);
    if (hal_lraw_send(f, (int)sizeof f)) x->n_tx++;
}

static void send_tcp(xt_link_t *x, uint32_t seq, uint32_t flags, const uint8_t *pl, int n)
{
    uint8_t f[128];
    int opt = (flags & TCP_SYN) ? 4 : 0;                  /* SYN 带 MSS 1460 */
    int thl = 20 + opt, tot = 20 + thl + n;
    if (14 + tot > (int)sizeof f) return;
    eth_hdr(x, f, x->peer_mac, 0x0800);
    uint8_t *ip = f + 14, *tcp = ip + 20;
    ip[0] = 0x45; ip[1] = 0; put16(ip + 2, (uint32_t)tot); put16(ip + 4, x->ip_id++);
    put16(ip + 6, 0x4000); ip[8] = 64; ip[9] = 6; put16(ip + 10, 0);
    put32(ip + 12, x->net.ip); put32(ip + 16, x->net.lidar_ip);
    put16(ip + 10, csum_fold(csum_add(0, ip, 20)));
    put16(tcp, x->lport); put16(tcp + 2, XL_PORT); put32(tcp + 4, seq);
    put32(tcp + 8, (flags & TCP_ACK) ? x->rcv_nxt : 0);
    tcp[12] = (uint8_t)((thl / 4) << 4); tcp[13] = (uint8_t)flags;
    put16(tcp + 14, 2048); put16(tcp + 16, 0); put16(tcp + 18, 0);
    if (opt) { tcp[20] = 2; tcp[21] = 4; put16(tcp + 22, 1460); }
    if (n) memcpy(tcp + thl, pl, (size_t)n);
    uint8_t ph[12];
    put32(ph, x->net.ip); put32(ph + 4, x->net.lidar_ip); ph[8] = 0; ph[9] = 6; put16(ph + 10, (uint32_t)(thl + n));
    put16(tcp + 16, csum_fold(csum_add(csum_add(0, ph, 12), tcp, thl + n)));
    if (hal_lraw_send(f, 14 + tot)) x->n_tx++;
}

static void begin(xt_link_t *x, uint32_t now)
{
    hal_netl_cfg(&x->net);
    x->n_start++;
    x->last_resp = -1;
    x->rx_n = 0;
    x->lport = (uint16_t)(49152u + ((x->n_start * 97u + (now >> 10)) & 0x3FFFu));
    go(x, XL_ARP, now);
    send_arp(x);
    x->tries = 1;
}

static void fail(xt_link_t *x, uint32_t now)
{
    x->n_fail++;
    go(x, XL_BACKOFF, now);
}

static void send_syn(xt_link_t *x)
{
    send_tcp(x, x->iss, TCP_SYN, NULL, 0);
}

static void close_conn(xt_link_t *x, uint32_t now)
{
    if (x->close_rst) {
        send_tcp(x, x->snd_nxt, TCP_RST | TCP_ACK, NULL, 0);
        watch(x, now, XL_GRACE_US);
    } else {
        send_tcp(x, x->snd_nxt, TCP_FIN | TCP_ACK, NULL, 0);
        x->snd_nxt++;
        go(x, XL_FIN, now);
    }
}

/* 在已收数据里找开始命令的应答帧：返回码，-1 = 还没有（其他命令号的上报帧跳过） */
static int find_resp(xt_link_t *x)
{
    int i = 0;
    while (i + 12 <= x->rx_n) {
        if (memcmp(x->rx + i, XT_START, 4) != 0) { i++; continue; }
        uint32_t n = le32(x->rx + i + 4);
        if (n < 4 || n > 1024) { i++; continue; }
        if (i + 12 + (int)n > x->rx_n) {
            if (12 + n > sizeof x->rx) i++;              /* 放不下的长上报：跳过开头标记，后面的内容当垃圾扫过去 */
            else break;                                  /* 还没收全 */
            continue;
        }
        const uint8_t *b = x->rx + i + 8;
        if (memcmp(b + n, XT_END, 4) == 0 && b[0] == 1) return b[n - 3];
        i += 12 + (int)n;
    }
    if (i > 0) {                                         /* 丢掉已经扫过的部分 */
        memmove(x->rx, x->rx + i, (size_t)(x->rx_n - i));
        x->rx_n -= i;
    }
    return -1;
}

static void on_tcp(xt_link_t *x, const uint8_t *f, int cap, uint32_t now)
{
    const uint8_t *ip = f + 14;
    int ihl = (ip[0] & 15) * 4, tot = (int)get16(ip + 2);
    if (ihl < 20 || tot < ihl + 20 || cap < 14 + ihl + 20) return;
    const uint8_t *tcp = ip + ihl;
    if (get16(tcp) != XL_PORT || get16(tcp + 2) != x->lport) return;
    int thl = (tcp[12] >> 4) * 4;
    int plen = tot - ihl - thl;
    if (thl < 20 || plen < 0) return;
    if (14 + tot <= cap) {                               /* 整段都收到了才能校验 */
        uint8_t ph[12];
        memcpy(ph, ip + 12, 8); ph[8] = 0; ph[9] = 6; put16(ph + 10, (uint32_t)(tot - ihl));
        if (csum_fold(csum_add(csum_add(0, ph, 12), tcp, tot - ihl)) != 0) return;
    }
    uint32_t fl = tcp[13], seq = get32(tcp + 4), ack = get32(tcp + 8);
    x->n_rx++;
    if (fl & TCP_RST) {
        if (x->st == XL_SYN || x->st == XL_CMD) fail(x, now);
        else if (x->st == XL_FIN) watch(x, now, XL_GRACE_US);
        return;
    }
    if (x->st == XL_SYN) {
        if ((fl & (TCP_SYN | TCP_ACK)) == (TCP_SYN | TCP_ACK) && ack == x->iss + 1) {
            x->rcv_nxt = seq + 1;
            send_tcp(x, x->iss + 1, TCP_ACK | TCP_PSH, XT_CMD_START, (int)CMD_LEN);
            x->snd_nxt = x->iss + 1 + CMD_LEN;
            x->data_acked = false;
            go(x, XL_CMD, now);
        }
        return;
    }
    if (x->st != XL_CMD && x->st != XL_FIN) return;
    if ((fl & TCP_ACK) && (int32_t)(ack - x->snd_nxt) >= 0) x->data_acked = true;
    bool peer_fin = false;
    if (plen > 0 || (fl & TCP_FIN)) {
        if (seq == x->rcv_nxt) {
            int have = cap - 14 - ihl - thl;             /* 超过 128 字节的段只存了前面一部分，应答不会这么长 */
            int n = plen < have ? plen : have;
            if (n > 0) {
                if (x->rx_n + n > (int)sizeof x->rx) x->rx_n = 0;
                if (n <= (int)sizeof x->rx) { memcpy(x->rx + x->rx_n, tcp + thl, (size_t)n); x->rx_n += n; }
            }
            x->rcv_nxt += (uint32_t)plen;
            if (fl & TCP_FIN) { x->rcv_nxt++; peer_fin = true; }
        }
        send_tcp(x, x->snd_nxt, TCP_ACK, NULL, 0);       /* 顺序不对时这是重复确认 */
    }
    if (x->st == XL_CMD) {
        int r = find_resp(x);
        if (r >= 0) {
            x->last_resp = r;
            if (r == 0) x->n_ok++; else x->n_fail++;
            close_conn(x, now);
        } else if (peer_fin) {
            close_conn(x, now);
        }
    }
    if (x->st == XL_FIN && peer_fin) watch(x, now, XL_GRACE_US);
}

static void on_frame(xt_link_t *x, const uint8_t *f, int cap, uint32_t now)
{
    if (cap < 42) return;
    uint32_t et = get16(f + 12);
    if (et == 0x0806) {
        if (get16(f + 20) == 2 && get32(f + 28) == x->net.lidar_ip) {
            memcpy(x->peer_mac, f + 22, 6);
            if (x->st == XL_ARP) {
                x->iss = now * 2654435761u + x->n_start;
                go(x, XL_SYN, now);
                send_syn(x);
                x->tries = 1;
            }
        }
    } else if (et == 0x0800 && f[23] == 6 && get32(f + 26) == x->net.lidar_ip) {
        on_tcp(x, f, cap, now);
    }
}

void xt_link_init(xt_link_t *x, uint32_t now_us)
{
    memset(x, 0, sizeof *x);
    hal_netl_cfg(&x->net);
    x->last_resp = -1;
    x->frames_last = hal_lidar_frames();
    watch(x, now_us, XL_BOOT_WAIT_US);
}

void xt_link_poll(xt_link_t *x, uint32_t now)
{
    uint8_t f[128];
    int len;
    if (x->st == XL_WATCH) {
        if (now - x->t_poll < XL_WATCH_EVERY_US) return;
        x->t_poll = now;
        while (hal_lraw_recv(f, (int)sizeof f) > 0) { }  /* 空闲时收到的段（上一次连接的余波）直接丢 */
        uint32_t fr = hal_lidar_frames();
        if (fr != x->frames_last) { x->frames_last = fr; x->t_frame = now; }
        hal_netl_cfg(&x->net);
        if ((int32_t)(now - x->t_frame) > (int32_t)XL_STREAM_TMO_US) {
            x->t_frame = now - XL_STREAM_TMO_US - 1u;    /* 钳住：一直没帧时差值约 36 min 后超出 int32，会永远不再启动 */
            if (x->net.tcp_en) begin(x, now);
        }
        return;
    }
    while ((len = hal_lraw_recv(f, (int)sizeof f)) > 0 && x->st != XL_WATCH)
        on_frame(x, f, len < (int)sizeof f ? len : (int)sizeof f, now);
    switch (x->st) {
    case XL_ARP:
        if (now - x->t_try >= XL_ARP_RETRY_US) {
            if (x->tries >= XL_ARP_TRIES) fail(x, now);
            else { send_arp(x); x->tries++; x->t_try = now; }
        }
        break;
    case XL_SYN:
        if (now - x->t_try >= XL_SYN_RETRY_US) {
            if (x->tries >= XL_SYN_TRIES) fail(x, now);
            else { send_syn(x); x->tries++; x->t_try = now; }
        }
        break;
    case XL_CMD:
        if (now - x->t_st >= XL_CMD_TMO_US) {
            x->n_fail++;                                 /* 没等到应答：命令可能已执行，交给出流监视判断 */
            send_tcp(x, x->snd_nxt, TCP_RST | TCP_ACK, NULL, 0);
            watch(x, now, XL_GRACE_US);
        } else if (!x->data_acked && now - x->t_try >= XL_CMD_RETX_US) {
            send_tcp(x, x->iss + 1, TCP_ACK | TCP_PSH, XT_CMD_START, (int)CMD_LEN);
            x->t_try = now;
        }
        break;
    case XL_FIN:
        if (now - x->t_st >= XL_FIN_TMO_US) watch(x, now, XL_GRACE_US);
        break;
    case XL_BACKOFF:
        if (now - x->t_st >= XL_BACKOFF_US) {
            go(x, XL_WATCH, now);                        /* t_frame 不变：还是没有帧就马上重来 */
            x->t_poll = now - XL_WATCH_EVERY_US;
        }
        break;
    default:
        break;
    }
}
