#include "prior.h"
#include "hal.h"

static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* zlib CRC32（反射多项式 0xEDB88320），半字节查表：16 项表、每字节两次查表 */
uint32_t prior_crc32_update(uint32_t crc, const uint8_t *d, int n)
{
    static const uint32_t T[16] = {
        0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
        0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu, 0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu};
    crc = ~crc;
    for (int i = 0; i < n; i++) {
        crc ^= d[i];
        crc = (crc >> 4) ^ T[crc & 15];
        crc = (crc >> 4) ^ T[crc & 15];
    }
    return ~crc;
}

void prior_rx_init(prior_rx_t *r) { *r = (prior_rx_t){0}; }

void prior_rx_begin(prior_rx_t *r, gmap_t *g, uint32_t total)
{
    prior_rx_init(r);
    r->state = PRIOR_RECV;
    r->total = total;
    hal_prior_begin();
    gmap_prior_drop(g);
}

static void body_bytes(prior_rx_t *r, gmap_t *g, const uint8_t *d, int n)
{
    r->crc = prior_crc32_update(r->crc, d, n);
    const uint32_t ns = r->n_struct * 5;
    for (int i = 0; i < n; i++, r->body_i++) {
        if (r->body_i < ns) {
            r->ent[r->ent_n++] = d[i];
            if (r->ent_n == 5) {
                uint32_t w = (uint32_t)r->ent[2] | ((uint32_t)r->ent[3] << 8) | ((uint32_t)r->ent[4] << 16);
                hal_prior_write(rd16(r->ent), w & 0x3FFFF);
                r->ent_n = 0;
                r->n_words++;
            }
        } else {
            gmap_prior_put(g, (int)(r->body_i - ns), d[i]);
        }
    }
}

bool prior_rx_data(prior_rx_t *r, gmap_t *g, uint32_t offset, const uint8_t *d, int n)
{
    if (r->state != PRIOR_RECV || offset != r->next || n <= 0 || offset + (uint32_t)n > r->total) return false;
    int i = 0;
    while (i < n && r->next < PRIOR_HDR_LEN) r->hdr[r->next++] = d[i++];
    if (!r->hdr_ok && r->next == PRIOR_HDR_LEN) {
        r->hdr_ok = true;
        const uint8_t *h = r->hdr;
        if (h[0] != 'N' || h[1] != 'V' || h[2] != 'M' || h[3] != 'P' || rd16(h + 4) != 2) { r->state = PRIOR_EFMT; return true; }   /* 版本 2 = 18 位槽字 */
        r->ox = (int32_t)rd32(h + 8); r->oy = (int32_t)rd32(h + 12);
        r->n_struct = rd32(h + 16);
        r->gox = (int32_t)rd32(h + 20); r->goy = (int32_t)rd32(h + 24);
        r->gnx = rd16(h + 28); r->gny = rd16(h + 30);
        r->crc_exp = rd32(h + 32); r->body_len = rd32(h + 36);
        if (r->gnx > GM_N || r->gny > GM_N || r->n_struct > 65536u ||
            r->body_len != r->n_struct * 5 + (uint32_t)r->gnx * r->gny || r->body_len + PRIOR_HDR_LEN != r->total) {
            r->state = PRIOR_EFMT;
            return true;
        }
        gmap_prior_begin(g, r->gox, r->goy, r->gnx, r->gny);
    }
    if (i < n) {
        body_bytes(r, g, d + i, n - i);
        r->next += (uint32_t)(n - i);
    }
    return true;
}

uint8_t prior_rx_end(prior_rx_t *r, gmap_t *g)
{
    if (r->state != PRIOR_RECV) return r->state;
    if (r->next != r->total || r->next < PRIOR_HDR_LEN) { r->state = PRIOR_EFMT; return r->state; }
    if (r->crc != r->crc_exp) {
        r->state = PRIOR_ECRC;
        hal_prior_enable(false, 0, 0);
        gmap_prior_drop(g);
        return r->state;
    }
    hal_prior_enable(true, r->ox, r->oy);
    gmap_prior_done(g);
    r->state = PRIOR_READY;
    return r->state;
}

void prior_rx_clear(prior_rx_t *r, gmap_t *g)
{
    prior_rx_init(r);
    hal_prior_enable(false, 0, 0);
    gmap_prior_drop(g);
}
