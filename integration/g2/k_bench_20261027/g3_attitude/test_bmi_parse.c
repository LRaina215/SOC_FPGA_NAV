/* Host test for the BMI internal IMU protocol in fw/nav/imu_parse.c.
 * Builds 0x5B/0x5C frames exactly like rtl bmi_att_uart from a known navigation
 * attitude (pitch/roll in the WIT convention used by the simulator) and checks
 * the parser recovers it. */
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "imu_parse.h"

static void frame(uint8_t *b, uint8_t type, int16_t a, int16_t c, int16_t e, uint16_t w)
{
    b[0] = 0x55; b[1] = type;
    b[2] = a & 0xFF; b[3] = (uint8_t)(a >> 8);
    b[4] = c & 0xFF; b[5] = (uint8_t)(c >> 8);
    b[6] = e & 0xFF; b[7] = (uint8_t)(e >> 8);
    b[8] = w & 0xFF; b[9] = (uint8_t)(w >> 8);
    uint8_t s = 0;
    for (int i = 0; i < 10; i++) s = (uint8_t)(s + b[i]);
    b[10] = s;
}

int main(void)
{
    imu_parser_t p;
    imu_parser_init(&p, IMU_PROTO_BMI_INTERNAL);
    int errors = 0, n = 0;
    double worst = 0;
    for (int pd = -40; pd <= 40; pd += 5) {
        for (int rd = -40; rd <= 40; rd += 5) {
            double pr = pd * M_PI / 180, rr = rd * M_PI / 180;
            /* navigation body gravity (x fwd, y left, z up) for R = Rz Ry(p) Rx(r) */
            double gxn = sin(pr), gyn = -sin(rr) * cos(pr), gzn = -cos(rr) * cos(pr);
            /* inverse of nav = (ctrl_y, -ctrl_x, ctrl_z): ctrl = (-nav_y, nav_x, nav_z) */
            int16_t cx = (int16_t)lround(-gyn * 4096), cy = (int16_t)lround(gxn * 4096), cz = (int16_t)lround(gzn * 4096);
            /* body rates: roll rate 0.5 rad/s about nav x, pitch rate -0.25 rad/s about nav y, yaw 1 rad/s */
            double wxn = 0.5, wyn = -0.25, wzn = 1.0;
            int16_t wcx = (int16_t)lround(-wyn * 512), wcy = (int16_t)lround(wxn * 512), wcz = (int16_t)lround(wzn * 512);
            uint8_t b[22];
            frame(b, 0x5B, cx, cy, cz, (uint16_t)n);
            frame(b + 11, 0x5C, wcx, wcy, wcz, 0);
            bool got = false;
            for (int i = 0; i < 22; i++) got |= imu_parser_feed(&p, b[i], i == 0, 1000u * n);
            if (!got || !p.gyro_new) { errors++; continue; }
            double pe = (int16_t)p.pitch * 360.0 / 65536 - pd, re = (int16_t)p.roll * 360.0 / 65536 - rd;
            /* expected WIT raw gyro: rad/s * 938.73 */
            double gxe = p.gx - wxn * 938.73, gye = p.gy - wyn * 938.73, gze = p.gz - wzn * 938.73;
            double e = fmax(fabs(pe), fabs(re));
            if (e > worst) worst = e;
            if (e > 0.15 || fabs(gxe) > 2 || fabs(gye) > 2 || fabs(gze) > 2) {
                errors++;
                if (errors < 10) printf("MISMATCH p=%d r=%d -> pitch %.3f roll %.3f gyro %d %d %d\n", pd, rd,
                                        (int16_t)p.pitch * 360.0 / 65536, (int16_t)p.roll * 360.0 / 65536, p.gx, p.gy, p.gz);
            }
            p.gyro_new = false;
            n++;
        }
    }
    printf("DONE cases=%d errors=%d worst_angle_err_deg=%.3f n_ok=%u n_bad=%u\n", n, errors, worst, p.n_ok, p.n_bad);
    return errors != 0;
}
