/* Isolated draft, not selected by the production FW or EDA inputs. */
#ifndef P11_COMMAND_GUARD_H
#define P11_COMMAND_GUARD_H
#include <stdbool.h>
#include <stdint.h>

#define P11_COMMAND_AGE_US 200000u
#define P11_FEEDBACK_AGE_US 50000u
#define P11_STILL_DWELL_US 500000u
#define P11_RAW_BYTE_QUOTA 32u
#define P11_RAW_QUEUE_LIMIT 32u
#define P11_RAW_PAIR_MAX_US 100u

typedef enum { P11_OWNER_OFF, P11_WAIT_ZERO, P11_ARMED, P11_LOCKED } p11_state_t;
typedef enum { P11_QUERY, P11_AUTHORIZE, P11_RELEASE } p11_action_t;
typedef enum { P11_OK, P11_NOT_TRUSTED, P11_NOT_IDLE, P11_NOT_STILL,
               P11_NOT_DRAINED, P11_BAD_SESSION, P11_BAD_ACTION } p11_result_t;
typedef struct {
    p11_state_t state;
    uint32_t session, authorized_us, arrival_us;
    uint32_t feedback_us, still_since_us, burst_us;
    uint32_t accepted, rejected, crc_bad, range_bad, time_bad, locks;
    int16_t forward_mmps, yaw_mradps;
    uint8_t bytes[10], count, total;
    bool have_arrival, have_feedback, still, have_burst, burst_used;
} p11_command_guard_t;

void p11_guard_init(p11_command_guard_t *g);
/* Call from the existing single odometry consumer; do not pop feedback twice. */
void p11_guard_feedback(p11_command_guard_t *g, int16_t vx, int16_t wz,
                        uint32_t sample_us, uint32_t now, bool published_zero);
bool p11_guard_stationary(const p11_command_guard_t *g, uint32_t now);
/* trusted_R0 requires a fresh accepted NV request and an unlocked R0 guard.
 * raw_drained_quiet is established by bounded draining, not assumed by this API. */
p11_result_t p11_guard_manage(p11_command_guard_t *g, p11_action_t action,
    uint32_t session, uint32_t now, bool trusted_R0, bool nav_idle, bool raw_drained_quiet);
/* Returns true only on the lock edge, for immediate common-HAL zero publication. */
bool p11_guard_poll(p11_command_guard_t *g, uint32_t now, bool R0_permit);
bool p11_guard_quarantine(p11_command_guard_t *g);
/* Each command is one idle-delimited burst. stamp is the actual raw FIFO FIRST
 * start timestamp, never CPU dequeue time. Partial/CRC/range errors do not renew. */
bool p11_guard_byte(p11_command_guard_t *g, uint8_t byte, bool first,
    uint32_t stamp, uint32_t now, bool R0_permit);
/* This is only a proposal: place it BEFORE the existing common nonzero
 * motion/support checks. The existing final hal_send_cmd guard is still applied. */
bool p11_guard_proposal(p11_command_guard_t *g, uint32_t now, bool R0_permit,
                        int16_t *forward, int16_t *yaw);
#endif
