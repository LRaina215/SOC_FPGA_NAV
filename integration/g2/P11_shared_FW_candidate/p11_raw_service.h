#ifndef P11_RAW_SERVICE_H
#define P11_RAW_SERVICE_H
#include "p11_command_guard.h"
typedef struct { uint32_t overflow_baseline; bool clean_epoch; } p11_raw_epoch_t;
/* These HAL wrappers map ONLY current MMIO: 6008,6004,600C,601C,SYS_TIME_US.
 * No timestamp-valid/overflow field is assumed to exist. */
uint32_t p11_raw_hal_count(void);
uint32_t p11_raw_hal_data(void);
uint32_t p11_raw_hal_timestamp(void);
uint32_t p11_raw_hal_overflow(void);
uint32_t p11_raw_hal_time_us(void);
/* A clean epoch must come from actual bounded drain + quiet-source protocol.
 * This component does not manufacture it from CPU dequeue time. */
unsigned p11_raw_service(p11_command_guard_t *g,p11_raw_epoch_t *epoch,
                        bool R0_permit,bool *lock_edge);
#endif
