#include "p11_raw_service.h"
static bool integrity(p11_raw_epoch_t *epoch)
{
    return epoch->clean_epoch&&p11_raw_hal_count()<=P11_RAW_QUEUE_LIMIT&&
        p11_raw_hal_overflow()==epoch->overflow_baseline;
}
unsigned p11_raw_service(p11_command_guard_t *g,p11_raw_epoch_t *epoch,
                        bool R0_permit,bool *lock_edge)
{
    unsigned count=0;*lock_edge=false;
    if(g->state==P11_OWNER_OFF)return 0;
    if(g->state==P11_LOCKED)return 0;
    for(unsigned i=0;i<P11_RAW_BYTE_QUOTA;i++){
        if(!integrity(epoch))goto bad;
        uint32_t data=p11_raw_hal_data();
        if(!(data&0x80000000u))break;
        bool first=(data&0x100u)!=0;
        uint32_t stamp=first?p11_raw_hal_timestamp():0;
        /* Zero is ambiguous with the current no-TS-valid MMIO. Reject it,
         * including the rare legitimate timestamp exactly at32-bit wrap. */
        if((first&&stamp==0)||!integrity(epoch))goto bad;
        p11_guard_byte(g,(uint8_t)data,first,stamp,p11_raw_hal_time_us(),R0_permit);
        count++;
        /* The byte helper may discover R0/feedback/arrival expiry itself.
         * Preserve that edge for immediate zero publication, rather than
         * asking a second poll after the state is already latched. */
        if(g->state==P11_LOCKED){*lock_edge=true;return count;}
    }
    *lock_edge=p11_guard_poll(g,p11_raw_hal_time_us(),R0_permit);
    return count;
bad:
    epoch->clean_epoch=false;*lock_edge=p11_guard_quarantine(g);return count;
}
