#include "om_cdc.h"
#include <string.h>

static int init(om_cdc *s,om_cdc_io io,const uint8_t nonce[16],
                const uint8_t digest[32],const om_joint *j,int observer) {
    memset(s,0,sizeof(*s)); s->io=io;
    if(!io.enter || !io.leave || !io.submit) {
        s->transport_fault=1;om_fault(&s->core,1);return -1;
    }
#ifdef OM_BENCH_OBSERVER_ONLY
    (void)observer;
    return om_init_observer(&s->core,nonce,digest,j);
#else
    return observer ? om_init_observer(&s->core,nonce,digest,j) : om_init(&s->core,nonce,digest,j);
#endif
}
int om_cdc_init(om_cdc *s,om_cdc_io io,const uint8_t nonce[16],const uint8_t digest[32],const om_joint *j) {
    return init(s,io,nonce,digest,j,0);
}
int om_cdc_init_observer(om_cdc *s,om_cdc_io io,const uint8_t nonce[16],const uint8_t digest[32],const om_joint *j) {
    return init(s,io,nonce,digest,j,1);
}
void om_cdc_receive(om_cdc *s,const uint8_t *data,size_t n) {
    uint32_t key=s->io.enter(s->io.context);
    if(!s->pending_fault && !s->transport_fault) {
        if(n>OM_CDC_RX_CAPACITY-s->count || (!data && n)) {
            s->pending_fault=8; /* whole incoming chunk rejected */
        } else {
            for(size_t i=0;i<n;++i) {
                s->rx[s->head]=data[i];
                s->head=(s->head+1u)%OM_CDC_RX_CAPACITY;
            }
            s->count+=(unsigned)n;
        }
    }
    s->io.leave(s->io.context,key);
}
void om_cdc_tx_complete(om_cdc *s) {
    uint32_t key=s->io.enter(s->io.context);
    s->completed=1;s->io.leave(s->io.context,key);
}
void om_cdc_link_fault(om_cdc *s) {
    uint32_t key=s->io.enter(s->io.context);
    s->pending_fault=8;s->io.leave(s->io.context,key);
}
static void latch(om_cdc *s,uint8_t reason) {
    uint32_t key=s->io.enter(s->io.context);
    s->transport_fault=reason;s->count=0;
    s->io.leave(s->io.context,key);
    om_fault(&s->core,reason);
    /* Do not modify tx: a disconnected or wedged endpoint may still own it. */
}
void om_cdc_poll(om_cdc *s,uint32_t now) {
    if(s->transport_fault) return;
    uint32_t key=s->io.enter(s->io.context);
    uint8_t fault=s->pending_fault,done=s->completed;s->completed=0;
    s->io.leave(s->io.context,key);
    if(fault) {latch(s,fault);return;}
    om_tick(&s->core,now); /* independent of TX_BUSY or queued input */
    if(s->active && done) {s->active=0;s->tx_len=0;}
    if(s->tx_len) {
        if((uint32_t)(now-s->tx_started)>=OM_CDC_TX_TIMEOUT_MS) {
            latch(s,8);return;
        }
        if(s->active) return;
    } else {
        for(unsigned consumed=0;consumed<OM_CDC_RX_CAPACITY;++consumed) {
            key=s->io.enter(s->io.context);
            fault=s->pending_fault;
            if(fault || !s->count) {
                s->io.leave(s->io.context,key);
                if(fault) latch(s,fault);
                return;
            }
            uint8_t byte=s->rx[s->tail];
            s->tail=(s->tail+1u)%OM_CDC_RX_CAPACITY;--s->count;
            s->io.leave(s->io.context,key);
            om_feed(&s->core,byte,now,s->tx,&s->tx_len);
            if(s->tx_len) {s->tx_started=now;break;}
        }
        if(!s->tx_len) return;
    }
    int result=s->io.submit(s->io.context,s->tx,s->tx_len);
    if(result==0) s->active=1;
    else if(result!=1) latch(s,8);
}
