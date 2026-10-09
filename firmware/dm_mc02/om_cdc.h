#ifndef OM_CDC_H
#define OM_CDC_H
#include "om_usb.h"
#define OM_CDC_RX_CAPACITY 256u
#define OM_CDC_TX_TIMEOUT_MS 20u /* bench deadline; not a control frequency */
/* submit: 0 accepted asynchronously, 1 busy, negative fatal.
 * Accepted bytes must remain owned by USB until om_cdc_tx_complete().
 * enter/leave must mask every IRQ which calls a cdc callback, and restore
 * the previous interrupt mask (including when invoked from an ISR). */
typedef struct {
    void *context;
    uint32_t (*enter)(void *);
    void (*leave)(void *, uint32_t);
    int (*submit)(void *, const uint8_t *, size_t);
} om_cdc_io;
typedef struct {
    om_usb core;
    om_cdc_io io;
    uint8_t rx[OM_CDC_RX_CAPACITY], tx[OM_MAX_FRAME];
    unsigned head, tail, count;
    uint8_t pending_fault, completed, transport_fault, active;
    size_t tx_len;
    uint32_t tx_started;
} om_cdc;
/* Initialize only after USB endpoints/IRQs are stopped and DMA releases tx.
 * All callbacks must then target this instance for its entire lifetime. */
int om_cdc_init(om_cdc *, om_cdc_io, const uint8_t nonce[16],
                const uint8_t digest[32], const om_joint *);
int om_cdc_init_observer(om_cdc *, om_cdc_io, const uint8_t nonce[16],
                        const uint8_t digest[32], const om_joint *);
/* ISR callbacks enqueue/copy only. No parser or transmit runs in an ISR. */
void om_cdc_receive(om_cdc *, const uint8_t *, size_t);
void om_cdc_tx_complete(om_cdc *);
void om_cdc_link_fault(om_cdc *); /* disconnect, bus error, endpoint error */
/* Call from one main-loop owner, continuously even with no USB traffic. */
void om_cdc_poll(om_cdc *, uint32_t now_ms);
#endif
