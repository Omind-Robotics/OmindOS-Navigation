#ifndef OM_USB_H
#define OM_USB_H
#include <stddef.h>
#include <stdint.h>
#define OM_MAX_FRAME 98u
#define OM_JOINTS 12u
/* Portable, locked bench core. There is deliberately no ARM or CAN-send API. */
typedef struct {
    uint8_t bus, motor_id;
    int8_t direction;
    float lower, upper, offset, ratio, kp, kd;
} om_joint;
typedef struct { uint8_t bus; uint32_t id; uint8_t data[8]; } om_can;
/* Optional read-only diagnostic provider. One task owns it and the core.
 * Return payload bytes (1..64), or 0 for an unsupported selector. */
typedef size_t (*om_status_provider)(void *, uint8_t, uint32_t, uint8_t[64]);
typedef struct {
    uint8_t nonce[16], digest[32], buffer[OM_MAX_FRAME];
    om_joint joints[OM_JOINTS];
    size_t used;
    uint32_t sequence, deadline, lease_deadline;
    uint8_t fault, configured, ticket_pending, lease;
    om_status_provider status;
    void *status_context;
} om_usb;
/* nonce must be a fresh boot nonce from the board RNG, never a constant. */
int om_init(om_usb *, const uint8_t nonce[16], const uint8_t digest[32], const om_joint *);
/* Call from the independent scheduler even when USB has no traffic. */
void om_tick(om_usb *, uint32_t now_ms);
/* A return of 1 means a full response; -1 means latched error. Never actuates. */
int om_feed(om_usb *, uint8_t byte, uint32_t now_ms, uint8_t *response, size_t *);
/* USB disconnect/overflow/bus error must invalidate tickets and latch. */
void om_fault(om_usb *, uint8_t reason);
/* Pure RS04 codec for calibrated dry-run fixtures; does not send frames. */
int om_encode_targets(const om_joint *, const float *positions, om_can *output);
#endif
