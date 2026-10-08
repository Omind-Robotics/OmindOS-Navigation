#ifndef OM_FEEDBACK_H
#define OM_FEEDBACK_H
#include "om_usb.h"
#define OM_FB_HOST 253u
#define OM_FB_TIMEOUT_MS 100u /* observation threshold, not a control rate */
#define OM_FB_MALFORMED 1u
#define OM_FB_OVERFLOW 2u
#define OM_FB_BUS_ERROR 4u
#define OM_FB_TIMEOUT 8u
#define OM_FB_MOTOR_ERROR 16u
#define OM_FB_INIT_ERROR 32u
typedef struct {
    uint8_t bus, motor, seen, mode, motor_fault, parameter_error;
    uint16_t parameter_index;
    uint32_t received, count, fault_bits, warning_bits, parameter_value, parameter_time;
    float position, velocity, torque, temperature;
} om_feedback_joint;
typedef struct {
    uint32_t accepted, ignored, malformed, overflow;
    uint8_t fault;
} om_feedback_bus;
typedef struct {
    om_feedback_joint joint[OM_JOINTS];
    om_feedback_bus bus[3];
    uint32_t started;
    uint8_t ready;
} om_feedback;
/* One task owns every function; HAL ISR only queues raw frames. No TX API. */
int om_feedback_init(om_feedback *, const om_joint *, uint32_t now);
void om_feedback_receive(om_feedback *, uint8_t bus, uint32_t id,
                         const uint8_t data[8], uint32_t received, uint32_t now);
void om_feedback_fault(om_feedback *, uint8_t bus, uint8_t reason);
void om_feedback_tick(om_feedback *, uint32_t now);
int om_feedback_faulted(const om_feedback *);
/* selector 0..11: one joint; 255: bus counters. Always 64 bytes on success. */
size_t om_feedback_status(void *, uint8_t selector, uint32_t now, uint8_t out[64]);
#endif
