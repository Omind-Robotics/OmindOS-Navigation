#ifndef OM_STM32_USB_H
#define OM_STM32_USB_H
#include "om_cdc.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rng.h"
/* Bench-only binding. Call once before starting USB, with initialized RNG.
 * No reinitialization in the same boot, no motor enable or CAN transmit. */
int om_stm32_usb_init(RNG_HandleTypeDef *, const uint8_t digest[32], const om_joint *);
void om_stm32_usb_poll(void); /* one main-loop/task owner; never USB ISR */
void om_stm32_usb_receive(const uint8_t *, size_t);
void om_stm32_usb_complete(const uint8_t *, size_t);
void om_stm32_usb_disconnect(void);
int om_stm32_usb_faulted(void); /* main-loop owner only */
/* Same task as poll; install once before USB starts. */
int om_stm32_usb_status_provider(om_status_provider, void *);
void om_stm32_usb_observation_fault(void);
#endif
