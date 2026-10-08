#ifndef OM_STM32_CAN_H
#define OM_STM32_CAN_H
#include "om_feedback.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#define OM_CAN_QUEUE_CAPACITY 32u
/* Fixed DM-MC02 vendor ALL layout, before can_bsp_init or any CAN Start.
 * Call once AFTER om_stm32_usb_init and MX_FDCAN1/2/3_Init, BEFORE USB starts.
 * Exclude vendor can_bsp.c and every sending task from the bench target.
 * Converts READY controllers to BUS_MONITORING, allocates no TX elements. */
int om_stm32_can_init(FDCAN_HandleTypeDef *const handles[3],const om_joint *);
/* Same task as USB poll. Run this first, then USB poll, at bounded intervals. */
void om_stm32_can_poll(void);
#endif
