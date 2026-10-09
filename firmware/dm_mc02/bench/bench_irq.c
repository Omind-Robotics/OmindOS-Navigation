#include "main.h"
#include "fdcan.h"
#include "om_stm32_usb.h"
extern TIM_HandleTypeDef htim23;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
void FDCAN1_IT0_IRQHandler(void) {HAL_FDCAN_IRQHandler(&hfdcan1);}
void FDCAN1_IT1_IRQHandler(void) {HAL_FDCAN_IRQHandler(&hfdcan1);}
void FDCAN2_IT0_IRQHandler(void) {HAL_FDCAN_IRQHandler(&hfdcan2);}
void FDCAN2_IT1_IRQHandler(void) {HAL_FDCAN_IRQHandler(&hfdcan2);}
void FDCAN3_IT0_IRQHandler(void) {HAL_FDCAN_IRQHandler(&hfdcan3);}
void FDCAN3_IT1_IRQHandler(void) {HAL_FDCAN_IRQHandler(&hfdcan3);}
void OTG_HS_IRQHandler(void) {HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);}
void TIM23_IRQHandler(void) {HAL_TIM_IRQHandler(&htim23);}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h) {if(h->Instance==TIM23) HAL_IncTick();}
void NMI_Handler(void) {Error_Handler();}
void HardFault_Handler(void) {Error_Handler();}
void MemManage_Handler(void) {Error_Handler();}
void BusFault_Handler(void) {Error_Handler();}
void UsageFault_Handler(void) {Error_Handler();}
