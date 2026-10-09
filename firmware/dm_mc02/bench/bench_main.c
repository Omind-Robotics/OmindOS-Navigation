/* Independent observer target. No vendor application/test tasks are linked. */
#include "main.h"
#include "fdcan.h"
#include "usb_device.h"
#include "om_stm32_can.h"
#include "om_stm32_usb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bench_config.h"

extern void SystemClock_Config(void);
extern uint32_t g_pfnVectors[];
static RNG_HandleTypeDef rng;
static StaticTask_t bench_tcb, idle_tcb;
static StackType_t bench_stack[4096], idle_stack[configMINIMAL_STACK_SIZE];
/* Read through SWD only. Values are evidence aids, not hardware certification. */
volatile struct {
    uint32_t phase, failure, rng_error, loops, hal_ms, stack_free_words;
} om_bench_debug;

void Error_Handler(void) {
    om_bench_debug.failure=om_bench_debug.phase;
    __disable_irq();
    for(;;) __NOP(); /* Physical power isolation required; no watchdog claim. */
}
void om_bench_assert(void) {om_bench_debug.phase=0xe0;Error_Handler();}
void vApplicationStackOverflowHook(TaskHandle_t task,char *name) {
    (void)task;(void)name;om_bench_debug.phase=0xe1;Error_Handler();
}
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,StackType_t **stack,uint32_t *depth) {
    *tcb=&idle_tcb;*stack=idle_stack;*depth=configMINIMAL_STACK_SIZE;
}
void HAL_RNG_MspInit(RNG_HandleTypeDef *h) {
    if(h->Instance==RNG) __HAL_RCC_RNG_CLK_ENABLE();
}
static void rng_init(void) {
    RCC_PeriphCLKInitTypeDef clocks={0};
    clocks.PeriphClockSelection=RCC_PERIPHCLK_RNG;
    clocks.RngClockSelection=RCC_RNGCLKSOURCE_HSI48;
    if(HAL_RCCEx_PeriphCLKConfig(&clocks)!=HAL_OK) Error_Handler();
    rng.Instance=RNG;rng.Init.ClockErrorDetection=RNG_CED_ENABLE;
    if(HAL_RNG_Init(&rng)!=HAL_OK || HAL_RNG_GetError(&rng)!=HAL_RNG_ERROR_NONE) {
        om_bench_debug.rng_error=HAL_RNG_GetError(&rng);Error_Handler();
    }
}
static void bench_task(void *argument) {
    (void)argument;
    om_bench_debug.phase=4;rng_init();
    om_bench_debug.phase=5;
    if(om_stm32_usb_init_observer(&rng,bench_digest,bench_joints)) {
        om_bench_debug.rng_error=HAL_RNG_GetError(&rng);Error_Handler();
    }
    om_bench_debug.phase=6;
    FDCAN_HandleTypeDef *handles[3]={&hfdcan1,&hfdcan2,&hfdcan3};
    /* Init failure latches CAN/core faults; USB may still expose diagnostics. */
    if(om_stm32_can_init(handles,bench_joints)) om_bench_debug.failure=6;
    om_bench_debug.phase=7;MX_USB_DEVICE_Init();
    om_bench_debug.phase=8;
    for(;;) {
        om_stm32_can_poll();om_stm32_usb_poll();
        ++om_bench_debug.loops;om_bench_debug.hal_ms=HAL_GetTick();
        om_bench_debug.stack_free_words=uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(1)); /* Requested delay, not a guaranteed period. */
    }
}
int main(void) {
    SCB->VTOR=(uint32_t)g_pfnVectors;__DSB();__ISB();
    om_bench_debug.phase=1;
    if(HAL_Init()!=HAL_OK) Error_Handler();
    om_bench_debug.phase=2;SystemClock_Config();
    /* No generic GPIO init: unknown power-switch polarity remains untouched. */
    om_bench_debug.phase=3;
    MX_FDCAN1_Init();MX_FDCAN2_Init();MX_FDCAN3_Init();
    if(!xTaskCreateStatic(bench_task,"om-observer",4096,NULL,3,bench_stack,&bench_tcb)) Error_Handler();
    vTaskStartScheduler();Error_Handler();
}
