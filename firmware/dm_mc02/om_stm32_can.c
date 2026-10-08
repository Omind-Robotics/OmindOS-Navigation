#include "om_stm32_can.h"
#include "om_stm32_usb.h"
#include <string.h>
typedef struct {uint32_t id,at;uint8_t bus,data[8];} received_frame;
static FDCAN_HandleTypeDef *controllers[3];
static om_feedback feedback;
static received_frame queue[OM_CAN_QUEUE_CAPACITY];
static unsigned head,tail,count;
static uint8_t attempted,ready,pending[3];
#define OM_CAN_NOTIFICATIONS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_FULL| \
    FDCAN_IT_RX_FIFO0_MESSAGE_LOST|FDCAN_IT_BUS_OFF|FDCAN_IT_ERROR_PASSIVE| \
    FDCAN_IT_ERROR_WARNING|FDCAN_IT_ARB_PROTOCOL_ERROR|FDCAN_IT_DATA_PROTOCOL_ERROR)
static uint32_t enter(void) {
    uint32_t key=__get_PRIMASK();__disable_irq();__DMB();return key;
}
static void leave(uint32_t key) {__DMB();__set_PRIMASK(key);}
static int bus_index(FDCAN_HandleTypeDef *h) {
    for(unsigned i=0;i<3;++i) if(h && controllers[i]==h) return (int)i;
    return -1;
}
static void note(unsigned bus,uint8_t reason) {
    uint32_t key=enter();pending[bus]|=reason;leave(key);
}
int om_stm32_can_init(FDCAN_HandleTypeDef *const handles[3],const om_joint *j) {
    if(attempted) return -1;
    attempted=1;
    if(om_feedback_init(&feedback,j,HAL_GetTick())) return -1;
    if(om_stm32_usb_status_provider(om_feedback_status,&feedback)) return -1;
    if(!handles) goto fail;
    for(unsigned b=0;b<3;++b) {
        if(!handles[b] || handles[b]->State!=HAL_FDCAN_STATE_READY ||
           handles[b]->Instance!=(b==0?FDCAN1:(b==1?FDCAN2:FDCAN3)) ||
           handles[b]->Init.MessageRAMOffset!=(b==0?0u:(b==1?0x406u:0x812u))) goto fail;
    }
    for(unsigned b=0;b<3;++b) {
        FDCAN_HandleTypeDef *h=handles[b];controllers[b]=h;
        h->Init.FrameFormat=FDCAN_FRAME_CLASSIC;
        h->Init.Mode=FDCAN_MODE_BUS_MONITORING;
        h->Init.AutoRetransmission=DISABLE;
        h->Init.StdFiltersNbr=0;h->Init.ExtFiltersNbr=1;
        h->Init.RxFifo0ElmtsNbr=32;h->Init.RxFifo0ElmtSize=FDCAN_DATA_BYTES_8;
        h->Init.RxFifo1ElmtsNbr=0;h->Init.RxBuffersNbr=0;
        h->Init.TxEventsNbr=0;h->Init.TxBuffersNbr=0;h->Init.TxFifoQueueElmtsNbr=0;
        if(HAL_FDCAN_Init(h)!=HAL_OK) goto fail;
        FDCAN_FilterTypeDef filter={0};
        filter.IdType=FDCAN_EXTENDED_ID;filter.FilterIndex=0;
        filter.FilterType=FDCAN_FILTER_MASK;filter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1=OM_FB_HOST;filter.FilterID2=255; /* host low byte */
        if(HAL_FDCAN_ConfigFilter(h,&filter)!=HAL_OK ||
           HAL_FDCAN_ConfigGlobalFilter(h,FDCAN_REJECT,FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)!=HAL_OK ||
           HAL_FDCAN_ConfigRxFifoOverwrite(h,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_BLOCKING)!=HAL_OK)
            goto fail;
    }
    ready=1;
    for(unsigned b=0;b<3;++b) {
        if(HAL_FDCAN_ActivateNotification(controllers[b],OM_CAN_NOTIFICATIONS,0)!=HAL_OK ||
           HAL_FDCAN_Start(controllers[b])!=HAL_OK) goto fail;
    }
    return 0;
fail:
    ready=0;
    for(unsigned b=0;b<3;++b) {
        if(controllers[b]) {
            (void)HAL_FDCAN_DeactivateNotification(controllers[b],OM_CAN_NOTIFICATIONS);
            (void)HAL_FDCAN_Stop(controllers[b]);
        }
        om_feedback_fault(&feedback,(uint8_t)b,OM_FB_INIT_ERROR);
    }
    om_stm32_usb_observation_fault();return -1;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h,uint32_t flags) {
    int index=bus_index(h);
    if(!ready || index<0) return;
    unsigned bus=(unsigned)index;
    if(flags&(FDCAN_IT_RX_FIFO0_FULL|FDCAN_IT_RX_FIFO0_MESSAGE_LOST))
        note(bus,OM_FB_OVERFLOW);
    if(!(flags&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;
    /* Bound ISR work; the level/new-message IRQ handles subsequent arrivals.
     * HAL copies DLC bytes, so keep a 64-byte scratch even for rejected FD. */
    for(unsigned n=0;n<8 && HAL_FDCAN_GetRxFifoFillLevel(h,FDCAN_RX_FIFO0);++n) {
        FDCAN_RxHeaderTypeDef header={0};uint8_t data[64];
        if(HAL_FDCAN_GetRxMessage(h,FDCAN_RX_FIFO0,&header,data)!=HAL_OK) {
            note(bus,OM_FB_BUS_ERROR);break;
        }
        if(header.IdType!=FDCAN_EXTENDED_ID || header.RxFrameType!=FDCAN_DATA_FRAME ||
           header.FDFormat!=FDCAN_CLASSIC_CAN || header.BitRateSwitch!=FDCAN_BRS_OFF ||
           header.DataLength!=FDCAN_DLC_BYTES_8 || header.Identifier>=0x20000000u) {
            note(bus,OM_FB_MALFORMED);continue;
        }
        uint32_t key=enter();
        if(count==OM_CAN_QUEUE_CAPACITY) pending[bus]|=OM_FB_OVERFLOW;
        else {
            received_frame *frame=&queue[head];
            frame->id=header.Identifier;frame->at=HAL_GetTick();frame->bus=(uint8_t)bus;
            memcpy(frame->data,data,8);head=(head+1u)%OM_CAN_QUEUE_CAPACITY;++count;
        }
        leave(key);
    }
    /* A backlog after the bounded drain is not silently assigned a fresh age.
     * Latch and let the task drain the hardware FIFO via this same callback. */
    if(HAL_FDCAN_GetRxFifoFillLevel(h,FDCAN_RX_FIFO0)) note(bus,OM_FB_OVERFLOW);
}
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *h) {
    int b=bus_index(h);if(ready && b>=0) note((unsigned)b,OM_FB_BUS_ERROR);
}
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *h,uint32_t flags) {
    if(flags) HAL_FDCAN_ErrorCallback(h);
}
void om_stm32_can_poll(void) {
    if(!ready) return;
    for(unsigned b=0;b<3;++b) {
        FDCAN_ProtocolStatusTypeDef status={0};
        if(HAL_FDCAN_GetProtocolStatus(controllers[b],&status)!=HAL_OK || status.BusOff ||
           status.ErrorPassive || status.Warning ||
           (status.LastErrorCode>0 && status.LastErrorCode<7)) note(b,OM_FB_BUS_ERROR);
        /* IRQs own the FIFO. Drain leftover backlog under the same mask. */
        uint32_t key=enter();
        if(HAL_FDCAN_GetRxFifoFillLevel(controllers[b],FDCAN_RX_FIFO0))
            HAL_FDCAN_RxFifo0Callback(controllers[b],FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        uint8_t faults=pending[b];pending[b]=0;leave(key);
        if(faults) om_feedback_fault(&feedback,(uint8_t)b,faults);
    }
    for(unsigned n=0;n<OM_CAN_QUEUE_CAPACITY;++n) {
        uint32_t key=enter();
        if(!count) {leave(key);break;}
        received_frame frame=queue[tail];tail=(tail+1u)%OM_CAN_QUEUE_CAPACITY;--count;
        leave(key);
        om_feedback_receive(&feedback,frame.bus,frame.id,frame.data,frame.at,HAL_GetTick());
    }
    om_feedback_tick(&feedback,HAL_GetTick());
    if(om_feedback_faulted(&feedback)) om_stm32_usb_observation_fault();
}
