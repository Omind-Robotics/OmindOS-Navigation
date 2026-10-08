"""Fault-injected actual HAL adapter; simulated peripherals, never hardware."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
from test_dm_mc02_stm32_usb import HAL

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT/'firmware/dm_mc02'
CAN = r'''
#ifndef MOCK_CAN_H
#define MOCK_CAN_H
#include "stm32h7xx_hal.h"
#define FDCAN1 ((void*)1)
#define FDCAN2 ((void*)2)
#define FDCAN3 ((void*)3)
#define HAL_FDCAN_STATE_READY 1
#define FDCAN_FRAME_CLASSIC 0
#define FDCAN_MODE_BUS_MONITORING 2
#define DISABLE 0
#define FDCAN_DATA_BYTES_8 4
#define FDCAN_EXTENDED_ID 1
#define FDCAN_FILTER_MASK 2
#define FDCAN_FILTER_TO_RXFIFO0 1
#define FDCAN_REJECT 2
#define FDCAN_REJECT_REMOTE 1
#define FDCAN_RX_FIFO0 0
#define FDCAN_RX_FIFO_BLOCKING 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_BRS_OFF 0
#define FDCAN_DLC_BYTES_8 (8u<<16)
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1u
#define FDCAN_IT_RX_FIFO0_FULL 2u
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 4u
#define FDCAN_IT_BUS_OFF 8u
#define FDCAN_IT_ERROR_PASSIVE 16u
#define FDCAN_IT_ERROR_WARNING 32u
#define FDCAN_IT_ARB_PROTOCOL_ERROR 64u
#define FDCAN_IT_DATA_PROTOCOL_ERROR 128u
/* No TX API is declared or defined: any call is a compile/link failure. */
typedef struct {
    uint32_t FrameFormat,Mode,AutoRetransmission,StdFiltersNbr,ExtFiltersNbr,
    RxFifo0ElmtsNbr,RxFifo0ElmtSize,RxFifo1ElmtsNbr,RxBuffersNbr,
    TxEventsNbr,TxBuffersNbr,TxFifoQueueElmtsNbr,MessageRAMOffset;
} CAN_Init;
typedef struct {void *Instance;uint32_t State;CAN_Init Init;} FDCAN_HandleTypeDef;
typedef struct {uint32_t IdType,FilterIndex,FilterType,FilterConfig,FilterID1,FilterID2;} FDCAN_FilterTypeDef;
typedef struct {uint32_t IdType,RxFrameType,FDFormat,BitRateSwitch,DataLength,Identifier;} FDCAN_RxHeaderTypeDef;
typedef struct {uint32_t BusOff,ErrorPassive,Warning,LastErrorCode;} FDCAN_ProtocolStatusTypeDef;
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *,FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *,uint32_t,uint32_t,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigRxFifoOverwrite(FDCAN_HandleTypeDef *,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *,uint32_t,FDCAN_RxHeaderTypeDef *,uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *,FDCAN_ProtocolStatusTypeDef *);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *,uint32_t);
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *,uint32_t);
#endif
'''
HARNESS = r'''
#include "om_stm32_can.h"
#include "om_stm32_usb.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
uint32_t irq_mask,now;
static int stage,call,stops,starts,scenario,rx_error,bus_error;
static FDCAN_HandleTypeDef h[3];
static unsigned pending_frames[3];
static FDCAN_RxHeaderTypeDef incoming;
static om_usb board;
static uint8_t report[64];
uint32_t HAL_GetTick(void) {return now;}
int om_stm32_usb_status_provider(om_status_provider provider,void *context) {
    board.status=provider;board.status_context=context;return 0;
}
void om_stm32_usb_observation_fault(void) {om_fault(&board,9);}
static HAL_StatusTypeDef result(void) {return ++call==stage?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *c) {
    assert(c->Init.Mode==FDCAN_MODE_BUS_MONITORING && c->Init.FrameFormat==FDCAN_FRAME_CLASSIC);
    assert(!c->Init.TxFifoQueueElmtsNbr && !c->Init.TxBuffersNbr && !c->Init.TxEventsNbr);
    assert(c->Init.ExtFiltersNbr==1 && c->Init.StdFiltersNbr==0 && c->Init.RxFifo0ElmtsNbr==32);
    return result();
}
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *c,FDCAN_FilterTypeDef *f) {
    (void)c;assert(f->IdType==FDCAN_EXTENDED_ID && f->FilterID1==253 && f->FilterID2==255);
    return result();
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *c,uint32_t a,uint32_t b,uint32_t d,uint32_t e) {
    (void)c;assert(a==FDCAN_REJECT && b==FDCAN_REJECT && d==FDCAN_REJECT_REMOTE && e==FDCAN_REJECT_REMOTE);
    return result();
}
HAL_StatusTypeDef HAL_FDCAN_ConfigRxFifoOverwrite(FDCAN_HandleTypeDef *c,uint32_t fifo,uint32_t mode) {
    (void)c;assert(fifo==FDCAN_RX_FIFO0 && mode==FDCAN_RX_FIFO_BLOCKING);return result();
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *c,uint32_t flags,uint32_t indexes) {
    (void)c;assert((flags&FDCAN_IT_RX_FIFO0_MESSAGE_LOST) && !indexes);return result();
}
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *c,uint32_t flags) {
    (void)c;(void)flags;return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *c) {(void)c;++starts;return result();}
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *c) {(void)c;++stops;return HAL_OK;}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *c,uint32_t fifo) {
    assert(fifo==FDCAN_RX_FIFO0);return pending_frames[c-h];
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *c,uint32_t fifo,FDCAN_RxHeaderTypeDef *head,uint8_t *data) {
    assert(fifo==FDCAN_RX_FIFO0 && pending_frames[c-h]);
    if(rx_error) return HAL_ERROR;
    --pending_frames[c-h];*head=incoming;
    /* Deliberately exercise HAL's DLC-driven 64-byte copy for invalid FD. */
    memset(data,0,scenario==6?64:8);
    data[0]=data[2]=data[4]=0x80;data[6]=1;data[7]=44;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *c,FDCAN_ProtocolStatusTypeDef *s) {
    (void)c;memset(s,0,sizeof(*s));s->BusOff=bus_error==1;s->LastErrorCode=bus_error==2?3:0;
    return bus_error==3?HAL_ERROR:HAL_OK;
}
static void irq(unsigned bus,unsigned frames) {
    pending_frames[bus]+=frames;
    HAL_FDCAN_RxFifo0Callback(h+bus,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}
static unsigned faults(unsigned bus) {
    assert(board.status(board.status_context,255,now,report)==64);return report[12+bus];
}
static void joint(unsigned index) {assert(board.status(board.status_context,index,now,report)==64);}
int main(int argc,char **argv) {
    assert(argc==2);scenario=atoi(argv[1]);
    om_joint j[12];uint8_t nonce[16]={1},digest[32]={0};
    for(unsigned i=0;i<12;++i) j[i]=(om_joint){i/4,i%4+1,1,-1,1,0,1,50,1};
    assert(!om_init(&board,nonce,digest,j));
    FDCAN_HandleTypeDef *handles[3]={h,h+1,h+2};
    for(unsigned b=0;b<3;++b) {
        h[b].Instance=b==0?FDCAN1:(b==1?FDCAN2:FDCAN3);h[b].State=HAL_FDCAN_STATE_READY;
        h[b].Init.MessageRAMOffset=b==0?0u:(b==1?0x406u:0x812u);
    }
    if(scenario>=100) stage=scenario-100;
    if(scenario==20) h[1].State=2;
    if(scenario==21) h[2].Init.MessageRAMOffset=0;
    int init=om_stm32_can_init(handles,j);
    if(scenario>=100 || scenario==20 || scenario==21) {
        assert(init==-1 && board.fault==9 && faults(0)&OM_FB_INIT_ERROR);
        assert(om_stm32_can_init(handles,j)==-1);
        om_stm32_can_poll();return 0;
    }
    assert(init==0 && starts==3 && !stops);
    assert(om_stm32_can_init(handles,j)==-1);
    incoming=(FDCAN_RxHeaderTypeDef){FDCAN_EXTENDED_ID,FDCAN_DATA_FRAME,
        FDCAN_CLASSIC_CAN,FDCAN_BRS_OFF,FDCAN_DLC_BYTES_8,0x020001fdu};
    now=1;
    if(scenario==1 || scenario==2) {
        if(scenario==2) irq_mask=1;
        for(unsigned b=0;b<3;++b) irq(b,1);
        om_stm32_can_poll();
        for(unsigned b=0;b<3;++b) {joint(4*b);assert((report[6]&9)==9 && !faults(b));}
        assert(irq_mask==(scenario==2?1u:0u));
    } else if(scenario>=3 && scenario<=7) {
        if(scenario==3) incoming.IdType=0;
        if(scenario==4) incoming.RxFrameType=1;
        if(scenario==5) incoming.DataLength=7u<<16;
        if(scenario==6) {incoming.FDFormat=1;incoming.DataLength=15u<<16;}
        if(scenario==7) incoming.Identifier=0x20000000u;
        irq(1,1);om_stm32_can_poll();joint(4);
        assert(!(report[6]&1) && faults(1)&OM_FB_MALFORMED && board.fault==9);
    } else if(scenario==8) {
        for(unsigned k=0;k<OM_CAN_QUEUE_CAPACITY+1;++k) irq(0,1);
        om_stm32_can_poll();assert(faults(0)&OM_FB_OVERFLOW && board.fault==9);
    } else if(scenario==9) {
        irq(2,20);om_stm32_can_poll();assert(faults(2)&OM_FB_OVERFLOW);
        om_stm32_can_poll();assert(!pending_frames[2]);
    } else if(scenario==10) {
        HAL_FDCAN_RxFifo0Callback(h,FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
        om_stm32_can_poll();assert(faults(0)&OM_FB_OVERFLOW);
    } else if(scenario==11) {
        rx_error=1;irq(1,1);om_stm32_can_poll();assert(faults(1)&OM_FB_BUS_ERROR);
    } else if(scenario==12) {
        HAL_FDCAN_ErrorCallback(h+2);om_stm32_can_poll();assert(faults(2)&OM_FB_BUS_ERROR);
    } else if(scenario==13) {
        HAL_FDCAN_ErrorStatusCallback(h+1,FDCAN_IT_BUS_OFF);
        om_stm32_can_poll();assert(faults(1)&OM_FB_BUS_ERROR);
    } else if(scenario>=14 && scenario<=16) {
        bus_error=scenario-13;om_stm32_can_poll();assert(faults(0)&OM_FB_BUS_ERROR);
    } else if(scenario==17) {
        irq(0,1);now=101;om_stm32_can_poll();joint(0);
        assert(!(report[6]&1) && faults(0)&OM_FB_TIMEOUT);
    } else if(scenario==18) {
        now=100;om_stm32_can_poll();assert(board.fault==9);
        assert(faults(0)&OM_FB_TIMEOUT && faults(1)&OM_FB_TIMEOUT && faults(2)&OM_FB_TIMEOUT);
    } else if(scenario==19) {
        FDCAN_HandleTypeDef unknown={0};
        HAL_FDCAN_RxFifo0Callback(&unknown,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        HAL_FDCAN_ErrorCallback(&unknown);om_stm32_can_poll();assert(!board.fault);
    } else assert(0);
    return 0;
}
'''


class STM32CAN(unittest.TestCase):
    def test_hal_receive_and_failures(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            (tmp/'stm32h7xx_hal.h').write_text(HAL)
            (tmp/'stm32h7xx_hal_rng.h').write_text('#include "stm32h7xx_hal.h"\n')
            (tmp/'stm32h7xx_hal_fdcan.h').write_text(CAN)
            (tmp/'harness.c').write_text(HARNESS)
            binary = tmp/'harness'
            subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
                            '-fsanitize=address,undefined','-fno-omit-frame-pointer','-no-pie',
                            '-I'+str(tmp),'-I'+str(CORE),str(tmp/'harness.c'),
                            *[str(CORE/(m+'.c')) for m in ('om_usb','om_feedback','om_stm32_can')],
                            '-o',str(binary)],check=True)
            # 21 behavior cases + failures at all 18 HAL setup calls.
            for scenario in [*range(1,22), *range(101,119)]:
                with self.subTest(scenario=scenario):
                    # No heap allocations in the firmware. Keep ASan bounds and
                    # UBSan checks; LSan cannot inspect /proc in some sandboxes.
                    subprocess.run([str(binary),str(scenario)],check=True,
                                   env={**os.environ, 'ASAN_OPTIONS':'detect_leaks=0',
                                        'UBSAN_OPTIONS':'halt_on_error=1'})


if __name__ == '__main__':
    unittest.main()
