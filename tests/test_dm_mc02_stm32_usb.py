"""Exercise HAL binding with simulated IRQ/USB/RNG; not a hardware test."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / 'firmware/dm_mc02'
HAL = r'''
#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
typedef struct { int unused; } RNG_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
extern uint32_t irq_mask;
static inline uint32_t __get_PRIMASK(void) { return irq_mask; }
static inline void __disable_irq(void) { irq_mask = 1; }
static inline void __set_PRIMASK(uint32_t key) { irq_mask = key; }
static inline void __DMB(void) {}
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *, uint32_t *);
#endif
'''
USB = r'''
#ifndef MOCK_USB_H
#define MOCK_USB_H
#include <stdint.h>
#define USBD_OK 0
#define USBD_BUSY 1
#define USBD_FAIL 2
#define USBD_STATE_CONFIGURED 3
typedef struct { uint32_t TxState; uint8_t *TxBuffer; uint32_t TxLength; } USBD_CDC_HandleTypeDef;
typedef struct { uint8_t dev_state, classId; void *pClassDataCmsit[1]; } USBD_HandleTypeDef;
extern USBD_HandleTypeDef hUsbDeviceHS;
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *, uint8_t *, uint32_t);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *);
#endif
'''
HARNESS = r'''
#include "om_stm32_usb.h"
#include "usbd_cdc.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
uint32_t irq_mask, now;
static int rng_mode, rng_calls, sends, fatal;
static USBD_CDC_HandleTypeDef cdc;
USBD_HandleTypeDef hUsbDeviceHS;
static uint8_t snapshot[OM_MAX_FRAME];
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *rng,uint32_t *word) {
    assert(rng); ++rng_calls;
    if(rng_mode==1) return HAL_ERROR;
    *word=rng_mode==2?0:(uint32_t)rng_calls; return HAL_OK;
}
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *dev,uint8_t *buf,uint32_t length) {
    assert(dev==&hUsbDeviceHS && irq_mask==1 && !cdc.TxState);
    cdc.TxBuffer=buf;cdc.TxLength=length;return USBD_OK;
}
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *dev) {
    assert(dev==&hUsbDeviceHS && irq_mask==1);++sends;
    memcpy(snapshot,cdc.TxBuffer,cdc.TxLength);cdc.TxState=1;
    return fatal?USBD_FAIL:USBD_OK;
}
static void configure(void) {
    hUsbDeviceHS.dev_state=USBD_STATE_CONFIGURED;
    hUsbDeviceHS.pClassDataCmsit[0]=&cdc;
}
static void receive_probe(void) {
    uint8_t p[34]={0};p[0]='O';p[1]='M';p[2]=1;p[3]=1;
    uint32_t crc=0xffffffffu;
    for(unsigned i=0;i<30;++i) {
        crc^=p[i];for(unsigned k=0;k<8;++k) crc=(crc>>1)^((crc&1)?0xedb88320u:0);
    }
    crc=~crc;for(unsigned i=0;i<4;++i)p[30+i]=(uint8_t)(crc>>(8*i));
    om_stm32_usb_receive(p,7);om_stm32_usb_receive(p+7,27);
}
static size_t diagnostic(void *context,uint8_t selector,uint32_t tick,uint8_t out[64]) {
    assert(context==&now && selector==0 && tick==2);
    memset(out,0,64);out[0]=out[1]=1;out[7]=8;return 64;
}
static void receive_status(void) {
    uint8_t p[35]={0};p[0]='O';p[1]='M';p[2]=1;p[3]=5;p[4]=1;
    for(unsigned i=0;i<4;++i)p[6+4*i]=(uint8_t)(i+1);
    p[22]=42;
    uint32_t crc=0xffffffffu;
    for(unsigned i=0;i<31;++i) {
        crc^=p[i];for(unsigned k=0;k<8;++k) crc=(crc>>1)^((crc&1)?0xedb88320u:0);
    }
    crc=~crc;for(unsigned i=0;i<4;++i)p[31+i]=(uint8_t)(crc>>(8*i));
    om_stm32_usb_receive(p,sizeof(p));
}
int main(int argc,char **argv) {
    assert(argc==2);int scenario=atoi(argv[1]);
    RNG_HandleTypeDef rng={0};uint8_t digest[32]={0};om_joint joints[12];
    for(unsigned i=0;i<12;++i)joints[i]=(om_joint){i/4,i%4+1,1,-1,1,0,1,50,1};
    if(scenario==3)rng_mode=1;
    if(scenario==4)rng_mode=2;
    if(scenario==9)configure();
    int result=om_stm32_usb_init(&rng,digest,joints);
    if(scenario==3 || scenario==4 || scenario==9) {
        assert(result==-1 && om_stm32_usb_faulted());
        om_stm32_usb_poll();assert(!sends);
        assert(om_stm32_usb_init(&rng,digest,joints)==-1);return 0;
    }
    assert(result==0 && rng_calls==4 && !om_stm32_usb_faulted());
    assert(om_stm32_usb_init(&rng,digest,joints)==-1);
    if(scenario==11) {
        assert(om_stm32_usb_status_provider(diagnostic,&now)==0);
        assert(om_stm32_usb_status_provider(diagnostic,&now)==-1);
        om_stm32_usb_observation_fault();
    }
    configure();
    if(scenario==12) assert(om_stm32_usb_status_provider(diagnostic,&now)==-1);
    if(scenario==2)hUsbDeviceHS.pClassDataCmsit[0]=NULL;
    if(scenario==5)irq_mask=1;
    if(scenario==10)fatal=1;
    receive_probe();now=1;om_stm32_usb_poll();
    assert(irq_mask==(scenario==5?1u:0u));
    if(scenario==2) {
        assert(!sends);now=21;om_stm32_usb_poll();assert(om_stm32_usb_faulted());return 0;
    }
    assert(sends==1);
    assert(cdc.TxLength==67 && snapshot[3]==129 && snapshot[6]==1);
    if(scenario==11) {
        assert(om_stm32_usb_faulted()); /* CAN failure does not hide diagnostics */
        cdc.TxState=0;om_stm32_usb_complete(cdc.TxBuffer,cdc.TxLength);
        receive_status();now=2;om_stm32_usb_poll();
        assert(sends==2 && cdc.TxLength==98 && snapshot[3]==133 && snapshot[22]==42);
        assert(snapshot[37]==8 && om_stm32_usb_faulted());
        om_stm32_usb_disconnect();receive_status();now=3;om_stm32_usb_poll();
        assert(sends==2); /* USB disconnect still terminal */
    } else if(scenario==1 || scenario==5 || scenario==12) {
        receive_probe();now=2;om_stm32_usb_poll();
        assert(sends==1 && !memcmp(snapshot,cdc.TxBuffer,cdc.TxLength));
        cdc.TxState=0;om_stm32_usb_complete(cdc.TxBuffer,cdc.TxLength);
        now=3;om_stm32_usb_poll();assert(sends==2 && !om_stm32_usb_faulted());
    } else if(scenario==6) {
        /* A callback before TxState clears cannot free the response buffer. */
        om_stm32_usb_complete(cdc.TxBuffer,cdc.TxLength);
        now=2;om_stm32_usb_poll();assert(om_stm32_usb_faulted() && sends==1);
    } else if(scenario==7) {
        om_stm32_usb_disconnect();configure();receive_probe();
        now=2;om_stm32_usb_poll();assert(om_stm32_usb_faulted() && sends==1);
    } else if(scenario==8) {
        hUsbDeviceHS.dev_state=0;now=2;om_stm32_usb_poll();
        configure();receive_probe();now=3;om_stm32_usb_poll();
        assert(om_stm32_usb_faulted() && sends==1);
    } else if(scenario==10) {
        assert(om_stm32_usb_faulted());
        receive_probe();now=2;om_stm32_usb_poll();assert(sends==1);
    } else assert(0);
    assert(!memcmp(snapshot,cdc.TxBuffer,cdc.TxLength));
    return 0;
}
'''


class STM32USB(unittest.TestCase):
    def test_hal_binding(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            (tmp/'stm32h7xx_hal.h').write_text(HAL)
            (tmp/'stm32h7xx_hal_rng.h').write_text('#include "stm32h7xx_hal.h"\n')
            for name in ('usb_device.h', 'usbd_cdc.h'):
                (tmp/name).write_text(USB)
            (tmp/'harness.c').write_text(HARNESS)
            binary = tmp/'harness'
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(tmp), '-I'+str(CORE), str(tmp/'harness.c'),
                            *[str(CORE/(name+'.c')) for name in
                              ('om_usb', 'om_cdc', 'om_stm32_usb')],
                            '-o', str(binary)], check=True)
            for scenario in range(1, 13):
                with self.subTest(scenario=scenario):
                    subprocess.run([str(binary), str(scenario)], check=True)


if __name__ == '__main__':
    unittest.main()
