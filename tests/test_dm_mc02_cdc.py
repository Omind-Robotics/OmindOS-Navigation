"""Compile and exercise the real C CDC transport, without motor hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / 'firmware/dm_mc02'
if not CORE.exists():
    CORE = Path(__file__).resolve().parent

HARNESS = r'''
#include "om_cdc.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static om_cdc board;
static int depth,calls,busy,fatal;
static const uint8_t *owned;
static size_t owned_len;
static uint8_t snapshot[OM_MAX_FRAME];
static uint32_t enter(void *p) {(void)p;assert(!depth);depth=1;return 73;}
static void leave(void *p,uint32_t k) {(void)p;assert(depth && k==73);depth=0;}
static int submit(void *p,const uint8_t *data,size_t n) {
    (void)p;assert(!depth);++calls;
    if(fatal) return -1;
    if(busy) return 1;
    owned=data;owned_len=n;memcpy(snapshot,data,n);return 0;
}
static void setup(void) {
    om_joint j[12];uint8_t nonce[16]={1},digest[32]={0};
    for(unsigned i=0;i<12;++i) j[i]=(om_joint){i/4,i%4+1,1,-1,1,0,1,50,1};
    assert(!om_cdc_init(&board,(om_cdc_io){NULL,enter,leave,submit},nonce,digest,j));
}
static void probe(uint8_t p[34]) {
    memset(p,0,34);p[0]='O';p[1]='M';p[2]=1;p[3]=1;
    uint32_t crc=0xffffffffu;
    for(unsigned i=0;i<30;++i) {
        crc^=p[i];for(unsigned k=0;k<8;++k) crc=(crc>>1)^((crc&1)?0xedb88320u:0);
    }
    crc=~crc;for(unsigned i=0;i<4;++i) p[30+i]=(uint8_t)(crc>>(8*i));
}
int main(int argc,char **argv) {
    assert(argc==2);int test=atoi(argv[1]);setup();uint8_t p[34];probe(p);
    switch(test) {
    case 1: /* split packet and async ownership, including queued next reply */
        om_cdc_receive(&board,p,7);om_cdc_poll(&board,1);assert(!calls);
        om_cdc_receive(&board,p+7,27);om_cdc_poll(&board,2);
        assert(calls==1 && board.active && owned_len==67 && owned[3]==129);
        om_cdc_receive(&board,p,34);om_cdc_poll(&board,3);
        assert(calls==1 && !memcmp(owned,snapshot,owned_len));
        om_cdc_tx_complete(&board);om_cdc_poll(&board,4);
        assert(calls==2 && board.active);break;
    case 2: /* overflow rejects whole chunk and cannot recover via a probe */
        {uint8_t bytes[257]={0};om_cdc_receive(&board,bytes,sizeof(bytes));}
        assert(!calls && !board.count);om_cdc_poll(&board,1);
        assert(board.transport_fault && board.core.fault);
        om_cdc_receive(&board,p,34);om_cdc_poll(&board,2);assert(!calls);break;
    case 3: /* busy deadline starts once, not on each retry */
        busy=1;om_cdc_receive(&board,p,34);om_cdc_poll(&board,10);
        om_cdc_poll(&board,29);assert(calls==2 && !board.transport_fault);
        om_cdc_poll(&board,30);assert(calls==2 && board.transport_fault);break;
    case 4: /* accepted transmission never completed; preserve DMA bytes */
        om_cdc_receive(&board,p,34);om_cdc_poll(&board,10);om_cdc_poll(&board,30);
        assert(board.transport_fault && !memcmp(owned,snapshot,owned_len));break;
    case 5: /* disconnect after USB owns response, no automatic restart */
        om_cdc_receive(&board,p,34);om_cdc_poll(&board,1);om_cdc_link_fault(&board);
        om_cdc_poll(&board,2);om_cdc_tx_complete(&board);
        om_cdc_receive(&board,p,34);om_cdc_poll(&board,3);
        assert(calls==1 && board.transport_fault && !memcmp(owned,snapshot,owned_len));break;
    case 6: /* independent watchdog runs while USB is busy */
        busy=1;board.core.lease=1;board.core.lease_deadline=15;
        om_cdc_receive(&board,p,34);om_cdc_poll(&board,10);om_cdc_poll(&board,15);
        assert(board.core.fault==5 && !board.transport_fault);break;
    case 7: /* unsigned TX timer wraps correctly */
        busy=1;om_cdc_receive(&board,p,34);om_cdc_poll(&board,0xfffffff8u);
        om_cdc_poll(&board,11);assert(!board.transport_fault);
        om_cdc_poll(&board,12);assert(board.transport_fault);break;
    case 8: /* fatal submit response */
        fatal=1;om_cdc_receive(&board,p,34);om_cdc_poll(&board,1);
        assert(board.transport_fault && board.core.fault);break;
    default: assert(0);
    }
    assert(!depth);return 0;
}
'''

class CDCTransport(unittest.TestCase):
    def test_transport_faults_and_buffer_lifetime(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'cdc_test.c'
            source.write_text(HARNESS)
            binary = Path(tmp) / 'cdc_test'
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(CORE), str(source), str(CORE/'om_usb.c'),
                            str(CORE/'om_cdc.c'), '-o', str(binary)], check=True)
            for scenario in range(1, 9):
                with self.subTest(scenario=scenario):
                    subprocess.run([str(binary), str(scenario)], check=True)

if __name__ == '__main__':
    unittest.main()
