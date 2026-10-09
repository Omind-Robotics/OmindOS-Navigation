"""Observer mode must work without fabricated motion calibration."""
import ctypes
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from dm_mc02_protocol import Packet, PROBE, PROBE_REPLY, STATUS, STATUS_REPLY, TICKET, ERROR, VALIDATE
from build_dm_mc02_bench import routing

WRAPPER = r'''
#include "om_usb.h"
static om_usb board;
static size_t status(void *c,uint8_t selector,uint32_t now,uint8_t p[64]) {
    (void)c;(void)now;p[0]=1;p[1]=1;p[2]=selector;return 3;
}
int reset(unsigned mode) {
    om_joint joints[12]={0};uint8_t nonce[16]={1},digest[32]={0};
    for(unsigned i=0;i<12;++i) {joints[i].bus=i/4;joints[i].motor_id=i%4+1;}
    if(mode==1) joints[1].motor_id=1;
    if(mode==2) joints[0].bus=3;
    if(mode==3) nonce[0]=0;
    int result=om_init_observer(&board,nonce,digest,joints);
    board.status=status;return result;
}
int feed(unsigned byte,uint8_t *out,size_t *n) {return om_feed(&board,byte,10,out,n);}
unsigned lease(void) {return board.lease|board.ticket_pending;}
'''


class Observer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        wrapper = Path(cls.tmp.name)/'wrapper.c'; wrapper.write_text(WRAPPER)
        cls.boards = []
        for index, flags in enumerate(([], ['-DOM_BENCH_OBSERVER_ONLY'])):
            shared = Path(cls.tmp.name)/f'board{index}.so'
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                            *flags, '-I'+str(ROOT/'firmware/dm_mc02'), str(wrapper),
                            str(ROOT/'firmware/dm_mc02/om_usb.c'), '-o', str(shared)], check=True)
            board = ctypes.CDLL(str(shared))
            board.feed.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_size_t)]
            cls.boards.append(board)

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def exchange(self, board, packet):
        replies = []
        for byte in packet.encode():
            out = (ctypes.c_uint8*98)(); n = ctypes.c_size_t()
            board.feed(byte, out, ctypes.byref(n))
            if n.value: replies.append(Packet.decode(bytes(out[:n.value])))
        self.assertEqual(len(replies), 1)
        return replies[0]

    def test_zero_calibration_is_sufficient_for_observer(self):
        for board in self.boards:
            self.assertEqual(board.reset(0), 0)
            self.assertEqual(self.exchange(board, Packet(PROBE)).kind, PROBE_REPLY)
            self.assertEqual(self.exchange(board, Packet(STATUS, b'\x01'+bytes(15), payload=b'\xff')).kind, STATUS_REPLY)

    def test_ticket_and_target_never_create_lease(self):
        for board in self.boards:
            for kind, payload in ((TICKET, b''), (VALIDATE, bytes(48))):
                board.reset(0)
                self.assertEqual(self.exchange(board, Packet(kind, b'\x01'+bytes(15), payload=payload)).kind, ERROR)
                self.assertEqual(board.lease(), 0)
                self.assertEqual(self.exchange(board, Packet(PROBE)).kind, PROBE_REPLY)

    def test_invalid_routing_and_zero_nonce_fail_closed(self):
        for board in self.boards:
            for mode in (1, 2, 3): self.assertEqual(board.reset(mode), -1)

    def test_routing_digest_and_no_calibration_fields(self):
        header, digest, config = routing(ROOT/'firmware/dm_mc02/bench/observer-routing.json')
        self.assertEqual(len(digest), 64)
        self.assertIn('.motor_id=4', header)
        self.assertFalse(config['routing_verified_on_hardware'])
        self.assertNotIn('kp=', header)

    def test_route_duplicates_and_extra_motion_fields_rejected(self):
        config = json.loads((ROOT/'firmware/dm_mc02/bench/observer-routing.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'routes.json'
            config['routes'][0]['kp'] = 50; path.write_text(json.dumps(config))
            with self.assertRaises(ValueError): routing(path)
            del config['routes'][0]['kp']
            config['routes'][1] = config['routes'][0]; path.write_text(json.dumps(config))
            with self.assertRaises(ValueError): routing(path)
