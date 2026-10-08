"""Cross-language tests exercise the actual C firmware core, without hardware."""
import ctypes
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from dm_mc02_protocol import *
from robstride04_driver import control

WRAPPER = r'''
#include "om_usb.h"
static om_usb board;
static om_joint joints[12];
void reset(unsigned seed) {
    uint8_t nonce[16]={0},digest[32]={0};nonce[0]=(uint8_t)seed;
    for(unsigned i=0;i<12;++i) joints[i]=(om_joint){i/4,i%4+1,1,-1,1,0,1,50,1};
    om_init(&board,nonce,digest,joints);
}
int feed(unsigned byte,unsigned now,uint8_t *out,size_t *n) {return om_feed(&board,(uint8_t)byte,now,out,n);}
void tick(unsigned now) {om_tick(&board,now);}
unsigned fault(void) {return board.fault;}
void disconnect(void) {om_fault(&board,8);}
int encode(float *q,om_can *out) {return om_encode_targets(joints,q,out);}
int invalid_config(unsigned mode) {
    uint8_t nonce[16]={1},digest[32]={0};
    if(mode==1) joints[0].ratio=0;
    if(mode==2) joints[1].motor_id=joints[0].motor_id;
    if(mode==3) joints[0].bus=2;
    if(mode==4) nonce[0]=0;
    return om_init(&board,nonce,digest,joints);
}
'''


class Can(ctypes.Structure):
    _fields_ = [('bus', ctypes.c_uint8), ('id', ctypes.c_uint32), ('data', ctypes.c_uint8*8)]


class Protocol(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        wrapper = Path(cls.tmp.name)/'wrapper.c'
        wrapper.write_text(WRAPPER)
        shared = Path(cls.tmp.name)/'board.so'
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                        '-I'+str(ROOT/'firmware/dm_mc02'), str(wrapper),
                        str(ROOT/'firmware/dm_mc02/om_usb.c'), '-o', str(shared)], check=True)
        cls.c = ctypes.CDLL(str(shared))
        cls.c.feed.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint8),
                               ctypes.POINTER(ctypes.c_size_t)]
        cls.c.encode.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(Can)]

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.c.reset(1)
        self.now = 100
        self.nonce = b'\x01'+bytes(15)

    def raw(self, raw):
        replies = []
        for byte in raw:
            out = (ctypes.c_uint8*98)()
            n = ctypes.c_size_t()
            self.c.feed(byte, self.now, out, ctypes.byref(n))
            if n.value:
                replies.append(Packet.decode(bytes(out[:n.value])))
        return replies

    def exchange(self, raw):
        replies = self.raw(raw)
        self.assertEqual(len(replies), 1)
        return replies[0].encode()

    def ticket(self):
        return Packet.decode(self.exchange(Packet(TICKET, self.nonce).encode()))

    def test_probe_and_validate(self):
        client = BenchClient(self.exchange, bytes(32))
        self.assertTrue(client.probe()['motion_locked'])
        self.assertFalse(client.validate_fresh(lambda: [0.]*12)['motor_output_enabled'])

    def test_firmware_crc_failure_latches(self):
        raw = bytearray(Packet(PROBE).encode());raw[-1] ^= 1
        self.assertEqual(self.raw(raw)[0].kind, ERROR)
        self.assertEqual(self.raw(Packet(PROBE).encode())[0].kind, ERROR)

    def test_stream_fragmentation_and_coalescing(self):
        raw = Packet(PROBE).encode()*100
        decoder = StreamDecoder(); packets = []
        for i in range(0, len(raw), 7): packets.extend(decoder.feed(raw[i:i+7]))
        self.assertEqual(len(packets), 100)
        self.assertEqual(len(decoder.buffer), 0)

    def test_decoder_corruption_latches(self):
        decoder = StreamDecoder()
        with self.assertRaises(ValueError): decoder.feed(b'XX'+Packet(PROBE).encode()[2:])
        with self.assertRaises(ValueError): decoder.feed(Packet(PROBE).encode())

    def test_ticket_expired_in_usb_queue(self):
        ticket = self.ticket();self.now += 50
        self.assertEqual(self.raw(validation_packet(ticket, [0.]*12).encode())[0].kind, ERROR)

    def test_replay(self):
        packet = validation_packet(self.ticket(), [0.]*12)
        self.assertEqual(self.raw(packet.encode())[0].kind, VALIDATE_REPLY)
        self.assertEqual(self.raw(packet.encode())[0].kind, ERROR)

    def test_previous_ticket_invalidated(self):
        old = self.ticket();self.ticket()
        self.assertEqual(self.raw(validation_packet(old, [0.]*12).encode())[0].kind, ERROR)

    def test_restart_nonce_invalidates_old_packet(self):
        packet = validation_packet(self.ticket(), [0.]*12)
        self.c.reset(2)
        self.assertEqual(self.raw(packet.encode())[0].kind, ERROR)

    def test_bad_last_joint_atomic(self):
        ticket = self.ticket()
        q = [0.]*11+[2.]
        self.assertEqual(self.raw(validation_packet(ticket, q).encode())[0].kind, ERROR)

    def test_nan_on_wire(self):
        ticket = self.ticket()
        packet = Packet(VALIDATE, ticket.nonce, ticket.sequence, ticket.deadline_ms,
                        struct.pack('<12f', *([0.]*11+[float('nan')])))
        self.assertEqual(self.raw(packet.encode())[0].kind, ERROR)

    def test_watchdog_without_usb_traffic(self):
        ticket = self.ticket();self.raw(validation_packet(ticket, [0.]*12).encode())
        self.c.tick(ticket.deadline_ms)
        self.assertEqual(self.c.fault(), 5)

    def test_new_ticket_cannot_refresh_validated_lease(self):
        first = self.ticket();self.raw(validation_packet(first, [0.]*12).encode())
        self.now += 40;self.ticket();self.c.tick(first.deadline_ms)
        self.assertEqual(self.c.fault(), 5)

    def test_tick_wrap(self):
        self.now = 0xfffffff0
        ticket = self.ticket()
        self.assertEqual(ticket.deadline_ms, 34)
        self.now = 10
        self.assertEqual(self.raw(validation_packet(ticket, [0.]*12).encode())[0].kind, VALIDATE_REPLY)
        self.c.tick(34);self.assertEqual(self.c.fault(), 5)

    def test_stop_latches(self):
        self.assertEqual(self.raw(Packet(STOP, self.nonce).encode())[0].kind, ERROR)
        self.assertEqual(self.c.fault(), 7)
        self.assertEqual(self.ticket().kind, ERROR)

    def test_disconnect_latches(self):
        self.ticket();self.c.disconnect()
        self.assertEqual(self.c.fault(), 8)
        self.assertEqual(self.ticket().kind, ERROR)

    def test_configuration_validation(self):
        for mode in (1, 2, 3, 4):
            self.c.reset(1)
            self.assertEqual(self.c.invalid_config(mode), -1)

    def test_configuration_digest_mismatch(self):
        client = BenchClient(self.exchange, bytes([1])*32)
        with self.assertRaises(RuntimeError): client.probe()
        self.assertTrue(client.failed)

    def test_rs04_codec_matches_existing_driver(self):
        rng = random.Random(42)
        for _ in range(100):
            q = [rng.uniform(-1, 1) for _ in range(12)]
            values = (ctypes.c_float*12)(*q);out = (Can*12)()
            self.assertEqual(self.c.encode(values, out), 0)
            for i, frame in enumerate(out):
                expected = control(i%4+1, position=values[i], kp=50, kd=1)
                self.assertEqual(frame.bus, i//4)
                self.assertEqual(frame.id, expected.arbitration_id)
                self.assertEqual(bytes(frame.data), expected.data)

    def test_invalid_batch_leaves_output_untouched(self):
        q = (ctypes.c_float*12)(*([0.]*11+[2.]));out = (Can*12)()
        ctypes.memset(out, 0xa5, ctypes.sizeof(out))
        before = bytes(out)
        self.assertEqual(self.c.encode(q, out), -1)
        self.assertEqual(bytes(out), before)

    def test_unsupported_motion_opcode_rejected(self):
        self.assertEqual(self.raw(Packet(5, self.nonce).encode())[0].kind, ERROR)


if __name__ == '__main__':
    unittest.main()
