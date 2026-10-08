"""Actual C CAN decoder -> existing USB framing -> Python client, no hardware."""
import ctypes
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from dm_mc02_protocol import BenchClient, Packet, STATUS, STATUS_REPLY, TICKET, TICKET_REPLY, ERROR
from robstride04_driver import Frame, feedback as reference_feedback

WRAPPER = r'''
#include "om_feedback.h"
static om_usb core;
static om_feedback feedback;
void reset(unsigned now) {
    uint8_t nonce[16]={1},digest[32]={0};om_joint j[12];
    for(unsigned i=0;i<12;++i) j[i]=(om_joint){i/4,i%4+1,1,-1,1,0,1,50,1};
    om_init(&core,nonce,digest,j);om_feedback_init(&feedback,j,now);
    core.status=om_feedback_status;core.status_context=&feedback;
}
void receive(unsigned bus,unsigned id,const uint8_t *data,unsigned at,unsigned now) {
    om_feedback_receive(&feedback,(uint8_t)bus,id,data,at,now);
}
void tick(unsigned now) {
    om_feedback_tick(&feedback,now);
    if(om_feedback_faulted(&feedback)) om_fault(&core,9);
    om_tick(&core,now);
}
void bad(unsigned bus,unsigned flags) {om_feedback_fault(&feedback,(uint8_t)bus,(uint8_t)flags);}
int feed(unsigned byte,unsigned now,uint8_t *out,size_t *n) {
    return om_feed(&core,(uint8_t)byte,now,out,n);
}
'''


class Feedback(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = Path(cls.tmp.name)
        (tmp/'wrapper.c').write_text(WRAPPER)
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                        '-I'+str(ROOT/'firmware/dm_mc02'), str(tmp/'wrapper.c'),
                        *[str(ROOT/'firmware/dm_mc02'/x) for x in ('om_usb.c','om_feedback.c')],
                        '-o', str(tmp/'feedback.so')], check=True)
        cls.c = ctypes.CDLL(str(tmp/'feedback.so'))
        cls.c.receive.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint8),
                                  ctypes.c_uint, ctypes.c_uint]
        cls.c.feed.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint8),
                               ctypes.POINTER(ctypes.c_size_t)]

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.now = 10
        self.c.reset(self.now)
        self.client = BenchClient(self.exchange, bytes(32))
        self.client.probe()

    def exchange(self, raw):
        result = []
        for byte in raw:
            out = (ctypes.c_uint8*98)(); size = ctypes.c_size_t()
            self.c.feed(byte, self.now, out, ctypes.byref(size))
            if size.value:
                result.append(bytes(out[:size.value]))
        self.assertEqual(len(result), 1)
        return result[0]

    def receive(self, bus=0, motor=1, kind=2, host=253, status=0, data=None, at=None):
        if data is None:
            data = struct.pack('>4H', 34000, 33000, 32000, 325)
        ident = kind<<24 | status<<16 | motor<<8 | host
        self.c.receive(bus, ident, (ctypes.c_uint8*8).from_buffer_copy(data),
                       self.now if at is None else at, self.now)
        return ident, data

    def test_unseen_is_not_zero_feedback(self):
        value = self.client.read_status(0)
        self.assertFalse(value['seen']); self.assertFalse(value['fresh'])
        self.assertIsNone(value['motor_position_rad'])
        self.assertFalse(value['hardware_adapter_ready'])

    def test_all_twelve_routes_and_reference_values(self):
        for b in range(3):
            for motor in range(1,5):
                ident, data = self.receive(b, motor)
                value = self.client.read_status(b*4+motor-1)
                expected = reference_feedback(Frame(ident, data))
                for key, field in [('position','motor_position_rad'),('velocity','motor_velocity_rad_s'),
                                   ('torque','motor_torque_nm'),('temperature','temperature_c')]:
                    self.assertAlmostEqual(value[field], expected[key], places=4)
                self.assertTrue(value['fresh']);self.assertEqual(value['bus'],b)
        self.assertEqual([b['accepted'] for b in self.client.read_status()['buses']], [4,4,4])

    def test_same_motor_id_different_bus_is_isolated(self):
        self.receive(2,1)
        self.assertFalse(self.client.read_status(0)['seen'])
        self.assertTrue(self.client.read_status(8)['seen'])

    def test_unknown_host_id_type_are_ignored(self):
        self.receive(host=254);self.receive(motor=99);self.receive(kind=24)
        self.assertEqual(self.client.read_status()['buses'][0]['ignored'],3)
        self.assertFalse(self.client.read_status(0)['seen'])

    def test_timeout_even_without_usb_traffic(self):
        self.receive();self.now += 100;self.c.tick(self.now)
        value = self.client.read_status(0)
        self.assertTrue(value['seen']);self.assertFalse(value['fresh'])
        self.assertEqual(value['bus_latched_fault'] & 8,8)
        self.assertEqual(Packet.decode(self.exchange(Packet(TICKET,self.client.nonce).encode())).kind,ERROR)
        self.assertTrue(self.client.probe()['motion_locked'])

    def test_late_frame_cannot_clear_latch(self):
        self.now += 100;self.c.tick(self.now);self.receive()
        value = self.client.read_status(0)
        self.assertTrue(value['fresh']);self.assertTrue(value['any_latched_fault'])

    def test_never_seen_times_out(self):
        self.now += 100;self.c.tick(self.now)
        self.assertTrue(all(b['latched_fault'] & 8 for b in self.client.read_status()['buses']))

    def test_expired_and_future_queue_data_rejected(self):
        self.receive(at=self.now-100);self.receive(at=self.now+1)
        self.assertFalse(self.client.read_status(0)['seen'])
        self.assertEqual(self.client.read_status()['buses'][0]['latched_fault'],8)

    def test_out_of_order_state_does_not_overwrite(self):
        self.receive();self.now += 2;self.receive(at=self.now-3)
        self.assertEqual(self.client.read_status(0)['sample_count'],1)

    def test_tick_wrap(self):
        self.now = 0xfffffff0;self.c.reset(self.now);self.receive()
        self.now = 20
        self.assertTrue(self.client.read_status(0)['fresh'])
        self.now = 84;self.c.tick(self.now)
        self.assertFalse(self.client.read_status(0)['fresh'])

    def test_fault_frame_and_warning_latch(self):
        self.receive(kind=21,data=struct.pack('<II',4,8));self.c.tick(self.now)
        value = self.client.read_status(0)
        self.assertTrue(value['fault_frame_seen']);self.assertEqual(value['fault_bits'],4)
        self.assertEqual(value['warning_bits'],8);self.assertFalse(value['seen'])
        self.assertEqual(value['bus_latched_fault'],16)

    def test_running_mode_and_saturation_are_not_healthy(self):
        self.receive(status=128)
        self.assertEqual(self.client.read_status(0)['motor_mode'],2)
        self.receive(1,1,data=struct.pack('>4H',0,32768,32768,300))
        self.assertEqual(self.client.read_status(4)['bus_latched_fault'],16)

    def test_passive_parameter_is_not_verified_readback(self):
        self.receive(kind=17,data=struct.pack('<H2xI',0x7028,400))
        value = self.client.read_status(0)
        p = value['parameter_observation']
        self.assertFalse(value['seen']);self.assertEqual(p['index'],0x7028)
        self.assertEqual(p['raw_value'],400);self.assertFalse(p['verified_readback'])
        self.now += 100
        self.assertFalse(self.client.read_status(0)['parameter_observation']['fresh'])

    def test_parameter_error_and_reserved_bytes(self):
        self.receive(kind=17,status=1,data=struct.pack('<H2xI',0x7028,400))
        self.assertEqual(self.client.read_status(0)['bus_latched_fault'],16)
        self.receive(1,1,kind=17,data=bytes([1,0,1,0,0,0,0,0]))
        self.assertEqual(self.client.read_status()['buses'][1]['malformed_reports'],1)

    def test_bus_errors_and_overflow_remain_visible(self):
        self.c.bad(0,4);self.c.bad(1,2);self.c.bad(2,1);self.c.tick(self.now)
        status = self.client.read_status()['buses']
        self.assertEqual([x['latched_fault'] for x in status],[4,2,1])
        self.assertEqual(status[1]['overflow_reports'],1)

    def test_status_nonce_and_correlation(self):
        raw = Packet(STATUS,self.client.nonce,47,0,b'\x00').encode()
        reply = Packet.decode(self.exchange(raw))
        self.assertEqual((reply.kind,reply.sequence,reply.deadline_ms),(STATUS_REPLY,47,0))
        self.assertEqual(Packet.decode(self.exchange(Packet(STATUS,bytes(16),0,0,b'\x00').encode())).kind,ERROR)

    def test_status_does_not_renew_validation_lease(self):
        self.client.validate_fresh(lambda:[0.]*12)
        self.now += 49;self.client.read_status(0)
        self.now += 1;self.c.tick(self.now)
        self.assertEqual(Packet.decode(self.exchange(Packet(TICKET,self.client.nonce).encode())).payload,b'\x05')

    def test_client_rejects_replayed_query_and_latches(self):
        first = self.exchange(Packet(STATUS,self.client.nonce,1,0,b'\x00').encode())
        self.client.exchange = lambda _:first
        self.client.read_status(0)
        with self.assertRaisesRegex(RuntimeError,'stale_validation_ack'):
            self.client.read_status(0)
        self.assertTrue(self.client.failed)

    def test_bad_payload_selector_rejected(self):
        with self.assertRaises(ValueError):self.client.read_status(12)
        reply=Packet.decode(self.exchange(Packet(STATUS,self.client.nonce,1,0,b'\xfe').encode()))
        self.assertEqual(reply.kind,ERROR)


if __name__ == '__main__':
    unittest.main()
