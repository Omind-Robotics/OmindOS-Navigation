"""RobStride04 private 29-bit CAN protocol and guarded multi-motor session.

Wire format: vendor RS04 User Manual 260713, sections 3.3.6, 4.1, 4.4.
No connection or transmission at import. No ID/zero/flash/fault-clear operations.
Actuator output-shaft units; ratio below is EXTERNAL transmission only.
"""
import math
import select
import socket
import struct
import time
from dataclasses import dataclass

POSITION = 12.57  # vendor C example; table rounds this to +/-4 pi
VELOCITY, TORQUE, KP, KD = 15., 120., 5000., 100.
HOST_ID = 253


@dataclass(frozen=True)
class Frame:
    arbitration_id: int
    data: bytes


def uint16(value, low, high):
    if isinstance(value, bool) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('CAN value outside range')
    return int((value-low)*65535/(high-low))


def frame(kind, motor_id, data=b'\x00'*8, data2=HOST_ID):
    if type(motor_id) is not int or not 1 <= motor_id <= 127 or len(data) != 8:
        raise ValueError('invalid motor ID or DLC')
    return Frame((kind << 24) | (data2 << 8) | motor_id, data)


def control(motor_id, position=0., velocity=0., kp=0., kd=0., torque=0.):
    return frame(1, motor_id, struct.pack('>4H', uint16(position, -POSITION, POSITION),
        uint16(velocity, -VELOCITY, VELOCITY), uint16(kp, 0., KP), uint16(kd, 0., KD)),
        uint16(torque, -TORQUE, TORQUE))


def read_parameter(motor_id, index):
    return frame(17, motor_id, struct.pack('<H6x', index))


def write_parameter(motor_id, index, value):
    # Deliberately narrow: volatile run mode and firmware communication watchdog.
    if index == 0x7005 and value == 0:
        raw = struct.pack('<B3x', value)
    elif index == 0x7028 and type(value) is int and 400 <= value <= 2000:
        raw = struct.pack('<I', value)
    else:
        raise ValueError('unsupported parameter write')
    return frame(18, motor_id, struct.pack('<H2x', index)+raw)


def feedback(packet):
    if not 0 <= packet.arbitration_id < 1 << 29 or len(packet.data) != 8:
        raise ValueError('invalid extended CAN frame')
    ident = packet.arbitration_id
    kind, motor, host = ident >> 24, (ident >> 8) & 255, ident & 255
    if host != HOST_ID:
        return None
    if kind == 2:
        p, v, t, temperature = struct.unpack('>4H', packet.data)
        return {'kind':kind, 'motor_id':motor, 'position':p*2*POSITION/65535-POSITION,
                'velocity':v*2*VELOCITY/65535-VELOCITY, 'torque':t*2*TORQUE/65535-TORQUE,
                'temperature':temperature/10, 'fault':(ident >> 16) & 63,
                'mode':(ident >> 22) & 3}
    if kind == 21:
        return {'kind':kind, 'motor_id':motor, 'fault':int.from_bytes(packet.data[:4], 'little'),
                'warning':int.from_bytes(packet.data[4:], 'little')}
    if kind == 17:
        return {'kind':kind, 'motor_id':motor, 'index':int.from_bytes(packet.data[:2], 'little'),
                'error':(ident >> 16) & 255, 'value':int.from_bytes(packet.data[4:], 'little')}
    return None


class SocketCAN:
    """Linux classic CAN. Kernel receive timestamps prevent stale queued frames.

    Interfaces must already exist at 1 Mbit/s. This never reconfigures the OS.
    A vendor USB adapter must expose SocketCAN; arbitrary USB serial is unsupported.
    """
    def __init__(self, channels):
        self.sockets = {}
        try:
            for channel in channels:
                sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sockets[channel] = sock
                sock.setsockopt(socket.SOL_SOCKET, 35, 1)  # SO_TIMESTAMPNS on Linux
                sock.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack('=I', 0x1fffffff))
                sock.bind((channel,)); sock.setblocking(False)
        except Exception:
            self.close(); raise

    def send(self, channel, packet):
        raw = struct.pack('=IB3x8s', packet.arbitration_id | 0x80000000, 8, packet.data)
        if self.sockets[channel].send(raw) != 16:
            raise OSError('short CAN write')

    def receive(self, timeout=0.):
        readable, _, _ = select.select(list(self.sockets.values()), [], [], timeout)
        if not readable:
            return None
        sock = readable[0]
        raw, ancillary, flags, _ = sock.recvmsg(16, 128)
        if len(raw) != 16 or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            raise OSError('truncated CAN frame')
        ident, dlc, data = struct.unpack('=IB3x8s', raw)
        if ident & 0x20000000:
            raise OSError('CAN bus error frame')
        if not ident & 0x80000000 or ident & 0x40000000 or dlc != 8:
            return ('', None, 0.)
        stamps = [struct.unpack('@ll', value[:struct.calcsize('@ll')])
                  for level, kind, value in ancillary if level == socket.SOL_SOCKET and kind == 35]
        if not stamps:
            raise OSError('missing CAN kernel receive timestamp')
        sec, ns = stamps[0]
        age = time.time()-(sec+ns/1e9)
        if not -.001 <= age <= .1:
            raise OSError('stale CAN receive queue')
        channel = next(k for k,v in self.sockets.items() if v is sock)
        return channel, Frame(ident & 0x1fffffff, data), time.monotonic()-max(0., age)

    def close(self):
        for sock in self.sockets.values():
            sock.close()
        self.sockets.clear()


def validate_calibration(config):
    if config.get('motor_model') != 'ROBSTRIDE04' or config.get('protocol') != 'private_29bit':
        raise ValueError('ROBSTRIDE04 private protocol required')
    if not config.get('robot_serial') or not config.get('calibration_record'):
        raise ValueError('machine identity and calibration record required')
    joints = config.get('joints', [])
    if len(joints) != 12:
        raise ValueError('expected twelve joints')
    names, addresses = set(), set()
    for joint in joints:
        if joint['name'] in names or (joint['channel'], joint['motor_id']) in addresses:
            raise ValueError('duplicate joint name or bus address')
        names.add(joint['name']); addresses.add((joint['channel'], joint['motor_id']))
        frame(4, joint['motor_id'])
        if not isinstance(joint['channel'], str) or not joint['channel']:
            raise ValueError('CAN channel required')
        for field in ('direction', 'offset_rad', 'external_ratio', 'lower', 'upper',
                      'effort_limit_nm', 'velocity_limit_rad_s', 'temperature_limit_c'):
            value = joint.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'calibration field required: {field}')
        if joint['direction'] not in (-1, 1) or not 0 < joint['external_ratio'] <= 100:
            raise ValueError('invalid direction or external ratio')
        if not joint['lower'] < joint['upper'] or not 0 < joint['temperature_limit_c'] <= 100:
            raise ValueError('invalid position or temperature limit')
        r, s, offset = joint['external_ratio'], joint['direction'], joint['offset_rad']
        if any(abs(offset+s*r*q) > POSITION for q in (joint['lower'], joint['upper'])):
            raise ValueError('calibrated joint range exceeds protocol position range')
        if not 0 < joint['effort_limit_nm']/r <= TORQUE or not 0 < joint['velocity_limit_rad_s']*r <= VELOCITY:
            raise ValueError('robot limits exceed motor encoding ranges')
    return joints


class MotorSession:
    """Only arm() writes setup/enable frames. Every failure latches and stops all.

    No hardware acceptance is implied by successful protocol transactions.
    Host PD is torque-limited before transmission (kp=kd=0 on the motor).
    """
    def __init__(self, config, transport, clock=time.monotonic):
        self.joints = validate_calibration(config)
        self.transport, self.clock = transport, clock
        self.addresses = {(j['channel'], j['motor_id']):j for j in self.joints}
        self.latest, self.parameters = {}, {}
        self.active, self.fault = False, None
        self.arming = False
        self.last_send = None

    def _send_all_stop(self):
        errors = []
        for channel, motor in self.addresses:
            try:
                self.transport.send(channel, frame(4, motor))
            except Exception as error:
                errors.append(str(error))
        self.active = False
        return errors

    def fail(self, reason):
        self.fault = str(reason)
        self._send_all_stop()
        raise RuntimeError(self.fault)

    def poll(self, timeout=0.):
        item = self.transport.receive(timeout)
        if item is None:
            return False
        channel, packet, received = item
        if packet is None:
            return True
        value = feedback(packet)
        if value is None:
            return True
        key = channel, value['motor_id']
        if key not in self.addresses:
            return True
        if not 0 <= self.clock()-received <= .1:
            raise RuntimeError('stale_or_future_motor_feedback')
        if value['kind'] == 17:
            if value['error']:
                raise RuntimeError('motor_parameter_read_failed')
            self.parameters[key+(value['index'],)] = (value['value'], received)
        elif value['kind'] == 21:
            if value['fault'] or value['warning']:
                raise RuntimeError('motor_fault_or_warning_frame')
        else:
            joint = self.addresses[key]
            if value['fault'] or value['temperature'] > joint['temperature_limit_c']:
                raise RuntimeError('motor_fault_or_temperature_limit')
            if self.active and value['mode'] != 2:
                raise RuntimeError('motor_left_run_mode')
            self.latest[key] = (value, received)
        return True

    def _wait(self, predicate, seconds=.15):
        deadline = self.clock()+seconds
        while not predicate():
            if self.clock() >= deadline:
                raise TimeoutError('motor_response_timeout')
            self.poll(min(.005, max(0., deadline-self.clock())))

    def _parameter(self, channel, motor, index, value):
        key = channel, motor, index
        self.parameters.pop(key, None)
        self.transport.send(channel, write_parameter(motor, index, value))
        sent = self.clock()
        self.transport.send(channel, read_parameter(motor, index))
        self._wait(lambda: key in self.parameters and self.parameters[key][1] >= sent)
        if self.parameters[key][0] != value:
            raise RuntimeError('motor_parameter_readback_mismatch')

    def arm(self, authorize=lambda: None):
        if self.fault or self.active:
            raise RuntimeError('session_not_armable')
        self.arming = True
        try:
            # Discard queued pre-arm packets before sending an explicit stop.
            for _ in range(1000):
                if not self.poll(): break
            else:
                raise RuntimeError('CAN_receive_flood')
            self.latest.clear()
            if self._send_all_stop():
                raise RuntimeError('could_not_stop_every_motor')
            self._wait(lambda: len(self.latest) == len(self.joints))
            if any(v['mode'] != 0 for v,_ in self.latest.values()):
                raise RuntimeError('motors_not_in_reset_mode')
            for channel, motor in self.addresses:
                self._parameter(channel, motor, 0x7028, 2000)  # 100 ms firmware watchdog
                self._parameter(channel, motor, 0x7005, 0)
                authorize()
            # Recheck every calibrated position before allowing enable frames.
            self.latest.clear()
            if self._send_all_stop():
                raise RuntimeError('could_not_stop_every_motor')
            self._wait(lambda: len(self.latest) == len(self.joints), .05)
            if any(v['mode'] != 0 for v,_ in self.latest.values()):
                raise RuntimeError('motors_not_in_reset_mode')
            self.sample()
            self.latest.clear()
            for channel, motor in self.addresses:
                authorize()
                self.transport.send(channel, frame(3, motor))
                self.transport.send(channel, control(motor))  # immediately replace retained PD/torque command
            self._wait(lambda: len(self.latest) == len(self.joints) and
                       all(v['mode'] == 2 for v,_ in self.latest.values()), .05)
            authorize()
            self.active, self.last_send = True, self.clock()
        except Exception as error:
            self.fail(error)
        finally:
            self.arming = False

    def sample(self, require_fresh=True):
        q, dq, times = [], [], []
        for joint in self.joints:
            value, received = self.latest[(joint['channel'], joint['motor_id'])]
            if require_fresh and not 0 <= self.clock()-received <= .05:
                raise RuntimeError('motor_feedback_timeout')
            r, s = joint['external_ratio'], joint['direction']
            pos = s*(value['position']-joint['offset_rad'])/r
            vel = s*value['velocity']/r
            if not joint['lower']-.03 <= pos <= joint['upper']+.03 or abs(vel) > joint['velocity_limit_rad_s']:
                raise RuntimeError('motor_mechanical_limit')
            q.append(pos); dq.append(vel); times.append(received)
        return q, dq, min(times)

    def cycle(self, effort):
        if not self.active or self.fault:
            raise RuntimeError('motor_session_inactive')
        try:
            if not 0 <= self.clock()-self.last_send <= .05:
                raise RuntimeError('motor_command_deadline_missed')
            for _ in range(256):
                if not self.poll(): break
            else:
                raise RuntimeError('CAN_receive_flood')
            self.sample()
            if len(effort) != len(self.joints):
                raise ValueError('effort count mismatch')
            packets = []
            for joint, torque in zip(self.joints, effort):
                if isinstance(torque, bool) or not math.isfinite(torque) or abs(torque) > joint['effort_limit_nm']:
                    raise ValueError('joint torque limit')
                motor_torque = joint['direction']*torque/joint['external_ratio']
                packets.append((joint['channel'], control(joint['motor_id'], torque=motor_torque)))
            for channel, packet in packets:
                self.transport.send(channel, packet)
            self.last_send = self.clock()
        except Exception as error:
            self.fail(error)

    def close(self):
        self._send_all_stop()
        self.transport.close()
