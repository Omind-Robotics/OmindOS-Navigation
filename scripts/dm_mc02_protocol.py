"""OmindOS USB CDC v1 bench protocol. No port opens or motor enables at import.

This protocol is project firmware, not the manufacturer's preinstalled protocol.
Only a locked bench validator is implemented; no hardware motion backend exists.
"""
from dataclasses import dataclass
import math
import struct
import zlib

MAGIC, VERSION = b'OM', 1
HEADER = struct.Struct('<2sBBH16sII')
MAX_PAYLOAD = 64
PROBE, TICKET, VALIDATE, STOP = 1, 2, 3, 4
PROBE_REPLY, TICKET_REPLY, VALIDATE_REPLY, ERROR = 0x81, 0x82, 0x83, 0xff
STATUS, STATUS_REPLY = 5, 0x85
ZERO_NONCE = bytes(16)


@dataclass(frozen=True)
class Packet:
    kind: int
    nonce: bytes = ZERO_NONCE
    sequence: int = 0
    deadline_ms: int = 0
    payload: bytes = b''

    def encode(self):
        if type(self.kind) is not int or not 0 <= self.kind <= 255:
            raise ValueError('packet_kind')
        if type(self.nonce) is not bytes or len(self.nonce) != 16:
            raise ValueError('boot_nonce')
        if any(type(v) is not int or not 0 <= v <= 0xffffffff
               for v in (self.sequence, self.deadline_ms)):
            raise ValueError('packet_counter')
        if type(self.payload) is not bytes or len(self.payload) > MAX_PAYLOAD:
            raise ValueError('payload_length')
        body = HEADER.pack(MAGIC, VERSION, self.kind, len(self.payload),
                           self.nonce, self.sequence, self.deadline_ms)+self.payload
        return body+struct.pack('<I', zlib.crc32(body))

    @classmethod
    def decode(cls, raw):
        if len(raw) < HEADER.size+4:
            raise ValueError('truncated_packet')
        magic, version, kind, length, nonce, seq, deadline = HEADER.unpack_from(raw)
        if magic != MAGIC or version != VERSION or length > MAX_PAYLOAD:
            raise ValueError('packet_header')
        if len(raw) != HEADER.size+length+4:
            raise ValueError('packet_length')
        if zlib.crc32(raw[:-4]) != struct.unpack_from('<I', raw, len(raw)-4)[0]:
            raise ValueError('packet_crc')
        return cls(kind, nonce, seq, deadline, raw[HEADER.size:-4])


class StreamDecoder:
    """Bounded CDC fragmentation/coalescing parser; corrupt input fails closed.

    The caller must terminate the connection on any exception. Resynchronizing
    past corrupt bytes could hide a lost STOP or malformed target batch.
    """
    def __init__(self):
        self.buffer = bytearray()
        self.failed = False

    def feed(self, data):
        if self.failed:
            raise ValueError('decoder_fault_latched')
        result = []
        try:
            for byte in data:
                self.buffer.append(byte)
                if len(self.buffer) >= HEADER.size:
                    magic, version, _, length, *_ = HEADER.unpack_from(self.buffer)
                    if magic != MAGIC or version != VERSION or length > MAX_PAYLOAD:
                        raise ValueError('packet_header')
                    if len(self.buffer) == HEADER.size+length+4:
                        result.append(Packet.decode(bytes(self.buffer)))
                        self.buffer.clear()
            return result
        except Exception:
            self.failed = True
            self.buffer.clear()
            raise


def validation_packet(ticket, positions):
    """A board-issued single-use ticket bounds time spent in the USB queues.

    Request the ticket immediately before constructing the fresh batch. Board
    expiry is absolute on the MCU clock; receipt cannot renew an old command.
    This does not accept a queued PositionCommand and restamp it as fresh.
    """
    if ticket.kind != TICKET_REPLY or ticket.payload or ticket.nonce == ZERO_NONCE:
        raise ValueError('invalid_ticket')
    if not isinstance(positions, (list, tuple)) or len(positions) != 12:
        raise ValueError('twelve_joint_positions_required')
    if any(isinstance(q, bool) or not isinstance(q, (int, float)) or not math.isfinite(q)
           for q in positions):
        raise ValueError('non_finite_joint_target')
    return Packet(VALIDATE, ticket.nonce, ticket.sequence, ticket.deadline_ms,
                  struct.pack('<12f', *positions))


class BenchClient:
    """Injected exchange transport, explicitly locked to non-actuating validation.

    exchange(bytes)->bytes must apply a finite I/O timeout and close on errors.
    This class does not open serial ports, request ARM, or emit CAN messages.
    """
    hardware_adapter_ready = False

    def __init__(self, exchange, expected_configuration_digest):
        if type(expected_configuration_digest) is not bytes or len(expected_configuration_digest) != 32:
            raise ValueError('configuration_digest_required')
        self.exchange = exchange
        self.digest = expected_configuration_digest
        self.nonce = None
        self.failed = False
        self.query_sequence = 0

    def _exchange(self, packet, kind):
        if self.failed:
            raise RuntimeError('bench_client_fault_latched')
        try:
            reply = Packet.decode(self.exchange(packet.encode()))
            if reply.kind != kind or reply.nonce == ZERO_NONCE:
                raise RuntimeError('board_error_or_wrong_reply')
            if self.nonce is not None and reply.nonce != self.nonce:
                raise RuntimeError('board_reboot_requires_new_client')
            if kind in (VALIDATE_REPLY, STATUS_REPLY) and (reply.sequence != packet.sequence or
                                          reply.deadline_ms != packet.deadline_ms):
                raise RuntimeError('stale_validation_ack')
            return reply
        except Exception:
            self.failed = True
            raise

    def probe(self):
        reply = self._exchange(Packet(PROBE), PROBE_REPLY)
        if reply.payload != b'\x01'+self.digest:
            self.failed = True
            raise RuntimeError('board_must_be_locked_with_matching_configuration')
        self.nonce = reply.nonce
        return {'protocol_version': VERSION, 'motion_locked': True,
                'configuration_digest': self.digest.hex(), 'hardware_adapter_ready': False}

    def validate_fresh(self, make_positions):
        if self.nonce is None:
            raise RuntimeError('probe_required')
        # Callback runs after the ticket arrives, avoiding a cached target batch.
        ticket = self._exchange(Packet(TICKET, self.nonce), TICKET_REPLY)
        reply = self._exchange(validation_packet(ticket, make_positions()), VALIDATE_REPLY)
        if reply.payload != b'\x01':
            self.failed = True
            raise RuntimeError('unexpected_bench_ack')
        return {'accepted_for_bench_validation': True, 'motor_output_enabled': False}

    def read_status(self, selector=255):
        """Read one joint (0..11) or bus diagnostics (255); never sends CAN.

        Observed parameter replies are passive captures, not a verified readback
        transaction. A fresh sample does not clear a latched CAN fault.
        """
        if self.nonce is None:
            raise RuntimeError('probe_required')
        if type(selector) is not int or selector not in (*range(12), 255):
            raise ValueError('status_selector')
        if self.query_sequence == 0xffffffff:
            self.failed = True
            raise RuntimeError('status_sequence_exhausted')
        self.query_sequence += 1
        reply = self._exchange(Packet(STATUS, self.nonce, self.query_sequence, 0,
                                      bytes([selector])), STATUS_REPLY)
        try:
            return decode_status(reply.payload, selector)
        except Exception:
            self.failed = True
            raise


def decode_status(data, selector):
    if len(data) != 64 or data[:2] != b'\x01\x01' or data[2] not in (0, 1) or data[3] != selector:
        raise ValueError('status_payload')
    result = {'motion_locked': True, 'receiver_initialized': bool(data[2]),
              'selector': selector, 'hardware_adapter_ready': False}
    if selector == 255:
        result.update(now_ms=int.from_bytes(data[4:8], 'little'),
                      timeout_ms=int.from_bytes(data[8:12], 'little'))
        result['buses'] = [dict(zip(('accepted', 'ignored', 'malformed_reports', 'overflow_reports'),
                                    struct.unpack_from('<4I', data, 16+16*b)),
                                bus=b, latched_fault=data[12+b]) for b in range(3)]
        return result
    if selector not in range(12) or data[4] >= 3 or data[6] & ~31:
        raise ValueError('status_joint')
    values = struct.unpack_from('<4f', data, 24)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('status_nonfinite')
    seen, fresh = bool(data[6] & 1), bool(data[6] & 8)
    if fresh and not seen:
        raise ValueError('status_fresh_without_sample')
    result.update(bus=data[4], motor_id=data[5], seen=seen, fresh=fresh,
                  bus_latched_fault=data[7], any_latched_fault=bool(data[8]),
                  motor_mode=data[9], motor_fault=data[10],
                  now_ms=int.from_bytes(data[12:16], 'little'),
                  received_ms=int.from_bytes(data[16:20], 'little') if seen else None,
                  sample_count=int.from_bytes(data[20:24], 'little'),
                  fault_frame_seen=bool(data[6] & 4),
                  fault_bits=int.from_bytes(data[40:44], 'little'),
                  warning_bits=int.from_bytes(data[44:48], 'little'))
    result.update(zip(('motor_position_rad', 'motor_velocity_rad_s', 'motor_torque_nm', 'temperature_c'),
                      values if seen else (None,)*4))
    result['parameter_observation'] = None
    if data[6] & 2:
        result['parameter_observation'] = {
            'fresh': bool(data[6] & 16), 'error': data[11],
            'index': int.from_bytes(data[48:50], 'little'),
            'raw_value': int.from_bytes(data[52:56], 'little'),
            'received_ms': int.from_bytes(data[56:60], 'little'),
            'verified_readback': False}
    return result
