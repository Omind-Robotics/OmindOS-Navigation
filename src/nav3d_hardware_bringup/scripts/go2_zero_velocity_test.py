#!/usr/bin/env python3
"""One-shot Go2 zero Move stream plus exactly one StopMove call."""

import argparse
import math
import threading
import time
from xml.sax.saxutils import escape

from go2_safety_supervisor_core import SafetyConfig, decode_remote


CONFIRM_TOKEN = 'ZERO_MOVE_AND_STOP_ONCE'
STATE_TIMEOUT_SECONDS = 3.0
RPC_TIMEOUT_SECONDS = 2.0
MAXIMUM_LINEAR_SPEED = 0.05
MAXIMUM_YAW_SPEED = 0.05
ZERO_COMMAND_SECONDS = 0.50
ZERO_COMMAND_RATE_HZ = 20.0


def _cyclonedds_config(interface, peer, allow_multicast):
    peer_section = ''
    if peer:
        peer_section = (
            '<Discovery><Peers>'
            f'<Peer Address="{escape(peer)}"/>'
            '</Peers></Discovery>'
        )
    return f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="{escape(interface)}"
          priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>{escape(allow_multicast)}</AllowMulticast>
    </General>
    {peer_section}
  </Domain>
</CycloneDDS>'''


class StateSnapshot:
    def __init__(self):
        self.lock = threading.Lock()
        self.lowstate_time = None
        self.remote_online = False
        self.remote_neutral = False
        self.sport_time = None
        self.mode = None
        self.gait_type = None
        self.velocity = None
        self.yaw_speed = None

    def lowstate_callback(self, message):
        now = time.monotonic()
        try:
            remote = decode_remote(message.wireless_remote, SafetyConfig())
            online = remote.online
            neutral = remote.online and remote.neutral
        except (TypeError, ValueError):
            online = False
            neutral = False
        with self.lock:
            self.lowstate_time = now
            self.remote_online = online
            self.remote_neutral = neutral

    def sport_callback(self, message):
        with self.lock:
            self.sport_time = time.monotonic()
            self.mode = int(message.mode)
            self.gait_type = int(message.gait_type)
            self.velocity = tuple(float(value) for value in message.velocity)
            self.yaw_speed = float(message.yaw_speed)

    def read(self):
        with self.lock:
            return {
                'lowstate_time': self.lowstate_time,
                'remote_online': self.remote_online,
                'remote_neutral': self.remote_neutral,
                'sport_time': self.sport_time,
                'mode': self.mode,
                'gait_type': self.gait_type,
                'velocity': self.velocity,
                'yaw_speed': self.yaw_speed,
            }


def wait_for_fresh(snapshot, timeout, newer_than=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = snapshot.read()
        received = (
            state['lowstate_time'] is not None
            and state['sport_time'] is not None
        )
        newer = (
            newer_than is None
            or (
                state['lowstate_time'] > newer_than
                and state['sport_time'] > newer_than
            )
        )
        if received and newer:
            return state
        time.sleep(0.01)
    raise RuntimeError(
        'timed out waiting for fresh LowState and SportModeState')


def stationary_failures(state):
    failures = []
    velocity = state.get('velocity')
    yaw_speed = state.get('yaw_speed')
    if not state.get('remote_online'):
        failures.append('remote is offline')
    if not state.get('remote_neutral'):
        failures.append('remote is not neutral')
    if velocity is None or yaw_speed is None:
        failures.append('motion state is incomplete')
    else:
        values = tuple(velocity) + (yaw_speed,)
        if not all(math.isfinite(value) for value in values):
            failures.append('motion state contains a non-finite value')
        else:
            linear_speed = math.hypot(velocity[0], velocity[1])
            if linear_speed > MAXIMUM_LINEAR_SPEED:
                failures.append(
                    f'linear speed {linear_speed:.6f} m/s exceeds threshold')
            if abs(yaw_speed) > MAXIMUM_YAW_SPEED:
                failures.append(
                    f'yaw speed {yaw_speed:.6f} rad/s exceeds threshold')
    if state.get('mode') != 0 or state.get('gait_type') != 0:
        failures.append(
            f"expected idle mode=0 gait=0, got mode={state.get('mode')} "
            f"gait={state.get('gait_type')}")
    return failures


def validate_stationary(state):
    failures = stationary_failures(state)
    if failures:
        raise RuntimeError('; '.join(failures))


def print_state(label, state):
    linear_speed = math.hypot(state['velocity'][0], state['velocity'][1])
    print(
        f"{label}: remote_online={state['remote_online']} "
        f"remote_neutral={state['remote_neutral']} mode={state['mode']} "
        f"gait={state['gait_type']} linear_speed_m_s={linear_speed:.9f} "
        f"yaw_speed_rad_s={state['yaw_speed']:.9f}",
        flush=True,
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Go2 literal-zero Move plus one StopMove acceptance')
    parser.add_argument('--interface', required=True)
    parser.add_argument('--peer', default='')
    parser.add_argument('--domain-id', type=int, default=0)
    parser.add_argument('--allow-multicast', default='spdp')
    parser.add_argument('--confirm-token', default='BLOCKED')
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv)
    print('GO2 ZERO VELOCITY: START (ZERO Move + ONE StopMove)', flush=True)
    if not args.interface or args.interface == '__REQUIRED__':
        print('GO2 ZERO VELOCITY: FAIL\nreason: interface is required')
        return 2
    if args.confirm_token != CONFIRM_TOKEN:
        print(
            f'GO2 ZERO VELOCITY: BLOCKED\nreason: explicit '
            f'confirm token {CONFIRM_TOKEN} is required',
            flush=True,
        )
        return 3

    from unitree_sdk2py.core.channel import ChannelSubscriber
    import unitree_sdk2py.core.channel as unitree_channel
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    unitree_channel.ChannelConfigHasInterface = _cyclonedds_config(
        args.interface, args.peer, args.allow_multicast)
    unitree_channel.ChannelFactoryInitialize(args.domain_id, args.interface)
    snapshot = StateSnapshot()
    lowstate_subscriber = ChannelSubscriber('rt/lf/lowstate', LowState_)
    sport_subscriber = ChannelSubscriber(
        'rt/lf/sportmodestate', SportModeState_)
    lowstate_subscriber.Init(snapshot.lowstate_callback, 10)
    sport_subscriber.Init(snapshot.sport_callback, 10)

    try:
        before = wait_for_fresh(snapshot, STATE_TIMEOUT_SECONDS)
        validate_stationary(before)
        print_state('PRE', before)

        client = SportClient()
        client.SetTimeout(RPC_TIMEOUT_SECONDS)
        client.Init()
        version_code, server_version = client.GetServerApiVersion()
        if (version_code != 0
                or server_version != client.GetApiVersion()):
            raise RuntimeError(
                f'Sport API version mismatch: code={version_code} '
                f'client={client.GetApiVersion()} server={server_version}')
        print(
            'Streaming literal Move(0.0, 0.0, 0.0); no nonzero command '
            'path and no SwitchJoystick call',
            flush=True,
        )
        interval = 1.0 / ZERO_COMMAND_RATE_HZ
        deadline = time.monotonic() + ZERO_COMMAND_SECONDS
        zero_calls = 0
        while time.monotonic() < deadline:
            code = int(client.Move(0.0, 0.0, 0.0))
            if code != 0:
                raise RuntimeError(f'zero Move returned code {code}')
            zero_calls += 1
            time.sleep(interval)
        zero_completed_at = time.monotonic()
        zero_after = wait_for_fresh(
            snapshot, STATE_TIMEOUT_SECONDS, newer_than=zero_completed_at)
        validate_stationary(zero_after)
        print_state(f'ZERO_POST calls={zero_calls}', zero_after)

        print('Calling official StopMove API 1003 exactly once', flush=True)
        stopped_at = time.monotonic()
        stop_code = int(client.StopMove())
        print(f'StopMove return_code={stop_code}', flush=True)
        after = wait_for_fresh(
            snapshot, STATE_TIMEOUT_SECONDS, newer_than=stopped_at)
        validate_stationary(after)
        print_state('POST', after)
        if stop_code != 0:
            raise RuntimeError(
                f'StopMove returned status={stop_code}; zero Move passed '
                'and state remained stationary')
    except Exception as error:
        print(f'GO2 ZERO VELOCITY: FAIL\nreason: {error}', flush=True)
        return 2
    finally:
        lowstate_subscriber.Close()
        sport_subscriber.Close()

    print('GO2 ZERO VELOCITY: PASS', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
