#!/usr/bin/env python3
"""Read-only Unitree Go2 SDK connectivity probe."""

import argparse
import time

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo2

from dds_config import add_dds_arguments, initialize_channel, resolved_interface


def parse_args():
    parser = argparse.ArgumentParser(description="Read-only Go2 SDK/DDS connectivity probe")
    add_dds_arguments(parser)
    parser.add_argument(
        "--topic",
        default="rt/lf/lowstate",
        help="LowState DDS topic to subscribe (default: rt/lf/lowstate)",
    )
    parser.add_argument(
        "--listen-seconds",
        type=float,
        default=6.0,
        help="seconds to listen for LowState messages",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=2.0,
        help="RobotState ServiceList timeout in seconds",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print(
        f"Initializing Go2 DDS: iface={resolved_interface(args)}, "
        f"peer={args.peer or 'none'}, domain={args.domain_id}",
        flush=True,
    )
    initialize_channel(args)

    print("Calling RobotState.ServiceList()...", flush=True)
    client = RobotStateClient()
    client.SetTimeout(args.service_timeout)
    client.Init()
    code, services = client.ServiceList()
    print(f"ServiceList code: {code}", flush=True)
    if services:
        for service in services:
            print(
                f"  {service.name}: status={service.status}, protect={service.protect}",
                flush=True,
            )

    counts = {"lowstate": 0}

    def lowstate_handler(msg):
        counts["lowstate"] += 1
        if counts["lowstate"] == 1:
            remote = list(msg.wireless_remote[:8])
            print(f"First {args.topic}: wireless_remote[:8]={remote}", flush=True)

    subscriber = ChannelSubscriber(args.topic, LowStateGo2)
    subscriber.Init(lowstate_handler, 10)
    print(f"Listening {args.listen_seconds:.1f}s on {args.topic}...", flush=True)
    time.sleep(args.listen_seconds)
    print(f"LowState messages: {counts['lowstate']}", flush=True)


if __name__ == "__main__":
    main()
