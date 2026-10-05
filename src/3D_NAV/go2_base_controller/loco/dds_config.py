#!/usr/bin/env python3
"""DDS initialization helpers for Unitree SDK2."""

import os
from xml.sax.saxutils import escape

import unitree_sdk2py.core.channel as unitree_channel


DEFAULT_INTERFACE = os.environ.get("GO2_SDK_IFACE", "enp86s0")
DEFAULT_DOMAIN_ID = int(os.environ.get("GO2_DOMAIN_ID", "0"))
DEFAULT_PEER = os.environ.get("GO2_PEER", "")
DEFAULT_ALLOW_MULTICAST = os.environ.get("GO2_ALLOW_MULTICAST", "spdp")
DEFAULT_DDS_TRACE = os.environ.get("GO2_DDS_TRACE", "")


def add_dds_arguments(parser):
    """Add common SDK/DDS arguments while keeping the old positional iface."""
    parser.add_argument(
        "interface",
        nargs="?",
        default=None,
        help="network interface for Unitree SDK DDS, kept for backward compatibility",
    )
    parser.add_argument(
        "--iface",
        default=None,
        help=f"network interface for Unitree SDK DDS (default: {DEFAULT_INTERFACE})",
    )
    parser.add_argument(
        "--peer",
        default=DEFAULT_PEER,
        help="optional CycloneDDS static peer address, e.g. 192.168.123.161",
    )
    parser.add_argument(
        "--domain-id",
        type=int,
        default=DEFAULT_DOMAIN_ID,
        help=f"DDS domain id (default: {DEFAULT_DOMAIN_ID})",
    )
    parser.add_argument(
        "--allow-multicast",
        default=DEFAULT_ALLOW_MULTICAST,
        help=f"CycloneDDS AllowMulticast value (default: {DEFAULT_ALLOW_MULTICAST})",
    )
    parser.add_argument(
        "--dds-trace",
        default=DEFAULT_DDS_TRACE,
        help="optional CycloneDDS trace output path, e.g. /tmp/cdds.LOG",
    )


def resolved_interface(args):
    return args.iface or args.interface or DEFAULT_INTERFACE


def initialize_channel(args):
    initialize_go2_channel(
        domain_id=args.domain_id,
        interface=resolved_interface(args),
        peer=args.peer,
        allow_multicast=args.allow_multicast,
        trace_file=args.dds_trace,
    )


def initialize_go2_channel(
    domain_id=0,
    interface=None,
    peer=None,
    allow_multicast="spdp",
    trace_file=None,
):
    """Initialize Unitree SDK2 DDS with optional static peer discovery."""
    interface = interface or None
    peer = peer or None
    allow_multicast = allow_multicast or "default"
    trace_file = trace_file or None

    if peer or trace_file or allow_multicast != "default":
        config = _build_cyclonedds_config(
            interface=interface,
            peer=peer,
            allow_multicast=allow_multicast,
            trace_file=trace_file,
        )
        if interface:
            unitree_channel.ChannelConfigHasInterface = config
        else:
            unitree_channel.ChannelConfigAutoDetermine = config

    unitree_channel.ChannelFactoryInitialize(domain_id, interface)


def _build_cyclonedds_config(interface=None, peer=None, allow_multicast="spdp", trace_file=None):
    if interface:
        iface_xml = (
            f'<NetworkInterface name="{escape(interface)}" '
            'priority="default" multicast="default"/>'
        )
    else:
        iface_xml = '<NetworkInterface autodetermine="true" priority="default" multicast="default"/>'

    peer_xml = ""
    if peer:
        peer_xml = (
            "<Discovery>"
            "<Peers>"
            f'<Peer Address="{escape(peer)}"/>'
            "</Peers>"
            "</Discovery>"
        )

    tracing_xml = ""
    if trace_file:
        tracing_xml = (
            "<Tracing>"
            "<Verbosity>config</Verbosity>"
            f"<OutputFile>{escape(trace_file)}</OutputFile>"
            "</Tracing>"
        )

    return f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        {iface_xml}
      </Interfaces>
      <AllowMulticast>{escape(allow_multicast)}</AllowMulticast>
    </General>
    {peer_xml}
    {tracing_xml}
  </Domain>
</CycloneDDS>'''
