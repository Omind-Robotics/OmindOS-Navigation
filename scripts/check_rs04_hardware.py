#!/usr/bin/env python3
"""Inventory or read-only RS04 communication assessment. Never enables motors.

Default only inspects Linux device metadata. --probe-readonly requires explicit
confirmation that motors are disabled. Even reads can refresh a motor watchdog;
this tool must not be used as a concurrent observer of a running robot.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import platform
import struct
import time

from robstride04_driver import SocketCAN, feedback, read_parameter, HOST_ID

PARAMETERS={0x7005:('run_mode','uint'),0x7019:('position_rad','float'),
            0x701B:('velocity_rad_s','float'),0x701C:('bus_voltage_v','float'),
            0x7028:('watchdog_ticks','uint')}


def inventory(sys_root=Path('/sys'),dev_root=Path('/dev')):
    can=[]; serial=[]
    for interface in sorted((sys_root/'class/net').glob('*')):
        try:
            if (interface/'type').read_text().strip()=='280':
                can.append({'name':interface.name,'state':(interface/'operstate').read_text().strip()})
        except OSError: pass
    for pattern in ('ttyUSB*','ttyACM*'):
        for device in sorted(dev_root.glob(pattern)):
            driver=sys_root/'class/tty'/device.name/'device/driver'
            serial.append({'device':str(device),'kernel_driver':driver.resolve().name if driver.exists() else None,
                           'SocketCAN_compatibility_confirmed':False})
    return {'can_interfaces':can,'usb_serial_candidates':serial,
            'hardware_connection_observed':bool(can or serial)}


def addresses(config):
    if config.get('motor_model')!='ROBSTRIDE04' or config.get('protocol')!='private_29bit':
        raise ValueError('RS04 private protocol configuration required')
    result=[]; seen=set()
    for joint in config.get('joints',[]):
        channel,motor=joint['channel'],joint['motor_id']
        if not isinstance(channel,str) or not channel or type(motor) is not int or not 1<=motor<=127:
            raise ValueError('invalid configured motor address')
        if (channel,motor) in seen: raise ValueError('duplicate motor address')
        seen.add((channel,motor)); result.append((channel,motor,joint['name']))
    if len(result)!=12: raise ValueError('expected twelve configured motors')
    return result


class ReadOnlyTransport:
    """Transmission allowlist: only type 17 with five documented read indices."""
    def __init__(self,transport,targets):
        self.transport=transport; self.targets={(c,m) for c,m,_ in targets}; self.sent=0

    def send(self,channel,packet):
        ident=packet.arbitration_id
        allowed=(ident>>24==17 and (ident>>8)&65535==HOST_ID and
                 (channel,ident&255) in self.targets and len(packet.data)==8 and
                 int.from_bytes(packet.data[:2],'little') in PARAMETERS and packet.data[2:]==b'\x00'*6)
        if not allowed: raise ValueError('read-only transport rejected command')
        self.transport.send(channel,packet); self.sent+=1

    def receive(self,timeout): return self.transport.receive(timeout)
    def close(self): self.transport.close()  # SocketCAN.close does not send a stop/enable frame.


def read_assessment(config,transport,confirmed_disabled=False,clock=time.monotonic):
    if not confirmed_disabled: raise ValueError('operator must confirm disabled motors before sending even reads')
    targets=addresses(config); link=ReadOnlyTransport(transport,targets)
    results=[]; fatal=None
    try:
        # Old responses must never satisfy a new transaction.
        for _ in range(1024):
            if link.receive(0.) is None: break
        else: raise RuntimeError('CAN receive queue flood')
        for channel,motor,name in targets:
            item={'channel':channel,'motor_id':motor,'joint':name,'parameters':{},'errors':[],
                  'round_trip_ms':[]}
            results.append(item)
            for index,(label,kind) in PARAMETERS.items():
                sent=clock(); link.send(channel,read_parameter(motor,index))
                deadline=sent+.15; value=None
                while clock()<deadline:
                    received=link.receive(min(.005,deadline-clock()))
                    if received is None: continue
                    bus,packet,stamp=received
                    if packet is None: continue
                    data=feedback(packet)
                    if not data or (bus,data['motor_id']) not in link.targets: continue
                    if not sent<=stamp<=clock(): continue
                    if data['kind']==2 and data['mode']!=0:
                        raise RuntimeError('motor_not_disabled_reading_aborted')
                    if data['kind']==2 and data['fault']:
                        raise RuntimeError('motor_fault_reading_aborted')
                    if data['kind']==21 and (data['fault'] or data['warning']):
                        raise RuntimeError('motor_fault_reading_aborted')
                    if bus!=channel or data['motor_id']!=motor: continue
                    if data['kind']!=17 or data['index']!=index: continue
                    if data['error']: raise RuntimeError('parameter_read_error')
                    value=data['value'] if kind=='uint' else struct.unpack('<f',struct.pack('<I',data['value']))[0]
                    if not math.isfinite(value): raise RuntimeError('non_finite_parameter_value')
                    item['parameters'][label]=value
                    item['round_trip_ms'].append((clock()-sent)*1000)
                    break
                if value is None: item['errors'].append(f'{label}:timeout')
            ticks=item['parameters'].get('watchdog_ticks')
            item['firmware_watchdog_enabled']=(ticks is not None and ticks>0)
            if ticks is not None: item['watchdog_seconds']=ticks/20000
    except Exception as error:
        fatal=str(error)
    finally:
        link.close()
    responsive=sum(len(r['parameters'])==len(PARAMETERS) and not r['errors'] for r in results)
    return {'status':'READS_COMPLETE' if responsive==12 and not fatal else 'INCOMPLETE',
            'responsive_motors':responsive,'expected_motors':12,'motors':results,'error':fatal,
            'sent_read_requests':link.sent,'operator_confirmed_disabled':True,
            'enable_commands_sent':0,'motion_commands_sent':0,'parameter_writes_sent':0,
            'hardware_motion_validated':False,'mechanical_calibration_validated':False,
            'firmware_version_verified':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--probe-readonly',action='store_true')
    parser.add_argument('--confirm-motors-disabled',action='store_true')
    args=parser.parse_args()
    report={'created_at':datetime.now(timezone.utc).isoformat(),'system':platform.system(),
            'architecture':platform.machine(),'inventory':inventory(),'hardware_motion_validated':False}
    code=0
    if args.probe_readonly:
        if not args.config or not args.confirm_motors_disabled:
            parser.error('read-only probing requires --config and --confirm-motors-disabled')
        config=json.loads(args.config.read_text()); targets=addresses(config)
        try:
            transport=SocketCAN(sorted({c for c,_,_ in targets}))
            report['assessment']=read_assessment(config,transport,True)
            code=0 if report['assessment']['status']=='READS_COMPLETE' else 1
        except Exception as error:
            report['assessment']={'status':'INCOMPLETE','error':str(error)}; code=1
    else:
        report['status']='INVENTORY_ONLY'
        report['can_frames_sent']=0
        report['blockers']=([] if report['inventory']['can_interfaces'] else ['no_SocketCAN_interface'])
        if report['inventory']['usb_serial_candidates']:
            report['blockers'].append('serial_adapter_protocol_not_verified')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return code


if __name__=='__main__': raise SystemExit(main())
