"""Draft hardware connection settings. No sockets, serial ports or SDKs are opened."""
import copy


def text_field(key, label, placeholder=''):
    return {'key': key, 'label': label, 'type': 'text', 'placeholder': placeholder}


def integer_field(key, label, minimum, maximum):
    return {'key': key, 'label': label, 'type': 'integer', 'min': minimum, 'max': maximum}


def choice_field(key, label, choices):
    return {'key': key, 'label': label, 'type': 'choice', 'choices': choices}


NETWORK = [text_field('host', 'IP / 主机名', '填写设备地址'), integer_field('port', '端口', 1, 65535)]
SERIAL = [text_field('device', '串口设备', '例如 COM3 或 /dev/ttyUSB0'),
          integer_field('baud_rate', '波特率', 50, 4000000),
          choice_field('data_bits', '数据位', ['7', '8']),
          choice_field('parity', '校验位', ['none', 'even', 'odd']),
          choice_field('stop_bits', '停止位', ['1', '2'])]
ARM = {
    'sdk_tcp': {'label': '厂家 SDK · TCP/IP', 'note': '连接机械臂控制器，由控制器管理七个关节；SDK 须与控制器版本匹配。',
                'fields': NETWORK + [text_field('controller_version', '控制器版本'), text_field('sdk_version', 'SDK 版本')]},
    'json_tcp': {'label': 'JSON 指令 · TCP/IP', 'note': '通过厂家 JSON 协议访问控制器，具体指令按控制器版本适配。',
                 'fields': NETWORK + [text_field('controller_version', '控制器版本')]},
    'json_rs485': {'label': 'JSON 指令 · RS-485', 'note': '控制器串口选项；接口和协议需按具体控制器版本核对。',
                   'fields': SERIAL + [text_field('controller_version', '控制器版本')]},
    'json_rs232_usb': {'label': 'JSON · RS232-USB（基础系列）', 'note': '厂家第四代文档标注基础系列支持；其他控制器不能默认套用。',
                       'fields': SERIAL + [text_field('controller_version', '控制器版本')]},
}
CATALOG = {
    'left_arm': {'label': '左机械臂', 'model': 'RM75', 'protocols': ARM},
    'right_arm': {'label': '右机械臂', 'model': 'RM75', 'protocols': ARM},
    'waist': {'label': '腰部', 'model': 'Elmo Gold DBTwitter', 'protocols': {
        'canopen': {'label': 'CANopen', 'note': '核对驱动器通信版本及总线参数后再接入。', 'fields': [
            text_field('interface', 'CAN 接口', '例如 can0'), integer_field('node_id', '节点 ID', 1, 127),
            integer_field('bitrate', '总线速率 / bit/s', 1000, 1000000), text_field('drive_model', '驱动器完整型号')]},
        'ethercat': {'label': 'EtherCAT', 'note': '仅适用于对应 EtherCAT 版本；网口与从站位置待实物确认。', 'fields': [
            text_field('interface', '主站网卡', '填写网卡名称'), integer_field('slave_position', '从站位置', 0, 65535),
            text_field('esi_reference', 'ESI 文件名 / 版本'), text_field('drive_model', '驱动器完整型号')]},
        'rs232': {'label': 'RS-232', 'note': '须确认标准 RS-232 / TTL 电平及对应协议；不能直接混接。', 'fields': SERIAL + [
            choice_field('signal_level', '接口电平', ['standard_rs232', 'ttl']), text_field('drive_model', '驱动器完整型号')]},
    }},
    'chassis': {'label': '底盘', 'model': 'DT-01 Pro', 'protocols': {
        'rs232': {'label': 'RS-232 · 底盘控制', 'note': '发送到底盘控制器，由控制器驱动车轮；需要厂家的帧格式和校验规则。',
                  'fields': SERIAL + [text_field('protocol_version', '通信协议版本')]},
        'mqtt': {'label': 'MQTT · 导航接口', 'note': '资料标注的是导航接口；不能据此假定支持直接轮速控制。', 'fields': [
            text_field('host', 'Broker 地址'), integer_field('port', '端口', 1, 65535),
            choice_field('security', '连接方式', ['tcp', 'tls']), text_field('client_id', '客户端 ID'),
            text_field('command_topic', '指令主题'), text_field('state_topic', '状态主题')]},
        'modbus': {'label': 'Modbus · 扩展待确认', 'note': '资料仅注明可扩展，RTU/TCP、寄存器及是否已安装均待厂家确认。', 'fields': [
            choice_field('variant', '传输方式', ['rtu', 'tcp']), text_field('endpoint', '串口设备 / 主机地址'),
            integer_field('port', 'TCP 端口', 1, 65535), integer_field('baud_rate', '串口波特率', 50, 4000000),
            integer_field('unit_id', '站号', 1, 247), text_field('register_map', '寄存器表版本')]},
    }},
}


def connection_catalog():
    return {'schema_version': 1, 'devices': copy.deepcopy(CATALOG), 'hardware_connected': False,
            'adapter_status': 'not_integrated'}


def validate_connections(config):
    """Allow unselected or incomplete drafts; reject invalid supplied values in every saved option."""
    if not isinstance(config, dict) or set(config) != {'schema_version', 'devices'}:
        raise ValueError('connections 需要 schema_version 和 devices。')
    if type(config['schema_version']) is not int or config['schema_version'] != 1:
        raise ValueError('connections.schema_version 必须为 1。')
    devices = config['devices']
    if not isinstance(devices, dict) or set(devices) - set(CATALOG):
        raise ValueError('未知设备通信配置。')
    for device, saved in devices.items():
        spec = CATALOG[device]
        if not isinstance(saved, dict) or set(saved) != {'selected', 'profiles'}:
            raise ValueError(spec['label'] + '需要 selected 和 profiles。')
        selected, profiles = saved['selected'], saved['profiles']
        if not isinstance(selected, str) or selected not in ('unselected', *spec['protocols']):
            raise ValueError(spec['label'] + '通信方式无效。')
        if not isinstance(profiles, dict) or set(profiles) - set(spec['protocols']):
            raise ValueError(spec['label'] + '含未知通信选项。')
        for protocol, values in profiles.items():
            fields = {field['key']: field for field in spec['protocols'][protocol]['fields']}
            if not isinstance(values, dict) or set(values) - set(fields):
                raise ValueError(spec['label'] + '含未知通信参数。')
            for key, value in values.items():
                field = fields[key]
                if value is None or value == '':
                    continue
                if field['type'] == 'integer':
                    valid = type(value) is int and field['min'] <= value <= field['max']
                elif field['type'] == 'choice':
                    valid = isinstance(value, str) and value in field['choices']
                else:
                    valid = isinstance(value, str) and len(value) <= 256 and not any(ord(c) < 32 for c in value)
                if not valid:
                    raise ValueError(f"{spec['label']} / {spec['protocols'][protocol]['label']} / {field['label']} 无效。")
    return {'configuration_only': True, 'hardware_connected': False, 'adapter_status': 'not_integrated'}
