"""
喵喵机P2新协议命令字定义 v3
适用于固件版本 V1.03.11
"""

class NewCommand:
    @staticmethod
    def find_command(cmd):
        for name, value in NewCommand.__dict__.items():
            if not name.startswith('_') and isinstance(value, int) and value == cmd:
                return name
        return f"CMD_{cmd:02X}"
    
    CMD_GET_VERSION = 0x05
    CMD_VERSION_RESP = 0x10
    
    CMD_GET_SN = 0x02
    CMD_SN_RESP = 0x16
    
    CMD_GET_INFO_07 = 0x07
    CMD_INFO_07_RESP = 0x08
    
    CMD_GET_PAPER_INFO = 0x28
    CMD_PAPER_INFO_RESP = 0x4E
    
    CMD_INFO_05_RESP = 0x0A
    
    CMD_GET_STATUS = 0x14
    CMD_STATUS_RESP = 0x0A
    
    CMD_GET_INFO_02 = 0x02
    CMD_INFO_02_RESP = 0x0D
    
    CMD_GET_DEVICE_INFO = 0x08
    CMD_DEVICE_INFO_RESP = 0x11
    
    CMD_GET_INFO_09 = 0x09
    CMD_INFO_09_RESP = 0x0A
    
    CMD_GET_SUPPORT_INFO = 0x01
    CMD_SUPPORT_INFO_RESP = 0x5E
    
    CMD_GET_INFO_0B = 0x0B
    CMD_INFO_0B_RESP = 0x0A
    
    CMD_GET_INFO_0C = 0x0C
    CMD_INFO_0C_RESP = 0x0A
    
    CMD_GET_DEVICE_ID = 0x17
    CMD_DEVICE_ID_RESP = 0x62
    
    CMD_SET_PARAM = 0x13
    CMD_SET_PARAM_RESP = 0x08
    
    CMD_SEND_DATA = 0x1F
    CMD_SEND_DATA_RESP = 0x08
    
    CMD_GET_INFO_2F = 0x2F
    CMD_INFO_2F_RESP = 0x08
    
    CMD_PRINT_DATA = 0x20
    CMD_PRINT_DATA_RESP = 0x08
    
    CMD_PRINT_DATA_V2 = 0xB9
    CMD_PRINT_CONTROL = 0x1B
    
    CMD_SET_PARAM_06 = 0x06
    CMD_GET_PARAM_07 = 0x07
    
    CMD_SET_HEAT = 0x11
    CMD_SET_SPEED = 0x19
    
    CMD_FEED_PAPER = 0x20
    
    CMD_GET_DEVICE_AUTH = 0x18
    CMD_DEVICE_AUTH_RESP = 0x62
    
    CMD_SEND_AUTH_DATA = 0x3B
    CMD_AUTH_DATA_RESP = 0x08
    
    CMD_HEARTBEAT = 0x05
    CMD_HEARTBEAT_RESP = 0x08
    
    CMD_GET_BATTERY = 0x15
    CMD_BATTERY_RESP = 0x0A
    
    CMD_GET_TEMPERATURE = 0x12
    CMD_TEMPERATURE_RESP = 0x0A
    
    CMD_SET_PAPER_TYPE = 0x1A
    CMD_PAPER_TYPE_RESP = 0x08
    
    CMD_GET_PRINT_COUNT = 0x1E
    CMD_PRINT_COUNT_RESP = 0x0A
    
    CMD_CLEAR_BUFFER = 0x21
    CMD_CLEAR_BUFFER_RESP = 0x08
    
    CMD_GET_PRINT_STATUS = 0x22
    CMD_PRINT_STATUS_RESP = 0x0A
    
    CMD_GET_PAPER_INFO_V2 = 0x28
    CMD_PAPER_INFO_V2_RESP = 0x4E
    
    CMD_GET_CAPABILITY = 0x01
    CMD_CAPABILITY_RESP = 0x5E
    
    CMD_UNKNOWN = 0xFF

    COMMAND_DESCRIPTIONS = {
        0x01: "获取支持信息",
        0x02: "获取序列号",
        0x05: "查询/心跳",
        0x06: "设置参数",
        0x07: "查询参数",
        0x08: "响应/设备信息",
        0x09: "查询信息",
        0x0A: "通用响应",
        0x0B: "查询信息0B",
        0x0C: "查询信息0C",
        0x0D: "信息响应",
        0x10: "版本响应",
        0x11: "设置加热/设备信息响应",
        0x12: "查询温度",
        0x13: "设置参数",
        0x14: "查询状态",
        0x15: "查询电量",
        0x16: "序列号响应",
        0x17: "获取设备ID",
        0x18: "设备认证请求",
        0x19: "设置速度",
        0x1A: "设置纸张类型",
        0x1B: "打印控制",
        0x1E: "获取打印次数",
        0x1F: "发送数据",
        0x20: "打印数据/走纸",
        0x21: "清除缓冲区",
        0x22: "获取打印状态",
        0x28: "获取纸张信息",
        0x2F: "查询信息2F",
        0x3B: "发送认证数据",
        0x4E: "纸张信息响应",
        0x5E: "设备能力响应",
        0x62: "设备ID响应",
        0xB9: "打印数据V2",
    }
    
    @staticmethod
    def get_description(cmd):
        return NewCommand.COMMAND_DESCRIPTIONS.get(cmd, "未知命令")
