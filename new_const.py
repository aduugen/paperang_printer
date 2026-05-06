"""
喵喵机P2新协议常量定义
适用于固件版本 V1.03.11
"""

class NewConst:
    PKT_START_BYTE = b'\xA5'
    PKT_START = 0xA5
    PKT_STOP_BYTE = b'\x5A'
    PKT_STOP = 0x5A
    
    PKT_ADDR = 0x01
    
    HEADER_LEN = 8
    FOOTER_LEN = 3
