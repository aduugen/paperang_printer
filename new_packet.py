"""
喵喵机P2新协议数据包解析类 v3
重新分析协议格式

数据包格式:
A5 ADDR CMD SEQ1 SEQ2 REF STATUS LEN1 LEN2 DATA... CHECK(4B) 5A

| 字节偏移 | 长度 | 字段 | 说明 |
|----------|------|------|------|
| 0 | 1 | START | 起始字节 0xA5 |
| 1 | 1 | ADDR | 地址 0x01 |
| 2 | 1 | CMD | 命令码 |
| 3-4 | 2 | SEQ | 序号 (小端) |
| 5 | 1 | REF | 参考字节 |
| 6 | 1 | STATUS | 状态 |
| 7-8 | 2 | LEN | 数据长度 (小端) |
| 9-n | LEN | DATA | 数据 |
| n+1-n+4 | 4 | CHECK | 校验 |
| n+5 | 1 | END | 结束字节 0x5A |
"""

import struct
from new_const import NewConst
from new_cmd import NewCommand

HEADER_LEN = 9
FOOTER_LEN = 5

class NewPacketV3:
    def __init__(self):
        self.start = NewConst.PKT_START
        self.addr = NewConst.PKT_ADDR
        self.cmd = 0
        self.seq = 0
        self.ref = 0
        self.status = 0
        self.length = 0
        self.data = bytes()
        self.checksum = bytes()
        self.end = NewConst.PKT_STOP
        self.raw = bytes()
    
    def __str__(self):
        cmd_name = NewCommand.find_command(self.cmd)
        cmd_desc = NewCommand.get_description(self.cmd)
        data_preview = self.data.hex()[:60] + "..." if len(self.data) > 30 else self.data.hex()
        return (f"Packet(cmd={cmd_name}[{cmd_desc}], seq={self.seq}, "
                f"ref=0x{self.ref:02X}, status=0x{self.status:02X}, "
                f"len={self.length}, data={data_preview})")
    
    def unpack(self, data):
        """解包字节数据"""
        if len(data) < HEADER_LEN + FOOTER_LEN:
            raise ValueError(f"数据太短: {len(data)} 字节, 需要至少 {HEADER_LEN + FOOTER_LEN} 字节")
        
        if data[0] != NewConst.PKT_START:
            raise ValueError(f"起始字节错误: 0x{data[0]:02X}, 应为 0xA5")
        
        if data[-1] != NewConst.PKT_STOP:
            raise ValueError(f"结束字节错误: 0x{data[-1]:02X}, 应为 0x5A")
        
        self.raw = data
        self.start = data[0]
        self.addr = data[1]
        self.cmd = data[2]
        self.seq = struct.unpack('<H', data[3:5])[0]
        self.ref = data[5]
        self.status = data[6]
        self.length = struct.unpack('<H', data[7:9])[0]
        
        expected_len = HEADER_LEN + self.length + FOOTER_LEN
        if len(data) < expected_len:
            raise ValueError(f"数据长度不匹配: 实际 {len(data)}, 期望 {expected_len}")
        
        self.data = data[9:9+self.length]
        self.checksum = data[9+self.length:9+self.length+4]
        self.end = data[-1]
        
        return self
    
    def is_print_data(self):
        """判断是否为打印数据包"""
        return self.cmd in [NewCommand.CMD_PRINT_DATA_V2, NewCommand.CMD_PRINT_DATA]
    
    def get_data_text(self):
        """尝试将数据解析为文本"""
        try:
            if len(self.data) > 3:
                text = self.data.decode('utf-8', errors='ignore')
                if text and all(c.isprintable() or c in '\r\n\t' for c in text):
                    return text
        except:
            pass
        return None
    
    def get_print_image_data(self):
        """提取打印图像数据"""
        if self.is_print_data() and len(self.data) > 0:
            return self.data
        return None


class NewPacketParserV3:
    """新协议数据包解析器 v3"""
    
    def __init__(self, logging=None, tag="PARSER"):
        self.logging = logging
        self.tag = tag
        self.buffer = bytearray()
        self.packets = []
    
    def parse(self, data):
        """解析数据流，返回找到的数据包列表"""
        self.buffer.extend(data)
        self._extract_packets()
        return self.packets
    
    def _extract_packets(self):
        """从缓冲区提取数据包"""
        while len(self.buffer) >= HEADER_LEN + FOOTER_LEN:
            start_idx = self._find_start()
            if start_idx == -1:
                self._log_debug(f"未找到起始字节，清空缓冲区 ({len(self.buffer)} 字节)")
                self.buffer.clear()
                return
            
            if start_idx > 0:
                self._log_debug(f"丢弃 {start_idx} 字节无效数据")
                del self.buffer[:start_idx]
            
            if len(self.buffer) < HEADER_LEN:
                return
            
            length = struct.unpack('<H', self.buffer[7:9])[0]
            total_len = HEADER_LEN + length + FOOTER_LEN
            
            if len(self.buffer) < total_len:
                self._log_debug(f"等待更多数据: {len(self.buffer)}/{total_len} 字节")
                return
            
            end_idx = total_len - 1
            if self.buffer[end_idx] != NewConst.PKT_STOP:
                self._log_warn(f"结束字节错误: 0x{self.buffer[end_idx]:02X}，尝试重新同步")
                del self.buffer[:1]
                continue
            
            packet_data = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]
            
            try:
                packet = NewPacketV3()
                packet.unpack(packet_data)
                self.packets.append(packet)
                self._log_packet(packet)
            except ValueError as e:
                self._log_warn(f"解析失败: {e}")
    
    def _find_start(self):
        """查找起始字节"""
        for i, b in enumerate(self.buffer):
            if b == NewConst.PKT_START:
                return i
        return -1
    
    def _log_packet(self, packet):
        """记录数据包日志"""
        cmd_name = NewCommand.find_command(packet.cmd)
        if packet.is_print_data():
            self._log_info(f"[{cmd_name}] seq={packet.seq}, len={packet.length} (打印数据)")
        else:
            self._log_info(f"{packet}")
            text = packet.get_data_text()
            if text:
                self._log_info(f"  文本: {text}")
    
    def _log_debug(self, msg):
        if self.logging:
            self.logging.debug(f"[{self.tag}] {msg}")
    
    def _log_info(self, msg):
        if self.logging:
            self.logging.info(f"[{self.tag}] {msg}")
    
    def _log_warn(self, msg):
        if self.logging:
            self.logging.warning(f"[{self.tag}] {msg}")


if __name__ == "__main__":
    print("=" * 60)
    print("测试新协议解析 v3")
    print("=" * 60)
    
    test_packets = [
        ("请求-查询版本", "a5010500011501000041d235195a"),
        ("响应-版本", "a50110000115020b0001080030312e30332e3131109896395a"),
        ("响应-序列号", "a50116000102021100010e0050323047483335313032323438318a10e7cb5a"),
        ("请求-设置参数", "a501060005110101005fdd69c7475a"),
        ("响应-设置参数", "a50108000511020300010000e21557db5a"),
    ]
    
    for name, hex_str in test_packets:
        print(f"\n{name}:")
        print(f"  原始: {hex_str}")
        data = bytes.fromhex(hex_str)
        packet = NewPacketV3()
        try:
            packet.unpack(data)
            print(f"  命令: {NewCommand.find_command(packet.cmd)} (0x{packet.cmd:02X})")
            print(f"  序号: {packet.seq}")
            print(f"  参考: 0x{packet.ref:02X}")
            print(f"  状态: 0x{packet.status:02X}")
            print(f"  长度: {packet.length}")
            print(f"  数据: {packet.data.hex()}")
            text = packet.get_data_text()
            if text:
                print(f"  文本: {text}")
        except ValueError as e:
            print(f"  错误: {e}")
