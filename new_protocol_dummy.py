"""
新协议模拟打印机 v13
- 激进流控制
- 断连后通过AT命令重置HC-06
- 支持多次重连
"""

import struct
import serial
import serial.threaded
import time
import json
import threading
import logging
import os
from datetime import datetime
from new_const import NewConst

COM_PORT = 'COM3'
BAUD = 115200

PRT_SN = 'P20GH351022481'
PRT_VERSION = '01.03.11'
PRT_NAME = 'P2'

HEADER_LEN = 9
FOOTER_LEN = 5

BASE_DELAY = 0.010
PRINT_DATA_DELAY = 0.050
LARGE_PACKET_DELAY = 0.030
VERY_LARGE_PACKET_DELAY = 0.080
LARGE_PACKET_THRESHOLD = 500
VERY_LARGE_PACKET_THRESHOLD = 800

CONNECTION_TIMEOUT = 30

LOG_DIR = 'logs'

CMD_NAMES = {
    0x05: 'QUERY',
    0x02: 'GET_SN',
    0x06: 'CMD_06',
    0x07: 'CMD_07',
    0x08: 'GET_INFO',
    0x09: 'CMD_09',
    0x17: 'GET_ID',
    0x18: 'GET_ID',
    0x3B: 'AUTH',
    0x71: 'CMD_71',
    0xB9: 'PRINT_DATA',
    0x1B: 'CMD_1B',
}

REF_NAMES = {
    0x15: 'VERSION',
    0x02: 'SN/INFO',
    0x07: 'VERSION2',
    0x20: 'STATUS',
    0x28: 'PAPER_INFO',
    0x04: 'CMD_04',
    0x05: 'CMD_05',
    0x14: 'CMD_14',
    0x08: 'DEVICE_INFO',
    0x09: 'CMD_09',
    0x01: 'CAPABILITY',
    0x0B: 'CMD_0B',
    0x0C: 'CMD_0C',
    0x2F: 'CMD_2F',
    0x19: 'CMD_19',
    0x1A: 'CMD_1A',
}

def setup_logger():
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(LOG_DIR, 'dummy_{}.log'.format(timestamp))
    
    logger = logging.getLogger('DummyPrinter')
    logger.setLevel(logging.DEBUG)
    
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    file_handler = logging.FileHandler(log_file, encoding='utf-8')
    file_handler.setLevel(logging.DEBUG)
    file_format = logging.Formatter('%(asctime)s | %(message)s', datefmt='%H:%M:%S')
    file_handler.setFormatter(file_format)
    
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_format = logging.Formatter('%(message)s')
    console_handler.setFormatter(console_format)
    
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger, log_file

class NewPacketBuilder:
    """数据包构建器"""
    
    @staticmethod
    def build_response(cmd, seq, ref, status, data):
        length = len(data)
        packet = bytearray()
        packet.append(NewConst.PKT_START)
        packet.append(NewConst.PKT_ADDR)
        packet.append(cmd)
        packet.extend(struct.pack('<H', seq))
        packet.append(ref)
        packet.append(status)
        packet.extend(struct.pack('<H', length))
        packet.extend(data)
        
        checksum = NewPacketBuilder._calc_checksum(packet[1:])
        packet.extend(struct.pack('<I', checksum))
        packet.append(NewConst.PKT_STOP)
        
        return bytes(packet)
    
    @staticmethod
    def _calc_checksum(data):
        return sum(data) & 0xFFFFFFFF

class FlowController:
    """激进流控制器"""
    
    def __init__(self):
        self.packet_count = 0
        self.print_count = 0
        self.lock = threading.Lock()
    
    def on_packet(self, is_print_data):
        with self.lock:
            self.packet_count += 1
            if is_print_data:
                self.print_count += 1
    
    def get_delay(self, packet_size, is_print_data=False):
        with self.lock:
            delay = BASE_DELAY
            
            if is_print_data:
                delay += PRINT_DATA_DELAY
                
                if self.print_count > 0 and self.print_count % 10 == 0:
                    delay += 0.100
            
            if packet_size > VERY_LARGE_PACKET_THRESHOLD:
                delay += VERY_LARGE_PACKET_DELAY
                extra = (packet_size - VERY_LARGE_PACKET_THRESHOLD) * 0.0001
                delay += extra
            elif packet_size > LARGE_PACKET_THRESHOLD:
                delay += LARGE_PACKET_DELAY
                extra = (packet_size - LARGE_PACKET_THRESHOLD) * 0.00005
                delay += extra
            
            if self.packet_count > 50:
                delay += 0.020
            elif self.packet_count > 30:
                delay += 0.010
            
            return delay
    
    def reset(self):
        with self.lock:
            self.packet_count = 0
            self.print_count = 0

class NewProtocolHandler(serial.threaded.Protocol):
    """协议处理器"""
    
    def __init__(self, printer, flow_controller, logger):
        self.printer = printer
        self.flow_controller = flow_controller
        self.logger = logger
        self.buffer = bytearray()
        self.lock = threading.Lock()
        self.count = 0
        self.print_count = 0
        self.last_time = 0
        self.start_time = time.time()
        self.last_packet_time = time.time()
        self.print_session_start = False
    
    def __call__(self):
        return self
    
    def data_received(self, data):
        self.last_packet_time = time.time()
        
        with self.lock:
            self.buffer.extend(data)
            self._process_buffer()
    
    def _process_buffer(self):
        while len(self.buffer) >= HEADER_LEN + FOOTER_LEN:
            start_idx = self._find_start()
            if start_idx == -1:
                self.buffer.clear()
                return
            
            if start_idx > 0:
                del self.buffer[:start_idx]
            
            if len(self.buffer) < HEADER_LEN:
                return
            
            length = struct.unpack('<H', self.buffer[7:9])[0]
            total_len = HEADER_LEN + length + FOOTER_LEN
            
            if len(self.buffer) < total_len:
                return
            
            packet_data = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]
            
            self._handle_packet(packet_data, length)
    
    def _find_start(self):
        for i, b in enumerate(self.buffer):
            if b == NewConst.PKT_START:
                return i
        return -1
    
    def _handle_packet(self, data, payload_len):
        try:
            cmd = data[2]
            seq = struct.unpack('<H', data[3:5])[0]
            ref = data[5]
            payload = data[9:9+payload_len]
            
            self.count += 1
            
            is_print_data = (cmd == 0xB9)
            is_control_cmd = (cmd in [0x05, 0x02, 0x06, 0x07, 0x08, 0x09, 0x17, 0x18, 0x3B])
            
            cmd_name = CMD_NAMES.get(cmd, 'CMD_{:02X}'.format(cmd))
            ref_name = REF_NAMES.get(ref, 'REF_{:02X}'.format(ref))
            
            self.flow_controller.on_packet(is_print_data)
            
            if is_print_data:
                if not self.print_session_start:
                    self.print_session_start = True
                    self.logger.info("=== 开始打印任务 ===")
                
                self.print_count += 1
                
                self.logger.debug('[RX] #{:04d} {} | SEQ:{:04d} | LEN:{:04d}'.format(
                    self.count, cmd_name, seq, payload_len))
                
                now = time.time()
                if now - self.last_time >= 1.0:
                    elapsed = now - self.start_time
                    rate = self.print_count / elapsed if elapsed > 0 else 0
                    self.logger.info('[{}] 打印#{}, 总包{}, 速率:{:.1f}包/秒'.format(
                        time.strftime('%H:%M:%S'), self.print_count, self.count, rate))
                    self.last_time = now
            else:
                if is_control_cmd and self.print_session_start:
                    self.print_session_start = False
                    self.logger.info("=== 打印任务结束 ===")
                    
                    if self.printer.serial:
                        try:
                            self.printer.serial.reset_input_buffer()
                            self.logger.debug("已清理串口输入缓冲区")
                        except:
                            pass
                
                self.logger.debug('[RX] #{:04d} {} {} | SEQ:{:04d} | LEN:{:04d}'.format(
                    self.count, cmd_name, ref_name, seq, payload_len))
            
            delay = self.flow_controller.get_delay(payload_len, is_print_data)
            time.sleep(delay)
            
            self.printer.handle_command(cmd, seq, ref, payload, self.logger)
            
        except Exception as e:
            self.logger.error('处理错误: {}'.format(e))
    
    def reset(self):
        with self.lock:
            self.buffer.clear()
            self.count = 0
            self.print_count = 0
            self.start_time = time.time()
            self.last_packet_time = time.time()
            self.print_session_start = False

class HC06Reset:
    """HC-06蓝牙模块重置工具"""
    
    @staticmethod
    def send_at_reset(port, baud=115200, logger=None):
        try:
            ser = serial.Serial(port, baud, timeout=1, write_timeout=1)
            time.sleep(0.2)
            
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            at_commands = [
                b'AT\r\n',
                b'AT+RESET\r\n',
                b'AT+ORGL\r\n',
            ]
            
            for cmd in at_commands:
                try:
                    ser.write(cmd)
                    time.sleep(0.5)
                    response = ser.read(ser.in_waiting)
                    if response and logger:
                        logger.debug("HC-06响应: {}".format(response))
                except:
                    pass
                time.sleep(0.3)
            
            ser.close()
            
            if logger:
                logger.info("已发送AT重置命令到HC-06")
            
            return True
            
        except Exception as e:
            if logger:
                logger.error("HC-06 AT重置失败: {}".format(e))
            return False

class NewProtocolDummyPrinter:
    """新协议模拟打印机 v13 - 支持AT重置"""
    
    def __init__(self, port=COM_PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self.serial = None
        self.serial_worker = None
        self.handler = None
        self.running = False
        
        self.logger, self.log_file = setup_logger()
        
        self.flow_controller = FlowController()
        self.write_lock = threading.Lock()
        
        self.device_id_1 = 'd1ffa69a01a7eb61'
        self.device_id_2 = '06a51aa4e254517f0c548830d13b569e'
        
        self._prebuild_responses()
    
    def _prebuild_responses(self):
        self.resp_010000 = bytes.fromhex('010000')
        self.resp_0102000000 = bytes.fromhex('0102000000')
        self.resp_0102000100 = bytes.fromhex('0102000100')
        self.resp_0102002c01 = bytes.fromhex('0102002c01')
        self.resp_0102004002 = bytes.fromhex('0102004002')
        self.resp_0102000004 = bytes.fromhex('0102000004')
        self.resp_0102009b02 = bytes.fromhex('0102009b02')
        self.resp_0102002c0f = bytes.fromhex('0102002c0f')
        
        self.resp_version = self._build_version_resp()
        self.resp_sn = self._build_sn_resp()
        self.resp_device_info = bytes.fromhex('010900010100010202005032')
        self.resp_paper_info = self._build_paper_info_resp()
        self.resp_capability = self._build_capability_resp()
        self.resp_device_id = self._build_device_id_resp()
        self.resp_info_02 = bytes.fromhex('0105000102003900')
        self.resp_auth_ok = bytes.fromhex('010800010100000000000001')
    
    def _build_version_resp(self):
        data = bytearray()
        data.extend(bytes.fromhex('010800'))
        data.extend(PRT_VERSION.encode('utf-8'))
        return bytes(data)
    
    def _build_sn_resp(self):
        data = bytearray()
        data.extend(bytes.fromhex('010e00'))
        data.extend(PRT_SN.encode('utf-8'))
        return bytes(data)
    
    def _build_paper_info_resp(self):
        paper_info = {
            "TPSizeInfo": [{
                "PaperWidth": "57",
                "HotSpot": "576",
                "OffsetStart": "0"
            }]
        }
        json_str = json.dumps(paper_info, separators=(',', ':'))
        data = bytearray()
        data.extend(bytes.fromhex('014600'))
        data.extend(json_str.encode('utf-8'))
        return bytes(data)
    
    def _build_capability_resp(self):
        capability = {
            "TPSupportSize": "57",
            "TPDevDPI": "300",
            "TPHotSpotNum": "576",
            "TPColorSupport": "000000"
        }
        json_str = json.dumps(capability, separators=(',', ':'))
        data = bytearray()
        data.extend(bytes.fromhex('015600'))
        data.extend(json_str.encode('utf-8'))
        return bytes(data)
    
    def _build_device_id_resp(self):
        data = bytearray()
        data.extend(bytes.fromhex('015a00'))
        data.extend(bytes.fromhex('011000'))
        data.extend(self.device_id_1.encode('utf-8'))
        data.extend(bytes.fromhex('022000'))
        data.extend(self.device_id_2.encode('utf-8'))
        data.extend(bytes.fromhex('030e00'))
        data.extend(PRT_SN.encode('utf-8'))
        data.extend(bytes.fromhex('041000'))
        data.extend('11111111d1ffa69a'.encode('utf-8'))
        return bytes(data)
    
    def connect_host(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.setDTR(False)
                self.serial.setRTS(False)
                time.sleep(0.1)
                self.serial.close()
                time.sleep(0.5)
            
            self.serial = serial.Serial(
                self.port, 
                self.baud, 
                timeout=1,
                write_timeout=1,
                dsrdtr=False,
                rtscts=False
            )
            self.serial.set_buffer_size(rx_size=4096, tx_size=4096)
            
            self.serial.setDTR(False)
            self.serial.setRTS(False)
            
            time.sleep(0.2)
            
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            print("串口 {} 已打开并清理缓冲区".format(self.port))
            return True
        except serial.SerialException as e:
            print("无法打开串口 {}: {}".format(self.port, e))
            return False
    
    def disconnect_host(self):
        if self.serial_worker:
            try:
                self.serial_worker.stop()
            except:
                pass
            self.serial_worker = None
        
        if self.serial:
            try:
                self.serial.setDTR(False)
                self.serial.setRTS(False)
                time.sleep(0.1)
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                self.serial.close()
            except:
                pass
            self.serial = None
            print("串口 {} 已关闭".format(self.port))
    
    def reset_connection(self):
        self.logger.info("检测到连接超时，正在重置...")
        print("\n[重置] 检测到连接超时，正在重置...")
        
        self.disconnect_host()
        
        print("[重置] 发送AT命令重置HC-06...")
        HC06Reset.send_at_reset(self.port, self.baud, self.logger)
        
        print("[重置] 等待HC-06模块重置...")
        time.sleep(3)
        
        for attempt in range(3):
            print("[重置] 尝试重新连接 ({}/3)...".format(attempt + 1))
            
            if self.connect_host():
                self.handler = NewProtocolHandler(self, self.flow_controller, self.logger)
                self.serial_worker = serial.threaded.ReaderThread(self.serial, self.handler)
                self.serial_worker.start()
                
                self.flow_controller.reset()
                
                self.logger.info("重置完成，等待新连接...")
                print("[重置] 完成，等待新连接...")
                print("[重置] 如果APP仍无法连接，请手动重新插拔HC-06模块")
                return True
            else:
                time.sleep(2)
        
        self.logger.error("重置失败，请手动重新插拔HC-06")
        print("[重置] 失败，请手动重新插拔HC-06模块")
        return False
    
    def start(self):
        if not self.connect_host():
            return
        
        self.handler = NewProtocolHandler(self, self.flow_controller, self.logger)
        self.serial_worker = serial.threaded.ReaderThread(self.serial, self.handler)
        self.serial_worker.start()
        
        self.running = True
        
        print("=" * 60)
        print("新协议模拟打印机 v13 (AT重置)")
        print("设备: {}, 版本: {}, SN: {}".format(PRT_NAME, PRT_VERSION, PRT_SN))
        print("日志文件: {}".format(self.log_file))
        print("超时检测: {}秒".format(CONNECTION_TIMEOUT))
        print("按 Ctrl+C 退出")
        print("=" * 60)
        
        self.logger.info("=" * 60)
        self.logger.info("新协议模拟打印机 v13 启动")
        self.logger.info("设备: {}, 版本: {}, SN: {}".format(PRT_NAME, PRT_VERSION, PRT_SN))
        self.logger.info("=" * 60)
        
        try:
            while self.running:
                time.sleep(1)
                
                if self.handler and self.handler.last_packet_time > 0:
                    idle_time = time.time() - self.handler.last_packet_time
                    if idle_time > CONNECTION_TIMEOUT and self.handler.count > 0:
                        self.logger.info("连接空闲 {} 秒，准备重置".format(int(idle_time)))
                        self.reset_connection()
                        
        except KeyboardInterrupt:
            print("\n正在停止...")
        finally:
            self.stop()
    
    def stop(self):
        self.running = False
        self.disconnect_host()
        self.logger.info("已停止")
        print("日志已保存: {}".format(self.log_file))
    
    def send_response(self, cmd, seq, ref, data, logger=None, is_print_resp=False):
        if not self.serial:
            return
        
        with self.write_lock:
            try:
                response = NewPacketBuilder.build_response(cmd, seq, ref, 0x02, data)
                self.serial.write(response)
                self.serial.flush()
                
                resp_cmd_name = CMD_NAMES.get(cmd, 'CMD_{:02X}'.format(cmd))
                
                if is_print_resp:
                    logger.debug('[TX] {} (PRINT_ACK) | SEQ:{:04d}'.format(
                        resp_cmd_name, seq))
                else:
                    logger.debug('[TX] {} | SEQ:{:04d} | LEN:{:04d}'.format(
                        resp_cmd_name, seq, len(data)))
                
                if len(data) > 0 and len(data) < 100:
                    try:
                        data_str = data.decode('utf-8', errors='ignore')
                        if data_str.isprintable() and len(data_str) > 3:
                            logger.debug('    DATA: {}'.format(data_str))
                    except:
                        pass
                        
            except Exception as e:
                logger.error("发送响应失败: {}".format(e))
    
    def handle_command(self, cmd, seq, ref, payload, logger):
        if cmd == 0x05:
            self._handle_query(seq, ref, logger)
        elif cmd == 0x02:
            self.send_response(0x16, seq, ref, self.resp_sn, logger)
        elif cmd == 0x06:
            self.send_response(0x08, seq, ref, self.resp_010000, logger)
        elif cmd == 0x07:
            self.send_response(0x0A, seq, ref, self.resp_0102000000, logger)
        elif cmd == 0x08:
            self.send_response(0x11, seq, ref, self.resp_device_info, logger)
        elif cmd == 0x09:
            self.send_response(0x08, seq, ref, self.resp_010000, logger)
        elif cmd == 0x17:
            self.send_response(0x62, seq, ref, self.resp_device_id, logger)
        elif cmd == 0x18:
            self.send_response(0x62, seq, ref, self.resp_device_id, logger)
        elif cmd == 0x3B:
            self.send_response(0x3C, seq, ref, self.resp_auth_ok, logger)
        elif cmd == 0x71:
            self.send_response(0x08, seq, ref, self.resp_010000, logger)
        elif cmd == 0xB9:
            self.send_response(0x08, seq, ref, self.resp_010000, logger, is_print_resp=True)
        elif cmd == 0x1B:
            self.send_response(0x08, seq, ref, self.resp_010000, logger)
        else:
            logger.debug('[TX] 未知命令 0x{:02X} 返回默认响应'.format(cmd))
            self.send_response(0x08, seq, ref, self.resp_010000, logger)
    
    def _handle_query(self, seq, ref, logger):
        if ref == 0x15:
            self.send_response(0x10, seq, ref, self.resp_version, logger)
        elif ref == 0x02:
            if seq == 256:
                self.send_response(0x16, seq, ref, self.resp_sn, logger)
            else:
                self.send_response(0x0D, seq, ref, self.resp_info_02, logger)
        elif ref == 0x07:
            self.send_response(0x10, seq, ref, self.resp_version, logger)
        elif ref == 0x20:
            self.send_response(0x0A, seq, ref, self.resp_0102000100, logger)
        elif ref == 0x28:
            self.send_response(0x4E, seq, ref, self.resp_paper_info, logger)
        elif ref == 0x04:
            self.send_response(0x0A, seq, ref, self.resp_0102002c01, logger)
        elif ref == 0x05:
            self.send_response(0x0A, seq, ref, self.resp_0102004002, logger)
        elif ref == 0x14:
            self.send_response(0x0A, seq, ref, self.resp_0102000004, logger)
        elif ref == 0x08:
            self.send_response(0x11, seq, ref, self.resp_device_info, logger)
        elif ref == 0x09:
            self.send_response(0x0A, seq, ref, self.resp_0102000000, logger)
        elif ref == 0x01:
            self.send_response(0x5E, seq, ref, self.resp_capability, logger)
        elif ref == 0x0B:
            self.send_response(0x0A, seq, ref, self.resp_0102009b02, logger)
        elif ref == 0x0C:
            self.send_response(0x0A, seq, ref, self.resp_0102002c0f, logger)
        elif ref == 0x2F:
            self.send_response(0x08, seq, ref, self.resp_010000, logger)
        elif ref == 0x19:
            self.send_response(0x0A, seq, ref, self.resp_0102000000, logger)
        elif ref == 0x1A:
            self.send_response(0x0A, seq, ref, self.resp_0102000000, logger)
        else:
            logger.debug('[TX] 未知查询 REF=0x{:02X} 返回默认响应'.format(ref))
            self.send_response(0x0A, seq, ref, self.resp_0102000000, logger)

if __name__ == "__main__":
    printer = NewProtocolDummyPrinter()
    printer.start()
