"""
新协议模拟打印机 v20
- 接收与发送完全解耦
- 发送独立线程，不阻塞接收
- ACK 立即入队，无延迟
"""

import struct
import serial
import serial.threaded
import time
import json
import threading
import queue
import logging
import os
from datetime import datetime
from new_const import NewConst

COM_PORT = 'COM4'
BAUD = 115200

PRT_SN = 'P20GH351022481'
PRT_VERSION = '01.03.11'
PRT_NAME = 'Paperang_P2'

HEADER_LEN = 9
FOOTER_LEN = 5

CONNECTION_TIMEOUT = 3600

SERIAL_RX_BUFFER_SIZE = 65536
SERIAL_TX_BUFFER_SIZE = 65536

LOG_DIR = 'logs'

_RESET_INPUT = object()

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
    file_format = logging.Formatter('%(asctime)s.%(msecs)03d | %(message)s', datefmt='%H:%M:%S')
    file_handler.setFormatter(file_format)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_format = logging.Formatter('%(message)s')
    console_handler.setFormatter(console_format)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    return logger, log_file

class NewPacketBuilder:

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

class TrafficStats:

    def __init__(self):
        self.total_bytes_received = 0
        self.total_bytes_sent = 0
        self.packet_count = 0
        self.lock = threading.Lock()

    def record_received(self, packet_size):
        with self.lock:
            self.total_bytes_received += packet_size
            self.packet_count += 1

    def record_sent(self, byte_count):
        with self.lock:
            self.total_bytes_sent += byte_count

    def get_stats(self):
        with self.lock:
            return {
                'received': self.total_bytes_received,
                'sent': self.total_bytes_sent,
                'packets': self.packet_count
            }

class NewProtocolHandler(serial.threaded.Protocol):

    def __init__(self, printer, logger, traffic_stats):
        self.printer = printer
        self.logger = logger
        self.traffic_stats = traffic_stats
        self.buffer = bytearray()
        self.lock = threading.Lock()
        self.count = 0
        self.print_count = 0
        self.last_time = 0
        self.start_time = time.time()
        self.last_packet_time = time.time()
        self.print_session_start = False
        self.data_receive_count = 0
        self.last_buffer_change_time = time.time()
        self.expected_packet_len = 0

    def __call__(self):
        return self

    def data_received(self, data):
        self.last_packet_time = time.time()
        self.data_receive_count += 1

        with self.lock:
            old_len = len(self.buffer)
            self.buffer.extend(data)
            if old_len != len(self.buffer):
                self.last_buffer_change_time = time.time()
            responses = self._process_buffer()

        for response in responses:
            self.printer.enqueue_response(response)

    def _process_buffer(self):
        responses = []
        while len(self.buffer) >= HEADER_LEN + FOOTER_LEN:
            start_idx = self._find_start()

            if start_idx == -1:
                self.buffer.clear()
                self.expected_packet_len = 0
                return responses

            if start_idx > 0:
                del self.buffer[:start_idx]

            if len(self.buffer) < HEADER_LEN:
                return responses

            length = struct.unpack('<H', self.buffer[7:9])[0]
            total_len = HEADER_LEN + length + FOOTER_LEN
            self.expected_packet_len = total_len

            if len(self.buffer) < total_len:
                return responses

            packet_data = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]
            self.expected_packet_len = 0

            packet_responses = self._handle_packet(packet_data, length)
            responses.extend(packet_responses)

        return responses

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
            self.traffic_stats.record_received(len(data))

            is_print_data = (cmd == 0xB9)
            is_control_cmd = (cmd in [0x05, 0x02, 0x06, 0x07, 0x08, 0x09, 0x17, 0x18, 0x3B])

            if is_print_data:
                if not self.print_session_start:
                    self.print_session_start = True
                    self.logger.info("=== 开始打印任务 ===")

                self.print_count += 1

                now = time.time()
                if now - self.last_time >= 1.0:
                    elapsed = now - self.start_time
                    rate = self.print_count / elapsed if elapsed > 0 else 0
                    stats = self.traffic_stats.get_stats()
                    self.logger.info('[{}] 打印#{}, 总包{}, 速率:{:.1f}包/秒, 收发:{}/{}KB'.format(
                        time.strftime('%H:%M:%S'), self.print_count, self.count, rate,
                        stats['received']//1024, stats['sent']//1024))
                    self.last_time = now

                response = self.printer.build_response(cmd, seq, ref)
                return [response]
            else:
                responses = []
                if is_control_cmd and self.print_session_start:
                    self.print_session_start = False
                    self.logger.info("=== 打印任务结束 ===")
                    responses.append(_RESET_INPUT)

                cmd_responses = self.printer.handle_command(cmd, seq, ref, payload)
                responses.extend(cmd_responses)
                return responses

        except Exception as e:
            self.logger.error('[ERR] 处理错误: {}'.format(e))
            return []

    def reset(self):
        with self.lock:
            self.buffer.clear()
            self.count = 0
            self.print_count = 0
            self.start_time = time.time()
            self.last_packet_time = time.time()
            self.print_session_start = False
            self.data_receive_count = 0
            self.expected_packet_len = 0
            self.last_buffer_change_time = time.time()

    def check_stuck_buffer(self, timeout=3):
        with self.lock:
            if len(self.buffer) == 0:
                return False

            stuck_time = time.time() - self.last_buffer_change_time

            if stuck_time > timeout:
                self.logger.warning('[STUCK] 缓冲区卡住 {} 秒, {} 字节'.format(
                    int(stuck_time), len(self.buffer)))

                if self.expected_packet_len > 0 and len(self.buffer) < self.expected_packet_len:
                    for i in range(1, len(self.buffer)):
                        if self.buffer[i] == NewConst.PKT_START:
                            self.logger.info('[STUCK] 跳过 {} 字节, 找到新起始符'.format(i))
                            del self.buffer[:i]
                            self.expected_packet_len = 0
                            self.last_buffer_change_time = time.time()
                            return True

                    self.logger.warning('[STUCK] 清空缓冲区')
                    self.buffer.clear()
                    self.expected_packet_len = 0
                else:
                    self.buffer.clear()
                    self.expected_packet_len = 0

                self.last_buffer_change_time = time.time()
                return True

            return False

class NewProtocolDummyPrinter:

    def __init__(self, port=COM_PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self.serial = None
        self.serial_worker = None
        self.handler = None
        self.running = False

        self.logger, self.log_file = setup_logger()
        self.traffic_stats = TrafficStats()

        self.response_queue = queue.Queue()
        self.sender_thread = None

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

    def build_response(self, cmd, seq, ref):
        return NewPacketBuilder.build_response(cmd, seq, ref, 0x02, self.resp_010000)

    def enqueue_response(self, item):
        self.response_queue.put(item)

    def _sender_loop(self):
        while self.running:
            try:
                item = self.response_queue.get(timeout=0.5)
                if item is None:
                    break
                if item is _RESET_INPUT:
                    if self.serial:
                        try:
                            self.serial.reset_input_buffer()
                            self.logger.info("已清理串口输入缓冲区")
                        except:
                            pass
                elif isinstance(item, bytes):
                    if self.serial:
                        self.serial.write(item)
                        self.serial.flush()
                        self.traffic_stats.record_sent(len(item))
            except queue.Empty:
                continue
            except Exception as e:
                self.logger.error('[TX] 发送失败: {}'.format(e))

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
                write_timeout=2,
                dsrdtr=False,
                rtscts=False
            )

            self.serial.set_buffer_size(
                rx_size=SERIAL_RX_BUFFER_SIZE,
                tx_size=SERIAL_TX_BUFFER_SIZE
            )

            self.serial.setDTR(False)
            self.serial.setRTS(False)

            time.sleep(0.2)

            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            self.logger.info("串口 {} 已打开 (RX缓冲:{}KB, TX缓冲:{}KB)".format(
                self.port, SERIAL_RX_BUFFER_SIZE//1024, SERIAL_TX_BUFFER_SIZE//1024))
            print("串口 {} 已打开 (缓冲区: {}KB)".format(self.port, SERIAL_RX_BUFFER_SIZE//1024))
            return True
        except serial.SerialException as e:
            print("无法打开串口 {}: {}".format(self.port, e))
            return False

    def disconnect_host(self):
        if self.sender_thread and self.sender_thread.is_alive():
            self.response_queue.put(None)
            self.sender_thread.join(timeout=3)
            self.sender_thread = None

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

    def start(self):
        if not self.connect_host():
            return

        self.handler = NewProtocolHandler(self, self.logger, self.traffic_stats)
        self.serial_worker = serial.threaded.ReaderThread(self.serial, self.handler)

        self.running = True

        self.serial_worker.start()

        self.sender_thread = threading.Thread(target=self._sender_loop, daemon=True, name='SenderThread')
        self.sender_thread.start()

        print("=" * 60)
        print("新协议模拟打印机 v20 (收发解耦)")
        print("设备: {}, 版本: {}, SN: {}".format(PRT_NAME, PRT_VERSION, PRT_SN))
        print("串口缓冲区: {}KB".format(SERIAL_RX_BUFFER_SIZE//1024))
        print("日志文件: {}".format(self.log_file))
        print("按 Ctrl+C 退出")
        print("=" * 60)

        self.logger.info("=" * 60)
        self.logger.info("新协议模拟打印机 v20 启动")
        self.logger.info("设备: {}, 版本: {}, SN: {}".format(PRT_NAME, PRT_VERSION, PRT_SN))
        self.logger.info("=" * 60)

        last_count = 0

        try:
            while self.running:
                time.sleep(1)

                if self.handler:
                    current_count = self.handler.count
                    buffer_size = len(self.handler.buffer)

                    if current_count == last_count and current_count > 0:
                        idle_time = time.time() - self.handler.last_packet_time
                        if idle_time > 5 and buffer_size > 0:
                            self.logger.warning('[MON] 无数据 {} 秒, 缓冲区: {} 字节'.format(
                                int(idle_time), buffer_size))

                    self.handler.check_stuck_buffer(timeout=3)
                    last_count = current_count

        except KeyboardInterrupt:
            print("\n正在停止...")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        self.disconnect_host()
        self.logger.info("已停止")
        print("日志已保存: {}".format(self.log_file))

    def handle_command(self, cmd, seq, ref, payload):
        if cmd == 0x05:
            return self._handle_query(seq, ref)
        elif cmd == 0x02:
            return [NewPacketBuilder.build_response(0x16, seq, ref, 0x02, self.resp_sn)]
        elif cmd == 0x06:
            return [NewPacketBuilder.build_response(0x08, seq, ref, 0x02, self.resp_010000)]
        elif cmd == 0x07:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102000000)]
        elif cmd == 0x08:
            return [NewPacketBuilder.build_response(0x11, seq, ref, 0x02, self.resp_device_info)]
        elif cmd == 0x09:
            return [NewPacketBuilder.build_response(0x08, seq, ref, 0x02, self.resp_010000)]
        elif cmd == 0x17:
            return [NewPacketBuilder.build_response(0x62, seq, ref, 0x02, self.resp_device_id)]
        elif cmd == 0x18:
            return [NewPacketBuilder.build_response(0x62, seq, ref, 0x02, self.resp_device_id)]
        elif cmd == 0x3B:
            return [NewPacketBuilder.build_response(0x3C, seq, ref, 0x02, self.resp_auth_ok)]
        elif cmd == 0x71:
            return [NewPacketBuilder.build_response(0x08, seq, ref, 0x02, self.resp_010000)]
        elif cmd == 0xB9:
            return [NewPacketBuilder.build_response(0x08, seq, ref, 0x02, self.resp_010000)]
        elif cmd == 0x1B:
            return [NewPacketBuilder.build_response(0x08, seq, ref, 0x02, self.resp_010000)]
        else:
            return [NewPacketBuilder.build_response(0x08, seq, ref, 0x02, self.resp_010000)]

    def _handle_query(self, seq, ref):
        if ref == 0x15:
            return [NewPacketBuilder.build_response(0x10, seq, ref, 0x02, self.resp_version)]
        elif ref == 0x02:
            if seq == 256:
                return [NewPacketBuilder.build_response(0x16, seq, ref, 0x02, self.resp_sn)]
            else:
                return [NewPacketBuilder.build_response(0x0D, seq, ref, 0x02, self.resp_info_02)]
        elif ref == 0x07:
            return [NewPacketBuilder.build_response(0x10, seq, ref, 0x02, self.resp_version)]
        elif ref == 0x20:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102000100)]
        elif ref == 0x28:
            return [NewPacketBuilder.build_response(0x4E, seq, ref, 0x02, self.resp_paper_info)]
        elif ref == 0x04:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102002c01)]
        elif ref == 0x05:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102004002)]
        elif ref == 0x14:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102000004)]
        elif ref == 0x08:
            return [NewPacketBuilder.build_response(0x11, seq, ref, 0x02, self.resp_device_info)]
        elif ref == 0x09:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102000000)]
        elif ref == 0x01:
            return [NewPacketBuilder.build_response(0x5E, seq, ref, 0x02, self.resp_capability)]
        elif ref == 0x0B:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102009b02)]
        elif ref == 0x0C:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102002c0f)]
        elif ref == 0x2F:
            return [NewPacketBuilder.build_response(0x08, seq, ref, 0x02, self.resp_010000)]
        elif ref == 0x19:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102000000)]
        elif ref == 0x1A:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102000000)]
        else:
            return [NewPacketBuilder.build_response(0x0A, seq, ref, 0x02, self.resp_0102000000)]

if __name__ == "__main__":
    printer = NewProtocolDummyPrinter()
    printer.start()
