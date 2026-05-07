"""
新协议桥接程序 v9
- 修复打印机沉默后无限超时循环
- 增加打印机沉默检测（30秒无数据判定断连）
- 增加连续超时计数和日志
- 记录双向每个包的CMD/SEQ/LEN
- 记录蓝牙异常完整信息
"""

import serial
import serial.threaded
import bluetooth
import time
import struct
import threading
import logging
import os
from datetime import datetime

COM_PORT = 'COM4'
BAUD = 115200
PAPERANG_ADDR = "03:0B:F8:E0:D3:E8"

RECV_TIMEOUT = 10
STUCK_TIMEOUT = 5
PRINTER_SILENCE_TIMEOUT = 30
HEADER_LEN = 9
FOOTER_LEN = 5
LOG_DIR = 'logs'

CMD_NAMES = {
    0x05: 'QUERY', 0x02: 'GET_SN', 0x06: 'CMD_06', 0x07: 'CMD_07',
    0x08: 'GET_INFO', 0x09: 'CMD_09', 0x0A: 'CMD_0A', 0x0B: 'CMD_0B',
    0x0C: 'CMD_0C', 0x10: 'CMD_10', 0x11: 'CMD_11', 0x16: 'CMD_16',
    0x17: 'GET_ID', 0x18: 'GET_ID', 0x19: 'CMD_19', 0x1A: 'CMD_1A',
    0x1B: 'CMD_1B', 0x2F: 'CMD_2F', 0x31: 'CMD_31', 0x3C: 'CMD_3C',
    0x3B: 'AUTH', 0x4E: 'CMD_4E', 0x5E: 'CMD_5E', 0x62: 'CMD_62',
    0x71: 'CMD_71', 0x0D: 'CMD_0D', 0xB9: 'PRINT_DATA',
    0xB5: 'CMD_B5', 0xE1: 'CMD_E1',
}


def setup_logger():
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(LOG_DIR, 'bridge_{}.log'.format(timestamp))
    logger = logging.getLogger('Bridge')
    logger.setLevel(logging.DEBUG)
    for h in logger.handlers[:]:
        logger.removeHandler(h)
    fh = logging.FileHandler(log_file, encoding='utf-8')
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter('%(asctime)s | %(message)s', datefmt='%H:%M:%S'))
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter('%(message)s'))
    logger.addHandler(fh)
    logger.addHandler(ch)
    return logger, log_file


def parse_packet_info(data):
    if len(data) < 9:
        return 'RAW({})'.format(len(data))
    cmd = data[2]
    seq = struct.unpack('<H', data[3:5])[0]
    ref = data[5]
    length = struct.unpack('<H', data[7:9])[0]
    cmd_name = CMD_NAMES.get(cmd, 'CMD_{:02X}'.format(cmd))
    return '{} SEQ:{:04d} REF:{:02X} LEN:{:04d}'.format(cmd_name, seq, ref, length)


class PhoneDataHandler(serial.threaded.Protocol):
    """处理手机数据"""

    def __init__(self, send_callback, logger):
        self.send_callback = send_callback
        self.logger = logger
        self.buffer = bytearray()
        self.lock = threading.Lock()
        self.count = 0
        self.print_count = 0
        self.last_time = 0
        self.last_packet_time = time.time()
        self.last_buffer_change_time = time.time()
        self.data_receive_count = 0
        self.last_data_receive_count = 0
        self.print_session_start = False

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
            self._process_buffer()

    def _process_buffer(self):
        while len(self.buffer) >= HEADER_LEN + FOOTER_LEN:
            start_idx = self._find_start()
            if start_idx == -1:
                self.logger.warning('[PHONE] 未找到起始符, 丢弃 {} 字节'.format(len(self.buffer)))
                self.buffer.clear()
                return

            if start_idx > 0:
                self.logger.info('[PHONE] 跳过 {} 字节到起始符'.format(start_idx))
                del self.buffer[:start_idx]

            if len(self.buffer) < HEADER_LEN:
                return

            length = struct.unpack('<H', self.buffer[7:9])[0]
            total_len = HEADER_LEN + length + FOOTER_LEN

            if len(self.buffer) < total_len:
                return

            packet_data = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]

            self._handle_packet(packet_data)

    def _find_start(self):
        for i, b in enumerate(self.buffer):
            if b == 0xA5:
                return i
        return -1

    def _handle_packet(self, data):
        try:
            cmd = data[2]
            self.count += 1

            is_print_data = (cmd == 0xB9)
            is_control_cmd = (cmd in [0x05, 0x02, 0x06, 0x07, 0x08, 0x09, 0x17, 0x18, 0x3B])

            if is_print_data:
                if not self.print_session_start:
                    self.print_session_start = True
                    self.logger.info('=== 开始打印任务 ===')

                self.print_count += 1
                now = time.time()
                if now - self.last_time >= 1.0:
                    self.logger.info('[PHONE→PRT] 打印#{}, 总包{}'.format(
                        self.print_count, self.count))
                    self.last_time = now
            else:
                if is_control_cmd and self.print_session_start:
                    self.print_session_start = False
                    self.logger.info('=== 打印任务结束 ===')

                info = parse_packet_info(data)
                self.logger.debug('[PHONE→PRT] #{} {}'.format(self.count, info))

            if self.send_callback:
                sent = self.send_callback(data)
                if sent != len(data):
                    self.logger.warning('[PHONE→PRT] 发送不完整: 期望{} 实际{}'.format(len(data), sent))

        except Exception as e:
            self.logger.error('[PHONE→PRT] 转发错误: {}'.format(e))

    def check_stuck_buffer(self, timeout=STUCK_TIMEOUT):
        with self.lock:
            if len(self.buffer) == 0:
                return

            stuck_time = time.time() - self.last_buffer_change_time
            if stuck_time <= timeout:
                return

            data_still_arriving = (self.data_receive_count != self.last_data_receive_count)
            self.last_data_receive_count = self.data_receive_count

            if data_still_arriving:
                self.last_buffer_change_time = time.time()
                return

            self.logger.warning('[STUCK] 缓冲区卡住 {} 秒, {} 字节, 清空'.format(
                int(stuck_time), len(self.buffer)))
            self.buffer.clear()
            self.last_buffer_change_time = time.time()


class NewProtocolBridge:
    """新协议桥接"""

    def __init__(self):
        self.ser = None
        self.sock = None
        self.running = False
        self.handler = None
        self.worker = None
        self.last_recv_time = time.time()
        self.printer_count = 0
        self.last_print_time = 0
        self.bt_connected = True
        self.logger, self.log_file = setup_logger()
        self.phone_count = 0

    def connect_serial(self):
        try:
            self.ser = serial.Serial(COM_PORT, BAUD, timeout=1)
            self.ser.set_buffer_size(rx_size=8192, tx_size=8192)
            self.logger.info("串口 {} 已打开".format(COM_PORT))
            return True
        except Exception as e:
            self.logger.error("无法打开串口: {}".format(e))
            return False

    def connect_printer(self):
        try:
            self.logger.info("正在连接打印机 {}...".format(PAPERANG_ADDR))
            self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.sock.connect((PAPERANG_ADDR, 1))
            self.sock.settimeout(RECV_TIMEOUT)
            self.bt_connected = True
            self.logger.info("打印机已连接")
            return True
        except Exception as e:
            self.logger.error("连接打印机失败: {}".format(e))
            return False

    def disconnect_printer(self):
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        self.bt_connected = False

    def send_to_printer(self, data):
        if self.sock and self.bt_connected:
            try:
                return self.sock.send(data)
            except bluetooth.BluetoothError as e:
                self.logger.error('[BT] 发送到打印机失败: {}'.format(e))
                self.bt_connected = False
                return 0
            except OSError as e:
                self.logger.error('[BT] 发送到打印机OS错误: {}'.format(e))
                self.bt_connected = False
                return 0
        return 0

    def _is_timeout_error(self, err_str):
        return ("timed out" in err_str or "一段时间后" in err_str or
                "timeout" in err_str or "10060" in err_str)

    def _check_printer_silence(self):
        silence_time = time.time() - self.last_data_from_printer_time
        if silence_time > PRINTER_SILENCE_TIMEOUT:
            self.logger.error('[BT] 打印机沉默 {} 秒, 连续超时 {} 次, 判定断连'.format(
                int(silence_time), self.consecutive_timeouts))
            self.bt_connected = False
            return True

        if self.consecutive_timeouts >= 1 and self.consecutive_timeouts % 3 == 0:
            self.logger.warning('[BT] 连续超时 {} 次, 打印机沉默 {} 秒'.format(
                self.consecutive_timeouts, int(silence_time)))

        return False

    def run(self):
        print("=" * 60)
        print("新协议桥接程序 v9 (详细日志+沉默检测)")
        print("=" * 60)

        if not self.connect_serial():
            return

        if not self.connect_printer():
            self.ser.close()
            return

        print("\n桥接已启动，按 Ctrl+C 退出")
        print("日志文件: {}".format(self.log_file))
        print("-" * 60)

        self.handler = PhoneDataHandler(self.send_to_printer, self.logger)
        self.worker = serial.threaded.ReaderThread(self.ser, self.handler)
        self.worker.start()

        self.running = True
        self.last_recv_time = time.time()
        self.consecutive_timeouts = 0
        self.last_data_from_printer_time = time.time()

        try:
            while self.running:
                try:
                    data = self.sock.recv(8192)

                    if not data:
                        self.logger.warning("[BT] 打印机断开连接 (recv返回空)")
                        self.bt_connected = False
                        break

                    self.last_recv_time = time.time()
                    self.last_data_from_printer_time = time.time()
                    self.consecutive_timeouts = 0
                    self.phone_count += 1
                    self.ser.write(data)

                    info = parse_packet_info(data)
                    self.logger.debug('[PRT→PHONE] #{} {} ({}字节)'.format(
                        self.phone_count, info, len(data)))

                    if len(data) >= 3 and data[2] == 0x08:
                        self.printer_count += 1
                        now = time.time()
                        if now - self.last_print_time >= 2.0:
                            self.logger.info('[PRT→PHONE] 响应#, 总包{}'.format(self.printer_count))
                            self.last_print_time = now

                except bluetooth.BluetoothError as e:
                    err_str = str(e).lower()
                    self.consecutive_timeouts += 1
                    self.logger.debug('[BT] 蓝牙异常 #{}: {} | {}'.format(
                        self.consecutive_timeouts, type(e).__name__, str(e)[:80]))
                    if self._is_timeout_error(err_str):
                        if self._check_printer_silence():
                            break
                        continue
                    else:
                        self.logger.error("[BT] 蓝牙错误(非超时): {} | {}".format(
                            type(e).__name__, str(e)))
                        self.bt_connected = False
                        break

                except OSError as e:
                    err_str = str(e).lower()
                    self.consecutive_timeouts += 1
                    self.logger.debug('[BT] OS异常 #{}: {} | {}'.format(
                        self.consecutive_timeouts, type(e).__name__, str(e)[:80]))
                    if self._is_timeout_error(err_str):
                        if self._check_printer_silence():
                            break
                        continue
                    else:
                        self.logger.error("[BT] OS错误(非超时): {} | {}".format(
                            type(e).__name__, str(e)))
                        self.bt_connected = False
                        break

                except Exception as e:
                    err_str = str(e).lower()
                    self.logger.debug('[BT] 未知异常: {} | {}'.format(
                        type(e).__name__, str(e)))
                    if self._is_timeout_error(err_str):
                        self.consecutive_timeouts += 1
                        if self._check_printer_silence():
                            break
                        continue
                    self.logger.error("[BT] 未知错误: {} | {}".format(
                        type(e).__name__, str(e)))

                if self.handler:
                    self.handler.check_stuck_buffer()

                if not self.bt_connected:
                    break

        except KeyboardInterrupt:
            print("\n\n正在停止...")
        finally:
            self.running = False
            if self.worker:
                self.worker.stop()
            if self.ser:
                self.ser.close()
            self.disconnect_printer()
            self.logger.info("已断开连接")
            print("日志已保存: {}".format(self.log_file))

if __name__ == "__main__":
    bridge = NewProtocolBridge()
    bridge.run()
