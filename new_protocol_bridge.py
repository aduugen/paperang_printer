"""
新协议桥接程序 v12
关键修复:
  - 修复线程安全: BT socket 读写加锁, 避免并发冲突导致断连
  - 手机→打印机数据改用发送队列, 不再阻塞 ReaderThread
  - 发送队列独立线程, 避免阻塞串口读取
诊断增强:
  - ACK 原始 hex 转储, 验证 SEQ 解析
  - 双向 SEQ 追踪 (请求 SEQ vs ACK SEQ)
  - 打印会话计数和间隔时间
  - 断连时完整状态转储
"""

import serial
import serial.threaded
import bluetooth
import time
import struct
import threading
import logging
import os
import queue
from datetime import datetime

COM_PORT = 'COM4'
BAUD = 115200
PAPERANG_ADDR = "03:0B:F8:E0:D3:E8"

RECV_TIMEOUT = 10
STUCK_TIMEOUT = 5
PRINTER_SILENCE_TIMEOUT = 30
PHONE_SILENCE_WARN = 15
HEADER_LEN = 9
FOOTER_LEN = 5
LOG_DIR = 'logs'
STAT_INTERVAL = 10
SEND_QUEUE_SIZE = 200
ACK_HEX_DUMP_COUNT = 10

CMD_NAMES = {
    0x05: 'QUERY', 0x02: 'GET_SN', 0x06: 'CMD_06', 0x07: 'CMD_07',
    0x08: 'ACK', 0x09: 'CMD_09', 0x0A: 'CMD_0A', 0x0B: 'CMD_0B',
    0x0C: 'CMD_0C', 0x10: 'CMD_10', 0x11: 'CMD_11', 0x16: 'CMD_16',
    0x17: 'GET_ID', 0x18: 'GET_ID', 0x19: 'CMD_19', 0x1A: 'CMD_1A',
    0x1B: 'CMD_1B', 0x2F: 'CMD_2F', 0x31: 'CMD_31', 0x3C: 'CMD_3C',
    0x3B: 'AUTH', 0x4E: 'CMD_4E', 0x5E: 'CMD_5E', 0x62: 'CMD_62',
    0x71: 'CMD_71', 0x0D: 'CMD_0D', 0xB9: 'PRINT',
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
    status = data[6]
    length = struct.unpack('<H', data[7:9])[0]
    cmd_name = CMD_NAMES.get(cmd, 'CMD_{:02X}'.format(cmd))
    return '{} SEQ:{} REF:{:02X} ST:{:02X} LEN:{}'.format(cmd_name, seq, ref, status, length)


class PhoneDataHandler(serial.threaded.Protocol):
    """处理手机→打印机方向的数据"""

    def __init__(self, enqueue_callback, logger):
        self.enqueue_callback = enqueue_callback
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
        self.print_data_sent = 0
        self.print_data_bytes = 0
        self.enqueue_fail_count = 0
        self.total_bytes_from_phone = 0
        self.last_data_from_phone_time = time.time()
        self.phone_seq_log = []
        self.print_session_count = 0
        self.last_print_session_end_time = 0

    def __call__(self):
        return self

    def data_received(self, data):
        self.last_packet_time = time.time()
        self.data_receive_count += 1
        self.total_bytes_from_phone += len(data)
        self.last_data_from_phone_time = time.time()

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
            seq = struct.unpack('<H', data[3:5])[0] if len(data) >= 5 else 0
            self.count += 1

            if len(self.phone_seq_log) < 50:
                self.phone_seq_log.append((cmd, seq))

            is_print_data = (cmd == 0xB9)
            is_control_cmd = (cmd in [0x05, 0x02, 0x06, 0x07, 0x08, 0x09, 0x17, 0x18, 0x3B])

            if is_print_data:
                if not self.print_session_start:
                    self.print_session_start = True
                    self.print_session_count += 1
                    self.print_data_sent = 0
                    self.print_data_bytes = 0
                    self.enqueue_fail_count = 0
                    gap = 0
                    if self.last_print_session_end_time > 0:
                        gap = time.time() - self.last_print_session_end_time
                    self.logger.info('=== 开始打印任务 #{} (距上次: {:.1f}秒) ==='.format(
                        self.print_session_count, gap))

                self.print_count += 1
                self.print_data_sent += 1
                self.print_data_bytes += len(data)

                info = parse_packet_info(data)
                self.logger.info('[PHONE→PRT] 打印包#{:03d} {} ({}字节)'.format(
                    self.print_data_sent, info, len(data)))
            else:
                if is_control_cmd and self.print_session_start:
                    self.print_session_start = False
                    self.last_print_session_end_time = time.time()
                    self.logger.info('=== 打印任务 #{} 结束 ({}个打印包, {}字节, 入队失败{}) ==='.format(
                        self.print_session_count, self.print_data_sent,
                        self.print_data_bytes, self.enqueue_fail_count))

                info = parse_packet_info(data)
                self.logger.debug('[PHONE→PRT] #{} {}'.format(self.count, info))

            if self.enqueue_callback:
                try:
                    self.enqueue_callback(data)
                except queue.Full:
                    self.enqueue_fail_count += 1
                    self.logger.warning('[PHONE→PRT] 发送队列满! 丢弃包 (累计失败{})'.format(
                        self.enqueue_fail_count))

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
    """新协议桥接 v12 - 线程安全 + 发送队列"""

    def __init__(self):
        self.ser = None
        self.sock = None
        self.running = False
        self.handler = None
        self.worker = None
        self.last_recv_time = time.time()
        self.printer_count = 0
        self.ack_count = 0
        self.last_print_time = 0
        self.bt_connected = True
        self.logger, self.log_file = setup_logger()
        self.phone_count = 0
        self.consecutive_timeouts = 0
        self.last_data_from_printer_time = time.time()
        self.total_bt_send_bytes = 0
        self.total_bt_send_calls = 0
        self.total_bt_send_fail = 0
        self.total_ser_write_bytes = 0
        self.total_ser_write_calls = 0
        self.total_ser_write_fail = 0
        self.last_stat_time = time.time()

        self.sock_lock = threading.Lock()
        self.send_queue = queue.Queue(maxsize=SEND_QUEUE_SIZE)
        self.sender_thread = None
        self.sender_running = False
        self.queue_drop_count = 0

        self.ack_hex_dump_remaining = ACK_HEX_DUMP_COUNT
        self.ack_seq_log = []

    def connect_serial(self):
        try:
            self.ser = serial.Serial(COM_PORT, BAUD, timeout=1, write_timeout=5)
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

    def _start_sender_thread(self):
        self.sender_running = True
        self.sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self.sender_thread.start()
        self.logger.info('[发送线程] 已启动')

    def _stop_sender_thread(self):
        self.sender_running = False
        try:
            self.send_queue.put(None, timeout=1)
        except queue.Full:
            pass
        if self.sender_thread and self.sender_thread.is_alive():
            self.sender_thread.join(timeout=3)
        self.logger.info('[发送线程] 已停止')

    def _sender_loop(self):
        while self.sender_running:
            try:
                item = self.send_queue.get(timeout=0.5)
                if item is None:
                    continue
                self._do_send_to_printer(item)
            except queue.Empty:
                continue
            except Exception as e:
                self.logger.error('[发送线程] 异常: {}'.format(e))

    def _do_send_to_printer(self, data):
        if not self.sock or not self.bt_connected:
            self.total_bt_send_fail += 1
            return

        try:
            with self.sock_lock:
                total_sent = 0
                remaining = data
                retries = 0
                while remaining and retries < 3:
                    sent = self.sock.send(remaining)
                    self.total_bt_send_bytes += sent
                    self.total_bt_send_calls += 1
                    total_sent += sent
                    if sent >= len(remaining):
                        break
                    remaining = remaining[sent:]
                    retries += 1
                    self.logger.warning('[BT] 部分发送: {}/{}, 重试{}'.format(
                        total_sent, len(data), retries))

                if total_sent < len(data):
                    self.total_bt_send_fail += 1
                    self.logger.error('[BT] 发送不完整: {}/{} 字节'.format(total_sent, len(data)))

        except bluetooth.BluetoothError as e:
            self.total_bt_send_fail += 1
            self.logger.error('[BT] 发送失败: {} | {}'.format(type(e).__name__, str(e)))
            self.bt_connected = False
        except OSError as e:
            self.total_bt_send_fail += 1
            self.logger.error('[BT] 发送OS错误: {} | {}'.format(type(e).__name__, str(e)))
            self.bt_connected = False
        except Exception as e:
            self.total_bt_send_fail += 1
            self.logger.error('[BT] 发送异常: {} | {}'.format(type(e).__name__, str(e)))
            self.bt_connected = False

    def enqueue_to_printer(self, data):
        try:
            self.send_queue.put_nowait(data)
        except queue.Full:
            self.queue_drop_count += 1
            self.logger.warning('[队列] 发送队列满! 丢弃数据 (累计丢弃{})'.format(
                self.queue_drop_count))
            raise queue.Full

    def write_to_serial(self, data):
        if not self.ser or not self.ser.is_open:
            self.logger.error('[SER] 串口未打开')
            return 0

        try:
            total_written = 0
            remaining = data
            retries = 0
            while remaining and retries < 3:
                written = self.ser.write(remaining)
                self.total_ser_write_bytes += written
                self.total_ser_write_calls += 1
                total_written += written
                if written >= len(remaining):
                    break
                remaining = remaining[written:]
                retries += 1
                self.logger.warning('[SER] 部分写入: {}/{}, 重试{}'.format(
                    total_written, len(data), retries))

            if total_written < len(data):
                self.total_ser_write_fail += 1
                self.logger.error('[SER] 写入不完整: {}/{} 字节 ← ACK可能丢失!'.format(
                    total_written, len(data)))

            return total_written
        except serial.SerialTimeoutException as e:
            self.total_ser_write_fail += 1
            self.logger.error('[SER] 写入超时: {} ← HC-06缓冲区可能满了!'.format(e))
            return 0
        except Exception as e:
            self.total_ser_write_fail += 1
            self.logger.error('[SER] 写入错误: {}'.format(e))
            return 0

    def _is_timeout_error(self, err_str):
        return ("timed out" in err_str or "一段时间后" in err_str or
                "timeout" in err_str or "10060" in err_str)

    def _check_printer_silence(self):
        silence_time = time.time() - self.last_data_from_printer_time
        if silence_time > PRINTER_SILENCE_TIMEOUT:
            self.logger.error('[BT] 打印机沉默 {} 秒, 连续超时 {} 次, 判定断连'.format(
                int(silence_time), self.consecutive_timeouts))
            self._dump_disconnect_state('打印机沉默超时')
            self.bt_connected = False
            return True

        if self.consecutive_timeouts >= 1 and self.consecutive_timeouts % 3 == 0:
            self.logger.warning('[BT] 连续超时 {} 次, 打印机沉默 {} 秒'.format(
                self.consecutive_timeouts, int(silence_time)))

        return False

    def _check_phone_silence(self):
        if not self.handler:
            return
        phone_silence = time.time() - self.handler.last_data_from_phone_time
        if phone_silence > PHONE_SILENCE_WARN:
            self.logger.warning('[PHONE] 手机沉默 {} 秒 (最后收包: {}个/{}字节)'.format(
                int(phone_silence), self.handler.count, self.handler.total_bytes_from_phone))

    def _log_stats(self):
        phone_bytes = self.handler.total_bytes_from_phone if self.handler else 0
        phone_pkts = self.handler.count if self.handler else 0
        phone_print = self.handler.print_data_sent if self.handler else 0
        qsize = self.send_queue.qsize()
        self.logger.info('[统计] 手机→{}包/{}字节, 打印包:{}, ACK:{}, 发送队列:{} | 串口写:{}次/{}字节/失败{}, BT发:{}次/{}字节/失败{}, 队列丢弃:{}'.format(
            phone_pkts, phone_bytes, phone_print, self.ack_count, qsize,
            self.total_ser_write_calls, self.total_ser_write_bytes, self.total_ser_write_fail,
            self.total_bt_send_calls, self.total_bt_send_bytes, self.total_bt_send_fail,
            self.queue_drop_count))

    def _periodic_stats(self):
        now = time.time()
        if now - self.last_stat_time >= STAT_INTERVAL:
            self._log_stats()
            self.last_stat_time = now

    def _dump_disconnect_state(self, reason):
        self.logger.error('=' * 50)
        self.logger.error('[断连诊断] 原因: {}'.format(reason))
        self.logger.error('[断连诊断] 打印会话: #{}'.format(
            self.handler.print_session_count if self.handler else '?'))
        self.logger.error('[断连诊断] 当前打印中: {}'.format(
            self.handler.print_session_start if self.handler else '?'))
        self.logger.error('[断连诊断] 手机→打印包: {}, ACK: {}'.format(
            self.handler.print_data_sent if self.handler else '?', self.ack_count))
        self.logger.error('[断连诊断] 发送队列: {}/{}, 队列丢弃: {}'.format(
            self.send_queue.qsize(), SEND_QUEUE_SIZE, self.queue_drop_count))
        self.logger.error('[断连诊断] 串口写失败: {}, BT发失败: {}'.format(
            self.total_ser_write_fail, self.total_bt_send_fail))
        self.logger.error('[断连诊断] 连续超时: {}, 打印机沉默: {:.1f}秒'.format(
            self.consecutive_timeouts,
            time.time() - self.last_data_from_printer_time))

        if self.handler and self.handler.phone_seq_log:
            self.logger.error('[断连诊断] 手机最近SEQ: {}'.format(
                self.handler.phone_seq_log[-10:]))
        if self.ack_seq_log:
            self.logger.error('[断连诊断] ACK最近SEQ: {}'.format(
                self.ack_seq_log[-10:]))
        self.logger.error('=' * 50)

    def run(self):
        print("=" * 60)
        print("新协议桥接程序 v12 (线程安全+发送队列+诊断增强)")
        print("=" * 60)

        if not self.connect_serial():
            return

        if not self.connect_printer():
            self.ser.close()
            return

        print("\n桥接已启动，按 Ctrl+C 退出")
        print("日志文件: {}".format(self.log_file))
        print("-" * 60)

        self.handler = PhoneDataHandler(self.enqueue_to_printer, self.logger)
        self.worker = serial.threaded.ReaderThread(self.ser, self.handler)
        self.worker.start()

        self._start_sender_thread()

        self.running = True
        self.last_recv_time = time.time()
        self.consecutive_timeouts = 0
        self.last_data_from_printer_time = time.time()
        self.last_stat_time = time.time()

        try:
            while self.running:
                try:
                    data = self.sock.recv(8192)

                    if not data:
                        self.logger.warning("[BT] 打印机断开连接 (recv返回空)")
                        self._dump_disconnect_state('recv返回空')
                        self.bt_connected = False
                        break

                    self.last_recv_time = time.time()
                    self.last_data_from_printer_time = time.time()
                    self.consecutive_timeouts = 0
                    self.phone_count += 1

                    written = self.write_to_serial(data)

                    info = parse_packet_info(data)
                    self.logger.debug('[PRT→PHONE] #{} {} ({}字节, 写入{})'.format(
                        self.phone_count, info, len(data), written))

                    if len(data) >= 3 and data[2] == 0x08:
                        self.ack_count += 1
                        ack_seq = struct.unpack('<H', data[3:5])[0] if len(data) >= 5 else 0
                        self.ack_seq_log.append(ack_seq)

                        if self.ack_hex_dump_remaining > 0:
                            self.ack_hex_dump_remaining -= 1
                            self.logger.info('[PRT→PHONE] ACK#{:03d} {} HEX:{}'.format(
                                self.ack_count, info, data.hex()))
                        else:
                            self.logger.info('[PRT→PHONE] ACK#{:03d} {}'.format(
                                self.ack_count, info))

                except bluetooth.BluetoothError as e:
                    err_str = str(e).lower()
                    self.consecutive_timeouts += 1
                    self.logger.debug('[BT] 蓝牙异常 #{}: {} | {}'.format(
                        self.consecutive_timeouts, type(e).__name__, str(e)[:80]))
                    if self._is_timeout_error(err_str):
                        if self._check_printer_silence():
                            break
                        self._check_phone_silence()
                        continue
                    else:
                        self.logger.error("[BT] 蓝牙错误(非超时): {} | {}".format(
                            type(e).__name__, str(e)))
                        self._dump_disconnect_state('蓝牙错误(非超时): {}'.format(str(e)[:100]))
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
                        self._check_phone_silence()
                        continue
                    else:
                        self.logger.error("[BT] OS错误(非超时): {} | {}".format(
                            type(e).__name__, str(e)))
                        self._dump_disconnect_state('OS错误(非超时): {}'.format(str(e)[:100]))
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
                    self._dump_disconnect_state('未知错误: {}'.format(str(e)[:100]))
                    self.bt_connected = False
                    break

                if self.handler:
                    self.handler.check_stuck_buffer()

                self._periodic_stats()

                if not self.bt_connected:
                    break

        except KeyboardInterrupt:
            print("\n\n正在停止...")
        finally:
            self.running = False
            self._log_stats()
            self._stop_sender_thread()
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
