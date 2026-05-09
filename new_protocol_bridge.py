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

CMD_DESCRIPTIONS = {
    0x01: '获取设备能力', 0x02: '获取序列号', 0x05: '查询/心跳',
    0x06: '设置参数', 0x07: '查询参数', 0x08: '通用响应/ACK',
    0x09: '查询信息', 0x0A: '通用响应', 0x0B: '查询信息0B',
    0x0C: '查询信息0C', 0x0D: '信息响应', 0x10: '版本响应',
    0x11: '设置加热参数', 0x12: '查询温度', 0x13: '设置参数',
    0x14: '查询状态', 0x15: '查询电量', 0x16: '序列号响应',
    0x17: '获取设备ID', 0x18: '设备认证请求', 0x19: '设置打印速度',
    0x1A: '设置纸张类型', 0x1B: '打印控制', 0x1E: '获取打印次数',
    0x1F: '发送数据', 0x20: '打印数据/走纸', 0x21: '清除缓冲区',
    0x22: '获取打印状态', 0x28: '获取纸张信息', 0x2F: '查询信息2F',
    0x31: '查询信息31', 0x3B: '发送认证数据', 0x3C: '查询信息3C',
    0x4E: '纸张信息响应', 0x5E: '设备能力响应', 0x62: '设备ID/认证响应',
    0x71: '查询信息71', 0xB5: '打印控制B5', 0xB9: '打印数据V2',
    0xE1: '查询信息E1',
}

REQUEST_RESPONSE_MAP = {
    0x01: 0x5E, 0x02: 0x16, 0x05: 0x10, 0x06: 0x08,
    0x07: 0x08, 0x09: 0x0A, 0x0B: 0x0A, 0x0C: 0x0A,
    0x17: 0x62, 0x18: 0x62, 0x3B: 0x08, 0x2F: 0x08,
    0x28: 0x4E, 0xB9: 0x08, 0x1B: 0x08, 0x1A: 0x08,
    0x19: 0x08, 0x11: 0x08, 0x13: 0x08, 0x1F: 0x08,
    0x20: 0x08, 0x21: 0x08, 0x22: 0x0A,
}

RESPONSE_REQUEST_MAP = {}
for _req, _resp in REQUEST_RESPONSE_MAP.items():
    if _resp not in RESPONSE_REQUEST_MAP:
        RESPONSE_REQUEST_MAP[_resp] = _req


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


class ProtocolLogger:
    DIRECTION_PHONE_TO_PRINTER = '手机→打印机'
    DIRECTION_PRINTER_TO_PHONE = '打印机→手机'

    SESSION_INIT = '初始化'
    SESSION_AUTH = '认证'
    SESSION_QUERY = '查询'
    SESSION_PRINT = '打印'
    SESSION_IDLE = '空闲'

    def __init__(self, logger):
        self.logger = logger
        self.phone_seq_map = {}
        self.printer_seq_map = {}
        self.pending_requests = {}
        self.session_state = self.SESSION_INIT
        self.session_state_time = time.time()
        self.total_phone_packets = 0
        self.total_printer_packets = 0
        self.protocol_flow_log = []
        self._last_flow_time = time.time()

    def _format_hex_dump(self, data, bytes_per_line=32):
        lines = []
        for i in range(0, len(data), bytes_per_line):
            chunk = data[i:i + bytes_per_line]
            hex_part = ' '.join('{:02X}'.format(b) for b in chunk)
            ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            if len(data) <= bytes_per_line:
                return hex_part
            lines.append('    {:04X}: {}  {}'.format(i, hex_part.ljust(bytes_per_line * 3 - 1), ascii_part))
        return '\n' + '\n'.join(lines) if lines else ''

    def _get_cmd_desc(self, cmd):
        name = CMD_NAMES.get(cmd, 'CMD_{:02X}'.format(cmd))
        desc = CMD_DESCRIPTIONS.get(cmd, '未知命令')
        return '{}({:02X}h) [{}]'.format(name, cmd, desc)

    def _is_request(self, cmd):
        return cmd in REQUEST_RESPONSE_MAP

    def _is_response(self, cmd):
        return cmd in RESPONSE_REQUEST_MAP

    def _detect_session_transition(self, direction, cmd, data):
        old_state = self.session_state
        if cmd == 0x3B or cmd == 0x18:
            self.session_state = self.SESSION_AUTH
        elif cmd in [0x05, 0x02, 0x17, 0x07, 0x09, 0x0B, 0x0C, 0x2F, 0x28, 0x01]:
            self.session_state = self.SESSION_QUERY
        elif cmd == 0xB9 or cmd == 0x1B:
            self.session_state = self.SESSION_PRINT
        elif cmd == 0x08 and direction == self.DIRECTION_PRINTER_TO_PHONE:
            if old_state == self.SESSION_PRINT:
                pass
            elif old_state != self.SESSION_AUTH:
                self.session_state = self.SESSION_IDLE
        elif cmd in [0x10, 0x16, 0x62, 0x5E, 0x4E, 0x0A, 0x0D]:
            if old_state == self.SESSION_QUERY:
                self.session_state = self.SESSION_IDLE

        if self.session_state != old_state:
            elapsed = time.time() - self.session_state_time
            self.logger.info('[协议流程] 会话状态变更: {} → {} (持续{:.1f}秒)'.format(
                old_state, self.session_state, elapsed))
            self.session_state_time = time.time()

    def _parse_data_payload(self, cmd, data_bytes, direction):
        parts = []
        if len(data_bytes) == 0:
            return '(无数据)'

        if cmd == 0x10 and direction == self.DIRECTION_PRINTER_TO_PHONE:
            if len(data_bytes) >= 3:
                version_len = data_bytes[0]
                if version_len > 0 and len(data_bytes) >= 1 + version_len:
                    try:
                        ver = data_bytes[1:1 + version_len].decode('utf-8', errors='replace')
                        parts.append('版本号: {}'.format(ver))
                    except:
                        pass
                offset = 1 + version_len
                if offset < len(data_bytes):
                    parts.append('附加数据({}字节): {}'.format(
                        len(data_bytes) - offset, data_bytes[offset:].hex()))

        elif cmd == 0x16 and direction == self.DIRECTION_PRINTER_TO_PHONE:
            try:
                sn = data_bytes.decode('utf-8', errors='replace')
                parts.append('序列号: {}'.format(sn))
            except:
                parts.append('SN数据: {}'.format(data_bytes.hex()))

        elif cmd == 0x62 and direction == self.DIRECTION_PRINTER_TO_PHONE:
            if len(data_bytes) >= 2:
                id_len = struct.unpack('<H', data_bytes[0:2])[0] if len(data_bytes) >= 2 else 0
                if id_len > 0 and len(data_bytes) >= 2 + id_len:
                    try:
                        device_id = data_bytes[2:2 + id_len].decode('utf-8', errors='replace')
                        parts.append('设备ID: {}'.format(device_id))
                    except:
                        pass
                else:
                    parts.append('ID数据: {}'.format(data_bytes.hex()))
            else:
                parts.append('ID数据: {}'.format(data_bytes.hex()))

        elif cmd == 0x5E and direction == self.DIRECTION_PRINTER_TO_PHONE:
            parts.append('能力数据({}字节): {}'.format(len(data_bytes), data_bytes.hex()))

        elif cmd == 0x4E and direction == self.DIRECTION_PRINTER_TO_PHONE:
            if len(data_bytes) >= 4:
                paper_type = data_bytes[0]
                paper_width = struct.unpack('<H', data_bytes[1:3])[0] if len(data_bytes) >= 3 else 0
                parts.append('纸张类型: {}, 纸张宽度: {}px'.format(paper_type, paper_width))
                if len(data_bytes) > 4:
                    parts.append('附加: {}'.format(data_bytes[4:].hex()))
            else:
                parts.append('纸张数据: {}'.format(data_bytes.hex()))

        elif cmd == 0x08 and direction == self.DIRECTION_PRINTER_TO_PHONE:
            if len(data_bytes) >= 2:
                result = data_bytes[0]
                sub_cmd = data_bytes[1] if len(data_bytes) > 1 else 0
                result_str = '成功' if result == 0x00 else '失败({:02X}h)'.format(result)
                parts.append('结果: {}, 子命令: {:02X}h'.format(result_str, sub_cmd))
                if len(data_bytes) > 2:
                    parts.append('附加: {}'.format(data_bytes[2:].hex()))
            else:
                parts.append('ACK数据: {}'.format(data_bytes.hex()))

        elif cmd == 0x0A and direction == self.DIRECTION_PRINTER_TO_PHONE:
            if len(data_bytes) >= 1:
                parts.append('响应数据({}字节): {}'.format(len(data_bytes), data_bytes.hex()))
            else:
                parts.append('空响应')

        elif cmd == 0x0D and direction == self.DIRECTION_PRINTER_TO_PHONE:
            parts.append('信息数据({}字节): {}'.format(len(data_bytes), data_bytes.hex()))

        elif cmd == 0x3B and direction == self.DIRECTION_PHONE_TO_PRINTER:
            parts.append('认证数据({}字节): {}'.format(len(data_bytes), data_bytes.hex()))

        elif cmd == 0xB9 and direction == self.DIRECTION_PHONE_TO_PRINTER:
            parts.append('打印图像数据({}字节)'.format(len(data_bytes)))

        elif cmd == 0x1B and direction == self.DIRECTION_PHONE_TO_PRINTER:
            if len(data_bytes) >= 1:
                ctrl = data_bytes[0]
                ctrl_names = {0x01: '开始打印', 0x02: '结束打印', 0x03: '取消打印'}
                ctrl_name = ctrl_names.get(ctrl, '控制{:02X}h'.format(ctrl))
                parts.append('控制类型: {} ({:02X}h)'.format(ctrl_name, ctrl))
                if len(data_bytes) > 1:
                    parts.append('附加: {}'.format(data_bytes[1:].hex()))
            else:
                parts.append('打印控制(空)')

        elif cmd == 0x06 and direction == self.DIRECTION_PHONE_TO_PRINTER:
            if len(data_bytes) >= 1:
                parts.append('参数数据: {}'.format(data_bytes.hex()))
            else:
                parts.append('设置参数(空)')

        elif cmd == 0x11 and direction == self.DIRECTION_PHONE_TO_PRINTER:
            if len(data_bytes) >= 1:
                parts.append('加热参数: {}'.format(data_bytes.hex()))
            else:
                parts.append('加热参数(空)')

        elif cmd == 0x19 and direction == self.DIRECTION_PHONE_TO_PRINTER:
            if len(data_bytes) >= 1:
                parts.append('速度参数: {}'.format(data_bytes.hex()))
            else:
                parts.append('速度参数(空)')

        elif cmd == 0x1A and direction == self.DIRECTION_PHONE_TO_PRINTER:
            if len(data_bytes) >= 1:
                parts.append('纸张类型: {}'.format(data_bytes.hex()))
            else:
                parts.append('纸张类型(空)')

        elif cmd == 0x05 and direction == self.DIRECTION_PHONE_TO_PRINTER:
            parts.append('心跳/查询请求')

        else:
            try:
                text = data_bytes.decode('utf-8', errors='strict')
                if text and all(c.isprintable() or c in '\r\n\t' for c in text):
                    parts.append('文本: {}'.format(text))
                else:
                    parts.append('数据({}字节): {}'.format(len(data_bytes), data_bytes.hex()))
            except:
                parts.append('数据({}字节): {}'.format(len(data_bytes), data_bytes.hex()))

        return ' | '.join(parts) if parts else '(无数据)'

    def _track_request_response(self, direction, cmd, seq, ref):
        flow_info = ''
        now = time.time()

        if direction == self.DIRECTION_PHONE_TO_PRINTER:
            if self._is_request(cmd):
                expected_resp = REQUEST_RESPONSE_MAP[cmd]
                self.pending_requests[seq] = {
                    'cmd': cmd, 'time': now, 'expected_resp': expected_resp
                }
                resp_name = CMD_NAMES.get(expected_resp, 'CMD_{:02X}'.format(expected_resp))
                flow_info = '请求 → 期望响应: {}({:02X}h)'.format(resp_name, expected_resp)

            elif cmd in self.pending_requests:
                pass

        elif direction == self.DIRECTION_PRINTER_TO_PHONE:
            if self._is_response(cmd):
                matched_req = RESPONSE_REQUEST_MAP.get(cmd)
                if matched_req is not None:
                    req_name = CMD_NAMES.get(matched_req, 'CMD_{:02X}'.format(matched_req))
                    flow_info = '响应 ← 对应请求: {}({:02X}h)'.format(req_name, matched_req)

                for pseq, preq in list(self.pending_requests.items()):
                    if preq['expected_resp'] == cmd:
                        elapsed = now - preq['time']
                        flow_info += ' | 匹配SEQ:{} (耗时{:.3f}秒)'.format(pseq, elapsed)
                        del self.pending_requests[pseq]
                        break

            elif self._is_request(cmd):
                flow_info = '打印机主动发送'

        return flow_info

    def log_raw_packet(self, direction, data):
        if len(data) < HEADER_LEN + FOOTER_LEN:
            self.logger.info('[原始报文] {} | 短数据 {}字节 | HEX: {}'.format(
                direction, len(data), data.hex()))
            return

        cmd = data[2]
        seq = struct.unpack('<H', data[3:5])[0]
        length = struct.unpack('<H', data[7:9])[0]
        cmd_desc = self._get_cmd_desc(cmd)

        hex_dump = self._format_hex_dump(data)

        self.logger.info('[原始报文] {} | {} | SEQ:{} | 总长:{}字节(数据{}) | HEX:{}'.format(
            direction, cmd_desc, seq, len(data), length, hex_dump))

    def log_parsed_packet(self, direction, data):
        if len(data) < HEADER_LEN + FOOTER_LEN:
            self.logger.info('[协议解析] {} | 数据过短 {}字节, 无法解析'.format(direction, len(data)))
            return

        start = data[0]
        addr = data[1]
        cmd = data[2]
        seq = struct.unpack('<H', data[3:5])[0]
        ref = data[5]
        status = data[6]
        length = struct.unpack('<H', data[7:9])[0]
        payload = data[9:9 + length]
        checksum = data[9 + length:9 + length + 4]
        end = data[-1]

        cmd_desc = self._get_cmd_desc(cmd)

        self.logger.info('[协议解析] {} | {}'.format(direction, cmd_desc))
        self.logger.info('  头部: START={:02X}h ADDR={:02X}h CMD={:02X}h SEQ={} REF={:02X}h STATUS={:02X}h LEN={}'.format(
            start, addr, cmd, seq, ref, status, length))
        self.logger.info('  校验: {} | 尾部: {:02X}h'.format(checksum.hex(), end))

        payload_info = self._parse_data_payload(cmd, payload, direction)
        self.logger.info('  载荷: {}'.format(payload_info))

        flow_info = self._track_request_response(direction, cmd, seq, ref)
        if flow_info:
            self.logger.info('  流程: {}'.format(flow_info))

        self._detect_session_transition(direction, cmd, data)

        if direction == self.DIRECTION_PHONE_TO_PRINTER:
            self.total_phone_packets += 1
            self.phone_seq_map[seq] = cmd
        else:
            self.total_printer_packets += 1
            self.printer_seq_map[seq] = cmd

        now = time.time()
        if now - self._last_flow_time >= 30:
            self._log_flow_summary()
            self._last_flow_time = now

    def _log_flow_summary(self):
        self.logger.info('[协议流程] ===== 交互统计 =====')
        self.logger.info('[协议流程] 手机→打印机: {}包 | 打印机→手机: {}包'.format(
            self.total_phone_packets, self.total_printer_packets))
        self.logger.info('[协议流程] 当前会话状态: {} | 待响应请求: {}个'.format(
            self.session_state, len(self.pending_requests)))
        if self.pending_requests:
            for pseq, preq in list(self.pending_requests.items()):
                req_name = CMD_NAMES.get(preq['cmd'], 'CMD_{:02X}'.format(preq['cmd']))
                resp_name = CMD_NAMES.get(preq['expected_resp'], 'CMD_{:02X}'.format(preq['expected_resp']))
                elapsed = time.time() - preq['time']
                self.logger.info('[协议流程]   待响应: {}(SEQ:{}) → {} (等待{:.1f}秒)'.format(
                    req_name, pseq, resp_name, elapsed))
        self.logger.info('[协议流程] ======================')

    def log_protocol_event(self, event_type, detail):
        self.logger.info('[协议流程] {} | {}'.format(event_type, detail))


class PhoneDataHandler(serial.threaded.Protocol):
    """处理手机→打印机方向的数据"""

    def __init__(self, enqueue_callback, logger, protocol_logger=None):
        self.enqueue_callback = enqueue_callback
        self.logger = logger
        self.protocol_logger = protocol_logger
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

            if self.protocol_logger:
                self.protocol_logger.log_raw_packet(
                    ProtocolLogger.DIRECTION_PHONE_TO_PRINTER, data)
                self.protocol_logger.log_parsed_packet(
                    ProtocolLogger.DIRECTION_PHONE_TO_PRINTER, data)

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
        self.protocol_logger = ProtocolLogger(self.logger)

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

    @staticmethod
    def _split_packets(data):
        packets = []
        offset = 0
        while offset < len(data):
            if data[offset] != 0xA5:
                offset += 1
                continue
            if offset + HEADER_LEN > len(data):
                break
            length = struct.unpack('<H', data[offset + 7:offset + 9])[0]
            total_len = HEADER_LEN + length + FOOTER_LEN
            if offset + total_len > len(data):
                break
            packets.append(data[offset:offset + total_len])
            offset += total_len
        if not packets and data:
            packets.append(data)
        return packets

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

        self.protocol_logger.log_protocol_event('连接建立', '串口={} 蓝牙={}'.format(COM_PORT, PAPERANG_ADDR))

        self.handler = PhoneDataHandler(self.enqueue_to_printer, self.logger, self.protocol_logger)
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

                    for pkt in self._split_packets(data):
                        self.protocol_logger.log_raw_packet(
                            ProtocolLogger.DIRECTION_PRINTER_TO_PHONE, pkt)
                        self.protocol_logger.log_parsed_packet(
                            ProtocolLogger.DIRECTION_PRINTER_TO_PHONE, pkt)

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
            self.protocol_logger.log_protocol_event('连接断开', '手机→打印机:{}包 打印机→手机:{}包'.format(
                self.protocol_logger.total_phone_packets,
                self.protocol_logger.total_printer_packets))
            self.protocol_logger._log_flow_summary()
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
