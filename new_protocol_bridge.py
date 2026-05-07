"""
新协议桥接程序 v6
- 优化大数据传输
- 增加超时时间
- 减少日志输出
"""

import serial
import serial.threaded
import bluetooth
import time
import struct

COM_PORT = 'COM4'
BAUD = 115200
PAPERANG_ADDR = "03:0B:F8:E0:D3:E8"

RECV_TIMEOUT = 120

class PhoneDataHandler(serial.threaded.Protocol):
    """处理手机数据"""
    
    def __init__(self, send_callback):
        self.send_callback = send_callback
        self.buffer = bytearray()
        self.count = 0
        self.print_count = 0
        self.last_time = 0
    
    def __call__(self):
        return self
    
    def data_received(self, data):
        self.buffer.extend(data)
        self._process_buffer()
    
    def _process_buffer(self):
        while len(self.buffer) >= 14:
            start_idx = self._find_start()
            if start_idx == -1:
                self.buffer.clear()
                return
            
            if start_idx > 0:
                del self.buffer[:start_idx]
            
            if len(self.buffer) < 9:
                return
            
            length = struct.unpack('<H', self.buffer[7:9])[0]
            total_len = 9 + length + 5
            
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
            
            if cmd == 0xB9:
                self.print_count += 1
                now = time.time()
                if now - self.last_time >= 2.0:
                    print('[{}] 手机→打印机: 打印#{}, 总包{}'.format(
                        time.strftime('%H:%M:%S'), self.print_count, self.count))
                    self.last_time = now
            
            if self.send_callback:
                self.send_callback(data)
                
        except Exception as e:
            print('转发错误: {}'.format(e))

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
    
    def connect_serial(self):
        try:
            self.ser = serial.Serial(COM_PORT, BAUD, timeout=1)
            self.ser.set_buffer_size(rx_size=8192, tx_size=8192)
            print("串口 {} 已打开".format(COM_PORT))
            return True
        except Exception as e:
            print("无法打开串口: {}".format(e))
            return False
    
    def connect_printer(self):
        try:
            print("正在连接打印机 {}...".format(PAPERANG_ADDR))
            self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.sock.connect((PAPERANG_ADDR, 1))
            self.sock.settimeout(RECV_TIMEOUT)
            print("打印机已连接")
            return True
        except Exception as e:
            print("连接打印机失败: {}".format(e))
            return False
    
    def disconnect_printer(self):
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
    
    def send_to_printer(self, data):
        if self.sock:
            return self.sock.send(data)
        return 0
    
    def run(self):
        print("=" * 60)
        print("新协议桥接程序 v6")
        print("=" * 60)
        
        if not self.connect_serial():
            return
        
        if not self.connect_printer():
            self.ser.close()
            return
        
        print("\n桥接已启动，按 Ctrl+C 退出")
        print("-" * 60)
        
        self.handler = PhoneDataHandler(self.send_to_printer)
        self.worker = serial.threaded.ReaderThread(self.ser, self.handler)
        self.worker.start()
        
        self.running = True
        self.last_recv_time = time.time()
        
        try:
            while self.running:
                try:
                    data = self.sock.recv(8192)
                    if data:
                        self.last_recv_time = time.time()
                        self.ser.write(data)
                        
                        if len(data) >= 3 and data[2] == 0x08:
                            self.printer_count += 1
                            now = time.time()
                            if now - self.last_print_time >= 2.0:
                                print('[{}] 打印机→手机: 响应#, 总包{}'.format(
                                    time.strftime('%H:%M:%S'), self.printer_count))
                                self.last_print_time = now
                
                except bluetooth.BluetoothError as e:
                    err_str = str(e).lower()
                    if "timed out" in err_str or "一段时间后" in err_str:
                        continue
                    else:
                        print("蓝牙错误: {}".format(e))
                        break
                
                except Exception as e:
                    print("错误: {}".format(e))
                        
        except KeyboardInterrupt:
            print("\n\n正在停止...")
        finally:
            self.running = False
            if self.worker:
                self.worker.stop()
            if self.ser:
                self.ser.close()
            self.disconnect_printer()
            print("已断开连接")

if __name__ == "__main__":
    bridge = NewProtocolBridge()
    bridge.run()
