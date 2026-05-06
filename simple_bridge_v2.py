"""
简化版桥接测试脚本 v2
增加超时处理和错误恢复
"""

import serial
import serial.threaded
import bluetooth
import time
from logger import Logger as Logger

COM_PORT = 'COM3'
BAUD = 115200
PAPERANG_ADDR = "03:0B:F8:E0:D3:E8"

class SimpleHandler(serial.threaded.Protocol):
    def __init__(self, logging, socket=None):
        self.socket = socket
        self.logging = logging
        self.count = 0
        self.recv_count = 0

    def __call__(self):
        return self

    def data_received(self, data):
        self.count += 1
        self.logging.info('[手机→打印机] #{} 长度:{} 数据: {}'.format(
            self.count, len(data), data.hex()))
        if self.socket:
            try:
                sent = self.socket.send(data)
                self.logging.info('  已转发 {} 字节到打印机'.format(sent))
            except Exception as e:
                self.logging.error('  转发失败: {}'.format(e))

def connect_printer():
    """连接打印机"""
    print("正在连接打印机 {}...".format(PAPERANG_ADDR))
    try:
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((PAPERANG_ADDR, 1))
        sock.settimeout(120)
        print("[OK] 打印机已连接")
        return sock
    except Exception as e:
        print("[错误] 连接失败: {}".format(e))
        return None

def main():
    log = Logger('simple_bridge_v2.log', level='info')
    
    print("=" * 50)
    print("简化版桥接测试 v2")
    print("=" * 50)
    
    print("\n[步骤1] 打开串口...")
    try:
        ser = serial.Serial(COM_PORT, BAUD, timeout=1)
        print("[OK] 串口 {} 已打开".format(COM_PORT))
    except Exception as e:
        print("[错误] 无法打开串口: {}".format(e))
        return
    
    print("\n[步骤2] 连接打印机...")
    sock = connect_printer()
    if not sock:
        ser.close()
        return
    
    print("\n[步骤3] 启动数据转发...")
    print("-" * 50)
    
    handler = SimpleHandler(log.logger, sock)
    worker = serial.threaded.ReaderThread(ser, handler)
    worker.start()
    
    print("桥接已启动，按 Ctrl+C 退出")
    print("-" * 50)
    
    recv_count = 0
    try:
        while True:
            try:
                data = sock.recv(1024)
                if data:
                    recv_count += 1
                    log.logger.info('[打印机→手机] #{} 长度:{} 数据: {}'.format(
                        recv_count, len(data), data.hex()))
                    ser.write(data)
                    log.logger.info('  已转发 {} 字节到手机'.format(len(data)))
            except bluetooth.BluetoothError as e:
                log.logger.error('蓝牙错误: {}'.format(e))
                print("\n打印机连接断开，尝试重连...")
                handler.socket = None
                sock = connect_printer()
                if sock:
                    handler.socket = sock
                    recv_count = 0
                else:
                    break
            except Exception as e:
                log.logger.error('错误: {}'.format(e))
                break
    except KeyboardInterrupt:
        print("\n\n正在停止...")
    finally:
        worker.stop()
        ser.close()
        try:
            sock.close()
        except:
            pass
        print("已断开连接")

if __name__ == "__main__":
    main()
