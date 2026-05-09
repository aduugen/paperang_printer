"""
HC-06 蓝牙模块配置脚本 (修正版)
HC-06的AT指令格式特殊，不需要换行符，查询不带问号

使用方法：
1. 确保HC-06已通过USB转TTL连接到电脑
2. 确保HC-06处于未配对状态（LED闪烁）
3. 运行此脚本：python hc06_config_v2.py
"""

import serial
import time

COM_PORT = 'COM4'
BAUD_RATE = 115200

def send_at_command(ser, cmd, wait_time=1):
    ser.write(cmd.encode())
    time.sleep(wait_time)
    response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    return response.strip()

def main():
    print("=" * 50)
    print("HC-06 蓝牙模块配置工具 v2")
    print("=" * 50)
    print(f"\n串口: {COM_PORT}")
    print(f"波特率: {BAUD_RATE}")
    print("\n请确保HC-06处于未配对状态（LED闪烁）")
    print("-" * 50)
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=2)
        print(f"\n[OK] 串口 {COM_PORT} 已打开")
    except Exception as e:
        print(f"\n[错误] 无法打开串口: {e}")
        return
    
    time.sleep(1)
    
    print("\n[步骤1] 测试AT通信...")
    response = send_at_command(ser, 'AT')
    print(f"AT响应: {response if response else 'OK'}")
    
    print("\n[步骤2] 查询当前配置...")
    
    response = send_at_command(ser, 'AT+VERSION')
    print(f"版本: {response if response else '无响应'}")
    
    response = send_at_command(ser, 'AT+NAME')
    print(f"名称: {response if response else '无响应'}")
    
    response = send_at_command(ser, 'AT+PIN')
    print(f"配对码: {response if response else '无响应'}")
    
    response = send_at_command(ser, 'AT+BAUD')
    baud_map = {'1': '1200', '2': '2400', '3': '4800', '4': '9600', 
                '5': '19200', '6': '38400', '7': '57600', '8': '115200'}
    baud_name = baud_map.get(response, response)
    print(f"波特率: {baud_name} (代码: {response if response else '无响应'})")
    
    print("\n[步骤3] 配置参数...")
    
    print("\n--- 设置设备名称为 Paperang_P2 ---")
    response = send_at_command(ser, 'AT+NAMEPaperang_P2')
    print(f"响应: {response if response else 'OK'}")
    
    print("\n--- 设置配对码为 0000 ---")
    response = send_at_command(ser, 'AT+PIN0000')
    print(f"响应: {response if response else 'OK'}")
    
    print("\n[步骤4] 验证新配置...")
    
    response = send_at_command(ser, 'AT+NAME')
    print(f"名称: {response if response else '无响应'}")
    
    response = send_at_command(ser, 'AT+PIN')
    print(f"配对码: {response if response else '无响应'}")
    
    ser.close()
    
    print("\n" + "=" * 50)
    print("配置完成！")
    print("=" * 50)
    print("\n下一步：")
    print("  1. 手机蓝牙中删除旧的配对记录")
    print("  2. 关闭手机蓝牙再打开")
    print("  3. 搜索设备 'P2'")
    print("  4. 配对码: 0000")

if __name__ == "__main__":
    main()
