"""列出可用串口，不打开设备。"""
from serial.tools.list_ports import comports


def main():
    ports = sorted(comports(), key=lambda port: port.device)
    for port in ports:
        print(f'{port.device}\t{port.description}')
    if not ports:
        print('未找到串口')
    return 0
