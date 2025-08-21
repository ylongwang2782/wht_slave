import tkinter as tk
from tkinter import scrolledtext
import serial
import serial.tools.list_ports
import binascii
import threading
import struct


def crc16_modbus(data):
    """计算CRC16-MODBUS校验"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(source_addr, target_addr, msg_id, payload_hex):
    """
    构建完整的协议帧
    Args:
        source_addr: 源地址 (u8)
        target_addr: 目标地址 (u8)
        msg_id: 消息ID (u8)
        payload_hex: payload数据的十六进制字符串，如 "01 02 00 20 01"
    Returns:
        完整帧的十六进制字符串
    """
    # 解析payload
    if payload_hex.strip():
        payload_bytes = bytes.fromhex(payload_hex.replace(" ", ""))
    else:
        payload_bytes = b""

    payload_length = len(payload_bytes)

    # 构建MSG字段 (source + target + msg_id + payload_length + payload)
    msg_data = (
        struct.pack("<BBBh", source_addr, target_addr, msg_id, payload_length)
        + payload_bytes
    )

    # 计算CRC16
    crc = crc16_modbus(msg_data)

    # 构建完整帧: SOF + MSG + CRC16 + EOF
    frame = b"\x55\xaa" + msg_data + struct.pack("<H", crc) + b"\xbb\x66"

    # 转换为十六进制字符串格式
    hex_str = binascii.hexlify(frame).decode().upper()
    formatted = " ".join([hex_str[i : i + 2] for i in range(0, len(hex_str), 2)])

    return formatted


# 示例字典 - 现在只需要填写payload数据
examples = {
    "General IO - 输入模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x10,
        "payload": "01 02 00 20 00",  # Sub-ID=0x01(设置引脚模式), Port=0x02, Pin Mask=0x0020, Value=0x00(输入)
        "expect": "",
    },
    "General IO - 输出模式": {
        "source": 0x01,  # 测试工装
        "target": 0x02,  # 被测底板
        "msg_id": 0x10,  # General IO Control
        "payload": "01 02 00 20 01",  # Sub-ID=0x01(设置引脚模式), Port=0x02, Pin Mask=0x0020, Value=0x01(输出)
        "expect": "",
    },
    "General IO - 模拟模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x10,
        "payload": "01 02 00 20 02",  # Sub-ID=0x01(设置引脚模式), Port=0x02, Pin Mask=0x0020, Value=0x02(模拟)
        "expect": "",
    },
    "General IO - 下拉模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x10,
        "payload": "02 02 00 20 00",  # Sub-ID=0x02(配置上下拉), Port=0x02, Pin Mask=0x0020, Value=0x00(下拉)
        "expect": "",
    },
    "General IO - 上拉模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x10,
        "payload": "02 02 00 20 01",  # Sub-ID=0x02(配置上下拉), Port=0x02, Pin Mask=0x0020, Value=0x01(上拉)
        "expect": "",
    },
    "General IO - 浮空模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x10,
        "payload": "02 02 00 20 02",  # Sub-ID=0x02(配置上下拉), Port=0x02, Pin Mask=0x0020, Value=0x01(上拉)
        "expect": "",
    },
    "General IO - 低电平": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x10,
        "payload": "03 02 00 20 00",  # Sub-ID=0x03(写输出电平), Port=0x02, Pin Mask=0x0020, Value=0x00(低电平)
        "expect": "",
    },
    "General IO - 高电平": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x10,
        "payload": "03 02 00 20 01",  # Sub-ID=0x03(写输出电平), Port=0x02, Pin Mask=0x0020, Value=0x01(高电平)
        "expect": "",
    },
    "General IO - 读取电平": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x10,
        "payload": "04 02",  # Sub-ID=0x04(读输入电平), Port=0x02
        "expect": "",
    },
    "64-Way IO - 输入模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "01 FF 00 00 00 00 00 00 00 00",
        "expect": "",
    },
    "64-Way IO - 输出模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "01 FF 00 00 00 00 00 00 00 01",
        "expect": "",
    },
    "64-Way IO - 模拟模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "01 FF 00 00 00 00 00 00 00 02",
        "expect": "",
    },
    "64-Way IO - 下拉模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "02 FF 00 00 00 00 00 00 00 00",
        "expect": "",
    },
    "64-Way IO - 上拉模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "02 FF 00 00 00 00 00 00 00 01",
        "expect": "",
    },
    "64-Way IO - 浮空模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "02 FF 00 00 00 00 00 00 00 02",
        "expect": "",
    },
    "64-Way IO - 低电平": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "03 FF 00 00 00 00 00 00 00 00",
        "expect": "",
    },
    "64-Way IO - 高电平": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "03 FF 00 00 00 00 00 00 00 01",
        "expect": "",
    },
    "64-Way IO - 读取电平": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x11,
        "payload": "04",
        "expect": "",
    },
    "DipSwitch - 读取电平": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x12,
        "payload": "04",
        "expect": "",
    },
    "进入测试模式": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x21,
        "payload": "",  # 无payload
        "expect": "",
    },
    "心跳查询": {
        "source": 0x01,
        "target": 0x02,
        "msg_id": 0x22,
        "payload": "",  # 无payload
        "expect": "",
    },
}

# 串口对象
ser = None
running = False


def open_serial(port, baudrate=115200):
    global ser, running
    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
        running = True
        threading.Thread(target=read_serial, daemon=True).start()
        log(f"串口已打开: {port}")
    except Exception as e:
        log(f"打开串口失败: {e}")


def close_serial():
    global ser, running
    running = False
    if ser and ser.is_open:
        ser.close()
        log("串口已关闭")


def read_serial():
    global ser, running
    while running and ser and ser.is_open:
        try:
            data = ser.read(1024)
            if data:
                hex_str = binascii.hexlify(data).decode().upper()
                formatted = " ".join(
                    [hex_str[i : i + 2] for i in range(0, len(hex_str), 2)]
                )
                log(f"接收: {formatted}")
        except Exception as e:
            log(f"串口读取错误: {e}")
            break


def send_example(name):
    global ser
    if not ser or not ser.is_open:
        log("请先打开串口")
        return

    example = examples[name]

    # 使用新的帧构建函数生成完整帧
    frame_hex = build_frame(
        example["source"], example["target"], example["msg_id"], example["payload"]
    )

    send_bytes = bytes.fromhex(frame_hex.replace(" ", ""))
    ser.write(send_bytes)

    log(f"发送指令: {name}")
    log(f"Payload: {example['payload']}")
    log(f"完整帧: {frame_hex}")
    log(f"期望: {example['expect']}")


def log(msg):
    text_area.insert(tk.END, msg + "\n")
    text_area.see(tk.END)


# GUI
root = tk.Tk()
root.title("串口验证小工具")

# 串口选择
frame_top = tk.Frame(root)
frame_top.pack(pady=5)

ports = [p.device for p in serial.tools.list_ports.comports()]
port_var = tk.StringVar(value=ports[0] if ports else "")

tk.Label(frame_top, text="串口:").pack(side=tk.LEFT)
port_menu = tk.OptionMenu(frame_top, port_var, *ports)
port_menu.pack(side=tk.LEFT)

tk.Button(frame_top, text="打开串口", command=lambda: open_serial(port_var.get())).pack(
    side=tk.LEFT, padx=5
)
tk.Button(frame_top, text="关闭串口", command=close_serial).pack(side=tk.LEFT, padx=5)

# 指令按钮区
frame_buttons = tk.Frame(root)
frame_buttons.pack(pady=5)

# 每行显示 4 个按钮
cols = 9
for i, name in enumerate(examples):
    row = i // cols
    col = i % cols
    tk.Button(
        frame_buttons,
        text=name,
        width=25,  # 按钮宽度可调
        command=lambda n=name: send_example(n),
    ).grid(row=row, column=col, padx=5, pady=5)

# 输出框
text_area = scrolledtext.ScrolledText(root, width=100, height=30)
text_area.pack(pady=5)

root.mainloop()
