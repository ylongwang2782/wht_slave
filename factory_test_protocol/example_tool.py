import tkinter as tk
from tkinter import scrolledtext
import serial
import serial.tools.list_ports
import binascii
import threading

# 示例字典
examples = {
    "General IO - SubID 0x01 设置引脚模式": {
        "send": "55 AA 01 02 10 05 00 01 01 F0 00 01 91 61 BB 66",
        "expect": "55 AA 02 01 10 02 00 01 00 F8 FA BB 66",
    },
    "General IO - SubID 0x02 配置上下拉": {
        "send": "55 AA 01 02 10 05 00 02 01 03 00 01 25 52 BB 66",
        "expect": "55 AA 02 01 10 02 00 02 00 F8 0A BB 66",
    },
    "General IO - SubID 0x03 写输出电平": {
        "send": "55 AA 01 02 10 05 00 03 01 F0 00 00 29 61 BB 66",
        "expect": "55 AA 02 01 10 02 00 03 00 F9 9A BB 66",
    },
    "General IO - SubID 0x04 读输入电平": {
        "send": "55 AA 01 02 10 02 00 04 01 09 59 BB 66",
        "expect": "55 AA 02 01 10 04 00 04 01 F3 00 D6 19 BB 66",
    },
    "64-Way IO - SubID 0x01 设置引脚模式": {
        "send": "55 AA 01 02 11 0A 00 01 FF 00 00 00 00 00 00 00 01 73 85 BB 66",
        "expect": "55 AA 02 01 11 02 00 01 00 C5 3A BB 66",
    },
    "Dip Switch - SubID 0x04 读输入电平": {
        "send": "55 AA 01 02 12 01 00 04 2D 71 BB 66",
        "expect": "55 AA 02 01 12 03 00 04 00 A5 17 DA BB 66",
    },
    "Additional Test - SubID 0x01 串口回环自测": {
        "send": "55 AA 01 02 20 01 00 01 E3 CA BB 66",
        "expect": "55 AA 02 01 20 02 00 01 00 B8 FE BB 66",
    },
    "Enter Test Mode": {
        "send": "55 AA 01 02 21 00 00 48 72 BB 66",
        "expect": "55 AA 02 01 21 01 00 00 67 C5 BB 66",
    },
    "Heartbeat": {
        "send": "55 AA 01 02 22 00 00 B8 72 BB 66",
        "expect": "55 AA 02 01 22 01 00 00 67 81 BB 66",
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
    send_bytes = bytes.fromhex(example["send"])
    ser.write(send_bytes)
    log(f"发送: {example['send']}")
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
cols = 4
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
