"""串口抓日志小工具：打开 COM 口打印若干秒，用来看开机自检 / 心跳。

用法:
    python tools/serial_dump.py            # 抓 COM9 12 秒
    python tools/serial_dump.py COM9 20    # 指定端口与时长
依赖: pip install pyserial
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

print(f"打开 {port} @115200，抓 {secs:.0f} 秒（Ctrl+C 提前结束）…")
try:
    ser = serial.Serial(port, 115200, timeout=0.2)
except Exception as e:  # noqa: BLE001
    print(f"打开失败: {e}")
    sys.exit(1)

# 注意: 这里绝对不要动 DTR/RTS——原生 USB 上那会直接把芯片复位,
# 抓到的就是开机日志而不是当前运行状态。
end = time.time() + secs
try:
    while time.time() < end:
        line = ser.readline()
        if line:
            print(line.decode(errors="ignore").rstrip())
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("--- 结束 ---")
