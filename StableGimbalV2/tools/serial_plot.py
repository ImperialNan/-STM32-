"""
serial_plot.py — JY901S real-time serial data plotter
  - Click buttons to control port connection
  - Scroll mouse wheel over a subplot to zoom its Y-axis
  - Right-click a subplot to reset its Y-axis to default
  - X-axis auto-scrolls; Y-axis stays fixed unless zoomed
"""

import sys
import re
import time
import threading
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# ====================== Config ======================
BAUD = 115200
MAX_POINTS = 200
INTERVAL = 50   # ms

# Default Y-axis ranges per channel
DEFAULT_YLIM = {
    "pitch": (-90, 90),
    "roll":  (-180, 180),
    "yaw":   (-180, 180),
    "wx":    (-500, 500),
    "wy":    (-500, 500),
    "wz":    (-500, 500),
}

# ====================== Global State ======================
portStatus = "not connected"
serialObj = None
serialLock = threading.Lock()

times = deque(maxlen=MAX_POINTS)
dataBuf = {
    "pitch": deque(maxlen=MAX_POINTS),
    "roll":  deque(maxlen=MAX_POINTS),
    "yaw":   deque(maxlen=MAX_POINTS),
    "wx":    deque(maxlen=MAX_POINTS),
    "wy":    deque(maxlen=MAX_POINTS),
    "wz":    deque(maxlen=MAX_POINTS),
}
dataLock = threading.Lock()
t0 = time.time()

PATTERN = re.compile(
    r"A:\s+P=\s*([-\d.]+)\s+R=\s*([-\d.]+)\s+Y=\s*([-\d.]+)\s*\|\s*"
    r"G:\s+Wx=\s*([-\d.]+)\s+Wy=\s*([-\d.]+)\s+Wz=\s*([-\d.]+)"
)

# ====================== Port Scanning ======================
def scanPorts():
    ports = serial.tools.list_ports.comports()
    usbPorts, otherPorts = [], []
    for p in ports:
        desc = p.description[:50] if p.description else ""
        item = (p.device, desc)
        isUsb = any(kw in desc.upper() for kw in
                    ("USB", "CH340", "CP210", "FTDI", "STLINK", "SERIAL", "UART"))
        (usbPorts if isUsb else otherPorts).append(item)
    return usbPorts + otherPorts


def checkPortAvailable(portName):
    try:
        s = serial.Serial(portName, BAUD, timeout=0.1)
        s.close()
        return True, ""
    except (serial.SerialException, PermissionError, OSError):
        return False, "OCCUPIED"


# ====================== Serial Connection ======================
def connectPort(portName):
    global serialObj, portStatus
    with serialLock:
        if serialObj is not None and serialObj.is_open:
            try:
                serialObj.close()
            except Exception:
                pass
            serialObj = None

        ok, msg = checkPortAvailable(portName)
        if not ok:
            portStatus = f"{portName}: {msg}"
            print(f"[ERROR] {portStatus}")
            return False

        try:
            serialObj = serial.Serial(portName, BAUD, timeout=1)
        except (serial.SerialException, PermissionError, OSError):
            portStatus = f"{portName}: open failed"
            print(f"[ERROR] {portStatus}")
            return False

        portStatus = f"CONNECTED -> {portName} @ {BAUD}"
        print(f"[INFO] {portStatus}")
        return True


def disconnectPort():
    global serialObj, portStatus
    with serialLock:
        if serialObj is not None and serialObj.is_open:
            serialObj.close()
            serialObj = None
    portStatus = "DISCONNECTED"
    print("[INFO] Disconnected")


# ====================== Serial Reader Thread ======================
def serialReader():
    global serialObj, portStatus
    while True:
        with serialLock:
            ser = serialObj
        if ser is None or not ser.is_open:
            time.sleep(0.3)
            continue
        try:
            raw = ser.readline()
        except serial.SerialException:
            portStatus = "serial error, retry in 5s..."
            time.sleep(5)
            continue
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        m = PATTERN.search(line)
        if not m:
            continue
        try:
            vals = [float(m.group(i)) for i in range(1, 7)]
        except ValueError:
            continue

        with dataLock:
            times.append(time.time() - t0)
            dataBuf["pitch"].append(vals[0])
            dataBuf["roll"].append(vals[1])
            dataBuf["yaw"].append(vals[2])
            dataBuf["wx"].append(vals[3])
            dataBuf["wy"].append(vals[4])
            dataBuf["wz"].append(vals[5])


# ====================== GUI Setup ======================
fig, axes = plt.subplots(2, 3, figsize=(14, 8))
fig.subplots_adjust(bottom=0.12)

portList = scanPorts()
portNames = [p[0] for p in portList]
portIdx = [0]

if not portNames:
    print("[FATAL] No COM ports! Connect USB-serial device and restart.")
    sys.exit(1)

# ---- Status bar ----
statusText = fig.text(
    0.01, 0.985, f"Available: {', '.join(portNames)} | {portStatus}",
    ha="left", va="top", fontsize=10, color="black",
    transform=fig.transFigure, fontweight="bold"
)

channels = [
    (axes[0, 0], "pitch", "Pitch (deg)"),
    (axes[0, 1], "roll",  "Roll (deg)"),
    (axes[0, 2], "yaw",   "Yaw (deg)"),
    (axes[1, 0], "wx",    "Wx (deg/s)"),
    (axes[1, 1], "wy",    "Wy (deg/s)"),
    (axes[1, 2], "wz",    "Wz (deg/s)"),
]

lines = {}
channelMap = {}  # key -> (ax, default_ylim)
for ax, key, ylabel in channels:
    ax.set_ylabel(ylabel)
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    yLo, yHi = DEFAULT_YLIM[key]
    ax.set_ylim(yLo, yHi)
    channelMap[key] = (ax, (yLo, yHi))
    (line,) = ax.plot([], [], linewidth=0.8)
    lines[key] = line

# ---- Mouse-wheel zoom on Y-axis ----
def onScroll(event):
    """Scroll wheel: zoom Y-axis of the subplot under cursor."""
    if event.inaxes is None:
        return  # cursor not over any subplot

    ax = event.inaxes
    yLo, yHi = ax.get_ylim()
    yCenter = (yLo + yHi) / 2.0
    yHalf = (yHi - yLo) / 2.0

    # Zoom factor: scroll up = zoom in (0.85), scroll down = zoom out (1.15)
    factor = 0.85 if event.button == "up" else 1.15
    yHalf *= factor

    yHalf = max(yHalf, 0.01)  # prevent zero range
    ax.set_ylim(yCenter - yHalf, yCenter + yHalf)
    fig.canvas.draw_idle()


def onRightClick(event):
    """Right-click: reset Y-axis to default range for that subplot."""
    if event.inaxes is None:
        return
    # Find which channel this axes belongs to
    for key, (ax, (yLo, yHi)) in channelMap.items():
        if ax is event.inaxes:
            ax.set_ylim(yLo, yHi)
            fig.canvas.draw_idle()
            return


fig.canvas.mpl_connect("scroll_event", onScroll)
fig.canvas.mpl_connect("button_press_event",
                        lambda e: onRightClick(e) if e.button == 3 else None)


# ---- Buttons ----
btnY = 0.02
btnH = 0.04
btnW = 0.08
gap = 0.01

portLabel = fig.text(
    0.40, btnY + btnH + 0.005,
    f"Port: {portNames[portIdx[0]]}",
    fontsize=11, fontweight="bold", color="#1565C0",
    transform=fig.transFigure, ha="center"
)

axPrev = fig.add_axes([0.46, btnY, btnW, btnH])
btnPrev = Button(axPrev, "< Prev", color="#E3F2FD", hovercolor="#90CAF9")
btnPrev.label.set_fontsize(9)

axNext = fig.add_axes([0.46 + btnW + gap, btnY, btnW, btnH])
btnNext = Button(axNext, "Next >", color="#E3F2FD", hovercolor="#90CAF9")
btnNext.label.set_fontsize(9)

axConnect = fig.add_axes([0.46 + 2*btnW + 2*gap + 0.02, btnY, 0.14, btnH])
btnConnect = Button(axConnect, "CONNECT", color="#4CAF50", hovercolor="#66BB6A")
btnConnect.label.set_color("white")
btnConnect.label.set_fontsize(10)

axDisconn = fig.add_axes([0.46 + 2*btnW + 0.14 + 3*gap + 0.03, btnY, 0.13, btnH])
btnDisconn = Button(axDisconn, "DISCONNECT", color="#f44336", hovercolor="#EF5350")
btnDisconn.label.set_color("white")
btnDisconn.label.set_fontsize(10)


def refreshUi():
    portLabel.set_text(f"Port: {portNames[portIdx[0]]}")
    statusText.set_text(
        f"Available: {', '.join(portNames)}   |   {portStatus}"
    )
    fig.canvas.draw_idle()


def onPrev(_event):
    portIdx[0] = (portIdx[0] - 1) % len(portNames)
    refreshUi()

def onNext(_event):
    portIdx[0] = (portIdx[0] + 1) % len(portNames)
    refreshUi()

def onConnect(_event):
    connectPort(portNames[portIdx[0]])
    refreshUi()

def onDisconn(_event):
    disconnectPort()
    refreshUi()

btnPrev.on_clicked(onPrev)
btnNext.on_clicked(onNext)
btnConnect.on_clicked(onConnect)
btnDisconn.on_clicked(onDisconn)


# ====================== Animation ======================
def animate(_frame):
    statusText.set_text(
        f"Available: {', '.join(portNames)}   |   {portStatus}"
    )
    with dataLock:
        if len(times) == 0:
            return list(lines.values())

        t = list(times)
        for key in dataBuf:
            lines[key].set_data(t, list(dataBuf[key]))

        # X-axis auto-scrolls
        xMin, xMax = t[0], max(t[-1], 1.0)
        for ax, _, _ in channels:
            ax.set_xlim(xMin, xMax)

        # Y-axis: user-controlled — no auto-scale! Only scale X.
    return list(lines.values())


# ====================== Start ======================
firstPort = portNames[0]
ok, _ = checkPortAvailable(firstPort)
if ok:
    connectPort(firstPort)
else:
    portStatus = f"{firstPort}: occupied"
refreshUi()

threading.Thread(target=serialReader, daemon=True).start()

print("[INFO] GUI ready  |  Scroll = zoom Y-axis  |  Right-click = reset Y-axis")

timer = fig.canvas.new_timer(interval=INTERVAL)
timer.add_callback(lambda: (animate(None), fig.canvas.draw_idle()))
timer.start()

plt.show()
