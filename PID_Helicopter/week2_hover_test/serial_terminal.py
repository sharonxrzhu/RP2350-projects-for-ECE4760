#!/usr/bin/env python3
"""Line-at-a-time serial terminal for the Pico's UART (via the Debug Probe).

    python3 tools/pico-serial.py                  # first /dev/tty.usbmodem*, 115200
    python3 tools/pico-serial.py /dev/tty.usbserial-XXXX

Type a line and press Enter; it is sent with a carriage return, which is what
the course protothreads serial thread waits for. Ctrl-C to quit.
Works in terminals where `screen` doesn't (e.g. the Claude app's panel).
Everything received is also appended to /tmp/pico-serial.log, so it can be
read back afterwards (or by Claude while you test).
"""
import glob, os, select, sys, termios, threading

ports = sorted(glob.glob("/dev/tty.usbmodem*"))
port = sys.argv[1] if len(sys.argv) > 1 else (ports[0] if ports else None)
if not port:
    sys.exit("No /dev/tty.usbmodem* port: is the Debug Probe plugged in?")
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[0] = a[1] = a[3] = 0
a[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
a[4] = a[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, a)
log = open("/tmp/pico-serial.log", "a", buffering=1)


def reader():
    while True:
        try:
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            data = os.read(fd, 4096)
        except BlockingIOError:
            continue
        except OSError:
            print("\n[Serial connection lost: is the Debug Probe still plugged in?]")
            os._exit(1)
        text = data.decode(errors="replace").replace("\r", "")
        sys.stdout.write(text)
        sys.stdout.flush()
        log.write(text)


threading.Thread(target=reader, daemon=True).start()
print(f"Connected to {port}. Type a command and press Enter. Ctrl-C to quit.")
os.write(fd, b"\r")
try:
    for line in sys.stdin:
        os.write(fd, line.rstrip("\n").encode() + b"\r")
except (KeyboardInterrupt, OSError):
    pass
print("\n[Serial terminal closed]")
os._exit(0)
