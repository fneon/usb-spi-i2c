#!/usr/bin/env python3
"""
Exercise the usb-spi-i2c bridge over I2C against an SSD1306 OLED controller.

The firmware (Core/Src/main2.c) enumerates as a USB CDC serial device with the
product string "uart-spi". This script:

  1. Finds that serial port by its USB product string.
  2. Scans the I2C bus and confirms the SSD1306 is present (default 0x3C).
  3. Initialises the panel and draws a couple of test patterns.

Bridge protocol (host -> device), fixed 4-byte header, payload from offset 4:
    'I', addr8, length, 0x00, <data...>
where addr8 = (7-bit addr << 1) | R/W (bit0: 0=write, 1=read), length 0..255.
Device replies: [error_code][read data...]  (error 0 = OK, 1 = failure/NACK).

Usage:
    python3 ssd1306_i2c.py [--port /dev/ttyACMx] [--addr 0x3C] [--retries N]

Requires pyserial:  pip install pyserial
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

PRODUCT_MATCH = "uart-spi"   # USB product string of the bridge firmware
SSD1306_ADDR = 0x3C          # default 7-bit address (0x3D if SA0 tied high)
RESP_TIMEOUT = 2.0           # s; firmware uses 1 s blocking HAL transfers

# SSD1306 control bytes (first payload byte after the I2C address)
CTRL_CMD = 0x00              # Co=0, D/C#=0 -> the rest of the stream is commands
CTRL_DATA = 0x40             # Co=0, D/C#=1 -> the rest of the stream is GDDRAM data


# --------------------------------------------------------------------------- #
# Port discovery
# --------------------------------------------------------------------------- #
def find_port(match=PRODUCT_MATCH):
    """Return the device path of the first port whose USB strings match."""
    for p in list_ports.comports():
        for field in (p.product, p.description, p.manufacturer):
            if field and match.lower() in field.lower():
                return p.device
    return None


def list_all_ports():
    return [
        f"  {p.device}: product={p.product!r} desc={p.description!r} "
        f"mfr={p.manufacturer!r}"
        for p in list_ports.comports()
    ]


# --------------------------------------------------------------------------- #
# Bridge transport
# --------------------------------------------------------------------------- #
class Bridge:
    def __init__(self, port, timeout=RESP_TIMEOUT):
        self.ser = serial.Serial(port, baudrate=115200, timeout=timeout)
        time.sleep(0.1)                 # let the CDC port settle
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    def _read_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                raise TimeoutError(
                    f"bridge response timeout (got {len(buf)}/{n} bytes)"
                )
            buf += chunk
        return bytes(buf)

    def i2c_write(self, addr7, data):
        """Write `data` (<=255 bytes) to a 7-bit address. True on ACK."""
        data = bytes(data)
        if len(data) > 255:
            raise ValueError("I2C write length max 255")
        addr8 = (addr7 << 1) | 0
        frame = bytes([ord("I"), addr8, len(data), 0x00]) + data
        self.ser.write(frame)
        return self._read_exact(1)[0] == 0

    def i2c_read(self, addr7, length):
        """Read `length` (<=255) bytes from a 7-bit address. None on NACK."""
        if length > 255:
            raise ValueError("I2C read length max 255")
        addr8 = (addr7 << 1) | 1
        frame = bytes([ord("I"), addr8, length, 0x00])
        self.ser.write(frame)
        resp = self._read_exact(1 + length)
        return resp[1:] if resp[0] == 0 else None

    def i2c_probe(self, addr7):
        """Address-only (zero-length) write: ACK => device present."""
        return self.i2c_write(addr7, b"")

    def i2c_scan(self, start=0x08, end=0x77):
        return [a for a in range(start, end + 1) if self.i2c_probe(a)]


# --------------------------------------------------------------------------- #
# SSD1306 driver (just enough to prove I2C access works)
# --------------------------------------------------------------------------- #
class SSD1306:
    WIDTH = 128
    HEIGHT = 64
    PAGES = HEIGHT // 8
    FB_SIZE = WIDTH * PAGES        # 1024 bytes

    def __init__(self, bridge, addr7=SSD1306_ADDR):
        self.b = bridge
        self.addr = addr7

    def cmd(self, *cmds):
        if not self.b.i2c_write(self.addr, bytes([CTRL_CMD]) + bytes(cmds)):
            raise IOError("SSD1306 command write NACKed")

    def init(self):
        for c in (
            (0xAE,),            # display off
            (0xD5, 0x80),       # display clock divide
            (0xA8, 0x3F),       # multiplex ratio 1/64
            (0xD3, 0x00),       # display offset 0
            (0x40,),            # start line 0
            (0x8D, 0x14),       # charge pump on
            (0x20, 0x00),       # memory addressing mode: horizontal
            (0xA1,),            # segment remap (column 127 -> SEG0)
            (0xC8,),            # COM scan direction remapped
            (0xDA, 0x12),       # COM pins config
            (0x81, 0xCF),       # contrast
            (0xD9, 0xF1),       # pre-charge period
            (0xDB, 0x40),       # VCOMH deselect level
            (0xA4,),            # output follows RAM
            (0xA6,),            # normal (non-inverted)
            (0xAF,),            # display on
        ):
            self.cmd(*c)

    def _set_window(self):
        self.cmd(0x21, 0x00, self.WIDTH - 1)   # column address range
        self.cmd(0x22, 0x00, self.PAGES - 1)   # page address range

    def blit(self, framebuffer):
        """Push a full WIDTH*PAGES framebuffer to GDDRAM."""
        framebuffer = bytes(framebuffer)
        if len(framebuffer) != self.FB_SIZE:
            raise ValueError(f"framebuffer must be {self.FB_SIZE} bytes")
        self._set_window()
        # I2C length is one byte, so chunk; 128 data + 1 control byte = 129.
        chunk = 128
        for i in range(0, len(framebuffer), chunk):
            piece = framebuffer[i:i + chunk]
            if not self.b.i2c_write(self.addr, bytes([CTRL_DATA]) + piece):
                raise IOError("SSD1306 data write NACKed")

    def fill(self, value=0x00):
        self.blit(bytes([value]) * self.FB_SIZE)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def open_bridge(port, retries):
    last = None
    for attempt in range(1, retries + 1):
        dev = port or find_port()
        if dev:
            try:
                return Bridge(dev), dev
            except serial.SerialException as e:
                last = e
        else:
            last = RuntimeError(
                f'no serial device with product string containing '
                f'"{PRODUCT_MATCH}"'
            )
        if attempt < retries:
            print(f"  attempt {attempt} failed ({last}); retrying...",
                  file=sys.stderr)
            time.sleep(1.0)
    raise last


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device (skip product-string search)")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=SSD1306_ADDR,
                    help="SSD1306 7-bit address (default 0x3C)")
    ap.add_argument("--retries", type=int, default=3,
                    help="connection/discovery attempts (default 3)")
    args = ap.parse_args()

    try:
        bridge, dev = open_bridge(args.port, args.retries)
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        print("available ports:", file=sys.stderr)
        for line in list_all_ports() or ["  (none)"]:
            print(line, file=sys.stderr)
        return 1

    print(f"bridge: {dev}")
    try:
        print("scanning I2C bus...")
        found = bridge.i2c_scan()
        print("  devices:", ", ".join(hex(a) for a in found) or "none")

        if args.addr not in found:
            print(f"warning: SSD1306 not detected at {hex(args.addr)}; "
                  "trying anyway", file=sys.stderr)

        disp = SSD1306(bridge, args.addr)
        print("initialising SSD1306...")
        disp.init()

        print("clearing screen...")
        disp.fill(0x00)
        time.sleep(0.3)

        print("drawing vertical stripes...")
        disp.blit(bytes([0xFF if (i % 2 == 0) else 0x00
                        for i in range(SSD1306.FB_SIZE)]))
        time.sleep(0.5)

        print("drawing checkerboard...")
        disp.blit(bytes([0xAA if ((i // SSD1306.WIDTH) % 2 == 0) else 0x55
                        for i in range(SSD1306.FB_SIZE)]))

        print("done.")
        return 0
    except (TimeoutError, IOError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    finally:
        bridge.close()


if __name__ == "__main__":
    sys.exit(main())
