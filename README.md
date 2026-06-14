# usb-spi-i2cI2C

A **USB-to-SPI / I2C / GPIO bridge** for the STM32F103C6T6A ("Blue Pill" class).
The board enumerates as a USB CDC virtual serial port; the host sends small
binary command packets and the firmware drives the on-chip **SPI1**, **I2C1**,
and **GPIO** peripherals, returning a status byte plus any data read back.

No drivers needed — it shows up as a standard CDC-ACM serial device on Linux,
macOS, and Windows.

> **Status:** firmware builds clean and has been exercised on hardware against
> an SSD1306 I2C display (see [`tests/`](tests/)). SPI runs at a conservative
> ~281 kbit/s and all transfers are blocking; see [Roadmap](#roadmap).

---

## Table of contents

- [Features](#features)
- [Hardware](#hardware)
- [Protocol](#protocol)
- [Building](#building)
- [Flashing](#flashing)
- [Host usage](#host-usage)
- [Repository layout](#repository-layout)
- [Roadmap](#roadmap)
- [Porting / regenerating](#porting--regenerating)
- [License](#license)

---

## Features

- **USB CDC (virtual COM port)** — no custom host driver required.
- **I2C master** — 7-bit addressing, 100 kHz, read and write.
- **SPI master** — mode 0, MSB-first, software chip-select, send / receive /
  full-duplex.
- **GPIO** — four user output bits (A0–A3).
- **Framed binary protocol** with multi-packet command reassembly, so transfers
  larger than one 64-byte USB packet work transparently.
- Fits the small **32 KB flash / 10 KB RAM** STM32F103C6 (≈71 % flash, ≈87 %
  RAM).

## Hardware

MCU: **STM32F103C6T6A**, LQFP48, 72 MHz (HSE 8 MHz × PLL9), USB clock 48 MHz.

| Function              | Pin(s)              | Notes                                            |
|-----------------------|---------------------|--------------------------------------------------|
| USB D− / D+           | PA11 / PA12         | CDC full-speed device                            |
| SPI1 SCK / MISO / MOSI| PA5 / PA6 / PA7     | Master, full duplex, software NSS                |
| SPI chip-select       | PA4                 | Push-pull output, **active-low CSN**             |
| GPIO outputs A0–A3    | PA0 / PA1 / PA2 / PA3| Push-pull, driven by the `G` command            |
| I2C1 SCL / SDA        | PB6 / PB7           | 100 kHz, 7-bit addressing                        |
| USART1 TX / RX        | PA9 / PA10          | Legacy UART bridge (dormant reference)           |
| SWD                   | PA13 / PA14         | Debug                                            |
| LSE / HSE             | PC14-15 / PD0-1     | Crystals                                         |

I2C and SPI bus pull-ups / wiring are board-dependent. The chip-select (PA4) is
asserted low around each SPI transfer.

## Protocol

Binary packets are exchanged over the CDC endpoint. **Every command has a fixed
4-byte header**; payload (if any) starts at offset 4.

### Host → device

| Offset | I2C (`'I'`)                         | SPI (`'S'`)                              | GPIO (`'G'`)            |
|:------:|-------------------------------------|------------------------------------------|-------------------------|
| 0      | `'I'`                               | `'S'`                                     | `'G'`                   |
| 1      | address `(7-bit << 1) \| R/W`       | mode: 0=send, 1=receive, 2=send+receive  | output bits A0–A3 (low nibble) |
| 2      | data length (0–255)                 | length LSB                               | —                       |
| 3      | reserved                            | length MSB (0–1024)                      | —                       |
| 4…     | data (write only)                   | data (send / send+receive)               | —                       |

- **I2C:** R/W is bit 0 of the address byte (0 = write, 1 = read). The firmware
  passes `addr & 0xFE` to HAL, which owns the direction bit.
- **SPI:** one `'S'` command is one CS-framed transfer (CS low before, high
  after). Max length 1024 bytes.
- **GPIO:** only A0–A3 (PA0–PA3) are driven; PA4 is the SPI CS and is never
  touched by `'G'`.

### Device → host

| Offset | Field                                   |
|:------:|-----------------------------------------|
| 0      | error code (0 = success, 1 = error)     |
| 1…n    | read data (I2C read, SPI receive)       |

The host knows the expected reply length from the command it sent, so no extra
framing is added.

## Host usage

The device appears as a CDC serial port (e.g. `/dev/ttyACM0`) with the USB
product string **`uart-spi`**. A reference host script driving an SSD1306 OLED
over I2C lives in [`tests/ssd1306_i2c.py`](tests/ssd1306_i2c.py):

```sh
pip install pyserial
python3 tests/ssd1306_i2c.py                       # auto-detect the bridge
python3 tests/ssd1306_i2c.py --port /dev/ttyACM0 --addr 0x3C
```

It finds the port by product string, scans the I2C bus, then initialises the
panel and draws test patterns. Minimal example of the protocol in Python:

```python
import serial
ser = serial.Serial("/dev/ttyACM0", timeout=2)

# I2C write: 0x3C, payload [0x00, 0xAF] (SSD1306 "display on")
addr8 = (0x3C << 1) | 0
ser.write(bytes([ord('I'), addr8, 2, 0x00, 0x00, 0xAF]))
assert ser.read(1) == b"\x00"          # error code 0 == success

# GPIO: set A0 and A2 high
ser.write(bytes([ord('G'), 0b0101, 0x00, 0x00]))
ser.read(1)
```

## Repository layout

| Path | Description |
|------|-------------|
| [`Core/Src/main.c`](Core/Src/main.c) | CubeMX entry: clocks and peripheral init |
| [`Core/Src/main2.c`](Core/Src/main2.c) | **Bridge application** — command dispatcher + USB CDC glue |
| [`USB_DEVICE/`](USB_DEVICE/) | USB CDC device class (RX hook in `usbd_cdc_if.c`) |
| [`tests/`](tests/) | Host-side Python tests (SSD1306 over I2C) |
| [`usb-spi-i2c.ioc`](usb-spi-i2c.ioc) | STM32CubeMX project |
| [`Makefile`](Makefile) | Build configuration |
| [`flash.sh`](flash.sh) | Build + flash helper |
| [`CLAUDE.md`](CLAUDE.md) | Detailed developer/architecture notes |

`Drivers/` and `Middlewares/` are generated by CubeMX and git-ignored.

## Roadmap

- [ ] Raise the SPI baud rate (prescaler is currently /256).
- [ ] Optional DMA for large transfers (disabled on this part to save space).
- [ ] Repeated-start / combined I2C register transactions.
- [ ] Retire the dormant USART1 bridge code to reclaim RAM.
- [ ] PC13 LED is toggled in firmware but not yet configured as an output.

## Porting / regenerating

This firmware was ported down from the 128 KB **STM32F103CBT6**. The application
(`main2.c` and the small hand-edits in the generated files) is chip-independent;
only the startup file, linker sizes, `-DSTM32F103x6` define, and the DMA-free
CubeMX config differ. Hand-written code lives in `main2.c` or inside
`/* USER CODE BEGIN/END */` guards so CubeMX regeneration preserves it. See
[`CLAUDE.md`](CLAUDE.md) for the full porting notes.

## License

The STM32 HAL, CMSIS, and USB device library are © STMicroelectronics and used
under their respective ST licenses (see headers in the generated files). The
application code in this repository (`main2.c`, `tests/`) is provided as-is; add
a `LICENSE` file to set explicit terms for your own use.
