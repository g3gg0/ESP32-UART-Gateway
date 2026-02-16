# ESP32-C3 UART Gateway Toolbox

This project turns an ESP32-C3 into a combined USB-CDC ↔ UART gateway and web-based flasher/configurator. It replaces the pile of USB/UART dongles with a single ESP32 you likely already have.

## Why
- I was tired of juggling USB/UART adapters; I own more ESP32s than USB UARTs.
- A ESP32-C3 can bridge USB to UART for programming other devices via UART or other protocols and expose a WebSerial UI for configuration.

## Features
- Web flasher: flashes bundled images (bootloader, partition table, app) directly from the browser using Web Serial.
- WebSerial config UI: query and set UART gateway config (baud + pins).
- UART gateway firmware supports multiple operating modes (simple bridge, extended packet mode, SWD tunneling).
- Built-in images: defaults embed build outputs (bootloader.bin, partition-table.bin, ESP32C3_UART.bin).

## Web tools included
- `flasher.html`: flash and configure the ESP32-C3 (via ROM bootloader protocol) using the embedded binaries.
- `swd.html`: SWD debug console over the UART gateway (memory view/editor + basic core debug UI).
- `cc3200.html`: CC3200 console UI (Experimental interface for accessing CC3200 via UART).

Static/self-contained variants (handy for offline use/hosting without external JS):
- `flasher.static.html`
- `swd.static.html`
- `cc3200.static.html`

## Operating modes
The firmware has three modes on the USB-CDC port:

1) **Simple USB UART bridge (default)**
- Raw USB-CDC bytes are bridged to the target UART and back.
- No framing/packet protocol is required.
- In this mode the firmware watches the incoming stream for the extended-mode activation magic.
- Replaces USB-UARTs, except RTS/DTR

2) **Extended mode (packet protocol + GPIO control)**
- After activation, all traffic is framed as `{length,type}+payload` packets, EspSerial.js as helper.
- Enables config read/write, control commands (BREAK + GPIO), logs, and SWD tunneling.

3) **SWD mode (tunneled over extended mode)**
- SWD commands/responses are carried inside extended-mode packets of type `SWD`.
- This is what `swd.html` uses.

## Building
1. Build firmware (generates `build/bootloader/bootloader.bin`, `build/partition_table/partition-table.bin`, `build/ESP32C3_UART.bin`).
2. Embed binaries into the web tool (optional if already embedded):
   ```
   python inject_binaries.py
   ```
3. Open `flasher.html` in a Chromium-based browser (Web Serial required).

## Using the web tools
1. Flash the firmware: open `flasher.html` and click "Connect & Flash".
2. Configure the running gateway: open `webserial_config.html` and connect; it can query and set the config.
3. SWD debugging: open `swd.html` and connect; it will switch the device into extended mode and use the SWD packet type.

## Hardware
- ESP32-C3 with native USB (USB-CDC).
- UART lines from the C3 to the target ESP32's RX/TX (crossed), and common GND.
- For convenience, all other GPIO are set to GND and can be used as UART GND.

## Protocol (basic)
**Extended mode activation**
- Send 12-byte magic packet (header+payload):
   - Header: length=0x000C, type=0x000A (both little-endian)
   - Payload: ASCII "UARTGWEX" (8 bytes)

**Packet format (extended mode)**
- 4-byte header: `uint16_t length` + `uint16_t type`, little-endian.
- `length` includes header + payload (minimum 4).
- Types: 0x00=DATA, 0x01=CONFIG, 0x02=CONTROL, 0x03=LOG, 0x04=SWD, 0x0A=EXTMODE.

**Send/receive serial data**
- Host → device: wrap raw UART bytes in a DATA packet (type 0x00).
- Device → host: UART bytes are emitted as DATA packets (type 0x00).

**CONTROL: BREAK + GPIO**
- CONTROL payload is a 16-byte ASCII command (null/zero padded).
- Commands:
   - `B:<ms>`  — drive TX low for `<ms>` to generate BREAK.
   - `R:0`/`R:1` — deassert/assert RESET GPIO (if configured).
   - `C:0`/`C:1` — deassert/assert CONTROL GPIO (if configured).

**CONFIG (12-byte payload)**
- `baud_rate` (u32 LE), `tx_gpio`, `rx_gpio`, `reset_gpio`, `control_gpio`, `led_gpio`, padding(2), `extended_mode`.
- `baud_rate=0` is query-only; device replies with current config.

## Notes
- Web Serial works in Chromium-based browsers (Chrome, Edge) when served from a file:// origin for this simple use case.
- If you rebuild firmware, rerun `inject_binaries.py` to refresh embedded images.
- The flasher disconnects after flashing to free the port for the app/config step.
