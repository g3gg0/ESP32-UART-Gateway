# ESP32-C3 UART Gateway Toolbox

This project turns an ESP32-C3 into a combined USB-CDC ↔ UART gateway and web-based flasher/configurator. It replaces the pile of USB/UART dongles with a single ESP32 you likely already have.

## Why
- I was tired of juggling USB/UART adapters; I own more ESP32s than USB UARTs.
- One ESP32-C3 can bridge USB to UART for programming other ESP32s and expose a WebSerial UI for configuration.

## Features
- Web flasher: flashes bundled images (bootloader, partition table, app) directly from the browser using Web Serial.
- Web config: query and set UART gateway config (baud + TX/RX pins) via 38-byte magic packet protocol.
- Built-in images: defaults embed build outputs (bootloader.bin, partition-table.bin, ESP32C3_UART.bin).
- Single page toolbox: flasher and config live in one HTML with tabs.

## Building
1. Build firmware (generates `build/bootloader/bootloader.bin`, `build/partition_table/partition-table.bin`, `build/ESP32C3_UART.bin`).
2. Embed binaries into the web tool (optional if already embedded):
   ```
   python inject_binaries.py
   ```
3. Open `flasher.html` in a Chromium-based browser (Web Serial required).

## Using the web toolbox
1. Open `flasher.html`.
2. Flasher tab: click "Connect & Flash" and select the ESP32-C3 target in bootloader mode; flashing uses embedded images.
3. After flashing, the tool resets to the application; disconnect happens automatically after a short delay.
4. Config tab: connect to the running gateway, it auto-queries current config; adjust baud/TX/RX and "Send Config".

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
- Types: 0x00=DATA, 0x01=CONFIG, 0x02=CONTROL, 0x03=LOG.

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
