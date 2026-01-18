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

## Protocol (config)
- 38-byte packed struct: 16-byte magic, 4-byte baud (LE, 0 for query), 1-byte TX, 1-byte RX, 16-byte inverted magic.
- Baud 0 = query only (no reconfigure; replies with current config).

## Notes
- Web Serial works in Chromium-based browsers (Chrome, Edge) when served from a file:// origin for this simple use case.
- If you rebuild firmware, rerun `inject_binaries.py` to refresh embedded images.
- The flasher disconnects after flashing to free the port for the app/config step.
