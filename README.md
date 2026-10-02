# ESP32-C3 UART Gateway Toolbox

This project turns an ESP32-C3 into a combined USB-CDC ↔ UART gateway and web-based flasher/configurator. It replaces the pile of USB/UART dongles with a single ESP32 you likely already have.

## Why
- I was tired of juggling USB/UART adapters; I own more ESP32s than USB UARTs.
- A ESP32-C3 can bridge USB to UART for programming other devices via UART or other protocols and expose a WebSerial UI for configuration.

## Features
- Web flasher: flashes bundled images (bootloader, partition table, app) directly from the browser using Web Serial.
- WebSerial config UI: query and set UART gateway config (baud + pins).
- UART gateway firmware supports multiple operating modes (simple bridge, extended packet mode, SWD tunneling).
- CAN receive mode via native ESP32-C3 TWAI hardware with configurable RX/TX GPIO.
- Built-in images: defaults embed build outputs (bootloader.bin, partition-table.bin, ESP32C3_UART.bin).

## Web tools

[Open the toolbox](https://g3gg0.github.io/ESP32-UART-Gateway/)

- [Flasher](https://g3gg0.github.io/ESP32-UART-Gateway/flasher.html): flash firmware and configure the gateway.
- [Multi-protocol console](https://g3gg0.github.io/ESP32-UART-Gateway/multiprotocol.html): SWD debugging, CAN, UART and Canon EF.
- [CC3200 console](https://g3gg0.github.io/ESP32-UART-Gateway/cc3200.html): CC3200 storage and SFFS tools.

The published tools are self-contained HTML files. The standalone SWD page is
replaced by the SWD tab in the multi-protocol console.

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
- This is what the SWD tab in `multiprotocol.html` uses.

4) **CAN mode (tunneled over extended mode)**
- CAN control and RX frames are carried in extended-mode packets of type `CAN`.
- This is what `multiprotocol.html` uses.
- Uses ESP32-C3 TWAI hardware directly (not UART bit-banging).

## Building
The flasher sources live in `web/flasher/`. `python package_web.py` builds all
three tools; `--config web/flasher/package.json` builds only the flasher, including
firmware from the build manifest. See the [flasher source guide](web/flasher/README.md).

The CC3200 sources live in `web/cc3200/`. Run `python package_web.py` to build
all three modular tools, or use `--config web/cc3200/package.json` for CC3200 only.
See the [CC3200 source guide](web/cc3200/README.md).

For the modular multi-protocol console, edit `web/multiprotocol/` and run
`python package_web.py`. The manifest merges the SWD/CAN/UART/EF views into
`multiprotocol.html` and creates the offline single-file
`multiprotocol.static.html`. See [source and packaging guide](web/multiprotocol/README.md).

1. Build firmware (generates `build/bootloader/bootloader.bin`, `build/partition_table/partition-table.bin`, `build/ESP32C3_UART.bin`).
2. Package all three web tools using the build manifest and binaries:
   ```
   python package_web.py
   ```
3. Open `flasher.html` in a Chromium-based browser (Web Serial required).

## Using the web tools
1. Flash the firmware: open `flasher.html` and click "Connect & Flash".
2. Configure the running gateway using the Configuration tab in `flasher.html`.
3. SWD debugging: open `multiprotocol.html`, select SWD and connect.

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
- Types: 0x00=DATA, 0x01=CONFIG, 0x02=CONTROL, 0x03=LOG, 0x04=SWD, 0x05=CAN, 0x0A=EXTMODE.

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

## GitHub Pages builds

`.github/workflows/pages.yml` builds ESP32-C3 firmware with ESP-IDF 5.5.2,
packages all three HTML tools, checks the outputs and stages them with
`python prepare_pages.py`. Pull requests build and upload artifacts; pushes to
the default branch deploy to GitHub Pages. The site publishes the static bundles
as `flasher.html`, `multiprotocol.html` and `cc3200.html`.

In repository Settings > Pages, select **GitHub Actions** as the publishing source.
No binaries are substituted if the build manifest or an image is missing.
