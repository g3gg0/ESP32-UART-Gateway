# Multi-protocol console sources

Edit these sources, then run from the repository root:

```powershell
python package_web.py
```

The JSON build manifest is `package.json` (no npm dependencies or installation
required). `shell.html` contains the shared page frame; `views/swd.html`,
`views/can.html`, `views/uart.html`, and `views/ef.html` are HTML fragments inserted
at build time. They share the same serial connection and application state.

`js/swd/Swd.js` contains the SWD transport and its pin-control class.
`js/swd/HexEditor.js` and `js/swd/MemoryScanner.js` contain the SWD tools.
`js/swd/protocol.js` owns SWD state, register definitions, CoreSight decoding,
diagnostics, detection, AP/memory UI, and GPIO initialization.
`js/can/protocol.js`, `js/uart/protocol.js`, and `js/ef/protocol.js` each own
their protocol state, handlers, configuration and controls. EF also owns its
lens catalog, telemetry and cyclic actions. Each protocol updates its own UI;
the shared dispatcher calls these updates after connection changes.
`js/app.js` contains shared connection/UI logic and dispatches protocol hooks.
The shared serial transport remains in root `EspSerial.js`. CSS lives in
`console.css`. There is no separate `SwdPins.js` dependency.

Outputs (generated; edit the sources instead):

- `multiprotocol.html`: merged HTML with local JS/CSS references, for development.
- `multiprotocol.static.html`: one file with inline JS/CSS, Capstone, and audio.

The ordered `scripts` list in `package.json` controls execution order. Classic
scripts preserve the existing global handlers used by HTML `onclick` attributes;
classes resolve application constants when their methods run. No runtime fetching
of HTML fragments or ES module server is required. Both outputs can be opened
locally in a browser supporting Web Serial.

Capstone 3.0.5 is vendored from the existing dependency URL recorded in the
manifest. It is reused without network access on subsequent builds. If missing,
`python package_web.py --fetch-vendor` fetches it once. The legacy
`python inject_binaries.py` also invokes this builder and leaves these outputs
to it, while continuing to package the other tools.

Validation:

```powershell
node --test --test-isolation=none tests/swd-web.test.cjs tests/web-package.test.cjs
```
