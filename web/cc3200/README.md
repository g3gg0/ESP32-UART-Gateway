# CC3200 console sources

Edit the files here rather than generated root HTML files.

```powershell
python package_web.py --config web/cc3200/package.json
```

Running `python package_web.py` without a config builds all three consoles.
`python inject_binaries.py` also builds all three modular consoles.

- `shell.html`: shared frame and template insertion points.
- `views/*.html`: eight inert command-tab templates, cloned during initialization.
- `console.css`: shared styling.
- `js/CC3200PacketAssembler.js`, `js/CC3200Serial.js`: packet and serial classes.
- `js/SffsFATParser.js`, `js/SffsClient.js`: filesystem classes.
- `js/protocol.js`, `helpers.js`: protocol constants and byte helpers.
- `js/storage.js`, `sffs.js`, `flash.js`: feature handlers and state.
- `js/commands.js`, `tabs.js`: command definitions and tab controls.
- `js/app.js`: connection, console and general input handling.

`package.json` defines source order and produces `cc3200.html` with local
JS/CSS references and `cc3200.static.html` with all code and styling embedded.
Shared `EspSerial.js` and `SparseImage.js` remain root dependencies, embedded
in the static output. No runtime fetching of HTML templates is needed.

```powershell
node --test --test-isolation=none tests/cc3200-package.test.cjs
```
