# Flasher sources

```powershell
python package_web.py --config web/flasher/package.json
```

`python package_web.py` builds all three modular tools. The legacy
`python inject_binaries.py` invokes the same builders.

Edit `shell.html`, `views/flash.html`, `views/config.html`, `console.css`, and
the feature files in `js/`. `flash.js` owns flashing and image selection;
`config.js` owns gateway configuration; `app.js` owns shared capabilities,
logging and tab setup. `js/firmware.js` is generated, not a source to edit.

The manifest lists scripts in execution order. Existing ESP32 chips/flasher
dependencies are retained in `web/vendor/` and embedded in the static output.
Missing vendor files can be downloaded with `--fetch-vendor`.

Firmware comes from `build/flasher_args.json`; the packager uses its offsets
and reads only the listed binaries. Packaging fails if the manifest is missing,
`flash_files` is missing or empty, or any listed binary is missing. There are no
fallback images or default offsets. Rebuild firmware and repackage to ship changes.

Outputs are `flasher.html` (local JS/CSS references) and
`flasher.static.html` (single file containing scripts, CSS and firmware).
Neither packaging nor tests connect to a device or flash hardware.

```powershell
node --test --test-isolation=none tests/flasher-package.test.cjs
```
