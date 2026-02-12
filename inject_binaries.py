import base64
import re
from pathlib import Path

# Paths relative to project root
PROJECT_ROOT = Path(__file__).resolve().parent
HTML_PATH = PROJECT_ROOT / "flasher.html"
CC3200_HTML_PATH = PROJECT_ROOT / "cc3200.html"

# Binary sources and offsets
BINARIES = [
    ("0x0", PROJECT_ROOT / "build/bootloader/bootloader.bin"),
    ("0x8000", PROJECT_ROOT / "build/partition_table/partition-table.bin"),
    ("0x10000", PROJECT_ROOT / "build/ESP32C3_UART.bin"),
]

EMBED_START = "/* EMBEDDED_BINARIES_START */"
EMBED_END = "/* EMBEDDED_BINARIES_END */"

JS_EMBED_START = "/* EMBEDDED_JS_START:"  # suffix with filename */
JS_EMBED_END = "/* EMBEDDED_JS_END:"      # suffix with filename */

JS_FILES = [
    PROJECT_ROOT / "SparseImage.js",
    PROJECT_ROOT / "EspSerial.js",
]


def read_binaries():
    encoded = {}
    for offset, path in BINARIES:
        if not path.is_file():
            raise FileNotFoundError(f"Missing binary: {path}")
        data = path.read_bytes()
        b64 = base64.b64encode(data).decode("ascii")
        encoded[path.name] = (offset, b64)
    return encoded


def build_embed_block(encoded):
    lines = [EMBED_START, "const EMBEDDED_BINARIES = {"]
    items = list(encoded.items())
    for idx, (name, (_offset, b64)) in enumerate(items):
        comma = "," if idx < len(items) - 1 else ""
        lines.append(f'    "{name}": "{b64}"{comma}')
    lines += ["};", EMBED_END]
    return "\n        ".join(lines)  # keep indentation similar to surrounding code


def read_js_files():
    js_map = {}
    for path in JS_FILES:
        if not path.is_file():
            raise FileNotFoundError(f"Missing JS file: {path}")
        js_map[path.name] = path.read_text(encoding="utf-8")
    return js_map


def build_js_block(filename, content):
    start = f"{JS_EMBED_START}{filename} */"
    end = f"{JS_EMBED_END}{filename} */"
    return "\n".join([
        start,
        content,
        end,
    ])


def replace_js_block(html, filename, content):
    start = f"{JS_EMBED_START}{filename} */"
    end = f"{JS_EMBED_END}{filename} */"
    block = build_js_block(filename, content)

    replaced = replace_block(html, start, end, block)
    if replaced is not None:
        return replaced

    # Replace script src tag if present
    src_tag = f'<script src="{filename}"></script>'
    if src_tag in html:
        return html.replace(src_tag, block)

    return html



def replace_block(content, start_marker, end_marker, new_block):
    if start_marker in content and end_marker in content:
        pattern = re.compile(re.escape(start_marker) + r"[\s\S]*?" + re.escape(end_marker), re.MULTILINE)
        return pattern.sub(new_block, content)
    return None


def inject_embedded(content, embed_block):
    replaced = replace_block(content, EMBED_START, EMBED_END, embed_block)
    if replaced is not None:
        return replaced
    # Insert after flashConfig declaration
    needle = "let flashConfig = null;"
    if needle not in content:
        raise ValueError("Could not find flashConfig declaration to insert embedded binaries block")
    return content.replace(needle, needle + "\n\n        " + embed_block)


def inject_loader(content, load_block):
    replaced = replace_block(content, LOAD_START, LOAD_END, load_block)
    if replaced is not None:
        return replaced
    pattern = re.compile(r"async\s+function\s+loadBinaryFile\s*\(filename\)\s*\{[\s\S]*?\}\s*", re.MULTILINE)
    if pattern.search(content):
        return pattern.sub(load_block + "\n\n        ", content, count=1)

    # Fallback: slice between loadBinaryFile and the next async function (connectAndFlash)
    needle = "async function loadBinaryFile"
    start = content.find(needle)
    if start == -1:
        raise ValueError("Could not locate loadBinaryFile function to replace")
    tail = content.find("async function connectAndFlash", start)
    if tail == -1:
        raise ValueError("Could not locate end of loadBinaryFile function (next async function not found)")
    return content[:start] + load_block + "\n\n        " + content[tail:]


def main():
    encoded = read_binaries()
    embed_block = build_embed_block(encoded)

    content = HTML_PATH.read_text(encoding="utf-8")
    content = inject_embedded(content, embed_block)
    HTML_PATH.write_text(content, encoding="utf-8")

    # Embed JS into cc3200.html
    js_files = read_js_files()
    cc_content = CC3200_HTML_PATH.read_text(encoding="utf-8")
    for name, js_content in js_files.items():
        cc_content = replace_js_block(cc_content, name, js_content)
    CC3200_HTML_PATH.write_text(cc_content, encoding="utf-8")

    # Embed EspSerial.js into flasher.html
    esp_serial_name = "EspSerial.js"
    if esp_serial_name in js_files:
        content = HTML_PATH.read_text(encoding="utf-8")
        content = replace_js_block(content, esp_serial_name, js_files[esp_serial_name])
        HTML_PATH.write_text(content, encoding="utf-8")

    sizes = {name: len(base64.b64decode(b64)) for name, (_off, b64) in encoded.items()}
    print("Embedded binaries injected into flasher.html:")
    for name, size in sizes.items():
        print(f"  {name}: {size} bytes")

    print("Embedded JS injected into cc3200.html:")
    for name in js_files.keys():
        print(f"  {name}")

    if esp_serial_name in js_files:
        print("Embedded JS injected into flasher.html:")
        print(f"  {esp_serial_name}")


if __name__ == "__main__":
    main()
