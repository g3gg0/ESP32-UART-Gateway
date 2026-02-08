import base64
import re
from pathlib import Path

# Paths relative to project root
PROJECT_ROOT = Path(__file__).resolve().parent
HTML_PATH = PROJECT_ROOT / "flasher.html"

# Binary sources and offsets
BINARIES = [
    ("0x0", PROJECT_ROOT / "build/bootloader/bootloader.bin"),
    ("0x8000", PROJECT_ROOT / "build/partition_table/partition-table.bin"),
    ("0x10000", PROJECT_ROOT / "build/ESP32C3_UART.bin"),
]

EMBED_START = "/* EMBEDDED_BINARIES_START */"
EMBED_END = "/* EMBEDDED_BINARIES_END */"


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

    sizes = {name: len(base64.b64decode(b64)) for name, (_off, b64) in encoded.items()}
    print("Embedded binaries injected into flasher.html:")
    for name, size in sizes.items():
        print(f"  {name}: {size} bytes")


if __name__ == "__main__":
    main()
