import base64
import json
import re
from pathlib import Path

# constants
PROJECT_ROOT = Path(__file__).resolve().parent
FLASHER_ARGS_JSON = PROJECT_ROOT / "build" / "flasher_args.json"

# Paths relative to project root - all files to patch
PATCH_FILES = [
    PROJECT_ROOT / "flasher.html",
    PROJECT_ROOT / "cc3200.html"
]

# Token markers for embedding
BINARY_EMBED_START = "/* binary_embed_start */"
BINARY_EMBED_END = "/* binary_embed_stop */"


def load_flash_config():
    """Load flash configuration from build/flasher_args.json."""
    if not FLASHER_ARGS_JSON.is_file():
        raise FileNotFoundError(f"Missing flash config: {FLASHER_ARGS_JSON}")
    
    with FLASHER_ARGS_JSON.open("r") as f:
        return json.load(f)


def read_binaries():
    """Read binaries from paths specified in flasher_args.json."""
    config = load_flash_config()
    flash_files = config.get("flash_files", {})
    
    if not flash_files:
        raise ValueError("No flash_files found in flasher_args.json")
    
    build_dir = PROJECT_ROOT / "build"
    encoded = {}
    
    for offset, rel_path in flash_files.items():
        path = build_dir / rel_path
        if not path.is_file():
            raise FileNotFoundError(f"Missing binary: {path}")
        
        data = path.read_bytes()
        b64 = base64.b64encode(data).decode("ascii")
        encoded[path.name] = (offset, b64)
    
    return encoded


def build_binary_embed_block(encoded):
    """Build the binary embed block with const EMBEDDED_BINARIES."""
    lines = [BINARY_EMBED_START, "const EMBEDDED_BINARIES = {"]
    items = list(encoded.items())
    for idx, (name, (_offset, b64)) in enumerate(items):
        comma = "," if idx < len(items) - 1 else ""
        lines.append(f'    "{name}": "{b64}"{comma}')
    lines += ["};", BINARY_EMBED_END]
    return "\n".join(lines)


def replace_block(content, start_marker, end_marker, new_block):
    """Replace content between start and end markers."""
    if start_marker in content and end_marker in content:
        pattern = re.compile(re.escape(start_marker) + r"[\s\S]*?" + re.escape(end_marker), re.MULTILINE)
        return pattern.sub(new_block, content)
    return None


def inject_binary_embeds(content, binary_block):
    """Inject binary embeds between binary_embed start/stop tokens."""
    return replace_block(content, BINARY_EMBED_START, BINARY_EMBED_END, binary_block)



def read_local_file(filename):
    """Read a local file from the project root."""
    path = PROJECT_ROOT / filename
    if path.is_file():
        return path.read_text(encoding="utf-8")
    return None


def inline_local_scripts(content):
    """Replace local script src tags with inline script content."""
    # Pattern to match <script src="filename.js"></script> where filename is local (no http/https)
    pattern = re.compile(r'<script\s+src="([^"]+)"></script>', re.IGNORECASE)
    
    def replace_script(match):
        src = match.group(1)
        
        # Skip external URLs
        if src.startswith('http://') or src.startswith('https://') or src.startswith('//'):
            return match.group(0)
        
        # Try to read the local file
        script_content = read_local_file(src)
        if script_content is not None:
            return f"<script>\n{script_content}\n</script>"
        else:
            # Keep original if file not found
            return match.group(0)
    
    return pattern.sub(replace_script, content)


def main():
    # Read binaries and build embed block
    encoded = read_binaries()
    binary_block = build_binary_embed_block(encoded)
    
    # Process each file in PATCH_FILES
    for patch_file in PATCH_FILES:
        if not patch_file.is_file():
            print(f"Warning: {patch_file} not found, skipping")
            continue
        
        content = patch_file.read_text(encoding="utf-8")
        injected = []
        
        # Try to inject binary embeds
        if BINARY_EMBED_START in content:
            new_content = inject_binary_embeds(content, binary_block)
            if new_content:
                content = new_content
                sizes = {name: len(base64.b64decode(b64)) for name, (_off, b64) in encoded.items()}
                for name, size in sizes.items():
                    injected.append(f"binary: {name} ({size} bytes)")
        
        # Write back and report if modified
        if injected:
            patch_file.write_text(content, encoding="utf-8")
            print(f"{patch_file.name}:")
            for item in injected:
                print(f"  - {item}")

        content = patch_file.read_text(encoding="utf-8")
        
        # Inline local scripts
        static_content = inline_local_scripts(content)
        
        # Create output filename with .static.html suffix
        output_file = patch_file.with_suffix('.static.html')
        output_file.write_text(static_content, encoding="utf-8")
        
        print(f"Created: {output_file.name}")                


if __name__ == "__main__":
    main()
