"""Build the modular console as a development page and a single-file static page."""
import argparse
import json
from pathlib import Path
import re
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parent


def build(config_path='web/multiprotocol/package.json', fetch_vendor=False):
    config = json.loads((ROOT / config_path).read_text(encoding='utf-8'))
    if 'firmware' in config:
        import base64
        firmware = config['firmware']
        manifest = ROOT / firmware['buildManifest']
        flash_files = json.loads(manifest.read_text())['flash_files']
        if not isinstance(flash_files, dict):
            raise ValueError('Firmware manifest flash_files must be an object')
        files = [(offset, manifest.parent / filename) for offset, filename in flash_files.items()]
        if not files:
            raise ValueError('Firmware manifest contains no images')
        binaries = {p.name: base64.b64encode(p.read_bytes()).decode('ascii') for _, p in files}
        settings = {'files': [{'offset': offset, 'file': p.name} for offset, p in files]}
        (ROOT / firmware['script']).write_text(
            '/* Generated firmware payload; do not edit. */\nconst EMBEDDED_BINARIES = '
            + json.dumps(binaries) + ';\nconst EMBEDDED_FLASH_CONFIG = '
            + json.dumps(settings) + ';\n', encoding='utf-8')
    for filename, url in config.get('vendor', {}).items():
        path = ROOT / filename
        if not path.is_file():
            if not fetch_vendor:
                raise FileNotFoundError(f'{filename}: run python package_web.py --fetch-vendor once')
            with urlopen(url, timeout=60) as response:
                data = response.read()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)

    def read(filename):
        return (ROOT / filename).read_text(encoding='utf-8')

    def inline_script(filename):
        content = read(filename)
        license_file = config.get('licenses', {}).get(filename)
        if license_file:
            content = '/*\n' + read(license_file).replace('*/', '* /') + '\n*/\n' + content
        return '<script>\n' + re.sub(r'</script', r'<\/script', content, flags=re.I) + '\n</script>'

    shell = read(config['template'])
    for name, filename in config['views'].items():
        token = '{{view:' + name + '}}'
        if shell.count(token) != 1:
            raise ValueError(f'Expected exactly one {token}')
        shell = shell.replace(token, read(filename))
    for static, output in [(False, config['output']), (True, config['staticOutput'])]:
        styles = '\n'.join('<style>\n' + read(p) + '\n</style>' if static
                           else f'<link rel="stylesheet" href="{p}">' for p in config['styles'])
        scripts = '\n'.join(inline_script(p)
                            if static else f'<script src="{p}"></script>' for p in config['scripts'])
        page = shell.replace('{{styles}}', styles).replace('{{scripts}}', scripts)
        if '{{view:' in page or '{{styles}}' in page or '{{scripts}}' in page:
            raise ValueError('Unresolved template placeholders')
        # Audio remains explicitly user-triggered, but works offline in the static page.
        if static:
            import base64
            audio = ROOT / 'tadaa.mp3'
            if audio.is_file():
                encoded = base64.b64encode(audio.read_bytes()).decode('ascii')
                page = page.replace("new Audio('tadaa.mp3')", f"new Audio('data:audio/mpeg;base64,{encoded}')")
        page = '\n'.join(line.rstrip() for line in page.splitlines()) + '\n'
        (ROOT / output).write_text(page, encoding='utf-8', newline='\n')
        print(f'Created: {output}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='Build only the specified manifest; otherwise build both consoles')
    parser.add_argument('--fetch-vendor', action='store_true')
    args = parser.parse_args()
    for config in ([args.config] if args.config else
                   ['web/multiprotocol/package.json', 'web/cc3200/package.json', 'web/flasher/package.json']):
        build(config, args.fetch_vendor)
