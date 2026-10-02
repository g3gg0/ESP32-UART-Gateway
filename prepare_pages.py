"""Stage only the generated, self-contained tools for GitHub Pages."""
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent


def main():
    site = ROOT / '_site'
    sources = [(ROOT / 'index.html', 'index.html')]
    sources += [(ROOT / f'{name}.static.html', f'{name}.html')
                for name in ('flasher', 'multiprotocol', 'cc3200')]
    # Check all inputs before writing the deployment directory.
    for source, _ in sources:
        if not source.is_file():
            raise FileNotFoundError(source)
    site.mkdir(exist_ok=True)
    for source, name in sources:
        shutil.copyfile(source, site / name)
    (site / '.nojekyll').write_text('')
    print('Pages site staged in _site/')


if __name__ == '__main__':
    main()
