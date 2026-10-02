"""Compatibility entry point for packaging all three modular web tools."""
from package_web import build


def main():
    for tool in ('multiprotocol', 'cc3200', 'flasher'):
        build(f'web/{tool}/package.json')


if __name__ == '__main__':
    main()
