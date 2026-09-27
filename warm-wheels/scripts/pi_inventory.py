"""Read-only Pi device inventory. Never open serial ports or move motors."""
import json
import platform
from pathlib import Path
from serial.tools import list_ports


def main():
    by_id = Path('/dev/serial/by-id')
    print(json.dumps({
        'os': platform.system(), 'architecture': platform.machine(),
        'serial_ports': [{'port': p.device, 'description': p.description, 'vid': p.vid, 'pid': p.pid}
                         for p in list_ports.comports()],
        'stable_paths': [str(p) for p in by_id.iterdir()] if by_id.exists() else [],
        'firmware': 'Not identified; cable presence does not establish firmware or motor readiness',
        'serial_ports_opened': 0, 'drive_commands_sent': 0,
    }, indent=2))


if __name__ == '__main__':
    main()
