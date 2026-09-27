"""Build a small application-only Pi update; no venv, keys or motor configuration."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
FILES = (
    'rover/room_mapping.py', 'rover/app.py', 'rover/pi_link.py',
    'dashboard/app.js', 'dashboard/scan.js', 'dashboard/scan.html',
    'scripts/pi_inventory.py', 'docs/retained-room-scan.md',
)


def main():
    destination = ROOT.parent / 'WARM-room-mapping-update.zip'
    with zipfile.ZipFile(destination, 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in FILES:
            archive.write(ROOT / name, name)
        archive.writestr('mapping-update-manifest.json', json.dumps({
            'version': 'experimental-room-odometry-1', 'autonomous_driving': False,
            'sha256': {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in FILES},
        }, indent=2))
    print(destination)
    print('SHA256:', hashlib.sha256(destination.read_bytes()).hexdigest())


if __name__ == '__main__':
    main()
