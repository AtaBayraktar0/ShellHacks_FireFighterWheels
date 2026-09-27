"""Apply the explicit mapping-only patch locally, saving replaced files first."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ALLOWED = {
    'rover/room_mapping.py', 'rover/app.py', 'rover/pi_link.py',
    'dashboard/app.js', 'dashboard/scan.js', 'dashboard/scan.html',
    'scripts/pi_inventory.py', 'docs/retained-room-scan.md',
}


def apply(archive_path, root):
    root = Path(root).resolve()
    if not (root / 'rover/runtime.py').is_file():
        raise ValueError('Run from the warm-wheels project directory')
    with zipfile.ZipFile(archive_path) as archive:
        entries = archive.namelist()
        if len(entries) != len(set(entries)) or set(entries) != ALLOWED | {'mapping-update-manifest.json'}:
            raise ValueError('Unexpected update contents')
        if sum(entry.file_size for entry in archive.infolist()) > 2_000_000:
            raise ValueError('Unexpected update size')
        manifest = json.loads(archive.read('mapping-update-manifest.json'))
        if set(manifest['sha256']) != ALLOWED:
            raise ValueError('Manifest file list mismatch')
        payloads = {name: archive.read(name) for name in ALLOWED}
    for name, data in payloads.items():
        if hashlib.sha256(data).hexdigest() != manifest['sha256'][name]:
            raise ValueError('Update checksum mismatch: ' + name)
        if not (root / name).resolve().is_relative_to(root):
            raise ValueError('Update destination escapes project: ' + name)
        if name.endswith('.py'):
            compile(data, name, 'exec')
    backup = root / 'captures' / ('before-room-mapping-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f') + '.zip')
    backup.parent.mkdir(exist_ok=True)
    with zipfile.ZipFile(backup, 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in ALLOWED:
            if (root / name).exists():
                archive.write(root / name, name)
    for name, data in payloads.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + '.mapping-update-tmp')
        with temporary.open('xb') as stream:
            stream.write(data)
        temporary.replace(path)
    print('Updated experimental retained scanning. Motor settings and firmware unchanged.')
    print('Replaced-file backup:', backup)
    print('Next: .venv-pi/bin/python -m scripts.pi_inventory')
    print('Then: bash scripts/start_pi_camera.sh')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('Usage: python3 scripts/apply_mapping_update.py /path/to/update.zip')
    apply(sys.argv[1], Path.cwd())
