"""Start our small room using the CODEX Isaac Sim 6.1 integration."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parent

if __name__ == '__main__':
    # Keep the working robot, sensors, and controller together.
    runner = ROOT / 'FIRE_ROBOT_CODEX' / 'run_isaac.py'
    sys.path.insert(0, str(runner.parent))
    if not any(arg == '--layout' or arg.startswith('--layout=') for arg in sys.argv):
        sys.argv.extend(['--layout', 'room'])
    runpy.run_path(str(runner), run_name='__main__')
