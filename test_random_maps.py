"""Launch the CODEX randomized-map batch tester from the repository root."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parent

if __name__ == '__main__':
    runner = ROOT/'FIRE_ROBOT_CODEX'/'test_random_maps.py'
    sys.path.insert(0, str(runner.parent))
    runpy.run_path(str(runner), run_name='__main__')
