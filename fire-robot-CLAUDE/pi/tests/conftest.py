"""Make the `rescue` package importable no matter where pytest is run from."""

import sys
from pathlib import Path

# pi/ holds the rescue package; put it first on the import path.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
