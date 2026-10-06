import sys
from pathlib import Path

# tests/synthetic_capture.py is shared by unit and integration tests
sys.path.insert(0, str(Path(__file__).resolve().parent))
