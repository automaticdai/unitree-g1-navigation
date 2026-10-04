import importlib.util
from pathlib import Path
import sys

if importlib.util.find_spec('g1_navigation') is None:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/g1_navigation'))
