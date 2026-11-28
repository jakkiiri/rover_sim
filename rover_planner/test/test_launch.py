"""astar_planner.launch.py: RViz window placement (no processes started)."""
import importlib.util
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('astar_planner_launch', PACKAGE / 'launch' / 'astar_planner.launch.py')
LAUNCH = importlib.util.module_from_spec(spec)
spec.loader.exec_module(LAUNCH)


def test_rviz_window_geometry_is_replaced_and_nothing_else():
    original = (PACKAGE / 'rviz' / 'astar_planner.rviz').read_text()
    placed = LAUNCH.place_rviz_window(original, (1350, 54, 1210, 1506))
    block = placed[placed.index('Window Geometry:'):].splitlines()
    assert '  X: 1350' in block and '  Y: 54' in block
    assert '  Width: 1210' in block and '  Height: 1506' in block
    changed = [(a, b) for a, b in zip(original.splitlines(), placed.splitlines()) if a != b]
    assert len(changed) == 4 and len(original.splitlines()) == len(placed.splitlines())
