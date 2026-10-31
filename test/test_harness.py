"""Validation harness: backend detection and process helpers (no simulator)."""
from rover_sim.validation.harness import SimulationLaunch, isolated_environment
from rover_sim.validation import metrics


def test_launch_backend_and_world_name_from_arguments():
    env = isolated_environment(domain=77, port=11577)
    classic = SimulationLaunch('rover_mars.launch.py', ['gui:=false', 'world:=mars', 'rock_density:=high'], env=dict(env))
    assert classic.backend == 'classic' and classic.world_name == 'mars_high'
    fortress = SimulationLaunch('rover_mars.launch.py', ['sim:=fortress', 'world:=step_test'], env=dict(env))
    assert fortress.backend == 'fortress' and fortress.world_name == 'step_test'
    assert fortress.env['IGN_PARTITION'] == 'rover_sim_77'
    default = SimulationLaunch('rover_mars.launch.py', [], env=dict(env))
    assert default.world_name == 'mars_medium'


def test_real_time_factor_and_cpu_math():
    assert metrics.real_time_factor([(0.0, 0.0), (10.0, 9.5)]) == 0.95
    record = metrics.summarize_window([(0.0, 1.0), (2.0, 3.0)], {1: 0.5}, 2.0)
    assert record['real_time_factor'] == 1.0 and record['cpu_cores_by_pid'] == {'1': 0.5}
