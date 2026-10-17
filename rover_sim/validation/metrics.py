"""ROS-free process and timing metrics used by the validation scripts.

Real-time factor is computed from simulation-clock samples against monotonic
wall time; CPU load is read from ``/proc`` for named processes. Nothing here
depends on ROS so the arithmetic can be unit tested.
"""
import os
import time


CLOCK_TICKS = os.sysconf('SC_CLK_TCK')


def find_processes(name):
    """Return PIDs whose executable name matches ``name`` (e.g. ``gzserver``)."""
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/comm') as handle:
                if handle.read().strip() == name:
                    pids.append(int(entry))
        except OSError:
            continue
    return pids


def cpu_seconds(pid):
    """Total user+system CPU seconds consumed by ``pid`` (including threads)."""
    try:
        with open(f'/proc/{pid}/stat') as handle:
            fields = handle.read().rsplit(')', 1)[1].split()
    except OSError:
        return None
    return (int(fields[11]) + int(fields[12])) / CLOCK_TICKS


def memory_mb(pid):
    try:
        with open(f'/proc/{pid}/status') as handle:
            for line in handle:
                if line.startswith('VmRSS:'):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        return None
    return None


class CpuSampler:
    """Measure the average CPU utilisation (in cores) of processes over a window."""

    def __init__(self, pids):
        self.pids = list(pids)
        self.start_wall = time.monotonic()
        self.start_cpu = {pid: cpu_seconds(pid) for pid in self.pids}

    def finish(self):
        elapsed = time.monotonic() - self.start_wall
        result = {}
        for pid in self.pids:
            before, after = self.start_cpu.get(pid), cpu_seconds(pid)
            if before is None or after is None or elapsed <= 0:
                result[pid] = None
            else:
                result[pid] = (after - before) / elapsed
        return elapsed, result


def real_time_factor(samples):
    """``samples`` is a sequence of (wall_seconds, sim_seconds) pairs."""
    if len(samples) < 2:
        raise ValueError('at least two clock samples are required')
    (w0, s0), (w1, s1) = samples[0], samples[-1]
    if w1 <= w0:
        raise ValueError('wall clock did not advance')
    return (s1 - s0) / (w1 - w0)


def summarize_window(samples, cpu_by_pid, elapsed):
    """Build the JSON-friendly performance record used by every validator."""
    return {
        'wall_seconds': elapsed,
        'real_time_factor': real_time_factor(samples),
        'clock_samples': len(samples),
        'cpu_cores_by_pid': {str(pid): value for pid, value in cpu_by_pid.items()},
    }
