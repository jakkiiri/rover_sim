"""Deterministic controller state; clocks and feedback are supplied explicitly."""
import math

from rover_sim.kinematics import inverse

DEFAULTS = dict(command_timeout=0.5, feedback_timeout=0.5,
                max_wheel_speed=12.0, wheel_acceleration=8.0,
                alignment_tolerance=0.08)
ZERO = (0.0, 0.0, 0.0)


def validate_parameters(values, modules):
    """Validate a whole proposed configuration before any value is committed."""
    for name in DEFAULTS:
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        if name in ('wheel_acceleration', 'max_wheel_speed') and value <= 0:
            raise ValueError(f'{name} must be positive')
        if name in ('command_timeout', 'feedback_timeout', 'alignment_tolerance') and value < 0:
            raise ValueError(f'{name} must be nonnegative')
    if values['max_wheel_speed'] > max(m.wheel_limit for m in modules):
        raise ValueError('max_wheel_speed must not exceed the largest URDF wheel limit')
    if values['alignment_tolerance'] >= math.pi / 2:
        raise ValueError('alignment_tolerance must be less than pi/2 radians')


class ControlState:
    def __init__(self, modules, parameters=None):
        self.modules = modules
        self.parameters = dict(DEFAULTS if parameters is None else parameters)
        validate_parameters(self.parameters, modules)
        self.reset()

    def configure(self, changes):
        candidate = dict(self.parameters, **changes)
        validate_parameters(candidate, self.modules)
        self.parameters = candidate

    def reset(self):
        self.initialized = False
        self.feedback_valid = False
        self.measured = {}
        self.feedback_stamp = None
        self.angles = [0.0] * len(self.modules)
        self.speeds = [0.0] * len(self.modules)
        self.command = ZERO
        self.command_at = -math.inf
        self.last_tick = None

    def on_command(self, values, wall_now):
        """Accept forward/yaw commands; the rover cannot command body-frame strafe.

        Check every supplied field before clamping so malformed/nonfinite
        lateral input retains the existing fail-closed stop behavior.
        """
        self.command = (values[0], 0.0, values[2]) if all(math.isfinite(v) for v in values) else ZERO
        self.command_at = wall_now

    def invalidate_feedback(self):
        self.feedback_valid = False
        self.speeds = [0.0] * len(self.modules)

    def on_feedback(self, names, positions, stamp, sim_now, velocities=(), efforts=()):
        try:
            valid = (len(names) == len(positions) and len(set(names)) == len(names)
                     and all(isinstance(n, str) and n for n in names)
                     and all(math.isfinite(v) for v in positions)
                     and all(not field or (len(field) == len(names) and
                             all(math.isfinite(v) for v in field)) for field in (velocities, efforts))
                     and math.isfinite(stamp) and stamp >= 0 and stamp <= sim_now + 0.1
                     and (self.feedback_stamp is None or stamp >= self.feedback_stamp))
            measured = dict(zip(names, positions))
            valid = valid and all(m.steering in measured for m in self.modules)
        except (TypeError, ValueError):
            valid = False
        if not valid:
            self.invalidate_feedback()
            return False
        self.measured = {m.steering: measured[m.steering] for m in self.modules}
        self.feedback_stamp = stamp
        self.feedback_valid = True
        if not self.initialized:
            self.angles = [self.measured[m.steering] for m in self.modules]
            self.speeds = [0.0] * len(self.modules)
            self.initialized = True
        return True

    def step(self, sim_now, wall_now):
        if self.last_tick is not None and sim_now < self.last_tick:
            self.reset()
        dt = 0.0 if self.last_tick is None else min(0.1, max(0.0, sim_now - self.last_tick))
        self.last_tick = sim_now
        if not self.initialized:
            # Do not invent steering positions. The plugin holds measured angles
            # and zero drive until a complete initialized command is available.
            return None
        p = self.parameters
        fresh_command = p['command_timeout'] > 0 and wall_now - self.command_at < p['command_timeout']
        fresh_feedback = (self.feedback_valid and p['feedback_timeout'] > 0 and
                          max(0.0, sim_now - self.feedback_stamp) < p['feedback_timeout'] and
                          all(m.steering in self.measured for m in self.modules))
        cmd = self.command if fresh_command and fresh_feedback else ZERO
        try:
            targets, speeds = inverse(self.modules, *cmd, self.angles, p['max_wheel_speed'])
            if not all(math.isfinite(v) for v in targets + speeds):
                raise ValueError('nonfinite inverse output')
        except (ValueError, OverflowError):
            targets, speeds, cmd = list(self.angles), [0.0] * len(self.modules), ZERO
        aligned = fresh_feedback and all(abs(t - self.measured[m.steering]) <= p['alignment_tolerance']
                                         for m, t in zip(self.modules, targets))
        for i, m in enumerate(self.modules):
            steering_step = m.steering_rate * dt
            self.angles[i] += max(-steering_step, min(steering_step, targets[i] - self.angles[i]))
            if not aligned or cmd == ZERO:
                self.speeds[i] = 0.0
            else:
                drive_step = p['wheel_acceleration'] * dt
                self.speeds[i] += max(-drive_step, min(drive_step, speeds[i] - self.speeds[i]))
        return list(self.angles), list(self.speeds)
