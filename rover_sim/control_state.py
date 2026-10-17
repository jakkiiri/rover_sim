"""Deterministic swerve controller core; clocks, feedback and IMU are supplied.

Pipeline per tick (``step``):

1. command / feedback freshness (fail closed to a zero twist),
2. drive mode (``ackermann`` clamps vy, ``crab`` clamps wz, ``point_turn``
   clamps vx/vy, ``auto`` passes the twist through),
3. tilt-dependent speed and acceleration scaling from the IMU,
4. body-level acceleration and jerk limiting (v and wz, not per wheel),
5. swerve inverse kinematics with steering-flip optimisation,
6. steering slew and continuous steer-error speed scaling
   ``max(0, cos e)^p`` (hard stop only beyond ``steer_error_cutoff``),
7. traction control: per-wheel speed reduction while slip exceeds a threshold,
8. stall detection, wheel odometry and diagnostics.
"""
import math

from rover_sim.kinematics import inverse
from rover_sim.odometry import WheelOdometry

DRIVE_MODES = ('ackermann', 'crab', 'point_turn', 'auto')
ZERO = (0.0, 0.0, 0.0)
DEFAULTS = dict(
    command_timeout=0.5,        # monotonic wall seconds; 0 inhibits drive
    feedback_timeout=0.5,       # simulation seconds; 0 inhibits drive
    max_speed=1.0,              # m/s body linear speed
    max_yaw_rate=1.2,           # rad/s
    max_accel=0.8,              # m/s^2
    max_yaw_accel=1.5,          # rad/s^2
    max_jerk=4.0,               # m/s^3
    max_yaw_jerk=8.0,           # rad/s^3
    max_wheel_speed=8.0,        # rad/s, must not exceed the URDF limit
    steer_error_power=2.0,      # exponent p in max(0, cos e)^p
    steer_error_cutoff=1.2,     # rad; hard stop beyond this module error
    twist_scaling=True,         # scale the body twist by the worst module
    tilt_slowdown_deg=15.0,     # start reducing speed/accel at this tilt
    tilt_cutoff_deg=32.0,       # stop above this tilt (fraction of ~47 deg tip)
    slip_threshold=0.35,        # slip ratio that triggers traction control
    slip_speed_min=0.15,        # m/s below which slip is not evaluated
    slip_reduction_rate=2.0,    # per second reduction of a slipping wheel's scale
    slip_recovery_rate=0.5,     # per second recovery
    slip_min_scale=0.3,
    stall_time=2.0,             # s of commanded-but-no-motion before a stall warning
    stall_speed_fraction=0.15,  # measured/commanded speed below this counts as no motion
    stall_scale=0.6,            # wheel speed scale applied while stalled
    odometry_use_imu=True,
    imu_velocity_tau=10.0,      # s, decay of the IMU/wheel velocity-discrepancy estimate
)
POSITIVE = ('max_speed', 'max_yaw_rate', 'max_accel', 'max_yaw_accel', 'max_jerk', 'max_yaw_jerk',
            'max_wheel_speed', 'slip_reduction_rate', 'slip_recovery_rate', 'stall_time', 'tilt_cutoff_deg',
            'imu_velocity_tau')
NONNEGATIVE = ('command_timeout', 'feedback_timeout', 'steer_error_power', 'steer_error_cutoff',
               'tilt_slowdown_deg', 'slip_threshold', 'slip_speed_min')
FRACTIONS = ('slip_min_scale', 'stall_speed_fraction', 'stall_scale')
BOOLEANS = ('twist_scaling', 'odometry_use_imu')


def validate_parameters(values, modules):
    """Validate a complete configuration; raise ValueError naming the offender."""
    for name in DEFAULTS:
        if name not in values:
            raise ValueError(f'{name} is missing')
        value = values[name]
        if name in BOOLEANS:
            if not isinstance(value, bool):
                raise ValueError(f'{name} must be a boolean')
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        if name in POSITIVE and value <= 0:
            raise ValueError(f'{name} must be positive')
        if name in NONNEGATIVE and value < 0:
            raise ValueError(f'{name} must be nonnegative')
        if name in FRACTIONS and not 0 <= value <= 1:
            raise ValueError(f'{name} must lie in [0, 1]')
    if values['max_wheel_speed'] > max(m.wheel_limit for m in modules):
        raise ValueError('max_wheel_speed must not exceed the largest URDF wheel limit')
    if values['steer_error_cutoff'] > math.pi / 2:
        raise ValueError('steer_error_cutoff must not exceed pi/2 radians')
    if values['tilt_cutoff_deg'] <= values['tilt_slowdown_deg']:
        raise ValueError('tilt_cutoff_deg must exceed tilt_slowdown_deg')
    if values['tilt_cutoff_deg'] >= 90:
        raise ValueError('tilt_cutoff_deg must be below 90')


def apply_mode(mode, command):
    vx, vy, wz = command
    if mode == 'ackermann':
        return (vx, 0.0, wz)
    if mode == 'crab':
        return (vx, vy, 0.0)
    if mode == 'point_turn':
        return (0.0, 0.0, wz)
    return (vx, vy, wz)


def rate_limit(current, target, slope, dt):
    step = slope * dt
    return current + max(-step, min(step, target - current))


class ControlState:
    def __init__(self, modules, parameters=None, drive_mode='ackermann'):
        self.modules = modules
        self.parameters = dict(DEFAULTS if parameters is None else parameters)
        validate_parameters(self.parameters, modules)
        if drive_mode not in DRIVE_MODES:
            raise ValueError(f'drive_mode must be one of {DRIVE_MODES}')
        self.drive_mode = drive_mode
        self.odometry = WheelOdometry(modules)
        self.reset()

    # -- configuration -----------------------------------------------------
    def configure(self, changes):
        candidate = dict(self.parameters, **changes)
        validate_parameters(candidate, self.modules)
        self.parameters = candidate

    def set_drive_mode(self, mode):
        if mode not in DRIVE_MODES:
            raise ValueError(f'drive_mode must be one of {DRIVE_MODES}')
        self.drive_mode = mode

    # -- state -------------------------------------------------------------
    def reset(self):
        n = len(self.modules)
        self.initialized = False
        self.feedback_valid = False
        self.measured = {}
        self.measured_rates = {}
        self.feedback_stamp = None
        self.angles = [0.0] * n
        self.speeds = [0.0] * n
        self.command = ZERO
        self.command_at = -math.inf
        self.last_tick = None
        self.twist = [0.0, 0.0, 0.0]          # limited body twist
        self.accel = [0.0, 0.0, 0.0]          # limited body acceleration
        self.tilt = 0.0
        self.imu_stamp = None
        self.imu_yaw_rate = None
        self.gravity_estimate = None
        self.velocity_correction = [0.0, 0.0]     # body-frame (vx, vy) IMU-vs-wheel discrepancy
        self._wheel_twist_at_imu = (0.0, 0.0)
        self.traction_scale = [1.0] * n
        self.slip = [0.0] * n
        self.stall_since = None
        self.stalled = False
        self.odometry.reset()
        self.diagnostics = dict(mode=self.drive_mode, tilt_scale=1.0, steer_scale=1.0, stalled=False,
                                slip=[0.0] * n, traction_scale=[1.0] * n, twist=ZERO, measured_twist=ZERO)

    def on_command(self, values, wall_now):
        """Store a body twist; nonfinite input becomes a zero (stop) command."""
        try:
            finite = all(math.isfinite(v) for v in values) and len(values) == 3
        except TypeError:
            finite = False
        self.command = tuple(float(v) for v in values) if finite else ZERO
        # A malformed command is an invalid command: fail closed immediately.
        self.command_at = wall_now if finite else -math.inf

    def on_imu(self, roll, pitch, yaw_rate, stamp, accel=None):
        """Store tilt and yaw rate; with ``accel`` (specific force, body frame,
        m/s^2) also track the IMU-vs-wheel velocity discrepancy.

        The gravity-compensated horizontal acceleration is integrated minus
        the change of the wheel-odometry velocity; a wall impact therefore
        leaves a persistent negative correction (wheels keep reporting motion,
        the IMU saw the stop) that decays with ``imu_velocity_tau`` so bias
        cannot accumulate. Gravity is estimated from the force norm.
        """
        if not all(math.isfinite(v) for v in (roll, pitch, yaw_rate, stamp)):
            return False
        if accel is not None and not all(math.isfinite(v) for v in accel):
            return False
        self.tilt = max(abs(roll), abs(pitch))
        self.imu_yaw_rate = yaw_rate
        previous = self.imu_stamp
        self.imu_stamp = stamp
        if accel is None:
            return True
        fx, fy, fz = accel
        norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        if self.gravity_estimate is None:
            self.gravity_estimate = norm
        elif abs(yaw_rate) < 0.05:
            self.gravity_estimate += 0.01 * (norm - self.gravity_estimate)
        g = self.gravity_estimate
        ax = fx + g * math.sin(pitch)
        ay = fy - g * math.cos(pitch) * math.sin(roll)
        wheel = self.odometry.wheel_twist[:2]
        if previous is not None:
            dt = min(0.1, max(0.0, stamp - previous))
            cx, cy = self.velocity_correction
            cx += ax * dt - (wheel[0] - self._wheel_twist_at_imu[0])
            cy += ay * dt - (wheel[1] - self._wheel_twist_at_imu[1])
            cx, cy = cx + yaw_rate * dt * cy, cy - yaw_rate * dt * cx     # body frame rotates
            decay = math.exp(-dt / self.parameters['imu_velocity_tau'])
            self.velocity_correction = [cx * decay, cy * decay]
        self._wheel_twist_at_imu = (wheel[0], wheel[1])
        return True

    def fused_twist(self):
        """Wheel twist corrected by the IMU discrepancy (yaw from the gyro)."""
        vx, vy, wz = self.odometry.twist
        return (vx + self.velocity_correction[0], vy + self.velocity_correction[1], wz)

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
        rates = dict(zip(names, velocities)) if velocities else {}
        previous_stamp = self.feedback_stamp
        self.measured = {m.steering: measured[m.steering] for m in self.modules}
        self.measured_rates = {m.drive: rates.get(m.drive, 0.0) for m in self.modules}
        self.feedback_stamp = stamp
        self.feedback_valid = True
        if not self.initialized:
            self.angles = [self.measured[m.steering] for m in self.modules]
            self.speeds = [0.0] * len(self.modules)
            self.initialized = True
        elif previous_stamp is not None and velocities:
            dt = min(0.1, max(0.0, stamp - previous_stamp))
            imu = self.imu_yaw_rate if self.parameters['odometry_use_imu'] else None
            self.odometry.update([self.measured[m.steering] for m in self.modules],
                                 [self.measured_rates[m.drive] for m in self.modules], dt, imu)
        return True

    # -- control tick ------------------------------------------------------
    def step(self, sim_now, wall_now):
        if self.last_tick is not None and sim_now < self.last_tick:
            self.reset()
        dt = 0.0 if self.last_tick is None else min(0.1, max(0.0, sim_now - self.last_tick))
        self.last_tick = sim_now
        if not self.initialized:
            return None
        p = self.parameters
        n = len(self.modules)
        fresh_command = p['command_timeout'] > 0 and wall_now - self.command_at < p['command_timeout']
        fresh_feedback = (self.feedback_valid and p['feedback_timeout'] > 0 and
                          max(0.0, sim_now - self.feedback_stamp) < p['feedback_timeout'] and
                          all(m.steering in self.measured for m in self.modules))
        if not (fresh_command and fresh_feedback):
            # Fail closed: an immediate zero twist, no deceleration ramp.
            desired = ZERO
            self.twist = [0.0, 0.0, 0.0]
            self.accel = [0.0, 0.0, 0.0]
        else:
            desired = apply_mode(self.drive_mode, self.command)

        # 3. tilt scaling
        tilt_deg = math.degrees(self.tilt)
        if tilt_deg <= p['tilt_slowdown_deg']:
            tilt_scale = 1.0
        elif tilt_deg >= p['tilt_cutoff_deg']:
            tilt_scale = 0.0
        else:
            tilt_scale = 1.0 - (tilt_deg - p['tilt_slowdown_deg']) / (p['tilt_cutoff_deg'] - p['tilt_slowdown_deg'])
        max_speed = p['max_speed'] * tilt_scale
        max_yaw = p['max_yaw_rate'] * tilt_scale
        max_accel = p['max_accel'] * max(tilt_scale, 0.25)
        max_yaw_accel = p['max_yaw_accel'] * max(tilt_scale, 0.25)

        # Clamp the desired twist to the (tilt-scaled) limits, preserving direction.
        vx, vy, wz = desired
        linear = math.hypot(vx, vy)
        if linear > max_speed and linear > 0:
            vx, vy = vx * max_speed / linear, vy * max_speed / linear
        wz = max(-max_yaw, min(max_yaw, wz))

        # 4. body-level acceleration and jerk limiting (zero dt during pauses)
        if desired != ZERO or any(self.twist):
            if dt > 0:
                for i, (target, a_max, j_max) in enumerate(((vx, max_accel, p['max_jerk']),
                                                             (vy, max_accel, p['max_jerk']),
                                                             (wz, max_yaw_accel, p['max_yaw_jerk']))):
                    remaining = target - self.twist[i]
                    if abs(remaining) <= j_max * dt * dt + 1e-12 and abs(self.accel[i]) <= j_max * dt + 1e-12:
                        self.twist[i], self.accel[i] = target, 0.0      # converged
                        continue
                    # Jerk-limited approach: with the acceleration stepping down
                    # by j*dt per tick, n steps cover j*dt^2*n(n+1)/2 of velocity,
                    # so the largest acceleration that can still ramp to zero
                    # exactly at the target is n*j*dt.
                    steps = math.floor((-1.0 + math.sqrt(1.0 + 8.0 * abs(remaining) / (j_max * dt * dt))) / 2.0)
                    bound = min(a_max, steps * j_max * dt)
                    wanted_accel = math.copysign(bound, remaining) if remaining else 0.0
                    self.accel[i] = rate_limit(self.accel[i], wanted_accel, j_max, dt)
                    self.twist[i] += self.accel[i] * dt
                    if (remaining > 0 and self.twist[i] > target) or (remaining < 0 and self.twist[i] < target):
                        self.twist[i] = target
        limited = tuple(self.twist)

        # 5./6. inverse kinematics, steering slew, steer-error scaling
        try:
            targets, speeds = inverse(self.modules, *limited, self.angles, p['max_wheel_speed'])
            if not all(math.isfinite(v) for v in targets + speeds):
                raise ValueError('nonfinite inverse output')
        except (ValueError, OverflowError):
            targets, speeds = list(self.angles), [0.0] * n
            limited = ZERO
            self.twist = [0.0, 0.0, 0.0]
        scales = []
        for i, m in enumerate(self.modules):
            self.angles[i] = rate_limit(self.angles[i], targets[i], m.steering_rate, dt)
            error = abs(targets[i] - self.measured[m.steering]) if fresh_feedback else math.pi
            if error >= p['steer_error_cutoff']:
                scales.append(0.0)
            else:
                scales.append(max(0.0, math.cos(error)) ** p['steer_error_power'])
        body_scale = min(scales) if scales else 0.0

        # 7. traction control: a wheel slips when it rolls faster than both the
        # other wheels (leave-one-out fit) and the IMU-fused body velocity say
        # the ground moves under it.
        fused = self.fused_twist()
        loo = self.odometry.predicted_module_speeds()
        predicted = [min(loo[i], math.hypot(fused[0] - fused[2] * m.y, fused[1] + fused[2] * m.x))
                     for i, m in enumerate(self.modules)]
        for i, m in enumerate(self.modules):
            rate = self.measured_rates.get(m.drive, 0.0)
            rolling = abs(rate * m.radius)
            if fresh_feedback and max(rolling, predicted[i]) > p['slip_speed_min']:
                self.slip[i] = (rolling - predicted[i]) / max(rolling, predicted[i])
            else:
                self.slip[i] = 0.0
            if self.slip[i] > p['slip_threshold']:
                self.traction_scale[i] = max(p['slip_min_scale'], self.traction_scale[i] - p['slip_reduction_rate'] * dt)
            else:
                self.traction_scale[i] = min(1.0, self.traction_scale[i] + p['slip_recovery_rate'] * dt)

        # 8. stall detection: commanded motion, no measured motion
        commanded_speed = math.hypot(limited[0], limited[1]) + 0.5 * abs(limited[2])
        measured_twist = fused
        measured_speed = math.hypot(measured_twist[0], measured_twist[1]) + 0.5 * abs(measured_twist[2])
        all_slipping = all(scale <= p['slip_min_scale'] + 1e-9 for scale in self.traction_scale)
        if commanded_speed > 0.05 and body_scale > 0.5 and (
                measured_speed < p['stall_speed_fraction'] * commanded_speed or all_slipping):
            self.stall_since = sim_now if self.stall_since is None else self.stall_since
            self.stalled = sim_now - self.stall_since >= p['stall_time']
        else:
            self.stall_since = None
            self.stalled = False
        stall_scale = p['stall_scale'] if self.stalled else 1.0

        for i in range(n):
            scale = (body_scale if p['twist_scaling'] else scales[i]) * self.traction_scale[i] * stall_scale
            self.speeds[i] = speeds[i] * scale
        self.diagnostics = dict(mode=self.drive_mode, tilt_scale=tilt_scale, steer_scale=body_scale,
                                stalled=self.stalled, slip=list(self.slip), traction_scale=list(self.traction_scale),
                                twist=limited, measured_twist=tuple(measured_twist), tilt_deg=tilt_deg,
                                command_fresh=fresh_command, feedback_fresh=fresh_feedback)
        return list(self.angles), list(self.speeds)
