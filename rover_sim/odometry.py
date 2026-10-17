"""Planar wheel odometry (ROS-free): integrates the least-squares body twist.

Pose integration uses the midpoint rule on the heading so arcs are followed
exactly for constant twists. An IMU yaw rate can replace the wheel yaw
estimate (``use_imu_yaw_rate``) because gyros are far better than wheel
geometry at measuring rotation, while the wheel estimate is kept for slip
detection.
"""
import math

from rover_sim.kinematics import forward, wrap_angle


class WheelOdometry:
    def __init__(self, modules):
        self.modules = modules
        self.reset()

    def reset(self):
        self.x = self.y = self.yaw = 0.0
        self.twist = (0.0, 0.0, 0.0)          # body frame (vx, vy, wz) used for integration
        self.wheel_twist = (0.0, 0.0, 0.0)    # pure wheel estimate
        self.residuals = [0.0] * len(self.modules)
        self.distance = 0.0
        self._last_measurement = None

    def update(self, angles, wheel_rates, dt, imu_yaw_rate=None):
        """Advance the pose by ``dt`` seconds using the measured joint states."""
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('dt must be finite and nonnegative')
        twist, residuals = forward(self.modules, angles, wheel_rates)
        self._last_measurement = (list(angles), list(wheel_rates))
        self.wheel_twist = twist
        self.residuals = residuals
        vx, vy, wz = twist
        if imu_yaw_rate is not None and math.isfinite(imu_yaw_rate):
            wz = imu_yaw_rate
        self.twist = (vx, vy, wz)
        if dt == 0.0:
            return self.pose()
        heading = self.yaw + 0.5 * wz * dt
        dx = (vx * math.cos(heading) - vy * math.sin(heading)) * dt
        dy = (vx * math.sin(heading) + vy * math.cos(heading)) * dt
        self.x += dx
        self.y += dy
        self.yaw = wrap_angle(self.yaw + wz * dt)
        self.distance += math.hypot(vx, vy) * dt
        return self.pose()

    def pose(self):
        return (self.x, self.y, self.yaw)

    def predicted_module_speeds(self):
        """Rolling speed each module should have according to the other modules.

        A leave-one-out fit keeps a spinning wheel from contaminating its own
        reference, so the slip ratio of one wheel at twice the rolling speed
        reads about 0.5 instead of 0.3.
        """
        if self._last_measurement is None:
            return [0.0] * len(self.modules)
        angles, rates = self._last_measurement
        predicted = []
        for i, m in enumerate(self.modules):
            others = [j for j in range(len(self.modules)) if j != i]
            twist, _ = forward([self.modules[j] for j in others], [angles[j] for j in others],
                               [rates[j] for j in others])
            vx, vy, wz = twist
            predicted.append(math.hypot(vx - wz * m.y, vy + wz * m.x))
        return predicted
