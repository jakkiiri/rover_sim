"""Drive the rover along /planner/path by publishing /cmd_vel.

Turn-then-drive steering, after the original's navigate() in rsx-rover's
local_astar.py: aim at a point `lookahead` metres along the path; if it is
more than `turn_angle` off the nose, turn in place, otherwise drive forward
while steering towards it. The rover cannot strafe (rover_sim's controller
drops sideways commands), so only linear.x and angular.z are used.

Stopping: an empty path (goal reached, unreachable or cleared), no new path
for `path_timeout`, or no /odom for `path_timeout` all stop the rover. The
controller also stops on its own 0.5 s after the last command, so if this
node dies the rover stops. When idle the node publishes nothing, so teleop
can drive; do not run both at once while following.

Look-around: when the planner reports no path but says turning in place is
safe (/planner/status "no_path_can_turn") for `scan_delay` seconds, the rover
turns in place at `scan_rate`, at most `scan_turn` radians, so the camera can
re-check ground behind it (a false obstacle there is only cleared by seeing
it). It stops as soon as a path appears; after a full scan without one it waits
until the planner's status changes. It looks around at most once until the
rover has driven `scan_travel` metres from where the last look-around started
(or the planner goes idle), so a path that keeps appearing and vanishing cannot
make it go, stop and spin over and over.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

PARAMETERS = {
    'lookahead': 0.5,        # m along the path to aim at
    'max_linear': 0.4,       # m/s, teleop's speed
    'max_angular': 0.5,      # rad/s, teleop's turn rate
    'min_angular': 0.15,     # rad/s, so in-place turns do not stall
    'turn_angle': 0.5,       # rad off the nose (~30 deg) above which it turns in place
    'heading_gain': 1.5,     # rad/s per rad of heading error
    'slow_radius': 0.6,      # m from the path end where it starts slowing down
    'stop_radius': 0.15,     # m from the path end where it stops
    'path_timeout': 1.0,     # s without a new path or pose before stopping
    'rate': 10.0,            # Hz of /cmd_vel while driving
    'scan_delay': 2.0,       # s of "no path, can turn" before looking around
    'scan_rate': 0.3,        # rad/s while looking around
    'scan_turn': 6.3,        # rad, the most it turns per look-around (one turn)
    'scan_travel': 1.0,      # m to drive after a look-around before another may start
}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def lookahead_point(points, x, y, lookahead):
    """Point to aim at: the first path point at least `lookahead` beyond the closest one."""
    closest = min(range(len(points)), key=lambda i: (points[i][0] - x) ** 2 + (points[i][1] - y) ** 2)
    for px, py in points[closest:]:
        if math.hypot(px - x, py - y) >= lookahead:
            return px, py
    return points[-1]


def steer(pose, target, end, p):
    """(linear, angular) command towards target; end is the path's last point."""
    x, y, yaw = pose
    error = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
    to_end = math.hypot(end[0] - x, end[1] - y)
    if to_end < p['stop_radius']:
        return 0.0, 0.0
    angular = max(-p['max_angular'], min(p['max_angular'], p['heading_gain'] * error))
    if abs(error) > p['turn_angle']:
        # Turn in place, fast enough not to stall.
        return 0.0, math.copysign(max(abs(angular), p['min_angular']), error)
    linear = p['max_linear'] * math.cos(error) * min(1.0, to_end / p['slow_radius'])
    return max(linear, 0.0), angular


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in PARAMETERS}
        if self.p['rate'] <= 0 or self.p['lookahead'] <= 0 or self.p['max_linear'] < 0:
            raise ValueError('rate and lookahead must be positive, max_linear nonnegative')
        self.pose = self.pose_at = None
        self.path, self.path_at = [], None
        self.driving = False
        self.status, self.status_at, self.stuck_since = None, None, None
        self.scan_yaw, self.scanned, self.scan_done = None, 0.0, False
        self.last_scan_xy = None  # where the last look-around started
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Path, '/planner/path', self.on_path, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/planner/status', self.on_status, 1)
        self.create_timer(1.0 / self.p['rate'], self.step)
        self.get_logger().info('Path follower ready: following /planner/path on /cmd_vel. '
                               'Close teleop while it drives.')

    def on_path(self, msg):
        self.path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.path_at = self.get_clock().now()

    def on_status(self, msg):
        now = self.get_clock().now()
        if msg.data == 'idle':
            self.last_scan_xy = None
        if msg.data != 'no_path_can_turn':
            self.stuck_since, self.scan_yaw, self.scanned, self.scan_done = None, None, 0.0, False
        elif self.stuck_since is None:
            self.stuck_since = now
        self.status, self.status_at = msg.data, now

    def scan_command(self, now):
        """Angular speed for a look-around, or None when not looking around."""
        if (self.scan_done or self.stuck_since is None or not self.fresh(self.status_at, now)
                or not self.fresh(self.pose_at, now)
                or (now - self.stuck_since).nanoseconds * 1e-9 < self.p['scan_delay']):
            return None
        x, y, yaw = self.pose
        if self.scan_yaw is None:
            if (self.last_scan_xy is not None and math.hypot(
                    x - self.last_scan_xy[0], y - self.last_scan_xy[1]) < self.p['scan_travel']):
                if not self.scan_done:
                    self.get_logger().info('No path, but already looked around here; waiting')
                self.scan_done = True
                return None
            self.last_scan_xy = (x, y)
            self.get_logger().info('No path: turning in place to look around')
        else:
            self.scanned += abs(wrap(yaw - self.scan_yaw))
        self.scan_yaw = yaw
        if self.scanned >= self.p['scan_turn']:
            self.scan_done = True
            self.get_logger().info('Looked all around and still no path; waiting')
            return None
        return self.p['scan_rate']

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_at = self.get_clock().now()

    def fresh(self, stamp, now):
        return stamp is not None and (now - stamp).nanoseconds * 1e-9 < self.p['path_timeout']

    def step(self):
        now = self.get_clock().now()
        reason = None
        if not self.path:
            reason = 'no path (goal reached, unreachable or not set)'
        elif not self.fresh(self.path_at, now):
            reason = 'path is stale (planner stopped?)'
        elif not self.fresh(self.pose_at, now):
            reason = 'no recent /odom'
        if reason is None:
            target = lookahead_point(self.path, self.pose[0], self.pose[1], self.p['lookahead'])
            linear, angular = steer(self.pose, target, self.path[-1], self.p)
            if linear == 0.0 and angular == 0.0:
                reason = 'at the end of the path'
        if reason is not None and not self.path:
            spin = self.scan_command(now)
            if spin is not None:
                cmd = Twist()
                cmd.angular.z = spin
                self.pub.publish(cmd)
                self.driving = True
                return
        if reason is not None:
            if self.driving:
                self.pub.publish(Twist())  # stop now rather than wait for the controller timeout
                self.get_logger().info(f'Stopped: {reason}')
                self.driving = False
            return
        if not self.driving:
            self.get_logger().info(f'Following a path of {len(self.path)} points')
            self.driving = True
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = linear, angular
        self.pub.publish(cmd)

    def stop(self):
        if self.driving:
            try:
                self.pub.publish(Twist())
            except Exception:  # context already shut down; the controller times out in 0.5 s
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
