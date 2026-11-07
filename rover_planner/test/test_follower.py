import math

import pytest

from rover_planner.path_follower import PARAMETERS, lookahead_point, steer

P = dict(PARAMETERS)
LINE = [(0.1 * i, 0.0) for i in range(51)]  # 5 m straight along x


def test_lookahead_skips_past_the_closest_point():
    assert lookahead_point(LINE, 1.02, 0.1, 0.5) == pytest.approx((1.5, 0.0), abs=0.11)
    assert lookahead_point(LINE, 4.9, 0.0, 0.5) == (5.0, 0.0)  # near the end: aim at the end


def test_drives_straight_when_facing_the_path():
    lin, ang = steer((0.0, 0.0, 0.0), (0.5, 0.0), (5.0, 0.0), P)
    assert lin == pytest.approx(P['max_linear']) and ang == pytest.approx(0.0)


def test_turns_in_place_when_facing_away():
    lin, ang = steer((0.0, 0.0, math.pi / 2), (0.5, 0.0), (5.0, 0.0), P)
    assert lin == 0.0 and ang < 0  # target is to the right: clockwise
    lin, ang = steer((0.0, 0.0, -0.6), (0.5, 0.0), (5.0, 0.0), P)
    assert lin == 0.0 and ang >= P['min_angular']


def test_steers_and_slows_near_the_end():
    lin, ang = steer((4.7, 0.0, 0.2), (5.0, 0.0), (5.0, 0.0), P)
    assert 0 < lin < P['max_linear'] * 0.6 and ang < 0
    assert steer((4.95, 0.0, 0.0), (5.0, 0.0), (5.0, 0.0), P) == (0.0, 0.0)


def test_never_exceeds_limits_or_reverses():
    for yaw in [i * 0.3 for i in range(-10, 11)]:
        lin, ang = steer((0.0, 0.0, yaw), (0.5, 0.2), (5.0, 0.0), P)
        assert 0.0 <= lin <= P['max_linear'] and abs(ang) <= P['max_angular']


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append((msg.linear.x, msg.angular.z))


def _follower(**overrides):
    import rclpy
    from rclpy.parameter import Parameter
    from rover_planner.path_follower import PathFollower
    if not rclpy.ok():
        rclpy.init()
    node = PathFollower.__new__(PathFollower)
    PathFollower.__init__(node)
    for name, value in overrides.items():
        node.set_parameters([Parameter(name, value=value)])
        node.p[name] = value
    node.pub = _Pub()
    return node


def _odom(yaw):
    from nav_msgs.msg import Odometry
    msg = Odometry()
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


def _status(text):
    from std_msgs.msg import String
    return String(data=text)


def test_looks_around_after_scan_delay_then_stops_after_a_full_turn():
    node = _follower(scan_delay=0.0, scan_turn=1.0)
    try:
        node.on_odom(_odom(0.0))
        node.on_status(_status('no_path_can_turn'))
        node.step()
        assert node.pub.sent[-1] == (0.0, P['scan_rate'])          # turns in place, no driving
        for yaw in (0.4, 0.8, 1.2):                                  # 1.2 rad > scan_turn
            node.on_odom(_odom(yaw))
            node.on_status(_status('no_path_can_turn'))
            node.step()
        assert node.scan_done and node.pub.sent[-1] == (0.0, 0.0)   # stopped
        sent = len(node.pub.sent)
        node.step()
        assert len(node.pub.sent) == sent                            # then stays quiet
    finally:
        node.destroy_node()


def test_no_look_around_when_turning_is_unsafe_or_too_soon():
    node = _follower(scan_delay=0.0)
    try:
        node.on_odom(_odom(0.0))
        node.on_status(_status('no_path'))
        node.step()
        assert node.pub.sent == []
    finally:
        node.destroy_node()
    node = _follower(scan_delay=30.0)
    try:
        node.on_odom(_odom(0.0))
        node.on_status(_status('no_path_can_turn'))
        node.step()
        assert node.pub.sent == []
    finally:
        node.destroy_node()


def _odom_at(x, y, yaw):
    msg = _odom(yaw)
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    return msg


def _full_scan(node, x, y):
    for yaw in (0.0, 0.6, 1.2):
        node.on_odom(_odom_at(x, y, yaw))
        node.on_status(_status('no_path_can_turn'))
        node.step()


def test_one_look_around_until_the_rover_has_driven_scan_travel():
    node = _follower(scan_delay=0.0, scan_turn=1.0)
    try:
        _full_scan(node, 0.0, 0.0)
        assert node.scan_done
        node.on_status(_status('path'))                  # a path flickers on near the same spot
        node.on_odom(_odom_at(0.3, 0.0, 1.2))
        node.on_status(_status('no_path_can_turn'))      # ... and off again
        sent = len(node.pub.sent)
        node.step()
        assert all(w == 0.0 for _, w in node.pub.sent[sent:])   # no second spin
        node.on_status(_status('path'))
        node.on_odom(_odom_at(1.5, 0.0, 1.2))             # drove more than scan_travel
        node.on_status(_status('no_path_can_turn'))
        node.step()
        assert node.pub.sent[-1] == (0.0, P['scan_rate'])
    finally:
        node.destroy_node()


def test_idle_allows_a_new_look_around_in_place():
    node = _follower(scan_delay=0.0, scan_turn=1.0)
    try:
        _full_scan(node, 0.0, 0.0)
        node.on_status(_status('idle'))                  # new goal on the way
        node.on_odom(_odom_at(0.0, 0.0, 1.2))
        node.on_status(_status('no_path_can_turn'))
        node.step()
        assert node.pub.sent[-1] == (0.0, P['scan_rate'])
    finally:
        node.destroy_node()
