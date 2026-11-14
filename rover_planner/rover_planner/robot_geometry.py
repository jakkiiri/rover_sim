"""Footprint and camera position of the simulated rover (robot:=real or proxy).

The real rover's numbers come from rover_sim's config/real_rover.yaml, written
by rover_sim/scripts/generate_real_rover.py from the rover's CAD.
"""
import math
import os

import yaml

# Proxy box rover (rover_sim urdf/proxy_rover.urdf.xacro): base_link to the
# front of the wheels / camera, and to the outside of the wheels.
PROXY = {'half_length': 0.58, 'half_width': 0.51, 'camera_x': 0.55, 'camera_y': 0.0,
         'shadow_box': [0.0] * 6}
# Stereo camera on the real rover's body front, beside the stowed arm
# (default stereo_xyz in rover_sim urdf/real_rover.urdf.xacro).
REAL_CAMERA_XY = (0.25, 0.16)
ROBOTS = ('real', 'proxy')
# m added round the stowed arm's box for the planner's shadow filter: covers LiDAR
# range noise (~3.5 cm seen) and stereo mismatches just off the arm's outline.
SHADOW_MARGIN = 0.05


def _ceil_cm(value):
    return math.ceil(value * 100.0 - 1e-6) / 100.0


def real_from_yaml(geometry):
    """Footprint centred on base_link that holds the arm, body and wheels."""
    extent = geometry['extent']
    centre, size = geometry['arm']['box']['xyz'], geometry['arm']['box']['size']
    shadow = []
    for c, half in zip(centre, (v / 2.0 for v in size)):
        shadow += [round(c - half - SHADOW_MARGIN, 4), round(c + half + SHADOW_MARGIN, 4)]
    return {'half_length': _ceil_cm(max(extent['front'], -extent['rear'])),
            'half_width': _ceil_cm(extent['half_width']),
            'camera_x': REAL_CAMERA_XY[0], 'camera_y': REAL_CAMERA_XY[1],
            'shadow_box': shadow}


def rover_geometry(robot):
    if robot == 'proxy':
        return dict(PROXY)
    if robot != 'real':
        raise ValueError(f"robot must be one of {ROBOTS}, not '{robot}'")
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('rover_sim'), 'config', 'real_rover.yaml')
    with open(path) as f:
        return real_from_yaml(yaml.safe_load(f))
