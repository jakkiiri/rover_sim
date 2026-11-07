"""Launch helpers for the stereo system, used by launch/rover_desert.launch.py."""
import math
import os

from launch.actions import LogInfo
from launch_ros.actions import Node

from rover_sim.stereo_geometry import validate_stereo_config

# 480x360 keeps each RGB frame (518,400 B) inside Fast DDS's default 512 KiB shared-memory
# segment; 640x480 frames went over UDP and delayed the controller's reset handshake.
SHM_SEGMENT_BYTES = 512 * 1024
LAUNCH_ARGS = dict(stereo_width='480', stereo_height='360', stereo_rate='15',
                   stereo_baseline='0.12', stereo_hfov=str(math.pi / 2))


def parse_stereo_args(values):
    """Convert launch-argument strings and validate them; raises ValueError before anything starts."""
    try:
        width, height = int(values['stereo_width']), int(values['stereo_height'])
        rate, baseline, hfov = (float(values[k]) for k in ('stereo_rate', 'stereo_baseline', 'stereo_hfov'))
    except (KeyError, ValueError) as error:
        raise ValueError(f'Invalid stereo launch argument: {error}') from error
    return validate_stereo_config(width, height, rate, baseline, hfov)


def xacro_mappings(enabled, config):
    """Xacro arguments for proxy_rover.urdf.xacro."""
    mappings = {'enable_stereo': 'true' if enabled else 'false'}
    if enabled:
        mappings.update(stereo_width=str(config['width']), stereo_height=str(config['height']),
                        stereo_rate=repr(config['rate']), stereo_baseline=repr(config['baseline']),
                        stereo_hfov=repr(config['hfov']))
    return mappings


def frame_size_warning(config):
    """Text of a launch warning when one RGB frame is too big for default shared memory, else None."""
    size = config['width'] * config['height'] * 3
    if size <= SHM_SEGMENT_BYTES - 1024:   # leave room for the message header
        return None
    return (f"WARNING: stereo frames of {size} bytes exceed Fast DDS's default {SHM_SEGMENT_BYTES}-byte "
            'shared-memory segment and will travel over UDP. In testing this delayed the controller reset '
            'handshake (docs/stereo.md). Use a smaller stereo_width/stereo_height or enlarge the segment.')


def stereo_nodes(share, description, config=None):
    """Right-camera relay and processor; both read geometry from the same robot_description."""
    common = {'robot_description': description, 'use_sim_time': True}
    warning = frame_size_warning(config) if config else None
    return ([LogInfo(msg=warning)] if warning else []) + [
        Node(package='rover_sim', executable='stereo_camera_relay', name='stereo_camera_relay',
             parameters=[common], output='screen'),
        Node(package='rover_sim', executable='stereo_processor', name='stereo_processor',
             parameters=[os.path.join(share, 'config', 'stereo.yaml'), common], output='screen'),
    ]
