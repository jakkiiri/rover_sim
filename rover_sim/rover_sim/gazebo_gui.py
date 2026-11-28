"""Gazebo Fortress GUI window: plugin set, screen placement, camera on the rover.

    ros2 run rover_sim gazebo_gui

opens a GUI on a sim that is already running (started with gui:=false), and
rover_desert.launch.py runs the same thing for gui:=true. It:

- loads the world's own <gui> plugins minus DROP_PLUGINS (gui_config());
- un-maximizes the window and moves it to the left half of the screen, so the
  planner's RViz on the right half is not hidden (Fortress maximizes its window
  whatever its config says, so this goes through the window manager);
- points the camera at the rover once it is spawned. A plain ``ign gazebo -g``
  leaves the camera at the world's default pose, which on the chunked usgs
  world is far outside the terrain loaded around the rover.
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ENTITY = 'proxy_rover'
GUI_WM_CLASS = 'ign-gazebo-gui'
# The entity tree lists every model, hundreds of terrain chunks and rocks on
# usgs, and costs real-time factor while it updates (docs/fortress.md).
DROP_PLUGINS = ('EntityTree',)
# Window title bar the desktop adds above the client area (GNOME: about 37 px).
TITLE_BAR = 40


def gazebo_environment(share, environ=os.environ):
    """IGN_GAZEBO_* search paths with this package's models and plugin library added."""
    lib = os.path.normpath(os.path.join(share, '..', '..', 'lib'))

    def extend(key, value):
        current = environ.get(key, '')
        return os.pathsep.join([value] + ([current] if current else []))
    return {'IGN_GAZEBO_RESOURCE_PATH': extend('IGN_GAZEBO_RESOURCE_PATH', os.path.join(share, 'models')),
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': extend('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', lib)}


def parse_workarea(text):
    """(x, y, width, height) from ``xprop -root _NET_WORKAREA`` output, or None."""
    if '=' not in text:
        return None
    numbers = [int(n) for n in re.findall(r'\d+', text.split('=', 1)[1])]
    return tuple(numbers[:4]) if len(numbers) >= 4 and numbers[2] > 0 and numbers[3] > 0 else None


def workarea():
    """The desktop's usable area (without top bar and dock), or None without X."""
    try:
        result = subprocess.run(['xprop', '-root', '_NET_WORKAREA'], capture_output=True, text=True, timeout=2)
    except (OSError, subprocess.SubprocessError):
        return None
    return parse_workarea(result.stdout)


def screen_half(area, side):
    """Window (x, y, width, height) filling the left or right half of area."""
    if side not in ('left', 'right'):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    x, y, width, height = area
    half = width // 2
    return (x if side == 'left' else x + half, y, half, height - TITLE_BAR)


def gui_config_text(world_file, drop=DROP_PLUGINS):
    """ign-gui config with the world's <gui> plugins minus those in drop.

    None when the world has no <gui> (Gazebo then uses its default layout).
    The window size and position are not set here: Fortress maximizes its
    window whatever the config says, so place_window() moves it once mapped.
    """
    gui = ET.parse(world_file).getroot().find('world/gui')
    if gui is None:
        return None
    plugins = [p for p in gui.findall('plugin') if p.get('filename') not in drop]
    return '<?xml version="1.0"?>\n' + '\n'.join(ET.tostring(p, encoding='unicode') for p in plugins) + '\n'


def gui_config(world_file):
    """Write gui_config_text() to a temporary file; its path, or None."""
    text = gui_config_text(world_file)
    if text is None:
        return None
    with tempfile.NamedTemporaryFile('w', prefix='rover_sim_gui_', suffix='.config', delete=False) as f:
        f.write(text)
        return f.name


def find_window(pid, tree_text, wm_class=GUI_WM_CLASS):
    """X window id of the GUI process pid, from ``xwininfo -root -tree`` output."""
    for match in re.finditer(r'^\s*(0x[0-9a-f]+) "[^"]*": \("' + re.escape(wm_class) + '"', tree_text, re.M):
        window = match.group(1)
        if re.search(rf'= {pid}\b', run(['xprop', '-id', window, '_NET_WM_PID'])):
            return window
    return None


def place_window(window, geometry):
    """Un-maximize X window and move/resize it to geometry (x, y, width, height), via EWMH."""
    import ctypes
    x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.c_void_p]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

    class ClientMessage(ctypes.Structure):
        _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong), ('send_event', ctypes.c_int),
                    ('display', ctypes.c_void_p), ('window', ctypes.c_ulong), ('message_type', ctypes.c_ulong),
                    ('format', ctypes.c_int), ('data', ctypes.c_long * 5), ('pad', ctypes.c_long * 19)]

    display = x11.XOpenDisplay(None)
    if not display:
        return False
    root = x11.XDefaultRootWindow(display)

    def send(message_type, *data):
        event = ClientMessage(type=33, send_event=1, display=display, window=int(window, 16),
                              message_type=x11.XInternAtom(display, message_type, 0), format=32)
        event.data[:len(data)] = data
        # SubstructureRedirectMask | SubstructureNotifyMask, as EWMH asks of clients.
        x11.XSendEvent(display, root, 0, (1 << 20) | (1 << 19), ctypes.byref(event))
    atom = lambda name: x11.XInternAtom(display, name, 0)  # noqa: E731
    send(b'_NET_WM_STATE', 0, atom(b'_NET_WM_STATE_MAXIMIZED_HORZ'), atom(b'_NET_WM_STATE_MAXIMIZED_VERT'), 2)
    # Gravity NorthWest (1), x/y/width/height given (bits 8-11), from a pager-like source (2, bit 12+).
    send(b'_NET_MOVERESIZE_WINDOW', 1 | 0xF00 | (2 << 12), *geometry)
    x11.XFlush(display)
    x11.XCloseDisplay(display)
    return True


def running_world(service_list):
    """Name of the world in ``ign service -l`` output, or None."""
    for line in service_list.splitlines():
        match = re.fullmatch(r'/world/([^/]+)/control', line.strip())
        if match:
            return match.group(1)
    return None


def parse_model_pose(text):
    """(x, y, z, yaw) from ``ign model -m NAME -p`` output, or None."""
    vectors = re.findall(r'\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]', text)
    if len(vectors) < 2:
        return None
    (x, y, z), (_, _, yaw) = [[float(v) for v in vec] for vec in vectors[:2]]
    if not all(math.isfinite(v) for v in (x, y, z, yaw)):
        return None
    return x, y, z, yaw


def wait_for(probe, timeout, period=1.0, alive=lambda: True):
    """Call probe() until it returns something truthy, the timeout passes or alive() fails."""
    deadline = time.monotonic() + timeout
    while alive() and time.monotonic() < deadline:
        value = probe()
        if value:
            return value
        time.sleep(period)
    return None


def run(command, timeout=10):
    try:
        return subprocess.run(command, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ''


def move_camera(x, y, z, yaw):
    from rover_sim import usgs_sites
    reply = run(['ign', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'ignition.msgs.GUICamera',
                 '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000',
                 '--req', usgs_sites.camera_request(x, y, z, yaw)])
    return 'true' in reply


def main(argv=None):
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description='Open the Gazebo GUI on a running rover_sim and point it at the rover.')
    parser.add_argument('--side', choices=['left', 'right'], default='left', help='Screen half for the window')
    parser.add_argument('--entity', default=ENTITY, help='Model to point the camera at')
    parser.add_argument('--world-file', help="World file whose <gui> plugins to load (default: this package's "
                        'worlds/<running world>.world)')
    parser.add_argument('--wait', type=float, default=120.0,
                        help='Seconds to wait for the world and for the entity to be spawned')
    args = parser.parse_args(argv)

    world = wait_for(lambda: running_world(run(['ign', 'service', '-l'])), timeout=args.wait)
    if world is None:
        sys.exit('gazebo_gui: no running Gazebo world found (start rover_desert.launch.py first)')
    share = get_package_share_directory('rover_sim')
    world_file = args.world_file or os.path.join(share, 'worlds', world + '.world')
    command = ['ign', 'gazebo', '-g']
    config = gui_config(world_file) if os.path.isfile(world_file) else None
    if config:
        command += ['--gui-config', config]
    gui = subprocess.Popen(command, env={**os.environ, **gazebo_environment(share)})
    # ros2 launch escalates SIGINT to SIGTERM; either one closes the window with us.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    alive = lambda: gui.poll() is None  # noqa: E731
    try:
        area = workarea()
        window = area and wait_for(lambda: find_window(gui.pid, run(['xwininfo', '-root', '-tree'])),
                                   timeout=60, alive=alive)
        if window:
            time.sleep(1.0)   # let it map and maximize first, or the un-maximize is lost
            place_window(window, screen_half(area, args.side))
        else:
            print('gazebo_gui: Gazebo window not found; left where the desktop put it', file=sys.stderr)
        pose = wait_for(lambda: parse_model_pose(run(['ign', 'model', '-m', args.entity, '-p'])),
                        timeout=args.wait, alive=alive)
        if pose is None:
            print(f'gazebo_gui: no pose for model {args.entity!r}; camera left at the world default', file=sys.stderr)
        elif not wait_for(lambda: move_camera(*pose), timeout=90, alive=alive):
            print('gazebo_gui: /gui/move_to/pose not available; camera not moved', file=sys.stderr)
        gui.wait()
    except KeyboardInterrupt:
        gui.terminate()
        gui.wait()
    finally:
        if config:
            os.unlink(config)
    return gui.returncode


if __name__ == '__main__':
    sys.exit(main())
