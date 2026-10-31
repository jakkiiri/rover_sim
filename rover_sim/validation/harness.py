"""Launch, observe and tear down an isolated headless Gazebo simulation.

Every validator uses this module so that measurements are taken the same
way: an isolated ``ROS_DOMAIN_ID``/``GAZEBO_MASTER_URI`` pair, a process group
that is terminated in ``finally``, a steady wall clock for command heartbeats
and the simulation clock for everything physical.
"""
import json
import math
import os
import random
import signal
import subprocess
import threading
import time

from rover_sim.validation import metrics


def isolated_environment(domain=None, port=None):
    env = dict(os.environ)
    domain = domain if domain is not None else random.randint(60, 200)
    port = port if port is not None else random.randint(11400, 11900)
    env['ROS_DOMAIN_ID'] = str(domain)
    env['GAZEBO_MASTER_URI'] = f'http://127.0.0.1:{port}'
    env.setdefault('RCUTILS_COLORIZED_OUTPUT', '0')
    return env


class SimulationLaunch:
    """Run ``ros2 launch rover_sim <file> <args>`` in its own process group."""

    def __init__(self, launch_file, launch_args=(), env=None, log_path=None, package='rover_sim'):
        self.env = env if env is not None else isolated_environment()
        self.command = ['ros2', 'launch', package, launch_file, *launch_args]
        self.launch_args = list(launch_args)
        args = dict(a.split(':=', 1) for a in launch_args if ':=' in a)
        self.backend = args.get('sim', 'classic').lower()
        world = args.get('world', 'mars')
        self.world_name = f"mars_{args.get('rock_density', 'medium')}" if world == 'mars' else world
        # gz-transport partitions by IGN_PARTITION; isolate like GAZEBO_MASTER_URI.
        self.env.setdefault('IGN_PARTITION', 'rover_sim_' + self.env['ROS_DOMAIN_ID'])
        self.log_path = log_path
        self.process = None
        self.log = None

    def __enter__(self):
        os.environ['ROS_DOMAIN_ID'] = self.env['ROS_DOMAIN_ID']
        os.environ['GAZEBO_MASTER_URI'] = self.env['GAZEBO_MASTER_URI']
        os.environ['IGN_PARTITION'] = self.env['IGN_PARTITION']
        self.log = open(self.log_path, 'w') if self.log_path else subprocess.DEVNULL
        self.process = subprocess.Popen(self.command, env=self.env, stdout=self.log,
                                        stderr=subprocess.STDOUT, start_new_session=True)
        return self

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def server_pids(self):
        """The physics server process(es): gzserver (Classic) or the ruby-hosted ign gazebo (Fortress)."""
        if self.backend == 'fortress':
            wanted = ('IGN_PARTITION=' + self.env['IGN_PARTITION']).encode()
            found = []
            for pid in metrics.find_processes('ruby'):
                try:
                    with open(f'/proc/{pid}/cmdline', 'rb') as handle:
                        cmdline = handle.read()
                    with open(f'/proc/{pid}/environ', 'rb') as handle:
                        env = handle.read().split(b'\0')
                except OSError:
                    continue
                if b'gazebo' in cmdline and wanted in env:
                    found.append(pid)
            return found
        return self.pids('gzserver')

    def pids(self, name):
        """PIDs of ``name`` (e.g. gzserver) whose GAZEBO_MASTER_URI matches ours."""
        wanted = ('GAZEBO_MASTER_URI=' + self.env['GAZEBO_MASTER_URI']).encode()
        found = []
        for pid in metrics.find_processes(name):
            try:
                with open(f'/proc/{pid}/environ', 'rb') as handle:
                    if wanted in handle.read().split(b'\0'):
                        found.append(pid)
            except OSError:
                continue
        return found

    def __exit__(self, *exc):
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.process.wait(timeout=20)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
            if self.process.poll() is None:
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=5)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        for pid in self.pids('gzserver') + self.pids('gzclient') + (self.server_pids() if self.backend == 'fortress' else []):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if self.log not in (None, subprocess.DEVNULL):
            self.log.close()
        return False


class Observer:
    """A spinning rclpy node that records the clock, ground truth and joints."""

    def __init__(self, name='validation_observer', command_rate=50.0):
        import rclpy
        from rclpy.clock import Clock, ClockType
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock as ClockMessage
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState, Imu
        self.rclpy = rclpy
        self.lock = threading.Lock()
        self.clock_samples = []
        self.ground_truth = []     # (wall, sim, x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz)
        self.joint_states = []     # (wall, sim, {name: (pos, vel, effort)})
        self.imu = []              # (wall, sim, roll, pitch, ax, ay, az)
        self.command = (0.0, 0.0, 0.0)
        self.errors = []
        rclpy.init()
        self.node = Node(name, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.node.create_subscription(ClockMessage, '/clock', self._on_clock, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 50)
        self.node.create_subscription(JointState, '/joint_states', self._on_joints, 50)
        self.node.create_subscription(Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 1)
        self.node.create_timer(1.0 / command_rate, self._publish,
                               clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    # -- callbacks -------------------------------------------------------
    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception as error:  # noqa: BLE001 - surfaced by wait_for
            self.errors.append(repr(error))

    def _on_clock(self, msg):
        with self.lock:
            self.clock_samples.append((time.monotonic(), msg.clock.sec + msg.clock.nanosec / 1e9))
            if len(self.clock_samples) > 400000:
                del self.clock_samples[:100000]

    def _on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        with self.lock:
            self.ground_truth.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                                      p.x, p.y, p.z, roll, pitch, yaw, v.x, v.y, v.z, w.x, w.y, w.z))
            if len(self.ground_truth) > 400000:
                del self.ground_truth[:100000]

    def _on_joints(self, msg):
        efforts = list(msg.effort) if len(msg.effort) == len(msg.name) else [0.0] * len(msg.name)
        velocities = list(msg.velocity) if len(msg.velocity) == len(msg.name) else [0.0] * len(msg.name)
        data = {n: (p, v, e) for n, p, v, e in zip(msg.name, msg.position, velocities, efforts)}
        with self.lock:
            self.joint_states.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9, data))
            if len(self.joint_states) > 400000:
                del self.joint_states[:100000]

    def _on_imu(self, msg):
        q, a = msg.orientation, msg.linear_acceleration
        roll, pitch, _ = quaternion_to_euler(q.x, q.y, q.z, q.w)
        with self.lock:
            self.imu.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                             roll, pitch, a.x, a.y, a.z))
            if len(self.imu) > 400000:
                del self.imu[:100000]

    def _publish(self):
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = map(float, self.command)
        self.publisher.publish(msg)

    # -- queries ---------------------------------------------------------
    def sim_time(self):
        with self.lock:
            return self.clock_samples[-1][1] if self.clock_samples else None

    def latest_truth(self):
        with self.lock:
            return self.ground_truth[-1] if self.ground_truth else None

    def latest_joints(self):
        with self.lock:
            return self.joint_states[-1][2] if self.joint_states else None

    def wait_for(self, predicate, timeout=60.0, what='condition', launch=None):
        deadline = time.monotonic() + timeout
        while not predicate():
            if self.errors:
                raise RuntimeError('observer failed: ' + self.errors[-1])
            if launch is not None and not launch.alive():
                raise RuntimeError('simulation launch exited while waiting for ' + what)
            if time.monotonic() > deadline:
                raise RuntimeError('timed out waiting for ' + what)
            time.sleep(0.02)

    def run_sim(self, seconds, command=None, timeout_factor=20.0):
        """Hold ``command`` for ``seconds`` of simulation time.

        The wall timeout scales with the requested simulation time so that a
        slow simulation (RTF down to 1/timeout_factor) still completes; a
        stalled clock raises after that.
        """
        if command is not None:
            self.command = command
        self.wait_for(lambda: self.sim_time() is not None, timeout=60.0, what='first /clock sample')
        start = self.sim_time()
        self.wait_for(lambda: self.sim_time() is not None and self.sim_time() - start >= seconds,
                      timeout=max(30.0, seconds * timeout_factor), what='simulation time to advance')

    def measure(self, seconds, pids):
        """Sample RTF and CPU of ``pids`` over ``seconds`` of wall time."""
        sampler = metrics.CpuSampler(pids)
        started = time.monotonic()
        with self.lock:
            first = len(self.clock_samples)
        time.sleep(seconds)
        elapsed, cpu = sampler.finish()
        with self.lock:
            window = self.clock_samples[first:]
        record = metrics.summarize_window(window, cpu, elapsed)
        record['memory_mb_by_pid'] = {str(pid): metrics.memory_mb(pid) for pid in pids}
        return record

    def close(self):
        self.command = (0.0, 0.0, 0.0)
        self._publish()
        if self.node.executor is not None:
            self.node.executor.shutdown()
        self.thread.join(timeout=2)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def quaternion_to_euler(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


class EntityControl:
    """Teleport, spawn, remove and query models.

    Classic: gazebo_ros_state / factory services. Fortress: gz-transport
    services (``/world/<name>/set_pose``, ``create``, ``remove``) through the
    ``ign`` CLI, and poses from the ground-truth stream or the dynamic pose
    topic.
    """

    def __init__(self, observer, timeout=30.0, launch=None):
        self.observer = observer
        self.launch = launch
        self.backend = launch.backend if launch is not None else 'classic'
        if self.backend == 'fortress':
            self.world = launch.world_name
            self.env = dict(os.environ, IGN_PARTITION=launch.env['IGN_PARTITION'])
            observer.wait_for(lambda: self._gz('service', '-l', timeout=10).find(f'/world/{self.world}/set_pose') >= 0,
                              timeout=timeout, what='gz world services')
            return
        from gazebo_msgs.srv import GetEntityState, SetEntityState, SpawnEntity, DeleteEntity
        node = observer.node
        self.get = node.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.set = node.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.spawn = node.create_client(SpawnEntity, '/spawn_entity')
        self.delete = node.create_client(DeleteEntity, '/delete_entity')
        observer.wait_for(lambda: self.get.service_is_ready() and self.set.service_is_ready()
                          and self.spawn.service_is_ready() and self.delete.service_is_ready(),
                          timeout=timeout, what='gazebo entity services')

    def _call(self, client, request, timeout=20.0):
        future = client.call_async(request)
        self.observer.wait_for(future.done, timeout=timeout, what='gazebo service reply')
        return future.result()

    def _gz(self, *args, timeout=20):
        import subprocess
        return subprocess.run(['ign', *args], capture_output=True, text=True, timeout=timeout, env=self.env).stdout

    def _gz_service(self, service, reqtype, reptype, request, timeout=20):
        out = self._gz('service', '-s', f'/world/{self.world}/{service}', '--reqtype', reqtype, '--reptype', reptype,
                       '--timeout', str(int(timeout * 1000)), '--req', request, timeout=timeout + 5)
        if 'data: true' not in out:
            raise RuntimeError(f'gz service {service} failed: {out.strip()[:200]}')

    def pose(self, name):
        if self.backend == 'fortress':
            import re
            out = self._gz('topic', '-e', '-t', f'/world/{self.world}/pose/info', '-n', '1')
            match = re.search(r'pose \{\s*name: "%s"[\s\S]*?position \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)\s*\}'
                              r'\s*orientation \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)\s*w: ([-\d.e+]+)' % re.escape(name), out)
            if match is None:
                raise RuntimeError(f'no pose for {name} on the dynamic pose topic')
            x, y, z, qx, qy, qz, qw = (float(match.group(i)) for i in range(1, 8))
            return (x, y, z, *quaternion_to_euler(qx, qy, qz, qw))
        from gazebo_msgs.srv import GetEntityState
        state = self._call(self.get, GetEntityState.Request(name=name, reference_frame='world')).state
        p, q = state.pose.position, state.pose.orientation
        return (p.x, p.y, p.z, *quaternion_to_euler(q.x, q.y, q.z, q.w))

    def teleport(self, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        qw, qx, qy, qz = (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                          cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
        if self.backend == 'fortress':
            self._gz_service('set_pose', 'ignition.msgs.Pose', 'ignition.msgs.Boolean',
                             f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
                             f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
            return
        from gazebo_msgs.msg import EntityState
        from gazebo_msgs.srv import SetEntityState
        state = EntityState()
        state.name = name
        state.reference_frame = 'world'
        state.pose.position.x, state.pose.position.y, state.pose.position.z = float(x), float(y), float(z)
        state.pose.orientation.w, state.pose.orientation.x = qw, qx
        state.pose.orientation.y, state.pose.orientation.z = qy, qz
        result = self._call(self.set, SetEntityState.Request(state=state))
        if not result.success:
            raise RuntimeError(f'set_entity_state failed for {name}')

    def spawn_box(self, name, x, y, z, sx, sy, sz, static=True, mu=0.65, mass=1.0, colour='0.5 0.47 0.44'):
        from gazebo_msgs.srv import SpawnEntity
        inertia = ''
        if not static:
            inertia = (f'<inertial><mass>{mass}</mass><inertia><ixx>{mass*(sy*sy+sz*sz)/12}</ixx>'
                       f'<iyy>{mass*(sx*sx+sz*sz)/12}</iyy><izz>{mass*(sx*sx+sy*sy)/12}</izz>'
                       '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
        sdf = (f'<sdf version="1.6"><model name="{name}"><static>{"true" if static else "false"}</static>'
               f'<link name="link">{inertia}<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
               f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>'
               f'<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
               f'<material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse></material></visual></link></model></sdf>')
        if self.backend == 'fortress':
            escaped = sdf.replace('"', '\\"')
            self._gz_service('create', 'ignition.msgs.EntityFactory', 'ignition.msgs.Boolean',
                             f'sdf: "{escaped}", name: "{name}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}')
            return
        request = SpawnEntity.Request(name=name, xml=sdf, reference_frame='world')
        request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = float(x), float(y), float(z)
        request.initial_pose.orientation.w = 1.0
        result = self._call(self.spawn, request)
        if not result.success:
            raise RuntimeError(f'spawn failed for {name}: {result.status_message}')

    def remove(self, name):
        if self.backend == 'fortress':
            self._gz_service('remove', 'ignition.msgs.Entity', 'ignition.msgs.Boolean', f'name: "{name}", type: MODEL')
            return
        from gazebo_msgs.srv import DeleteEntity
        self._call(self.delete, DeleteEntity.Request(name=name))

    def reset_world(self):
        """Reset model poses (Classic /reset_world, Fortress world control)."""
        if self.backend == 'fortress':
            self._gz_service('control', 'ignition.msgs.WorldControl', 'ignition.msgs.Boolean', 'reset: {all: true}')
            return
        from std_srvs.srv import Empty
        client = self.observer.node.create_client(Empty, '/reset_world')
        self.observer.wait_for(client.service_is_ready, timeout=10, what='/reset_world')
        self._call(client, Empty.Request())


def truth_since(observer, sim_start):
    with observer.lock:
        return [g for g in observer.ground_truth if g[1] >= sim_start]


def joints_since(observer, sim_start):
    with observer.lock:
        return [j for j in observer.joint_states if j[1] >= sim_start]
