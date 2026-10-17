# Focused recovery and shutdown reliability fix

Validated on 2026-10-11 with ROS 2 Humble and Gazebo Classic 11.10.2. The
normal workspace was built with `colcon build --packages-select rover_sim`
from `~/rover_ws`, then `~/rover_ws/install/setup.bash` was sourced. No build,
install or log files were edited directly. No sensors, kinematics, keyboard
mapping, no-strafe policy, model geometry or actuator gains were changed.

## Watchdog root cause and correction

The earlier failure was a real stale-command recovery race, not merely a
short observation timeout. The old `/cmd_vel` callback timestamped an unstamped
Twist when Python executed its callback. A command queued while the controller
was suspended could therefore appear fresh on SIGCONT. The actuator bridge
would accept resumed joint heartbeats immediately, even after its watchdog
had stopped the wheels. Depending on DDS delivery/callback ordering, the old
test either replayed motion or passed. It also accepted a cached zero-target
snapshot before observing only one later wheel-state value.

The strengthened test confirms the controller is SIGSTOPped using `/proc`,
then publishes one forward Twist while it cannot execute callbacks. The
reliable DDS writer remains alive throughout suspension and recovery. This
publication precedes watchdog expiry; it is not a fresh user command on
resume. No acknowledgement is required while the reader's own DDS threads
are frozen. The test examines every new state and target frame in the existing
0.4-second recovery window. No watchdog/command timeout or observation sleep
was increased to make the check pass.

Against the old controller/plugin, the wheels first stopped, then replayed
forward motion on SIGCONT: maximum wheel speed **2.36124 rad/s**, with **20 of
21 nonzero target frames** and no generation change. See
[baseline watchdog results](reliability_before.json).

The plugin now makes watchdog expiry a generation transition under its mutex.
It checks expiry both on physics updates and before a command callback can
refresh freshness. Expiry clears targets/integrators and inhibits drive until
the new generation is acknowledged. The controller resets its state and
replaces its volatile `/cmd_vel` reader before acknowledging. Old-reader
queued samples are discarded, and already-taken old-generation callbacks
are ignored. Duplicate acknowledgements do not reset a live generation.
Steering is initialized again from complete measured feedback. A new
publication through the new reader can then restore motion.

With the fix, all **10 Gazebo integration checks passed** in
[recovery results](reliability_after.json):

| Check | Result |
| --- | --- |
| Invalid atomic parameter update | Rejected; both values unchanged |
| Valid runtime update | Accepted and restored |
| Pause/resume deadman | Zero targets; resumed wheels below 0.05 rad/s |
| Reset all while moving | New generation; zero drive until fresh input |
| Reset models while moving | New generation; zero drive until fresh input |
| Reset time while moving | New generation; zero drive until fresh input |
| Slow simulation | RTF 0.01998; 250 wall heartbeat frames; maximum gap 0.02494 s |
| Independent watchdog | 0.6 s configured; wheels below 0.05 rad/s after 0.800 wall seconds including braking |
| Delayed Twist on recovery | All 21 target frames zero; maximum wheel speed 0.01163 rad/s while settling |
| Fresh command after recovery | All four wheels resumed above 2 rad/s |

These are measured flat-ground integration results, separate from unit tests.
The interface remains unstamped Twist: replacing the local reader prevents
this proven recovery replay, but cannot establish end-to-end age or physical
key-press freshness for arbitrary upstream publishers. A continually active
publisher may resume on its next new publication after matching the new
reader. One-shot publishers must publish again after recovery.

## Repeated SIGINT root cause and correction

The old main caught KeyboardInterrupt only around `rclpy.spin`. A subsequent
SIGINT could raise another KeyboardInterrupt inside the finally block while
subscriptions were being destroyed. The normal automatic ROS signal handler
could also shut down the context before a final zero command was sent.

The real-process regression instruments only the destruction boundary and
injects actual SIGINT, SIGINT and SIGTERM there. The old installed controller
exited **-2**, logged KeyboardInterrupt and emitted no final zero target. See
[baseline shutdown result](shutdown_before.json). The instrumentation is
confined to the validation child, not production code.

The executable now installs non-raising signal handlers before ROS
initialization and disables rclpy's automatic signal shutdown. The handlers
only set stop flags. Main performs one guarded cleanup while the ROS context
is still live: inhibit callbacks, cancel the wall timer, clear chassis input,
publish zero wheel velocity using initialized steering targets, and wait up
to 0.2 wall seconds for DDS acknowledgement. It destroys the node once and
then shuts down the context. Repeated or reentrant cleanup calls are harmless.
No steering command is fabricated if feedback never initialized the node.

Both installed-controller process tests passed in
[shutdown results](shutdown_reliability.json): moving before shutdown, and
shutdown before any steering feedback. Each exited **0**, reached destruction
exactly once, and produced no traceback or KeyboardInterrupt despite repeated
signals during teardown. The moving case delivered a final zero wheel target.
The other case published no fabricated actuator targets. These tests use
synthetic ROS feedback and DDS; they do not claim physical Gazebo braking.

An additional [moving Gazebo shutdown check](shutdown_gazebo.json) sent
SIGINT, SIGINT and SIGTERM to the actual controller while leaving Gazebo
running. The final actuator target was zero, actual wheel speed dropped from
2.15194 to 0.04286 rad/s in 0.2051 wall seconds, and launch reported controller
exit **0**. One already-in-flight nonzero frame arrived before the final zero;
the zero was delivered before destruction. No cleanup exception occurred.

## Unit tests and scope

All **117 unit tests passed**: 22 kinematics, 22 teleop and 73 controller.
All previous 103 tests remain, with the reset fixture updated to assert one
reader replacement. Fourteen new cases cover generation handoff ordering,
already-taken stale callbacks, fresh no-strafe commands after recovery,
initial reader creation, idempotent/reentrant cleanup, missing feedback or
ROS context, publishing/acknowledgement failures, callbacks during shutdown,
forced zero publication, and repeated signals during main-loop and cleanup.

Normal `colcon test` and `colcon test-result --verbose` report **120 results,
0 errors, 0 failures, 0 skipped**; the extra three results are CTest wrappers.
The existing installed Xacro/Python compatibility warning remains. Backward
time warnings from robot_state_publisher are expected during reset tests;
watchdog-expiry warnings are expected when deliberately suspending control.

Modified source/configuration: `rover_sim/controller.py`,
`src/swerve_gazebo_plugin.cpp`, `test/test_controller.py`,
`scripts/validate_hardening.py`, `CMakeLists.txt`. Added executable:
`scripts/validate_shutdown.py`. Documentation: `README.md`,
`docs/hardening.md`, a follow-up pointer in `docs/validation/no_strafe.md`, this
report and its five linked JSON records.

## Safety and Phase 5 readiness

The watchdog issue was an actual runtime motion-safety problem: suspension
recovery could restart motion without a new operator publication. The SIGINT
issue was an actual cleanup/stop-delivery problem: teardown could be interrupted
and the final stop omitted. The independent actuator watchdog still provided
fallback braking; repeated SIGINT did not disable that watchdog.

Both documented blockers are fixed and Phase 5 simulated LiDAR can proceed.
Existing prototype limitations remain deferred: slope parking, idealized
2 cm tread contact, steering alignment pauses, no odometry, single command
publisher and lack of command arbitration. Local RViz continues to use
`base_link`; robot_state_publisher owns fixed transforms. No new capability
was added in this fix.
