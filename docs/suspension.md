# Suspension design

Goal: keep all four wheels in contact over uneven Martian ground with a
mechanism that is numerically stable in ODE and cheap to simulate. The
rover is a four-module swerve (each wheel steered and driven), which
constrains the choice more than it would on a skid-steer rover.

## Candidates

| | (a) Rocker / differential rockers | (b) Independent sprung vertical travel | (c) Hybrid (rockers + springs) |
| --- | --- | --- | --- |
| Kinematics | Left/right rockers pivot about a lateral axis; a differential (gearbox or bar) averages their angles into the body pitch. Wheels move on arcs, so the **kingpin axis tilts** by the rocker angle and a steered wheel cambers. Module x-positions change with rocker angle. | Each module translates vertically on a prismatic joint; the kingpin stays vertical and module (x, y) is constant at every travel position. | As (a) plus springs on each wheel; the most joints and the most coupled dynamics. |
| Travel | ±15–25° pivots give large terrain accommodation (one wheel can climb an obstacle while the other stays down). | ±10 cm per module; larger obstacles are handled by the body pitching/rolling as a whole. | Largest. |
| Static stability | Averaging keeps the body at the mean pitch of the two sides; no stored energy, so no bounce. | Body height varies with load; springs store energy and need damping. | Both. |
| ODE joint stability | Needs either a `gearbox` joint (ODE gearbox constraints are notoriously drifty) or a plugin-applied differential torque, plus two revolute pivots carrying 10 kg each. Revolute chains with large masses at the end are the configuration ODE handles least well without high solver iterations. | One prismatic joint per module with ODE's implicit spring-damper (`<springStiffness>`, `implicitSpringDamper`); unconditionally stable for the chosen k and c at 1 ms steps, no extra constraints. | Worst case of both. |
| Swerve interaction | Steering geometry changes with rocker angle; inverse kinematics and odometry need the rocker angle. | None: kinematics and odometry use the nominal module positions. | As (a). |
| Cost | 2 pivots + 1 differential constraint + plugin logic. | 4 prismatic joints, no plugin logic. | 6+ joints. |

## Decision: (b) independent sprung vertical travel

Chosen because it keeps the steering axes vertical, leaves the swerve
kinematics and the odometry exact, uses only ODE's built-in implicit spring
damper, and is the cheapest to simulate. The cost is less articulation
than a rocker: the body pitches when one wheel climbs a 20 cm step, which
the mechanical-limit targets accept (`docs/mechanics.md`).

## Parameters

Sprung mass: body + deck + e-box + mast = 33 kg → **m ≈ 8.25 kg per
module**; unsprung mass per module 5 kg (strut 1.0, fork 1.5, wheel 2.5).

```
k = 1500 N/m            ω_n = sqrt(k/m) = 13.5 rad/s  (2.1 Hz)
ζ = 0.5                 c = 2 ζ sqrt(k m) = 111 N s/m  (110 used)
static deflection       Mars: m g / k = 8.25·3.71/1500 = 2.0 cm   Earth: 5.4 cm
spring reference        +0.0198 m (preload = Mars weight, so the rest position is 0)
travel                  ±0.10 m (prismatic joint limits)
```

Damping ratio 0.5 gives a 2 % settling time of about `4/(ζ ω_n) = 0.6 s`.
With Earth gravity the rest position is −3.4 cm, still inside the travel.

Implementation: `urdf/modules/swerve_module.xacro` — a prismatic
`<prefix>_suspension` joint between `base_link` and the strut with
`<dynamics damping="110">` and Gazebo `<springStiffness>1500</springStiffness>`,
`<springReference>0.0198</springReference>`, `<implicitSpringDamper>true</implicitSpringDamper>`.
The actuator plugin publishes the suspension positions in `/joint_states`
so `robot_state_publisher` moves the modules in TF, and leaves them passive.

## Verification

`scripts/validate_suspension.py` drives over a 12 cm bump and a rock field
while logging wheels in contact (from the suspension spring forces and
contact state), body pitch/roll and vertical acceleration. Target: 4/4
wheels in contact ≥ 95 % of the time on moderately uneven terrain and
oscillations damped within 1–2 s. Measured results are in
`docs/validation/mars_mechanics.md`.
