# Rover mechanics and mobility limits

The rover's obstacle and slope limits come from its parameters (mass, CG,
wheel radius, friction, motor stall torque and no-load speed) acting through
ODE physics. Nothing in the controller or plugin refuses an obstacle by
height. This page derives the limits, lists the parameters that produce them
and records the measured thresholds from the test worlds.

## Parameters (`urdf/mars_rover.urdf.xacro`)

| Symbol | Value | Source |
| --- | --- | --- |
| total mass m | 53.0 kg | body 26 + deck 3 + e-box 3 + mast 1 + 4 × (strut 1.0 + fork 1.5 + wheel 2.5) |
| CG height h_cg | ≈ 0.37 m above ground | body group CoM 0.45 m (33 kg), module CoM 0.225 m (20 kg) |
| track / wheelbase | 1.00 m / 0.80 m | kingpins at (±0.40, ±0.50) |
| wheel radius R, width | 0.20 m, 0.12 m | spherical collision, cylindrical visual with 10 grousers |
| ground clearance | 0.30 m | body bottom to ground at Mars preload |
| tyre μ / ground μ | 0.8 / 0.65 | Gazebo ODE uses min(μ₁, μ₂) = **0.65** |
| stall torque τ_stall | `wheel_stall_torque` (see measured table) | URDF effort limit, read by the plugin |
| no-load speed ω_nl | 10 rad/s (2.0 m/s) | URDF velocity limit |
| gravity g | 3.71 m/s² (`gravity:=9.81` for Earth) | world file |
| suspension | k = 1500 N/m, c = 110 N s/m, ±0.10 m | `docs/suspension.md` |

The motor model in the plugin is a DC-motor derating
`τ_avail(ω) = τ_stall (1 − |ω| / ω_nl)` when the torque drives in the
direction of rotation; braking torque is limited to τ_stall. Stalling is
therefore physical: the PI velocity loop saturates at τ_avail and the rover
slows to the speed at which the available thrust balances the load.

## Slope limits

Three independent limits apply to climbing a slope of angle θ:

1. **Traction**: the wheels slip when `tan θ > μ`. With μ = 0.65:
   **θ_slip = 33.0°** (μ = 0.6 → 31.0°, μ = 0.8 → 38.7°).
2. **Torque** (quasi-static, all four wheels driven):
   `4 τ_stall / R ≥ m g sin θ + F_rr`. Rolling resistance from the joint
   friction (0.05 N m per wheel) is `F_rr = 4·0.05/R = 1 N`, negligible;
   contact damping adds a little more. Stall angle
   `θ_stall = asin((4 τ_stall / R − F_rr) / (m g))`:

   | τ_stall | 4 τ/R | θ_stall (Mars) | θ_stall (Earth) |
   | --- | --- | --- | --- |
   | 4.8 N m | 96 N | 28.9° | 10.5° |
   | 6.0 N m | 120 N | 37.3° | 13.2° |
   | 6.5 N m | 130 N | 41.0° | 14.4° |
   | 7.0 N m | 140 N | 45.0° | 15.5° |

   Below the stall angle the climb speed is reduced by the derating:
   `ω = ω_nl (1 − m g sin θ R / (4 τ_stall))`.
3. **Tipping**: `tan θ_tip = (L/2) / h_cg` longitudinally
   (0.40 / 0.37 → **47°**) and `(T/2) / h_cg` laterally (0.50 / 0.37 → **54°**);
   both far above the traction limit, so the rover slides before it tips.

Holding position with the command removed uses the same motors: the PI loop
holds zero wheel speed with up to τ_stall of braking torque, so the rover
holds on any slope below θ_stall and creeps back above it. There is no
separate parking brake.

## Step / boulder limits

For a rigid wheel of radius R against a rigid step of height h the hard
geometric limit is `h < R`: at `h = R` the edge normal is horizontal and no
push can lift the wheel. Below that, quasi-static equilibrium of the front
wheel at the edge (contact angle φ with `cos φ = (R − h)/R`, wheel load W_f,
edge normal N_e, edge friction T ≤ min(τ/R, μ N_e), horizontal push F from
the rear wheels) gives

```
x:  F = N_e sin φ − T cos φ
z:  W_f = N_e cos φ + T sin φ      =>   F = W_f tan φ − T / cos φ
```

so the push the rear wheels must supply falls linearly with the front
wheel's own edge torque. The rear pair can push at most `μ N_rear ≈ 0.65 ·
(m g / 2) = 64 N`. With `W_f ≈ m g / 4 = 49 N`:

| h | h/R | φ | push needed with τ = 4.8 N m | with τ = 6.5 N m | verdict |
| --- | --- | --- | --- | --- | --- |
| 0.10 m | 0.50 | 60.0° | 37 N | 20 N | climbs |
| 0.15 m | 0.75 | 75.5° | 94 N | 60 N | needs ≥ 6.1 N m (quasi-static) |
| 0.20 m | 1.00 | 90.0° | ∞ | ∞ | marginal: only dynamic/suspension effects |
| 0.25–0.40 m | > 1 | — | — | — | impossible; body (0.30 m clearance) also hits |

The 15 cm step therefore demands τ_stall ≳ 6 N m, while a torque-limited
slope stall at 28–30° would need ≈ 4.8 N m. The chosen compromise is
τ_stall = 6.5 N m with μ_eff = 0.65: the **slope limit is set by traction
(33°)** and the **step limit by torque and geometry** (reliable at 15 cm,
marginal at 20 cm, impossible at ≥ 25 cm). Momentum at the 0.5 m/s approach
speed and suspension compliance make the measured thresholds slightly more
permissive than the quasi-static numbers.

## Measured thresholds

MEASURED_SECTION

## Limits of the ODE model

- Rigid terrain and rigid wheels: no sinkage, no bulldozing, no
  slip-sinkage. Traction is a Coulomb friction pyramid with a single μ.
- The wheel is a sphere for collision (chosen in the wheel-shape comparison
  below); edge contacts are therefore point contacts without a tyre patch.
- Contact stiffness (`kp = 10⁶ N/m`, `kd = 100`, `min_depth = 2 mm`,
  `max_vel = 1 m/s`) and the quick solver (60 iterations, SOR 1.3, 1 ms
  step) were chosen for stability; they slightly soften impacts.
- The motor model is linear; no thermal limits, gearbox backlash or current
  limits.

## Wheel collision shape comparison

Measured 2026-10-17 (`wheel_collision:=` xacro/launch argument), headless,
after the strut-inertia fix. "Rest" is 5 s stationary on flat ground;
"flat drive" is 8 s at 0.5 m/s; "rough" is a 30 s drive at 0.5 m/s across a
14 m segment of the Mars heightmap with 7° mean / 12° maximum slope starting
at (−24, 28) m. a_z is the body vertical acceleration from ground truth.

| Collision | Rest z jitter (std) | Flat drive speed | Rough a_z RMS / peak | Rough pitch / roll range | Rough distance |
| --- | --- | --- | --- | --- | --- |
| **sphere** (default) | 2.4e-5 m | 0.495 m/s | 0.165 / 0.98 m/s² | 18.8° / 11.4° | 13.49 m |
| multisphere (3 spheres) | 5.8e-7 m | 0.495 m/s | 0.143 / 1.06 m/s² | 18.9° / 13.4° | 13.54 m |
| cylinder (full width) | 3.2e-7 m | 0.500 m/s | 0.187 / 1.22 m/s² | 18.9° / 15.8° | 13.50 m |
| band (2 cm cylinder) | 1.6e-7 m | 0.500 m/s | 0.213 / 1.26 m/s² | 18.7° / 14.9° | 13.50 m |

All four are stable (no tunnelling, no sliding at rest, drift < 0.2 mm in
5 s). Cylinders show the expected extra roughness against the heightfield
(a_z RMS +13–30 %). The sphere is kept: one analytic contact per wheel, the
lowest roughness after the three-sphere variant, and no cylinder-edge
scrub when steering in place. Before the fix, the strut's thin-cylinder
inertia (3e-4 kg m²) between the 33 kg body and the 4 kg fork/wheel made
every variant explode within 0.3 s; a 0.10 m cube inertia for the 1 kg
strut cured it, which is recorded here because it is the single most
important ODE stability lesson of this model.
