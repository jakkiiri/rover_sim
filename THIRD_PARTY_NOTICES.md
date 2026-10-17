# Upstream model provenance

Repository: https://github.com/Kettenhoax/gazebo_ros_four_wheel_steering

Exact commit: `9802a276510915126a706df640cb90c42d1b2788`

Upstream paths: `urdf/wheel_steered.urdf.xacro`,
`urdf/primitives.urdf.xacro`, `urdf/inertia_tensors.urdf.xacro`.
Local copies are under `urdf/upstream/`.

License: Apache-2.0, per upstream README License and Copyright and package.xml.
Copyright holder for files without their own marking: 2020 AIT Austrian
Institute of Technology GmbH. See `LICENSES/Apache-2.0.txt`.
Existing comments/attributions are preserved, including the inertia helper
attribution to Martin Guenther. These three files contain no explicit license
headers upstream. No upstream controller code is included.

## Every local modification

- `wheel_steered.urdf.xacro`: change the primitive include to the relative path
  `primitives.urdf.xacro`, removing the upstream package-prefix argument.
- `wheel_steered.urdf.xacro`: replace kingpin dynamics `friction="0"` with
  `friction="0.02" damping="0.1"` for low passive drag under active steering control.
- `wheel_steered.urdf.xacro`: move steering-link inertia origin from
  `0 ${-0.1 * left_right} 0` to `0 0 0`, centering it on the steering axis.
- `wheel_steered.urdf.xacro`: change the kingpin from revolute to continuous,
  remove the +/-0.6 rad limits, reduce effort limit from 4000 to 25 Nm and
  increase velocity limit from 1 to 3 rad/s for full-angle swerve steering.
- `primitives.urdf.xacro`: unchanged.
- `wheel_steered.urdf.xacro`: Phase 4.5 adds a prominent in-file modification
  notice pointing to this change list. No additional mechanical changes.
- `inertia_tensors.urdf.xacro`: Phase 4.5 adds a prominent local-change notice
  and inherited BSD attribution/license references. All formulas/macros remain
  unchanged; original Martin Guenther attribution is preserved.

## Inherited UOS inertia helpers (BSD)

The original inertia comment links `uos/uos_tools`, `uos_common_urdf`.
Historical revision `e5088e7404bb0f096363982018e68bc0269df927` (`fuerte` at
investigation) contains `uos_common_urdf/common.xacro`; its `manifest.xml`
declares BSD and names Jochen Sprickerhof and Martin Günther as authors.
The historical tree contains no standalone LICENSE/COPYING file, so no
historical copyright year or precise BSD variant is invented here.

The same six sphere/cylinder/box inertia helpers survive at revision
`e0cc5d44d8bc60975a2a2357a606655c4f46431c`, under
`uos_common_urdf/urdf/common.xacro`. That repository publishes a BSD-3-Clause
license, Copyright (c) 2020, Osnabrück University. Its complete original text is
preserved verbatim in `LICENSES/UOS-BSD.txt`; this is a published 2020 notice,
not a claim that the helpers were first authored in 2020. The license was added
by commit `bbad716ab55ec169ca030b36cb8f85f62b623f82` on 2020-04-27.
Preserve this inherited notice alongside the Apache-2.0 AIT adaptation notice.
The package manifest records both licenses for the corresponding portions.

Sources inspected on 2026-10-10:
- https://github.com/uos/uos_tools/blob/e5088e7404bb0f096363982018e68bc0269df927/uos_common_urdf/manifest.xml
- https://github.com/uos/uos_tools/blob/e5088e7404bb0f096363982018e68bc0269df927/uos_common_urdf/common.xacro
- https://github.com/uos/uos_tools/blob/e0cc5d44d8bc60975a2a2357a606655c4f46431c/uos_common_urdf/urdf/common.xacro
- https://github.com/uos/uos_tools/blob/e0cc5d44d8bc60975a2a2357a606655c4f46431c/LICENSE

## Local assembly and differences from the upstream complete vehicle

`urdf/proxy_rover.urdf.xacro` is a new local assembly, not a copy of the
complete upstream test vehicle. It uses the vendored steering, geometry and
inertia macros. It overrides steering-link mass from 10 to 1 kg and sphere
radius from 0.1 to 0.06 m. The steering inertia is centered on the link.
It adds a 20 kg, 1.0 x 0.5 x 0.2 m chassis and four 1 kg cylindrical wheels,
radius 0.18 m and width 0.12 m, attached by continuous drive joints.
Wheel steering axes are at x=+/-0.38, y=+/-0.45, z=-0.15 m; wheels are
centered on their kingpins, eliminating steering scrub offset. In Phase 3 wheel visuals remain 0.12 m wide, while the cylindrical collision
tread is narrowed to a centered 0.02 m contact band at the same 0.18 m radius
to approximate rounded tires and reduce ODE scrub during center rotation.
Wheel inertia still uses the full visible tire dimensions.
The wheel-frame rotation aligns the cylinder and spin axis with the axle.
Local materials, contact settings and passive wheel joint friction (0.02 Nm)
and damping (0.05) are added. Phase 2 passive braking was reduced for active drive control; steering is
continuous in Phase 3.
The coupled axle steering rods/gears and controller are omitted, leaving four
independent kingpins and four drive joints. Phase 3 adds a local ROS swerve controller and a local Gazebo Classic joint
actuator plugin. No upstream controller code, teleop, sensors, or control
transmissions are included.

A massless, coincident ROS root and fixed joint avoid KDL root-link-inertia
warnings without changing spawn coordinates. Phase 4.5 renames the misleading
`base_footprint`/`base_footprint_joint` to `base_root`/`base_root_joint`; this
dummy frame does not represent a projected ground footprint. A configurable
`actuator_watchdog` Xacro argument is passed to the local actuator plugin.
