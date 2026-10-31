"""Gazebo Fortress (gz-sim 6, SDF 1.8) world generation.

Differences from the Classic worlds (``world.py``):

* gz-sim system plugins instead of Classic world plugins (Physics with DART,
  UserCommands, SceneBroadcaster, Sensors with ogre2, Imu);
* terrain collision as **tiled OBJ meshes** (float vertices) because Fortress
  reads neither 16-bit PNG nor GeoTIFF heightmaps correctly (measured:
  16-bit misread, GeoTIFF "unknown image format", 8-bit quantised to 6 cm);
  the visual is the same textured mesh as Classic;
* rocks reuse the chunk models; boulders reuse their models; friction tags
  are shared (sdformat).
"""
import math

import numpy as np

from rover_sim.terrain.world import MARS_GRAVITY, MODEL_URI, box_model as _classic_box, friction

GZ_SYSTEMS = '''    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics">
      <engine><filename>ignition-physics-dartsim-plugin</filename></engine>
    </plugin>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
'''


GUI_BLOCK = """    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></ignition-gui>
        <engine>ogre2</engine><scene>scene</scene>
        <ambient_light>0.5 0.45 0.4</ambient_light><background_color>0.80 0.60 0.44</background_color>
        <camera_pose>CAMERA_POSE</camera_pose>
      </plugin>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <ignition-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></ignition-gui>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <ignition-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></ignition-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <ignition-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></ignition-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <ignition-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></ignition-gui>
      </plugin>
      <plugin filename="MarkerManager" name="Marker manager">
        <ignition-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></ignition-gui>
      </plugin>
      <plugin filename="SelectEntities" name="Select Entities">
        <ignition-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></ignition-gui>
      </plugin>
      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
        <ignition-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></ignition-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <ignition-gui><title>World control</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">72</property>
          <property type="double" key="width">121</property><property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors></ignition-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">110</property>
          <property type="double" key="width">290</property><property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors></ignition-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations>
      </plugin>
      <!-- Native teleoperation: gz-sim's Teleop panel publishes ignition.msgs.Twist on /cmd_vel,
           bridged to the controller. Tick "Keyboard" and hold W/A/S/D or the arrow keys. -->
      <plugin filename="Teleop" name="Teleop">
        <ignition-gui><title>Rover teleop</title><property type="string" key="state">docked</property></ignition-gui>
        <topic>/cmd_vel</topic>
      </plugin>
    </gui>
"""


def gui_block(camera_pose):
    return GUI_BLOCK.replace('CAMERA_POSE', camera_pose)


# Measured on the Mars world (docs/fortress.md): DART with the Bullet collision
# detector at a 2 ms step holds RTF 0.98 while driving with 12 dynamic probes
# (ODE detector at 1 ms: 0.64), with identical rest height and traverse.
PHYSICS_STEP = 0.002
PHYSICS_COLLISION_DETECTOR = 'bullet'


def physics_block(step=PHYSICS_STEP, detector=PHYSICS_COLLISION_DETECTOR):
    return f'''    <physics name="{step*1000:g}ms" type="dart">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>{int(round(1.0/step))}</real_time_update_rate>
      <dart><collision_detector>{detector}</collision_detector></dart>
    </physics>
'''


def scene_block(shadows, mars=True):
    if mars:
        return f'''    <scene>
      <ambient>0.55 0.46 0.40 1</ambient>
      <background>0.80 0.60 0.44 1</background>
      <shadows>{'true' if shadows else 'false'}</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>{'true' if shadows else 'false'}</cast_shadows>
      <pose>0 0 60 0 0 0</pose>
      <diffuse>0.95 0.80 0.64 1</diffuse>
      <specular>0.08 0.07 0.06 1</specular>
      <direction>-0.62 0.45 -0.55</direction>
    </light>
'''
    return f'''    <scene>
      <ambient>0.5 0.45 0.4 1</ambient>
      <background>0.80 0.60 0.44 1</background>
      <shadows>{'true' if shadows else 'false'}</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>{'true' if shadows else 'false'}</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.95 0.85 0.75 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.3 -0.8</direction>
    </light>
'''


def write_collision_tiles(directory, height, size, tiles_per_side=6, step=2):
    """Write one OBJ (with normals; DART's ODE mesh loader dereferences them)
    per tile and return the (name, cx, cy) list."""
    directory.mkdir(parents=True, exist_ok=True)
    n = height.shape[0]
    half = size / 2.0
    cells = (n - 1) // tiles_per_side
    tiles = []
    for ti in range(tiles_per_side):
        for tj in range(tiles_per_side):
            r0, c0 = ti * cells, tj * cells
            rows = np.arange(r0, r0 + cells + 1, step)
            cols = np.arange(c0, c0 + cells + 1, step)
            if rows[-1] != r0 + cells:
                rows = np.append(rows, r0 + cells)
            if cols[-1] != c0 + cells:
                cols = np.append(cols, c0 + cells)
            sub = height[np.ix_(rows, cols)]
            m0, m1 = sub.shape
            xs = -half + cols * size / (n - 1)
            ys = half - rows * size / (n - 1)
            X, Y = np.meshgrid(xs, ys)
            cx, cy = float(X.mean()), float(Y.mean())
            V = np.stack([(X - cx).ravel(), (Y - cy).ravel(), sub.ravel()], axis=1)
            index = np.arange(m0 * m1).reshape(m0, m1)
            a, b, c, d = index[:-1, :-1], index[:-1, 1:], index[1:, :-1], index[1:, 1:]
            F = np.concatenate([np.stack([a, c, b], -1).reshape(-1, 3), np.stack([b, c, d], -1).reshape(-1, 3)])
            N = np.zeros_like(V)
            fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
            for k in range(3):
                np.add.at(N, F[:, k], fn)
            N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
            name = f'terrain_tile_{ti}_{tj}'
            (directory / f'{name}.obj').write_text(
                'o tile\n' + ''.join(f'v {v[0]:.3f} {v[1]:.3f} {v[2]:.3f}\n' for v in V)
                + ''.join(f'vn {q[0]:.4f} {q[1]:.4f} {q[2]:.4f}\n' for q in N)
                + ''.join(f'f {f[0]+1}//{f[0]+1} {f[1]+1}//{f[1]+1} {f[2]+1}//{f[2]+1}\n' for f in F))
            tiles.append((name, cx, cy, int(len(F))))
    return tiles


def terrain_models(tiles, mu=0.65):
    parts = ['    <model name="mars_terrain"><static>true</static><link name="terrain">\n'
             f'      <visual name="visual"><geometry><mesh><uri>{MODEL_URI}/meshes/terrain_visual.obj</uri></mesh></geometry></visual>\n']
    for name, cx, cy, _ in tiles:
        parts.append(f'      <collision name="{name}"><pose>{cx:.4f} {cy:.4f} 0 0 0 0</pose><geometry><mesh>'
                     f'<uri>{MODEL_URI}/meshes/collision/{name}.obj</uri></mesh></geometry>{friction(mu)}</collision>\n')
    parts.append('    </link></model>\n')
    return ''.join(parts)


def mars_world(name, tiles, rocks_sdf, boulders_sdf, gravity=MARS_GRAVITY, shadows=False,
               camera_pose='-14 -14 12 0 0.45 0.78'):
    text = (f'<?xml version="1.0"?>\n<!-- Generated by rover_sim/scripts/generate_terrain.py (Gazebo Fortress); do not edit. -->\n'
            f'<sdf version="1.8">\n  <world name="{name}">\n'
            f'    <gravity>0 0 {-gravity}</gravity>\n' + physics_block() + GZ_SYSTEMS + scene_block(shadows)
            + gui_block(camera_pose)
            + terrain_models(tiles) + rocks_sdf + '\n' + boulders_sdf + '\n  </world>\n</sdf>\n')
    return text


def ground_plane(size=80.0, mu=0.65, colour='0.76 0.56 0.38'):
    return f'''    <model name="ground"><static>true</static><link name="ground">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>{friction(mu)}</collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
        <material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse></material></visual>
    </link></model>
'''


def flat_world(name, models, gravity=MARS_GRAVITY, shadows=False, camera_pose='-6 -10 6 0 0.4 1.0', size=80.0):
    return (f'<?xml version="1.0"?>\n<!-- Generated by rover_sim/scripts/generate_terrain.py (Gazebo Fortress); do not edit. -->\n'
            f'<sdf version="1.8">\n  <world name="{name}">\n'
            f'    <gravity>0 0 {-gravity}</gravity>\n' + physics_block() + GZ_SYSTEMS + scene_block(shadows, mars=False)
            + gui_block(camera_pose)
            + ground_plane(size) + ''.join(models) + '  </world>\n</sdf>\n')


def step_test_world(gravity=MARS_GRAVITY):
    from rover_sim.terrain.world import STEP_LENGTH, STEP_WIDTH, STEP_X_START, box_model, step_lanes
    models = [box_model(f'step_{int(round(h * 100)):02d}cm', STEP_X_START + STEP_LENGTH / 2, y, h / 2,
                        STEP_LENGTH, STEP_WIDTH, h, colour='0.5 0.47 0.44') for h, y in step_lanes()]
    return flat_world('step_test', models, gravity, camera_pose='-8 -22 10 0 0.5 0.9')


def slope_test_world(gravity=MARS_GRAVITY):
    from rover_sim.terrain.world import RAMP_ANGLES, box_model, ramp_geometry, ramp_lanes
    models = []
    for angle, y in ramp_lanes():
        (cx, cz, length, width, thickness, theta), (tx, tz, tl, tw, tt), _ = ramp_geometry(angle)
        models.append(box_model(f'ramp_{angle:02d}deg', cx, y, cz, length, width, thickness, pitch=-theta, colour='0.52 0.45 0.40'))
        models.append(box_model(f'ramp_{angle:02d}deg_top', tx, y, tz, tl, tw, tt, colour='0.45 0.42 0.40'))
    return flat_world('slope_test', models, gravity, camera_pose='-10 -26 14 0 0.55 0.9', size=100.0)


def boulder_garden_world(boulders_sdf, gravity=MARS_GRAVITY):
    return flat_world('boulder_garden', [boulders_sdf + '\n'], gravity, camera_pose='4 -14 8 0 0.5 1.3')
