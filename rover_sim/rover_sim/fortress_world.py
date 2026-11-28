"""Shared Gazebo Fortress (gz-sim 6) world blocks.

Every rover_sim world (worlds/*.world, scripts/generate_mars_world.py and
rover_planner's slope_test world writer) is built from these, so physics,
systems and GUI stay identical across worlds:

* ``SYSTEMS``: the gz-sim world systems. Classic loaded physics, rendering and
  sensors implicitly; Fortress loads only what the world lists.
* ``physics_block``: DART (Fortress's default engine), 1 ms step as on
  Classic. The collision detector is set explicitly; see PHYSICS_DETECTOR.
* ``gui_block``: the GUI layout. A world without a <gui> gets gz-sim's default
  GUI; one with a <gui> gets exactly the plugins listed, so the full set is
  here (3D view, world control/stats, entity tree, camera follow, teleop).
* ``pbr``: a textured material. Fortress's ogre2 ignores Classic OGRE
  material scripts (<material><script>), so textures are PBR albedo maps.

URIs: ``model://rover_sim_materials/...`` resolves through
IGN_GAZEBO_RESOURCE_PATH, which the launch file sets to the package's models
folder.
"""

PHYSICS_STEP = 0.001
# DART collision detector. On the Mars world with the real rover (no sensors)
# Bullet and DART's default ODE detector both ran at RTF 1.00 and gave the same
# rest height (0.346 m) and drive (7.36 m in 15 s at 0.5 m/s), 2026-11-21.
# Bullet is kept because it was clearly faster with many dynamic bodies on
# mesh terrain (jakkiiri's feat/mars-overhaul measurements: RTF 0.98 vs 0.64).
PHYSICS_DETECTOR = 'bullet'

# SceneBroadcaster sends the GUI the scene at 30 Hz instead of its default 60:
# it runs in the sim loop, and the usgs world has hundreds of terrain chunks
# and rocks (docs/fortress.md).
SYSTEMS = '''    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics">
      <engine><filename>ignition-physics-dartsim-plugin</filename></engine>
    </plugin>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster">
      <state_hertz>30</state_hertz><dynamic_pose_hertz>30</dynamic_pose_hertz>
    </plugin>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
'''


def physics_block(step=PHYSICS_STEP, detector=PHYSICS_DETECTOR):
    return f'''    <physics name="{step * 1000:g}ms" type="dart">
      <max_step_size>{step:g}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>{int(round(1.0 / step))}</real_time_update_rate>
      <dart><collision_detector>{detector}</collision_detector></dart>
    </physics>
'''


def _floating(name, filename, extra=''):
    return f'''      <plugin filename="{filename}" name="{name}">
        <ignition-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property>
          <property key="resizable" type="bool">false</property></ignition-gui>{extra}
      </plugin>
'''


def gui_block(camera_pose, background='0.83 0.66 0.50'):
    """GUI with the 3D view looking from ``camera_pose`` ("x y z roll pitch yaw")."""
    hidden = ''.join(_floating(n, f) for n, f in (
        ('Scene Manager', 'GzSceneManager'), ('Interactive view control', 'InteractiveViewControl'),
        ('Camera Tracking', 'CameraTracking'), ('Marker manager', 'MarkerManager'),
        ('Select Entities', 'SelectEntities'), ('Visualization Capabilities', 'VisualizationCapabilities'),
        ('Entity context menu', 'EntityContextMenuPlugin')))
    return f'''    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></ignition-gui>
        <engine>ogre2</engine><scene>scene</scene>
        <ambient_light>0.6 0.55 0.5</ambient_light><background_color>{background}</background_color>
        <camera_pose>{camera_pose}</camera_pose>
      </plugin>
{hidden}      <plugin filename="WorldControl" name="World control">
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
      <plugin filename="EntityTree" name="Entity tree">
        <ignition-gui><property type="string" key="state">docked_collapsed</property></ignition-gui>
      </plugin>
      <!-- gz-sim's Teleop panel publishes ignition.msgs.Twist on /cmd_vel (bridged
           to ROS by the launch file). Tick "Keyboard" and hold W/A/S/D. -->
      <plugin filename="Teleop" name="Teleop">
        <ignition-gui><title>Rover teleop</title><property type="string" key="state">docked_collapsed</property></ignition-gui>
        <topic>/cmd_vel</topic>
      </plugin>
    </gui>
'''


def pbr(albedo_uri, roughness=0.9, indent='          '):
    """PBR material with an albedo texture (replaces a Classic material script)."""
    return (f'{indent}<material><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>'
            f'<pbr><metal><albedo_map>{albedo_uri}</albedo_map><roughness>{roughness}</roughness>'
            f'<metalness>0</metalness></metal></pbr></material>')


TEXTURES = 'model://rover_sim_materials/materials/textures'
