"""Deterministic, asymmetric Gazebo arena and its prebuilt occupancy map."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

# Geometry and map share one definition. These are fixture landmarks, not pose seeds.
BOXES = [
    (0, -4, 10.2, .2), (0, 4, 10.2, .2),
    (-5, 0, .2, 8), (5, 0, .2, 8),
    (-1.6, 1.5, .5, 2), (2.4, -.7, 1.2, .5),
    (3.6, 2.6, .7, .7), (-3.2, -2.4, 1, .6),
]
GOALS = {'east': [1.0, -2.0, 0.0], 'north': [1.0, 2.4, 0.0],
         'home': [-2.5, 0.0, 0.0]}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c*x-s*y, s*x+c*y


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def clearance(x, y):
    return min(math.hypot(max(abs(x-bx)-sx/2, 0),
                          max(abs(y-by)-sy/2, 0)) for bx, by, sx, sy in BOXES)


def make_arena(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    models = []
    for i, (x, y, sx, sy) in enumerate(BOXES + [(0, 0, 11, 9)]):
        floor = i == len(BOXES)
        z, height = (-.05, .1) if floor else (.5, 1.)
        geometry = f'<geometry><box><size>{sx} {sy} {height}</size></box></geometry>'
        models.append(f'''<model name="box_{i}"><static>true</static>
          <pose>{x} {y} {z} 0 0 0</pose><link name="body">
          <collision name="collision">{geometry}</collision>
          <visual name="visual">{geometry}</visual></link></model>''')
    plugins = ''.join(f'<plugin filename="libignition-gazebo-{file}-system.so" '
                      f'name="ignition::gazebo::systems::{name}"/>'
                      for file, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                                         ('scene-broadcaster', 'SceneBroadcaster'), ('imu', 'Imu')])
    (directory/'arena.sdf').write_text(f'''<sdf version="1.7"><world name="default">
      <physics name="physics" type="ignored"><max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor></physics>{plugins}
      <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>{''.join(models)}</world></sdf>''')
    resolution, width, height = .05, 208, 168
    ox, oy = -5.2, -4.2
    pixels = bytearray()
    for row in range(height):
        y = oy+(height-row-.5)*resolution
        for col in range(width):
            x = ox+(col+.5)*resolution
            pixels.append(0 if clearance(x, y) == 0 else 254)
    (directory/'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels)
    (directory/'map.yaml').write_text(yaml.safe_dump(dict(
        image='map.pgm', mode='trinary', resolution=resolution, origin=[ox, oy, 0.],
        negate=0, occupied_thresh=.65, free_thresh=.25)))
    (directory/'landmarks.yaml').write_text(yaml.safe_dump({'frame': 'map', 'goals': GOALS}))


def make_robot(description_dir):
    from xmacro.xmacro4sdf import XMLMacro4sdf
    from sdformat_tools.urdf_generator import UrdfGenerator
    macro = XMLMacro4sdf()
    macro.set_xml_file(str(Path(description_dir)/'resource/xmacro/simulation_robot.sdf.xmacro'))
    macro.generate({'global_initial_color': 'red'})
    root = ET.fromstring(macro.to_string())
    model = root.find('model')
    # The light-bar plugin crashes Fortress's headless renderer. Omit cosmetics/fire.
    for plugin in list(model.findall('plugin')):
        if plugin.get('filename') in ('LightBarController', 'ProjectileShooter'):
            model.remove(plugin)
    for link in model.findall('link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('name') not in ('front_rplidar_a2', 'gimbal_imu'):
                link.remove(sensor)
    sdf = ET.tostring(root, encoding='unicode')
    generator = UrdfGenerator()
    generator.parse_from_sdf_string(sdf)
    return sdf, generator.to_string()
