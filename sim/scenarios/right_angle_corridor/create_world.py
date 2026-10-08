#!/usr/bin/env python3
"""Build an isolated L-shaped corridor scene around the existing MID360 robot."""
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

workspace = Path(__file__).resolve().parents[3]
out = Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory(prefix='mid360_l_corridor_') as tmp:
    base = Path(tmp) / 'base.sdf'
    subprocess.run([sys.executable, str(workspace / 'sim/create_classic_world.py'), str(base), 'lab'], check=True)
    root = ET.parse(base).getroot()

world = root.find('world')
robot = world.find("model[@name='turtlebot3_waffle_pi']")
if robot is None:
    raise RuntimeError('The base world did not contain turtlebot3_waffle_pi')
robot.find('pose').text = '-3.5 -2 0.015 0 0 0'

# Keep world physics, gravity, the Gazebo state plugin and the robot only.
for child in list(world):
    if child is robot or child.tag in ('physics', 'gravity', 'plugin'):
        continue
    world.remove(child)


def add_static_box(name, x, y, z, sx, sy, sz, color=(0.68, 0.70, 0.74, 1.0)):
    model = ET.SubElement(world, 'model', {'name': name})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 0'
    link = ET.SubElement(model, 'link', {'name': f'{name}_link'})
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, {'name': f'{name}_{kind}'})
        item.append(ET.fromstring(f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'))
    visual = link.find('visual')
    material = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(material, tag).text = ' '.join(map(str, color))


def add_cylinder(name, x, y, radius, height, color):
    model = ET.SubElement(world, 'model', {'name': name})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} {height / 2} 0 0 0'
    link = ET.SubElement(model, 'link', {'name': f'{name}_link'})
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, {'name': f'{name}_{kind}'})
        item.append(ET.fromstring(f'<geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>'))
    material = ET.SubElement(link.find('visual'), 'material')
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(material, tag).text = ' '.join(map(str, color))


# L corridor: horizontal centerline y=-2, then vertical centerline x=0.5.
add_static_box('corridor_floor', -1.5, 0, -0.075, 12.4, 12.4, 0.15, (0.45, 0.45, 0.45, 1))
wall_z = 1.5
walls = [
    # Horizontal arm x=[-5.5, 2.5], y=[-4, 0].
    ('wall_south', -1.5, -4.1, wall_z, 8.2, 0.2, 3.0),
    ('wall_west_end', -5.6, -2.0, wall_z, 0.2, 4.2, 3.0),
    ('wall_horizontal_north', -3.55, 0.1, wall_z, 4.1, 0.2, 3.0),
    # Vertical arm x=[-1.5, 2.5], y=[-4, 4].
    ('wall_vertical_west', -1.6, 2.05, wall_z, 0.2, 4.1, 3.0),
    ('wall_east', 2.6, 0.0, wall_z, 0.2, 8.2, 3.0),
    ('wall_north_end', 0.5, 4.1, wall_z, 4.2, 0.2, 3.0),
]
for wall in walls:
    add_static_box(*wall)

# Asymmetric landmarks visible from both legs without blocking the centerline.
add_cylinder('landmark_cylinder_1', -3.1, -0.95, 0.20, 1.45, (0.9, 0.2, 0.15, 1))
add_cylinder('landmark_cylinder_2', -1.0, -3.05, 0.28, 1.9, (0.15, 0.45, 0.9, 1))
add_cylinder('landmark_cylinder_3', 1.65, -0.8, 0.16, 1.15, (0.2, 0.8, 0.3, 1))
add_cylinder('landmark_cylinder_4', 1.6, 1.05, 0.25, 1.7, (0.9, 0.65, 0.1, 1))
add_cylinder('landmark_cylinder_5', -0.65, 3.0, 0.18, 1.35, (0.65, 0.25, 0.8, 1))
light = ET.SubElement(world, 'light', {'name': 'corridor_sun', 'type': 'directional'})
ET.SubElement(light, 'pose').text = '0 0 12 0 0 0'
ET.SubElement(light, 'diffuse').text = '0.9 0.9 0.9 1'
ET.SubElement(light, 'direction').text = '-0.3 -0.4 -0.8'

ET.indent(root)
out.parent.mkdir(parents=True, exist_ok=True)
ET.ElementTree(root).write(out, encoding='unicode', xml_declaration=True)
print(out)
