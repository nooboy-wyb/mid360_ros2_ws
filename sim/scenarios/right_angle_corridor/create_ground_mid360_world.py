#!/usr/bin/env python3
"""Create the corridor with a standalone, static MID360 at floor level."""
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

workspace = Path(__file__).resolve().parents[3]
out = Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory(prefix='mid360_ground_test_') as tmp:
    base = Path(tmp) / 'corridor.sdf'
    subprocess.run([
        sys.executable,
        str(workspace / 'sim/scenarios/right_angle_corridor/create_world.py'),
        str(base),
    ], check=True)
    root = ET.parse(base).getroot()

world = root.find('world')
robot = world.find("model[@name='turtlebot3_waffle_pi']")
if robot is None:
    raise RuntimeError('Corridor world has no TurtleBot model to extract the MID360 sensor from')
mount = robot.find("link[@name='mid360_mount']")
if mount is None:
    raise RuntimeError('TurtleBot model has no MID360 mount link')

# Keep only the corridor and ground truth state plugin. The standalone model,
# MID360 link, and sensor origin all use z=0. The 7 cm housing sits on the
# floor with its visual center 3.5 cm above the sensor origin.
world.remove(robot)
standalone = ET.SubElement(world, 'model', {'name': 'mid360_standalone'})
ET.SubElement(standalone, 'static').text = 'true'
ET.SubElement(standalone, 'pose').text = '0 0 0 0 0 0'
mount.attrib.pop('pose', None)
link_pose = mount.find('pose')
if link_pose is not None:
    link_pose.text = '0 0 0 0 0 0'
for child in list(mount):
    if child.tag in ('inertial', 'collision'):
        mount.remove(child)
visual = mount.find("visual[@name='mid360_housing']")
if visual is not None:
    ET.SubElement(visual, 'pose').text = '0 0 0.035 0 0 0'
standalone.append(mount)

ET.indent(root)
out.parent.mkdir(parents=True, exist_ok=True)
ET.ElementTree(root).write(out, encoding='unicode', xml_declaration=True)
print(out)
