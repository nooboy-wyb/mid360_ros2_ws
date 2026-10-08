#!/usr/bin/env python3
"""Create a Gazebo Classic test world with TurtleBot3 Waffle Pi and MID360."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


workspace = Path(__file__).resolve().parents[1]
scene = sys.argv[2] if len(sys.argv) > 2 else 'house'
if scene not in ('house', 'lab', 'box'):
    raise SystemExit('Scene must be house, lab, or box')
source = (Path('/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world')
          if scene == 'house' else workspace / 'sim/worlds/mesh_lab.sdf')
root = ET.parse(source).getroot()
root.set('version', '1.6')
world = root.find('world')
world.find('physics').set('type', 'ode')
for plugin in list(world.findall('plugin')):
    world.remove(plugin)
# Publish Gazebo world-state ground truth independently of wheel odometry.
state_plugin = ET.SubElement(world, 'plugin', {
    'name': 'gazebo_ros_state', 'filename': 'libgazebo_ros_state.so'
})
state_ros = ET.SubElement(state_plugin, 'ros')
ET.SubElement(state_ros, 'namespace').text = '/ground_truth'
ET.SubElement(state_ros, 'remapping').text = 'model_states:=model_states'
ET.SubElement(state_ros, 'remapping').text = 'link_states:=link_states'
ET.SubElement(state_plugin, 'update_rate').text = '50.0'
if scene == 'box':
    # Empty 40 m square enclosure. Each wall is a real collision box as well
    # as a visual, so it provides a simple ray-intersection test target.
    for child in list(world):
        if child.tag in ('model', 'include', 'light'):
            world.remove(child)
    floor = ET.SubElement(world, 'model', {'name': 'test_floor'})
    ET.SubElement(floor, 'static').text = 'true'
    floor_link = ET.SubElement(floor, 'link', {'name': 'floor_link'})
    ET.SubElement(floor_link, 'pose').text = '0 0 -0.05 0 0 0'
    for kind in ('collision', 'visual'):
        item = ET.SubElement(floor_link, kind, {'name': f'floor_{kind}'})
        item.append(ET.fromstring('<geometry><box><size>42 42 0.1</size></box></geometry>'))
    walls = [
        ('wall_north', '0 20.1 2 0 0 0', '40.4 0.2 4'),
        ('wall_south', '0 -20.1 2 0 0 0', '40.4 0.2 4'),
        ('wall_east', '20.1 0 2 0 0 0', '0.2 40 4'),
        ('wall_west', '-20.1 0 2 0 0 0', '0.2 40 4'),
    ]
    for name, pose, size in walls:
        wall = ET.SubElement(world, 'model', {'name': name})
        ET.SubElement(wall, 'static').text = 'true'
        ET.SubElement(wall, 'pose').text = pose
        link = ET.SubElement(wall, 'link', {'name': 'wall_link'})
        for kind in ('collision', 'visual'):
            item = ET.SubElement(link, kind, {'name': f'wall_{kind}'})
            item.append(ET.fromstring(f'<geometry><box><size>{size}</size></box></geometry>'))
        visual = link.find('visual')
        ET.SubElement(visual, 'material').extend([
            ET.fromstring('<ambient>0.7 0.7 0.7 1</ambient>'),
            ET.fromstring('<diffuse>0.7 0.7 0.7 1</diffuse>'),
        ])
    light = ET.SubElement(world, 'light', {'name': 'sun', 'type': 'directional'})
    ET.SubElement(light, 'pose').text = '0 0 15 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.9 0.9 0.9 1'
    ET.SubElement(light, 'direction').text = '-0.3 -0.4 -0.8'
elif scene == 'house':
    for include in list(world.findall('include')):
        world.remove(include)
    floor = ET.SubElement(world, 'model', {'name': 'house_floor'})
    ET.SubElement(floor, 'static').text = 'true'
    floor_link = ET.SubElement(floor, 'link', {'name': 'floor_link'})
    ET.SubElement(floor_link, 'pose').text = '0 0 -0.05 0 0 0'
    for kind in ('collision', 'visual'):
        item = ET.SubElement(floor_link, kind, {'name': f'floor_{kind}'})
        item.append(ET.fromstring('<geometry><box><size>30 30 0.1</size></box></geometry>'))
    light = ET.SubElement(world, 'light', {'name': 'sun', 'type': 'directional'})
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.9 0.9 0.9 1'
    ET.SubElement(light, 'direction').text = '-0.3 -0.4 -0.8'
    world.remove(world.find("model[@name='turtlebot3_house']"))
    house_source = Path('/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_house/model.sdf')
    house = ET.parse(house_source).getroot().find('model')
    ET.SubElement(house, 'static').text = 'true'
    world.append(house)
else:
    world.remove(world.find("model[@name='mid360_agv']"))
waffle_source = Path('/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf')
model = ET.parse(waffle_source).getroot().find('model')
model.find('pose').text = ('-2 -0.5 0.015 0 0 0' if scene == 'house'
                           else ('0 0 0.015 0 0 0' if scene == 'box'
                                 else '0 -2 0.015 0 0 0'))
world.append(model)
# Keep Waffle Pi's chassis, four contact points, wheels, and drive plugin.
# Remove its stock LDS and stock IMU; the MID360 and IMU share a new mount.
stock_lidar = model.find("link[@name='base_scan']")
for element in list(stock_lidar):
    if element.tag in ('sensor', 'visual', 'collision'):
        stock_lidar.remove(element)
model.find("link[@name='imu_link']").remove(model.find("link[@name='imu_link']/sensor"))
camera = model.find(".//sensor[@name='camera']")
if camera is not None:
    for link in model.findall('link'):
        if camera in link:
            link.remove(camera)
            break
drive = model.find("plugin[@name='turtlebot3_diff_drive']")
drive.find('publish_odom_tf').text = 'false'
drive.find('odometry_topic').text = '/wheel_odom_debug'
drive.find('odometry_frame').text = 'wheel_odom_debug'
mount = ET.SubElement(model, 'link', {'name': 'mid360_mount'})
# Put the MID360 housing on the Waffle Pi upper deck. The deck is at z~=0.092 m
# in base_link; the 0.07 m housing therefore has its center at z=0.127 m.
# Keep LiDAR and IMU co-located.
ET.SubElement(mount, 'pose').text = '0 0 0.127 0 0 0'
ET.SubElement(mount, 'inertial').extend([
    ET.fromstring('<mass>0.25</mass>'),
    ET.fromstring('<inertia><ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz></inertia>'),
])
ET.SubElement(mount, 'visual', {'name': 'mid360_housing'}).append(
    ET.fromstring('<geometry><cylinder><radius>0.05</radius><length>0.07</length></cylinder></geometry>')
)
joint = ET.SubElement(model, 'joint', {'name': 'mid360_mount_joint', 'type': 'fixed'})
ET.SubElement(joint, 'parent').text = 'base_link'
ET.SubElement(joint, 'child').text = 'mid360_mount'
sensor = ET.SubElement(mount, 'sensor', {'name': 'mid360_pattern', 'type': 'ray'})
ET.SubElement(sensor, 'always_on').text = 'true'
# The plugin fires subsets of a 10 Hz scan at each physics-scale callback.
ET.SubElement(sensor, 'update_rate').text = '1000'
ET.SubElement(sensor, 'visualize').text = 'false'
plugin = ET.SubElement(sensor, 'plugin', {'name': 'mid360_plugin', 'filename': 'libros2_livox.so'})
ET.SubElement(plugin, 'ray').append(ET.fromstring('''<scan><horizontal><samples>100</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>6.2831853</max_angle></horizontal><vertical><samples>360</samples><resolution>1</resolution><min_angle>-0.1260</min_angle><max_angle>0.9638</max_angle></vertical></scan>'''))
max_range = 30 if scene == 'box' else 12
ET.SubElement(plugin.find('ray'), 'range').extend([
    ET.fromstring('<min>0.2</min>'), ET.fromstring(f'<max>{max_range}</max>'), ET.fromstring('<resolution>0.01</resolution>')
])
ET.SubElement(plugin, 'samples').text = '4000'
ET.SubElement(plugin, 'downsample').text = '1'
ET.SubElement(plugin, 'scan_period_sec').text = '0.1'
ET.SubElement(plugin, 'raycast_update_rate').text = '1000'
ET.SubElement(plugin, 'csv_file_name').text = str(workspace / 'src/ros2_livox_simulation/scan_mode/mid360.csv')
ET.SubElement(plugin, 'topic').text = '/mid360_plugin/points'
imu = ET.SubElement(mount, 'sensor', {'name': 'mid360_imu', 'type': 'imu'})
ET.SubElement(imu, 'always_on').text = 'true'
ET.SubElement(imu, 'update_rate').text = '100'
imu_data = ET.SubElement(imu, 'imu')
for quantity in ('angular_velocity', 'linear_acceleration'):
    axes = ET.SubElement(imu_data, quantity)
    for axis in 'xyz':
        ET.SubElement(ET.SubElement(axes, axis), 'noise', {'type': 'gaussian'}).extend([
            ET.fromstring('<mean>0</mean>'), ET.fromstring('<stddev>0.001</stddev>')
        ])
ET.SubElement(imu, 'plugin', {'name': 'mid360_imu_ros', 'filename': 'libgazebo_ros_imu_sensor.so'})
ET.SubElement(imu.find('plugin'), 'ros').append(ET.fromstring('<remapping>~/out:=/mid360_sim/imu</remapping>'))
output = Path(sys.argv[1])
ET.indent(root)
ET.ElementTree(root).write(output, encoding='unicode', xml_declaration=True)
