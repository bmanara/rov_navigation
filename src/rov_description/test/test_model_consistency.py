"""Checks that the Gazebo model, the URDF and the allocator config agree.

These are the three places thruster geometry shows up; if they drift apart the
allocator computes thrusts for a vehicle that isn't the one being simulated.
Also sanity-checks buoyancy (must be slightly positive).
"""
import math
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(PKG, 'models', 'rov', 'model.sdf')
URDF = os.path.join(PKG, 'urdf', 'rov.urdf')
YAML = os.path.join(PKG, 'config', 'thrusters.yaml')


def vec(text):
    return [float(v) for v in text.split()]


@pytest.fixture(scope='module')
def params():
    with open(YAML) as f:
        return yaml.safe_load(f)['thruster_allocator']['ros__parameters']


@pytest.fixture(scope='module')
def sdf_model():
    return ET.parse(SDF).getroot().find('model')


@pytest.fixture(scope='module')
def urdf_robot():
    return ET.parse(URDF).getroot()


def test_directions_are_unit(params):
    for name in params['thruster_names']:
        d = params[name]['direction']
        assert math.isclose(math.sqrt(sum(c * c for c in d)), 1.0, abs_tol=1e-6), name


def test_sdf_thrusters_match_yaml(params, sdf_model):
    links = {l.get('name'): l for l in sdf_model.findall('link')}
    joints = {j.get('name'): j for j in sdf_model.findall('joint')}
    for name in params['thruster_names']:
        pose = vec(links[name].find('pose').text)
        assert pose[:3] == pytest.approx(params[name]['position'], abs=1e-6), name
        assert pose[3:] == pytest.approx([0, 0, 0], abs=1e-9), \
            f'{name}: link must be unrotated so the joint axis is in base_link frame'
        joint = joints[f'{name}_joint']
        assert joint.find('parent').text == 'base_link'
        axis = vec(joint.find('axis').find('xyz').text)
        assert axis == pytest.approx(params[name]['direction'], abs=1e-6), name


def test_sdf_thruster_limits_match_yaml(params, sdf_model):
    plugins = [p for p in sdf_model.findall('plugin')
               if p.get('name') == 'gz::sim::systems::Thruster']
    assert len(plugins) == len(params['thruster_names'])
    for p in plugins:
        assert p.find('namespace').text == 'rov'
        assert float(p.find('max_thrust_cmd').text) == pytest.approx(params['max_forward_thrust'])
        assert float(p.find('min_thrust_cmd').text) == pytest.approx(-params['max_reverse_thrust'])
        assert p.find('use_angvel_cmd').text.strip().lower() == 'false'


def test_command_topics_match_gazebo_default(params):
    # Thruster system default topic: /model/<namespace>/joint/<joint>/cmd_thrust
    pattern = params['command_topic_pattern']
    assert pattern.format(name='thruster1') == '/model/rov/joint/thruster1_joint/cmd_thrust'


def test_urdf_thruster_frames_match_yaml(params, urdf_robot):
    joints = {j.find('child').get('link'): j for j in urdf_robot.findall('joint')}
    for name in params['thruster_names']:
        origin = vec(joints[name].find('origin').get('xyz'))
        assert origin == pytest.approx(params[name]['position'], abs=1e-6), name


def test_urdf_camera_matches_sdf(urdf_robot, sdf_model):
    joints = {j.find('child').get('link'): j for j in urdf_robot.findall('joint')}
    cam = [s for s in sdf_model.iter('sensor') if s.get('type') == 'camera'][0]
    assert vec(joints['camera_link'].find('origin').get('xyz')) == \
        pytest.approx(vec(cam.find('pose').text)[:3])


def test_slightly_positively_buoyant(sdf_model):
    """Displaced mass of the collision volume must exceed total mass by 0-5 %."""
    total_mass = sum(float(l.find('inertial').find('mass').text)
                     for l in sdf_model.findall('link') if l.find('inertial') is not None)
    volume = 0.0
    for col in sdf_model.iter('collision'):
        box = col.find('geometry').find('box')
        assert box is not None, 'buoyancy check only handles box collisions'
        sx, sy, sz = vec(box.find('size').text)
        volume += sx * sy * sz
    displaced = 1000.0 * volume
    margin = (displaced - total_mass) / total_mass
    assert 0.0 < margin < 0.05, f'displaced {displaced:.2f} kg vs mass {total_mass:.2f} kg'


def test_centre_of_buoyancy_above_centre_of_mass(sdf_model):
    base = [l for l in sdf_model.findall('link') if l.get('name') == 'base_link'][0]
    com_z = vec(base.find('inertial').find('pose').text)[2]
    cob_z = vec(base.find('collision').find('pose').text)[2]
    assert cob_z > com_z
