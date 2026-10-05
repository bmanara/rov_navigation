"""sonar_link in the URDF must match the sonar emulator's mount parameters."""
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SONAR_YAML = os.path.join(HERE, 'config', 'sonar.yaml')
URDF = os.path.join(os.path.dirname(HERE), 'rov_description', 'urdf', 'rov.urdf')


def test_sonar_mount_matches_urdf():
    params = yaml.safe_load(open(SONAR_YAML))['sonar_emulator']['ros__parameters']
    joint = [j for j in ET.parse(URDF).getroot().findall('joint')
             if j.find('child').get('link') == params['frame_id']]
    assert len(joint) == 1, f"no URDF joint for {params['frame_id']}"
    origin = joint[0].find('origin')
    assert [float(v) for v in origin.get('xyz').split()] == pytest.approx(params['mount_xyz'])
    assert [float(v) for v in origin.get('rpy').split()] == pytest.approx(params['mount_rpy'])
    assert joint[0].find('parent').get('link') == 'base_link'
