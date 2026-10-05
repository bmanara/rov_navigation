"""Unit tests for rov_gazebo.raycast (no ROS, no Gazebo)."""
import math
import os
import textwrap

import numpy as np
import pytest

from rov_gazebo.raycast import (Pose, Primitive, cast, fan_directions,
                                load_world_primitives, quaternion_to_matrix,
                                rpy_to_matrix)

POOL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'worlds', 'pool.sdf')


def ray(o, d):
    d = np.asarray(d, dtype=float)
    return np.array([o], dtype=float), np.array([d / np.linalg.norm(d)])


def box(center, size, rpy=(0, 0, 0)):
    return Primitive('box', 'box', Pose(rpy_to_matrix(*rpy), np.array(center, float)),
                     size=np.array(size, float))


# ---- rotations ----

def test_rpy_matches_quaternion():
    r, p, y = 0.3, -0.2, 1.1
    R = rpy_to_matrix(r, p, y)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
         cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    assert np.allclose(R, quaternion_to_matrix(*q))
    assert np.allclose(R @ R.T, np.eye(3))


# ---- primitives ----

def test_box_hit_and_miss():
    b = box([5, 0, 0], [2, 2, 2])
    assert cast([b], *ray([0, 0, 0], [1, 0, 0]))[0] == pytest.approx(4.0)
    assert np.isinf(cast([b], *ray([0, 0, 0], [-1, 0, 0]))[0])  # behind
    assert np.isinf(cast([b], *ray([0, 5, 0], [1, 0, 0]))[0])   # beside


def test_box_axis_parallel_edges():
    b = box([5, 0, 0], [2, 2, 2])
    # Grazing exactly along a face plane still counts as a hit
    assert cast([b], *ray([0, 1, 0], [1, 0, 0]))[0] == pytest.approx(4.0)
    assert np.isinf(cast([b], *ray([0, 1.001, 0], [1, 0, 0]))[0])


def test_rotated_box():
    # 45 deg yawed unit-half-width box: nearest corner at x = 5 - sqrt(2)
    b = box([5, 0, 0], [2, 2, 2], rpy=(0, 0, math.pi / 4))
    assert cast([b], *ray([0, 0, 0], [1, 0, 0]))[0] == pytest.approx(5 - math.sqrt(2))


def test_origin_inside_is_blocked():
    b = box([0, 0, 0], [2, 2, 2])
    assert cast([b], *ray([0, 0, 0], [1, 0, 0]))[0] == 0.0
    s = Primitive('s', 'sphere', Pose(t=np.zeros(3)), radius=1.0)
    assert cast([s], *ray([0, 0, 0], [0, 1, 0]))[0] == 0.0


def test_sphere():
    s = Primitive('s', 'sphere', Pose(t=np.array([0, 3.0, 0])), radius=0.5)
    assert cast([s], *ray([0, 0, 0], [0, 1, 0]))[0] == pytest.approx(2.5)
    assert np.isinf(cast([s], *ray([0, 0, 0], [1, 0, 0]))[0])


def test_cylinder_side_and_caps():
    c = Primitive('c', 'cylinder', Pose(t=np.array([4.0, 0, 0])), radius=0.5, length=2.0)
    assert cast([c], *ray([0, 0, 0], [1, 0, 0]))[0] == pytest.approx(3.5)   # side
    assert np.isinf(cast([c], *ray([0, 0, 1.5], [1, 0, 0]))[0])              # above it
    assert cast([c], *ray([4, 0, 5], [0, 0, -1]))[0] == pytest.approx(4.0)   # top cap


def test_lying_cylinder():
    # Crossbar-style: cylinder rotated 90 deg about x lies along y
    c = Primitive('c', 'cylinder', Pose(rpy_to_matrix(math.pi / 2, 0, 0), np.array([3.0, 0, 0])),
                  radius=0.1, length=4.0)
    assert cast([c], *ray([0, 1.5, 0], [1, 0, 0]))[0] == pytest.approx(2.9)
    assert np.isinf(cast([c], *ray([0, 2.5, 0], [1, 0, 0]))[0])


def test_nearest_of_several():
    prims = [box([8, 0, 0], [1, 1, 1]), box([3, 0, 0], [1, 1, 1])]
    assert cast(prims, *ray([0, 0, 0], [1, 0, 0]))[0] == pytest.approx(2.5)


# ---- fan ----

def test_fan_geometry():
    d = fan_directions(math.radians(90), 3, math.radians(20), 5)
    assert d.shape == (15, 3)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    beam0 = d[:5]  # first beam = rightmost (-45 deg azimuth)
    assert np.allclose(np.arctan2(beam0[:, 1], beam0[:, 0]), math.radians(-45))
    assert np.max(beam0[:, 2]) == pytest.approx(math.sin(math.radians(10)))


def test_fan_azimuth_subrays_stay_within_beam():
    fov, n = math.radians(90), 3          # beam centres -45, 0, 45 deg; width 45 deg
    d = fan_directions(fov, n, math.radians(20), 5, n_azimuth=3)
    assert d.shape == (3 * 3 * 5, 3)
    az = np.degrees(np.arctan2(d[:, 1], d[:, 0])).reshape(3, 15)
    for centre, beam in zip((-45, 0, 45), az):
        assert np.all(np.abs(beam - centre) < 22.5)       # inside the beam width
        assert sorted(set(np.round(beam - centre, 6))) == pytest.approx([-11.25, 0, 11.25])


def test_dense_fan_sees_gate_but_sparse_misses_crossbar():
    prims = load_world_primitives(POOL, exclude_models=['rov'])
    o = np.array([0.21, 0.0, -0.56])  # sonar on a ROV at the spawn pose
    beam = math.radians(-15.5)        # through the gate opening, below the crossbar
    def beam_range(n_el):
        d = fan_directions(math.radians(1e-6), 1, math.radians(20), n_el)
        c, s = math.cos(beam), math.sin(beam)
        d = d @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T
        return cast(prims, np.broadcast_to(o, d.shape), d).min()
    assert beam_range(5) > 10.0                    # rays straddle the 10 cm bar
    assert beam_range(41) == pytest.approx(8.05, abs=0.05)


# ---- SDF parsing ----

def test_parse_nested_poses(tmp_path):
    f = tmp_path / 'w.sdf'
    f.write_text(textwrap.dedent('''\
        <sdf version="1.10"><world name="w">
          <model name="m"><pose>1 0 0 0 0 1.5707963</pose>
            <link name="l"><pose>2 0 0 0 0 0</pose>
              <collision name="c"><pose>0 0 3 0 0 0</pose>
                <geometry><box><size>1 1 1</size></box></geometry></collision>
              <collision name="mesh"><geometry><mesh><uri>x.dae</uri></mesh></geometry></collision>
            </link></model>
          <model name="rov"><link name="l"><collision name="c">
            <geometry><sphere><radius>1</radius></sphere></geometry></collision></link></model>
        </world></sdf>'''))
    with pytest.warns(UserWarning, match='unsupported'):
        prims = load_world_primitives(str(f), exclude_models=['rov'])
    assert len(prims) == 1
    # model yaw 90 deg maps link offset (2,0,0) to (0,2,0)
    assert np.allclose(prims[0].pose.t, [1, 2, 3], atol=1e-6)


def test_pool_world_loads():
    prims = load_world_primitives(POOL, exclude_models=['rov'])
    names = {p.name.split('::')[0] for p in prims}
    assert {'pool', 'pillar', 'reef_block', 'gate', 'shelf', 'buoy'} <= names


def test_pool_sees_walls_and_pillar():
    prims = load_world_primitives(POOL)
    # From the origin at 1 m depth, straight ahead (+x) the inner wall face is at x = 10
    assert cast(prims, *ray([0, 0, -1], [1, 0, 0]))[0] == pytest.approx(10.0)
    # Pillar at (5, 3), r = 0.3
    assert cast(prims, *ray([5, 0, -1], [0, 1, 0]))[0] == pytest.approx(2.7)
    # Floor top at z = -4
    assert cast(prims, *ray([0, 0, -1], [0, 0, -1]))[0] == pytest.approx(3.0)
    # Gate crossbar (r 0.05) at z = -1: looking down from z = -0.5 hits its top at -0.95
    assert cast(prims, *ray([8, -2, -0.5], [0, 0, -1]))[0] == pytest.approx(0.45)
