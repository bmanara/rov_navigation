"""Unit tests for the 3D sonar model (no ROS, no Gazebo)."""
import math
import os

import numpy as np
import pytest

from rov_gazebo.raycast import Pose, Primitive, load_world_primitives
from rov_gazebo.sonar_model import SonarConfig, SonarModel

POOL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'worlds', 'pool.sdf')
LEVEL = np.eye(3)


def noiseless(**kw):
    return SonarModel(SonarConfig(range_stddev=0.0, **kw), seed=0)


def wall(x, half_width=25.0):
    """A wall facing the sonar (normal -x), front face at distance x."""
    return Primitive('wall', 'box', Pose(t=np.array([x + 0.5, 0, 0])),
                     size=np.array([1.0, 2 * half_width, 2 * half_width]))


def test_shapes_and_point_placement():
    m = noiseless(num_beams=8, num_elevation_beams=4, sub_rays_azimuth=1, sub_rays_elevation=1,
                  horizontal_fov_deg=60, vertical_fov_deg=20)   # all beams within 10 m of x = 5
    s = m.scan([wall(5.0)], np.zeros(3), LEVEL)
    assert s.ranges.shape == (4, 8) and s.points.shape == (4, 8, 3)
    # Every beam hits the plane x = 5; points lie on it and along the beam centre
    assert np.allclose(s.points[..., 0], 5.0, atol=1e-6)
    dirs = s.points / np.linalg.norm(s.points, axis=-1, keepdims=True)
    assert np.allclose(dirs, m.centres)


def test_range_limits_rep117():
    m = noiseless(num_beams=4, num_elevation_beams=2, range_min=0.5, range_max=3.0)
    assert np.all(np.isposinf(m.scan([wall(5.0)], np.zeros(3), LEVEL).ranges))
    far = m.scan([], np.zeros(3), LEVEL)
    assert np.all(np.isposinf(far.ranges)) and far.valid_points().shape == (0, 3)
    close = m.scan([wall(0.2)], np.zeros(3), LEVEL)
    assert np.all(np.isneginf(close.ranges))
    assert np.all(np.isnan(close.points))  # too-close returns are not turned into points


def test_scan_2d_takes_nearest_per_column():
    m = noiseless(num_beams=3, num_elevation_beams=3)
    s = m.scan([wall(5.0)], np.zeros(3), LEVEL)
    s.ranges[:, 0] = [np.inf, 4.0, 6.0]
    s.ranges[:, 1] = [np.inf, np.inf, np.inf]
    s.ranges[:, 2] = [7.0, -np.inf, 2.0]
    assert np.array_equal(s.scan_2d(), [4.0, np.inf, -np.inf])


def test_sensor_orientation_applied():
    # Sensor yawed 90 deg left faces +y: a wall in +y is seen, one in +x is not.
    m = noiseless(num_beams=5, num_elevation_beams=3, horizontal_fov_deg=60, vertical_fov_deg=20)
    yaw90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
    wall_y = Primitive('w', 'box', Pose(t=np.array([0, 4.5, 0])), size=np.array([50, 1.0, 50]))
    s = m.scan([wall_y], np.zeros(3), yaw90)
    assert np.isfinite(s.ranges).all()
    assert np.all(np.isposinf(m.scan([wall(4.0, half_width=1.0)], np.zeros(3), yaw90).ranges))


def test_pool_gate_resolved_in_3d():
    """The point the 2D LaserScan couldn't make: the crossbar is ABOVE the opening."""
    prims = load_world_primitives(POOL, exclude_models=['rov'])
    m = noiseless()  # default grid
    origin = np.array([0.21, 0.0, -0.56])            # sonar on the ROV at spawn
    pts = m.scan(prims, origin, LEVEL).valid_points() + origin   # -> world frame
    near_gate = pts[(np.abs(pts[:, 0] - 8.0) < 0.3) & (pts[:, 1] > -3.2) & (pts[:, 1] < -0.8)]
    assert len(near_gate) > 0
    bar = near_gate[np.abs(near_gate[:, 2] + 1.0) < 0.3]
    posts = near_gate[near_gate[:, 2] < -1.3]
    assert len(bar) > 0, 'crossbar at z = -1 not seen'
    assert len(posts) > 0, 'gate posts not seen'
    # Through the middle of the opening, below the bar: nothing at the gate
    opening = near_gate[(np.abs(near_gate[:, 1] + 2.0) < 0.5) & (near_gate[:, 2] < -1.4)]
    assert len(opening) == 0


def test_pool_floor_visible_in_lower_rows():
    prims = load_world_primitives(POOL, exclude_models=['rov'])
    m = noiseless()
    origin = np.array([0.21, 0.0, -2.5])             # 1.5 m above the floor
    s = m.scan(prims, origin, LEVEL)
    floor = s.valid_points()[:, 2] + origin[2]
    assert np.any(np.abs(floor + 4.0) < 0.3)         # bottom rows reach z = -4
