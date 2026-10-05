"""CPU ray casting against the static collision primitives of an SDF world.

Used by sonar_emulator.py so the sonar needs no rendering (no gpu_lidar).
Pure numpy, no ROS, so it can be unit tested on its own.

Supported collision geometry: box, cylinder, sphere. Anything else (mesh,
plane, ...) is skipped with a warning; the sonar won't see it.

Conventions: SDF poses are `x y z roll pitch yaw`, rotation R = Rz(yaw) Ry(pitch)
Rx(roll). All distances in meters.
"""
from dataclasses import dataclass, field
import math
import warnings
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def quaternion_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@dataclass
class Pose:
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __mul__(self, other):
        return Pose(self.R @ other.R, self.R @ other.t + self.t)


def parse_pose(elem):
    """Parse an SDF <pose> element (None -> identity)."""
    if elem is None or not (elem.text or '').strip():
        return Pose()
    if elem.get('relative_to'):
        warnings.warn(f"pose relative_to='{elem.get('relative_to')}' ignored")
    v = [float(s) for s in elem.text.split()]
    if len(v) != 6:
        raise ValueError(f'expected 6 pose values, got {len(v)}')
    if elem.get('degrees', 'false').lower() == 'true':
        v[3:] = [math.radians(a) for a in v[3:]]
    return Pose(rpy_to_matrix(*v[3:]), np.array(v[:3]))


@dataclass
class Primitive:
    """A collision shape posed in the world. `kind` in {'box', 'cylinder', 'sphere'}."""
    name: str
    kind: str
    pose: Pose
    size: np.ndarray = None   # box: full extents (x, y, z)
    radius: float = 0.0       # cylinder / sphere
    length: float = 0.0       # cylinder (along local z)


def load_world_primitives(sdf_path, exclude_models=()):
    """Collect collision primitives of all top-level models in the world file."""
    world = ET.parse(sdf_path).getroot().find('world')
    if world is None:
        raise ValueError(f'{sdf_path}: no <world> element')
    prims = []
    for model in world.findall('model'):
        mname = model.get('name')
        if mname in exclude_models:
            continue
        model_pose = parse_pose(model.find('pose'))
        for link in model.findall('link'):
            link_pose = model_pose * parse_pose(link.find('pose'))
            for col in link.findall('collision'):
                pose = link_pose * parse_pose(col.find('pose'))
                name = f"{mname}::{link.get('name')}::{col.get('name')}"
                geom = col.find('geometry')
                if geom.find('box') is not None:
                    size = np.array([float(s) for s in geom.find('box/size').text.split()])
                    prims.append(Primitive(name, 'box', pose, size=size))
                elif geom.find('cylinder') is not None:
                    c = geom.find('cylinder')
                    prims.append(Primitive(name, 'cylinder', pose,
                                           radius=float(c.find('radius').text),
                                           length=float(c.find('length').text)))
                elif geom.find('sphere') is not None:
                    prims.append(Primitive(name, 'sphere', pose,
                                           radius=float(geom.find('sphere/radius').text)))
                else:
                    kinds = [child.tag for child in geom]
                    warnings.warn(f'{name}: unsupported geometry {kinds}; invisible to sonar')
    return prims


# ---------------------------------------------------------------------------
# Ray / primitive intersection. All functions take ray origins O (N,3) and unit
# directions D (N,3) in the primitive's LOCAL frame and return the distance to
# the first hit along each ray (inf = miss). A ray starting inside a solid
# returns 0 (the sensor is blocked).
# ---------------------------------------------------------------------------

def _ray_box(O, D, half):
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / D
        t1 = (-half - O) * inv
        t2 = (half - O) * inv
    near = np.minimum(t1, t2)
    far = np.maximum(t1, t2)
    # Axis-parallel rays: the slab constrains nothing if the origin is inside
    # it, and excludes the ray entirely if outside.
    parallel = D == 0.0
    inside_slab = np.abs(O) <= half
    near = np.where(parallel, np.where(inside_slab, -np.inf, np.inf), near)
    far = np.where(parallel, np.where(inside_slab, np.inf, -np.inf), far)
    tnear = np.max(near, axis=1)
    tfar = np.min(far, axis=1)
    hit = (tfar >= tnear) & (tfar >= 0.0)
    return np.where(hit, np.maximum(tnear, 0.0), np.inf)


def _ray_sphere(O, D, r):
    b = np.einsum('ij,ij->i', O, D)
    c = np.einsum('ij,ij->i', O, O) - r * r
    disc = b * b - c
    sq = np.sqrt(np.maximum(disc, 0.0))
    t0, t1 = -b - sq, -b + sq
    t = np.where(t0 >= 0.0, t0, np.where(t1 >= 0.0, 0.0, np.inf))  # t1>=0>t0: inside
    return np.where(disc >= 0.0, t, np.inf)


def _ray_cylinder(O, D, r, length):
    h = length / 2.0
    best = np.full(len(O), np.inf)

    # Side wall
    a = D[:, 0] ** 2 + D[:, 1] ** 2
    b = O[:, 0] * D[:, 0] + O[:, 1] * D[:, 1]
    c = O[:, 0] ** 2 + O[:, 1] ** 2 - r * r
    disc = b * b - a * c
    ok = (a > 1e-12) & (disc >= 0.0)
    with np.errstate(divide='ignore', invalid='ignore'):
        sq = np.sqrt(np.maximum(disc, 0.0))
        for t in ((-b - sq) / a, (-b + sq) / a):
            z = O[:, 2] + t * D[:, 2]
            valid = ok & (t >= 0.0) & (np.abs(z) <= h)
            best = np.where(valid & (t < best), t, best)

        # End caps
        for zc in (-h, h):
            t = (zc - O[:, 2]) / D[:, 2]
            x = O[:, 0] + t * D[:, 0]
            y = O[:, 1] + t * D[:, 1]
            valid = (np.abs(D[:, 2]) > 1e-12) & (t >= 0.0) & (x * x + y * y <= r * r)
            best = np.where(valid & (t < best), t, best)

    inside = (c <= 0.0) & (np.abs(O[:, 2]) <= h)
    return np.where(inside, 0.0, best)


def cast(primitives, origins, directions):
    """Distance from each ray to the nearest primitive (inf = no hit).

    origins, directions: (N,3) arrays in the world frame; directions unit length.
    """
    origins = np.atleast_2d(np.asarray(origins, dtype=float))
    directions = np.atleast_2d(np.asarray(directions, dtype=float))
    best = np.full(len(origins), np.inf)
    for p in primitives:
        Rt = p.pose.R.T
        O = (origins - p.pose.t) @ Rt.T
        D = directions @ Rt.T
        if p.kind == 'box':
            d = _ray_box(O, D, p.size / 2.0)
        elif p.kind == 'sphere':
            d = _ray_sphere(O, D, p.radius)
        else:
            d = _ray_cylinder(O, D, p.radius, p.length)
        best = np.minimum(best, d)
    return best


def fan_directions(h_fov, n_beams, v_aperture, n_elevations, n_azimuth=1):
    """Unit ray directions (sensor frame, x forward, z up) for a sonar fan.

    Beam centres run from -h_fov/2 (right) to +h_fov/2 (left), matching
    LaserScan's counter-clockwise angle convention. Each beam is sampled by
    n_azimuth x n_elevations rays: n_elevations across the vertical aperture,
    n_azimuth across the beam's horizontal width (the beam spacing
    h_fov / (n_beams - 1)). Dense sampling matters: thin structures (10 cm
    gate posts / crossbar) fall between sparse rays.

    Returns (n_beams * n_azimuth * n_elevations, 3), beam-major: each
    consecutive block of n_azimuth * n_elevations rows is one beam.
    """
    centres = np.linspace(-h_fov / 2.0, h_fov / 2.0, n_beams)
    width = h_fov / max(n_beams - 1, 1)
    sub_az = (np.linspace(-width / 2.0, width / 2.0, n_azimuth + 2)[1:-1]
              if n_azimuth > 1 else np.zeros(1))
    el = (np.linspace(-v_aperture / 2.0, v_aperture / 2.0, n_elevations)
          if n_elevations > 1 else np.zeros(1))
    C, S, E = np.meshgrid(centres, sub_az, el, indexing='ij')
    A, E = (C + S).ravel(), E.ravel()
    return np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=1)
