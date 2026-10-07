"""3D multibeam sonar model (no ROS). Used by scripts/sonar_emulator.py.

A grid of n_el x n_az beams. Each beam returns the range of the nearest echo
inside it (CPU ray cast against the world's collision primitives), and that
echo is placed along the beam CENTRE, like a real sonar: position within a
beam's width is unknown, so lateral error grows with range x beam width.
"""
from dataclasses import dataclass
import math

import numpy as np

from rov_gazebo.raycast import beam_grid, cast


@dataclass
class SonarConfig:
    horizontal_fov_deg: float = 130.0
    num_beams: int = 64                 # azimuth beams (columns)
    vertical_fov_deg: float = 40.0
    num_elevation_beams: int = 24       # elevation beams (rows)
    sub_rays_azimuth: int = 3
    sub_rays_elevation: int = 3
    range_min: float = 0.1              # [m]
    range_max: float = 10.0             # [m]
    range_stddev: float = 0.02          # [m]


@dataclass
class SonarScan:
    ranges: np.ndarray   # (n_el, n_az): range [m], +inf no return, -inf too close
    points: np.ndarray   # (n_el, n_az, 3) in sensor frame; NaN where no valid return

    def valid_points(self):
        """(N, 3) points of beams with a valid return."""
        p = self.points.reshape(-1, 3)
        return p[np.isfinite(p[:, 0])]

    def scan_2d(self):
        """Nearest return per azimuth column, i.e. the fan collapsed to a LaserScan."""
        r = self.ranges.copy()
        r[np.isneginf(r)] = 0.0          # keep 'too close' as the nearest thing
        out = r.min(axis=0)
        out[out == 0.0] = -np.inf
        return out


class SonarModel:

    def __init__(self, cfg: SonarConfig, seed=None):
        self.cfg = cfg
        self.h_fov = math.radians(cfg.horizontal_fov_deg)
        self.v_fov = math.radians(cfg.vertical_fov_deg)
        self.shape = (cfg.num_elevation_beams, cfg.num_beams)
        self.rays, self.centres = beam_grid(
            self.h_fov, cfg.num_beams, self.v_fov, cfg.num_elevation_beams,
            cfg.sub_rays_azimuth, cfg.sub_rays_elevation)
        self.rays_per_beam = cfg.sub_rays_azimuth * cfg.sub_rays_elevation
        self.rng = np.random.default_rng(seed)

    @property
    def num_rays(self):
        return len(self.rays)

    def scan(self, primitives, origin, R_world_sensor):
        """Cast one scan from a sensor at `origin` (world) with orientation R_world_sensor."""
        cfg = self.cfg
        dirs = self.rays @ R_world_sensor.T
        dist = cast(primitives, np.broadcast_to(origin, dirs.shape), dirs)
        ranges = dist.reshape(-1, self.rays_per_beam).min(axis=1).reshape(self.shape)

        hit = np.isfinite(ranges)
        if cfg.range_stddev > 0.0:
            ranges[hit] += self.rng.normal(0.0, cfg.range_stddev, int(hit.sum()))
        ranges[ranges > cfg.range_max] = np.inf
        ranges[hit & (ranges < cfg.range_min)] = -np.inf

        valid = np.isfinite(ranges)
        points = np.full(self.shape + (3,), np.nan)
        points[valid] = self.centres[valid] * ranges[valid][:, None]
        return SonarScan(ranges, points)
