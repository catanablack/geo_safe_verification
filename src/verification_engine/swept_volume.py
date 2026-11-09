"""Swept-volume sampling and conservative voxel-grid clearance queries."""

from __future__ import annotations

from collections.abc import Sequence
from itertools import pairwise

import numpy as np

from src.common.types import Trajectory, VoxelGrid, Waypoint
from src.verification_engine.signed_distance_field import SignedDistanceField


class SweptVolumeAnalyzer:
    """Samples a filled planar footprint along an interpolated trajectory."""

    def __init__(
        self,
        vehicle_footprint_xy: np.ndarray,
        z_min_m: float = 0.0,
        z_max_m: float = 1.5,
        sampling_step_m: float | None = None,
    ) -> None:
        footprint = np.asarray(vehicle_footprint_xy, dtype=np.float64)
        if footprint.ndim != 2 or footprint.shape[1] != 2 or len(footprint) < 3:
            raise ValueError("vehicle_footprint_xy must have shape (N, 2), N >= 3")
        if not np.isfinite(footprint).all():
            raise ValueError("vehicle_footprint_xy must contain finite coordinates")
        if abs(_polygon_area(footprint)) <= np.finfo(float).eps:
            raise ValueError("vehicle_footprint_xy must enclose a non-zero area")
        if not np.isfinite((z_min_m, z_max_m)).all() or z_max_m < z_min_m:
            raise ValueError("z bounds must be finite and z_max_m >= z_min_m")
        if sampling_step_m is not None and (
            not np.isfinite(sampling_step_m) or sampling_step_m <= 0
        ):
            raise ValueError("sampling_step_m must be finite and positive")

        self.vehicle_footprint_xy = footprint
        self.z_min_m = float(z_min_m)
        self.z_max_m = float(z_max_m)
        self.sampling_step_m = sampling_step_m
        self._footprint_radius_m = float(np.linalg.norm(footprint, axis=1).max())

    def compute_swept_volume(
        self, trajectory: Trajectory, resolution_m: float = 0.1
    ) -> np.ndarray:
        """Return sampled XYZ points covering the filled swept footprint.

        The returned points approximate a continuous swept volume. Use
        the error bound returned by ``sample_trajectory`` when converting
        their clearance to a conservative bound.
        """
        points, _, _ = self.sample_trajectory(trajectory, resolution_m)
        return points

    def sample_trajectory(
        self,
        trajectory: Trajectory,
        resolution_m: float,
        additional_times_s: Sequence[float] = (),
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Sample the filled footprint and return points, times, and error bound."""
        waypoints = self._validate_trajectory(trajectory)
        if not np.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("resolution_m must be finite and positive")
        error_bound = self.sampling_error_bound_m(resolution_m)
        step = error_bound / 2.0
        local_points = self._sample_footprint(step)

        extras = np.asarray(additional_times_s, dtype=np.float64)
        if extras.ndim != 1 or not np.isfinite(extras).all():
            raise ValueError("additional_times_s must be a finite 1D sequence")
        times = self._sample_times(waypoints, step, extras)
        poses = np.asarray([_pose_at(waypoints, time_s) for time_s in times])

        cos_heading = np.cos(poses[:, 2])
        sin_heading = np.sin(poses[:, 2])
        transformed = np.empty((len(times), len(local_points), 3), dtype=np.float64)
        transformed[:, :, 0] = (
            poses[:, None, 0]
            + local_points[None, :, 0] * cos_heading[:, None]
            - local_points[None, :, 1] * sin_heading[:, None]
        )
        transformed[:, :, 1] = (
            poses[:, None, 1]
            + local_points[None, :, 0] * sin_heading[:, None]
            + local_points[None, :, 1] * cos_heading[:, None]
        )
        transformed[:, :, 2] = local_points[None, :, 2]
        return (
            transformed.reshape(-1, 3),
            np.repeat(times, len(local_points)),
            error_bound,
        )

    def sampling_error_bound_m(self, resolution_m: float) -> float:
        """Return the conservative footprint-and-time sampling allowance."""
        if not np.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("resolution_m must be finite and positive")
        step = min(resolution_m / 2.0, self.sampling_step_m or resolution_m / 2.0)
        return 2.0 * step

    def min_clearance_to_occupancy(
        self, trajectory: Trajectory, occupancy: VoxelGrid
    ) -> float:
        """Return a conservative lower bound on swept-volume clearance."""
        points, _, sampling_error = self.sample_trajectory(
            trajectory, occupancy.resolution_m
        )
        sdf = SignedDistanceField.from_voxel_grid(occupancy)
        point_clearances = sdf.query_batch_lower_bound(points)
        return float(np.min(point_clearances) - sampling_error)

    def _sample_footprint(self, step: float) -> np.ndarray:
        min_xy = self.vehicle_footprint_xy.min(axis=0)
        max_xy = self.vehicle_footprint_xy.max(axis=0)
        x_values = _inclusive_grid(min_xy[0], max_xy[0], step)
        y_values = _inclusive_grid(min_xy[1], max_xy[1], step)
        xx, yy = np.meshgrid(x_values, y_values, indexing="xy")
        interior = np.column_stack((xx.ravel(), yy.ravel()))
        interior = interior[_points_in_polygon(interior, self.vehicle_footprint_xy)]

        boundary: list[np.ndarray] = [self.vehicle_footprint_xy]
        for start, end in zip(
            self.vehicle_footprint_xy,
            np.roll(self.vehicle_footprint_xy, -1, axis=0),
        ):
            count = max(1, int(np.ceil(np.linalg.norm(end - start) / step)))
            boundary.append(
                start + np.linspace(0.0, 1.0, count + 1)[:, None] * (end - start)
            )
        points_xy = np.unique(np.vstack((interior, *boundary)), axis=0)

        height = self.z_max_m - self.z_min_m
        z_count = max(1, int(np.ceil(height / step)))
        z_values = np.linspace(self.z_min_m, self.z_max_m, z_count + 1)
        return np.column_stack(
            (
                np.tile(points_xy, (len(z_values), 1)),
                np.repeat(z_values, len(points_xy)),
            )
        )

    def _sample_times(
        self,
        waypoints: Sequence[Waypoint],
        step: float,
        additional_times_s: np.ndarray,
    ) -> np.ndarray:
        if len(waypoints) == 1:
            return np.asarray([waypoints[0].timestamp_s], dtype=np.float64)

        radius = self._footprint_radius_m
        sampled: list[float] = [waypoints[0].timestamp_s]
        for first, second in pairwise(waypoints):
            segment_times = additional_times_s[
                (additional_times_s > first.timestamp_s)
                & (additional_times_s < second.timestamp_s)
            ]
            breakpoints = np.unique(
                np.concatenate(
                    (
                        [first.timestamp_s],
                        segment_times,
                        [second.timestamp_s],
                    )
                )
            )
            for start_time, end_time in pairwise(breakpoints):
                start_pose = _pose_at(waypoints, float(start_time))
                end_pose = _pose_at(waypoints, float(end_time))
                translation = float(np.linalg.norm(end_pose[:2] - start_pose[:2]))
                rotation = abs(_angle_delta(start_pose[2], end_pose[2]))
                count = max(1, int(np.ceil((translation + radius * rotation) / step)))
                sampled.extend(np.linspace(start_time, end_time, count + 1)[1:])
        return np.unique(np.asarray(sampled, dtype=np.float64))

    @staticmethod
    def _validate_trajectory(trajectory: Trajectory) -> Sequence[Waypoint]:
        waypoints = trajectory.waypoints
        if not waypoints:
            raise ValueError("trajectory must contain at least one waypoint")
        values = np.asarray(
            [
                (
                    waypoint.pose.x,
                    waypoint.pose.y,
                    waypoint.pose.heading_rad,
                    waypoint.timestamp_s,
                    waypoint.velocity_mps,
                )
                for waypoint in waypoints
            ],
            dtype=np.float64,
        )
        if not np.isfinite(values).all():
            raise ValueError("trajectory waypoint values must be finite")
        if np.any(np.diff(values[:, 3]) <= 0):
            raise ValueError(
                "trajectory waypoint timestamps must be strictly increasing"
            )
        return waypoints


def _polygon_area(polygon: np.ndarray) -> float:
    x, y = polygon.T
    return float(0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1)))


def _inclusive_grid(start: float, stop: float, step: float) -> np.ndarray:
    values = np.arange(start, stop, step, dtype=np.float64)
    return np.append(values, stop) if not len(values) or values[-1] < stop else values


def _points_in_polygon(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    inside = np.zeros(len(points), dtype=bool)
    x, y = points.T
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        crosses = (start[1] > y) != (end[1] > y)
        denominator = end[1] - start[1]
        intersection_x = start[0] + (y - start[1]) * (end[0] - start[0]) / (
            denominator if denominator else 1.0
        )
        inside ^= crosses & (x < intersection_x)
    return inside


def _angle_delta(start: float, end: float) -> float:
    return float((end - start + np.pi) % (2.0 * np.pi) - np.pi)


def _pose_at(waypoints: Sequence[Waypoint], timestamp_s: float) -> np.ndarray:
    if timestamp_s <= waypoints[0].timestamp_s:
        waypoint = waypoints[0]
        return np.asarray((waypoint.pose.x, waypoint.pose.y, waypoint.pose.heading_rad))
    if timestamp_s >= waypoints[-1].timestamp_s:
        waypoint = waypoints[-1]
        return np.asarray((waypoint.pose.x, waypoint.pose.y, waypoint.pose.heading_rad))
    right = next(
        index
        for index, waypoint in enumerate(waypoints)
        if waypoint.timestamp_s >= timestamp_s
    )
    first, second = waypoints[right - 1], waypoints[right]
    fraction = (timestamp_s - first.timestamp_s) / (
        second.timestamp_s - first.timestamp_s
    )
    start = np.asarray((first.pose.x, first.pose.y, first.pose.heading_rad))
    end = np.asarray((second.pose.x, second.pose.y, second.pose.heading_rad))
    return np.asarray(
        (
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
            start[2] + fraction * _angle_delta(start[2], end[2]),
        )
    )
