"""Signed distance fields derived from probabilistic voxel occupancy."""

from __future__ import annotations

import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates

from src.common.types import VoxelGrid


class SignedDistanceField:
    """A trilinearly interpolated distance field over a bounded voxel grid.

    Positive samples denote free space and non-positive samples denote occupied
    space or its boundary. Queries outside the known grid return a negative
    value so callers cannot certify trajectories beyond observed map coverage.
    """

    def __init__(
        self,
        sdf_values: np.ndarray,
        resolution_m: float,
        origin: tuple[float, float, float],
        occupied: np.ndarray | None = None,
    ) -> None:
        values = np.asarray(sdf_values, dtype=np.float64)
        if values.ndim != 3 or any(size == 0 for size in values.shape):
            raise ValueError("sdf_values must be a non-empty 3D array")
        if not np.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("resolution_m must be finite and positive")
        if len(origin) != 3 or not np.isfinite(origin).all():
            raise ValueError("origin must contain three finite coordinates")
        if np.isnan(values).any():
            raise ValueError("sdf_values must not contain NaN")

        self.sdf_values = values
        self.resolution_m = float(resolution_m)
        self.origin = tuple(float(value) for value in origin)
        self.occupied = occupied
        self.voxel_error_bound_m = np.sqrt(3.0) * self.resolution_m / 2.0
        self.interpolation_error_bound_m = np.sqrt(3.0) * self.resolution_m / 2.0
        self.total_error_bound_m = (
            self.voxel_error_bound_m + self.interpolation_error_bound_m
        )

    @classmethod
    def from_voxel_grid(
        cls, grid: VoxelGrid, occupancy_threshold: float = 0.5
    ) -> SignedDistanceField:
        """Build a Euclidean distance transform from thresholded occupancy.

        Occupied voxels are treated as closed cubes. The free-space transform
        is reduced by the voxel half-diagonal; ``query_lower_bound`` also
        accounts for interpolation error between voxel centers.
        """
        occupancy = np.asarray(grid.occupancy)
        if occupancy.ndim != 3 or any(size == 0 for size in occupancy.shape):
            raise ValueError("grid.occupancy must be a non-empty 3D array")
        if not np.isfinite(occupancy).all():
            raise ValueError("grid.occupancy must contain only finite values")
        if not np.isfinite(grid.resolution_m) or grid.resolution_m <= 0:
            raise ValueError("grid.resolution_m must be finite and positive")
        if len(grid.origin) != 3 or not np.isfinite(grid.origin).all():
            raise ValueError("grid.origin must contain three finite coordinates")
        if not np.isfinite(occupancy_threshold) or not 0 <= occupancy_threshold <= 1:
            raise ValueError("occupancy_threshold must be between 0 and 1")
        if not np.isfinite(occupancy).all() or np.any(
            (occupancy < 0) | (occupancy > 1)
        ):
            raise ValueError("grid.occupancy values must be between 0 and 1")

        occupied = occupancy >= occupancy_threshold
        resolution = float(grid.resolution_m)
        half_voxel_diagonal = np.sqrt(3.0) * resolution / 2.0
        if not occupied.any():
            axes = [
                np.minimum(
                    (np.arange(size) + 0.5) * resolution,
                    (size - np.arange(size) - 0.5) * resolution,
                )
                for size in occupied.shape
            ]
            values = np.minimum(
                np.minimum(axes[0][:, None, None], axes[1][None, :, None]),
                axes[2][None, None, :],
            )
            return cls(values, resolution, grid.origin, occupied)

        padded = np.pad(occupied, 1, constant_values=False)

        distance_to_occupied = distance_transform_edt(~padded, sampling=resolution)[
            1:-1, 1:-1, 1:-1
        ]
        distance_to_free = distance_transform_edt(padded, sampling=resolution)[
            1:-1, 1:-1, 1:-1
        ]
        values = np.where(
            occupied,
            -np.maximum(0.0, distance_to_free - half_voxel_diagonal),
            distance_to_occupied - half_voxel_diagonal,
        )
        return cls(values, resolution, grid.origin, occupied)

    def query(self, point_xyz: np.ndarray) -> float:
        """Return the trilinearly interpolated signed distance estimate."""
        point = np.asarray(point_xyz, dtype=np.float64)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError("point_xyz must be a finite 3D coordinate")
        if not self._inside_grid(point[None, :])[0]:
            return -self.resolution_m

        indices = self._center_indices(point[None, :])[0]
        value = float(
            map_coordinates(
                self.sdf_values,
                indices[:, None],
                order=1,
                mode="nearest",
                prefilter=False,
            )[0]
        )
        if self._is_occupied(point[None, :])[0]:
            return min(value, 0.0)
        return value

    def query_lower_bound(self, point_xyz: np.ndarray) -> float:
        """Return a conservative free-space clearance bound in meters.

        The stored field already subtracts the voxel half-diagonal. This query
        also subtracts the trilinear interpolation error.
        """
        value = self.query(point_xyz)
        if value <= 0:
            return value
        return value - self.interpolation_error_bound_m

    def query_batch(self, points_xyz: np.ndarray) -> np.ndarray:
        """Vectorized signed-distance query for an ``(N, 3)`` point array."""
        points = np.asarray(points_xyz, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_xyz must have shape (N, 3)")
        if not np.isfinite(points).all():
            raise ValueError("points_xyz must contain only finite coordinates")
        if len(points) == 0:
            return np.empty(0, dtype=np.float64)

        inside = self._inside_grid(points)
        values = np.full(len(points), -self.resolution_m, dtype=np.float64)
        if inside.any():
            indices = self._center_indices(points[inside])
            values[inside] = map_coordinates(
                self.sdf_values,
                indices.T,
                order=1,
                mode="nearest",
                prefilter=False,
            )
        occupied = self._is_occupied(points)
        values[occupied] = np.minimum(values[occupied], 0.0)
        return values

    def query_batch_lower_bound(self, points_xyz: np.ndarray) -> np.ndarray:
        """Vectorized conservative clearance bounds for safety decisions."""
        values = self.query_batch(points_xyz)
        positive = values > 0
        values[positive] -= self.interpolation_error_bound_m
        return values

    def _center_indices(self, points_xyz: np.ndarray) -> np.ndarray:
        return (points_xyz - np.asarray(self.origin)) / self.resolution_m - 0.5

    def _inside_grid(self, points_xyz: np.ndarray) -> np.ndarray:
        coordinates = (points_xyz - np.asarray(self.origin)) / self.resolution_m
        shape = np.asarray(self.sdf_values.shape)
        return np.all((coordinates >= 0) & (coordinates < shape), axis=1)

    def _is_occupied(self, points_xyz: np.ndarray) -> np.ndarray:
        if self.occupied is None:
            return np.zeros(len(points_xyz), dtype=bool)
        indices = np.floor(
            (points_xyz - np.asarray(self.origin)) / self.resolution_m
        ).astype(np.intp)
        valid = self._inside_grid(points_xyz)
        result = np.zeros(len(points_xyz), dtype=bool)
        result[valid] = self.occupied[tuple(indices[valid].T)]
        return result
