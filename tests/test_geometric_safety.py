from __future__ import annotations

import numpy as np
import pytest

from src.common.config import SafetyConfig
from src.common.types import (
    DynamicAgentForecast,
    Pose2D,
    Trajectory,
    VehicleClass,
    VoxelGrid,
    Waypoint,
)
from src.verification_engine.safety_certifier import GeometricSafetyCertifier
from src.verification_engine.signed_distance_field import SignedDistanceField
from src.verification_engine.swept_volume import SweptVolumeAnalyzer


def _grid(
    occupancy: np.ndarray,
    resolution: float = 0.25,
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> VoxelGrid:
    return VoxelGrid(
        resolution_m=resolution,
        origin=origin,
        occupancy=occupancy.astype(np.float32),
        uncertainty=np.zeros_like(occupancy, dtype=np.float32),
    )


def _trajectory(
    start: tuple[float, float],
    end: tuple[float, float] | None = None,
    start_time: float = 0.0,
    end_time: float = 1.0,
) -> Trajectory:
    points = [start] if end is None else [start, end]
    times = [start_time] if end is None else [start_time, end_time]
    return Trajectory(
        trajectory_id="test",
        waypoints=[
            Waypoint(Pose2D(x, y, 0.0), timestamp, 0.0)
            for (x, y), timestamp in zip(points, times)
        ],
        vehicle_class=VehicleClass.ROBOTAXI,
    )


def _rectangle_intersects_aabb(
    center_x: float,
    center_y: float,
    half_length: float,
    half_width: float,
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
) -> bool:
    return (
        center_x - half_length <= xmax
        and center_x + half_length >= xmin
        and center_y - half_width <= ymax
        and center_y + half_width >= ymin
    )


def test_sdf_box_clearance_is_a_conservative_bound() -> None:
    occupancy = np.zeros((12, 12, 12), dtype=np.float32)
    occupancy[4:8, 4:8, 4:8] = 1.0
    grid = _grid(occupancy, resolution=0.25)
    sdf = SignedDistanceField.from_voxel_grid(grid)

    occupied_center = np.asarray((1.375, 1.375, 1.375))
    assert sdf.query(occupied_center) <= 0.0

    box_min = np.asarray((1.0, 1.0, 1.0))
    box_max = np.asarray((2.0, 2.0, 2.0))
    for point in (
        (0.375, 1.375, 1.375),
        (2.375, 2.375, 1.375),
        (2.125, 1.625, 2.75),
    ):
        point_array = np.asarray(point)
        outside_distance = np.maximum(
            np.maximum(box_min - point_array, point_array - box_max), 0.0
        )
        exact_clearance = float(np.linalg.norm(outside_distance))
        assert sdf.query_lower_bound(point_array) <= exact_clearance + 1e-12


def test_sdf_sphere_clearance_is_a_conservative_bound() -> None:
    resolution = 0.1
    axes = (np.arange(30) + 0.5) * resolution
    xx, yy, zz = np.meshgrid(axes, axes, axes, indexing="ij")
    center = np.asarray((1.5, 1.5, 1.5))
    occupancy = (xx - center[0]) ** 2 + (yy - center[1]) ** 2 + (
        zz - center[2]
    ) ** 2 <= 0.5**2
    sdf = SignedDistanceField.from_voxel_grid(_grid(occupancy, resolution))

    for point in ((2.5, 1.5, 1.5), (2.1, 2.2, 1.5), (1.5, 1.5, 1.5)):
        clearance = sdf.query_lower_bound(np.asarray(point))
        analytic_clearance = max(
            float(np.linalg.norm(np.asarray(point) - center)) - 0.5, 0.0
        )
        assert clearance <= analytic_clearance + 1e-12


def test_certifier_has_no_false_accepts_for_brute_force_box_intersections() -> None:
    resolution = 0.25
    occupancy = np.zeros((64, 64, 4), dtype=np.float32)
    footprint = np.asarray(((-0.4, -0.25), (0.4, -0.25), (0.4, 0.25), (-0.4, 0.25)))
    analyzer = SweptVolumeAnalyzer(footprint, z_min_m=0.0, z_max_m=0.5)
    certifier = GeometricSafetyCertifier(
        analyzer,
        SafetyConfig(min_safety_margin_m=0.0, dynamic_agent_radius_m=0.0),
    )
    origin = (-5.0, -5.0, 0.0)
    rng = np.random.default_rng(2026)
    false_accepts = 0
    for _ in range(128):
        occupancy.fill(0.0)
        x = float(rng.uniform(-2.0, 2.0))
        y = float(rng.uniform(-2.0, 2.0))
        obstacle_x = int(
            np.floor((x + rng.uniform(-0.9, 0.9) - origin[0]) / resolution)
        )
        obstacle_y = int(
            np.floor((y + rng.uniform(-0.7, 0.7) - origin[1]) / resolution)
        )
        obstacle = (obstacle_x, obstacle_y, 0)
        occupancy[obstacle] = 1.0
        xmin = origin[0] + obstacle[0] * resolution
        ymin = origin[1] + obstacle[1] * resolution
        exact_overlap = _rectangle_intersects_aabb(
            x,
            y,
            half_length=0.4,
            half_width=0.25,
            xmin=xmin,
            xmax=xmin + resolution,
            ymin=ymin,
            ymax=ymin + resolution,
        )
        verdict = certifier.verify(
            _trajectory((x, y)),
            _grid(occupancy, resolution, origin),
            [],
        )
        false_accepts += int(verdict.is_safe and exact_overlap)
        assert np.isfinite(verdict.min_safety_margin_m)
    assert false_accepts == 0


def test_dynamic_forecast_is_interpolated_across_the_trajectory() -> None:
    resolution = 0.25
    occupancy = np.zeros((40, 40, 8), dtype=np.float32)
    analyzer = SweptVolumeAnalyzer(
        np.asarray(((-0.2, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.2, 0.1))),
        z_min_m=0.0,
        z_max_m=0.5,
    )
    certifier = GeometricSafetyCertifier(
        analyzer,
        SafetyConfig(min_safety_margin_m=0.0, dynamic_agent_radius_m=0.3),
    )
    forecast = DynamicAgentForecast(
        agent_id="crossing",
        predicted_positions=(Pose2D(0.5, 0.0, 0.0), Pose2D(1.5, 0.0, 0.0)),
        horizon_s=1.0,
        confidence=1.0,
    )

    verdict = certifier.verify(
        _trajectory((0.0, 0.0), (2.0, 0.0)),
        _grid(occupancy, resolution, (-5.0, -5.0, 0.0)),
        [forecast],
    )
    assert not verdict.is_safe
    assert verdict.violation_timestamp_s is not None


def test_forecast_footprint_radius_overrides_the_configured_default() -> None:
    resolution = 0.25
    occupancy = np.zeros((40, 40, 8), dtype=np.float32)
    analyzer = SweptVolumeAnalyzer(
        np.asarray(((-0.2, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.2, 0.1))),
        z_min_m=0.0,
        z_max_m=0.5,
    )
    certifier = GeometricSafetyCertifier(
        analyzer,
        SafetyConfig(min_safety_margin_m=0.0, dynamic_agent_radius_m=0.1),
    )
    trajectory = _trajectory((0.0, 0.0))
    grid = _grid(occupancy, resolution, (-5.0, -5.0, -1.0))
    small_agent = DynamicAgentForecast(
        "small",
        (Pose2D(0.6, 0.0, 0.0),),
        1.0,
        1.0,
        footprint_radius_m=0.1,
    )
    large_agent = DynamicAgentForecast(
        "large",
        (Pose2D(0.6, 0.0, 0.0),),
        1.0,
        1.0,
        footprint_radius_m=1.1,
    )

    small_margin = certifier.verify(trajectory, grid, [small_agent]).min_safety_margin_m
    large_margin = certifier.verify(trajectory, grid, [large_agent]).min_safety_margin_m

    assert small_margin - large_margin == pytest.approx(1.0)


def test_sdf_queries_fail_closed_outside_the_grid() -> None:
    sdf = SignedDistanceField.from_voxel_grid(_grid(np.zeros((4, 4, 4))))
    assert sdf.query(np.asarray((-0.01, 0.5, 0.5))) < 0
    assert sdf.query_batch(np.asarray(((0.5, 0.5, 0.5), (2.0, 0.5, 0.5))))[1] < 0


def test_invalid_trajectory_is_rejected() -> None:
    analyzer = SweptVolumeAnalyzer(np.asarray(((0, 0), (1, 0), (0, 1))))
    duplicate_times = Trajectory(
        trajectory_id="invalid",
        waypoints=(
            Waypoint(Pose2D(0, 0, 0), 0.0, 0.0),
            Waypoint(Pose2D(1, 0, 0), 0.0, 0.0),
        ),
        vehicle_class=VehicleClass.ROBOTAXI,
    )

    with pytest.raises(ValueError, match="strictly increasing"):
        analyzer.compute_swept_volume(duplicate_times)
