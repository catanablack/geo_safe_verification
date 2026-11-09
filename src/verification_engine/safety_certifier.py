"""Conservative geometric safety certification for sampled trajectories."""

from __future__ import annotations

import numpy as np

from src.common.config import SafetyConfig
from src.common.types import DynamicAgentForecast, SafetyVerdict, Trajectory, VoxelGrid
from src.verification_engine.signed_distance_field import SignedDistanceField
from src.verification_engine.swept_volume import SweptVolumeAnalyzer


class GeometricSafetyCertifier:
    """Certifies trajectories against voxel occupancy and forecast agent discs.

    The result is conservative for the modeled occupancy and deterministic
    linear agent forecasts. It is not a guarantee about unmodeled perception
    errors or agents outside the supplied forecast horizon.
    """

    def __init__(
        self,
        swept_volume_analyzer: SweptVolumeAnalyzer,
        config: SafetyConfig | None = None,
    ) -> None:
        config = config if config is not None else SafetyConfig()
        if (
            not np.isfinite(config.dynamic_agent_radius_m)
            or config.dynamic_agent_radius_m < 0
        ):
            raise ValueError("dynamic_agent_radius_m must be finite and non-negative")
        if (
            not np.isfinite(config.occupancy_threshold)
            or not 0 <= config.occupancy_threshold <= 1
        ):
            raise ValueError("occupancy_threshold must be between 0 and 1")
        self.swept_volume_analyzer = swept_volume_analyzer
        self.config = config

    def verify(
        self,
        trajectory: Trajectory,
        current_occupancy: VoxelGrid,
        forecasted_agents: list[DynamicAgentForecast],
        min_margin_m: float | None = None,
    ) -> SafetyVerdict:
        """Return a safety verdict with discretization and sampling bounds applied."""
        margin_threshold = (
            min_margin_m
            if min_margin_m is not None
            else self.config.min_safety_margin_m
        )
        if not np.isfinite(margin_threshold) or margin_threshold < 0:
            raise ValueError("min_margin_m must be finite and non-negative")

        step = current_occupancy.resolution_m / 2.0
        extra_times: list[float] = []
        forecast_segments: list[
            tuple[DynamicAgentForecast, np.ndarray, np.ndarray, float]
        ] = []
        trajectory_start = (
            trajectory.waypoints[0].timestamp_s if trajectory.waypoints else 0.0
        )
        trajectory_end = (
            trajectory.waypoints[-1].timestamp_s if trajectory.waypoints else 0.0
        )
        for forecast in forecasted_agents:
            positions = np.asarray(
                [(pose.x, pose.y) for pose in forecast.predicted_positions],
                dtype=np.float64,
            )
            if positions.ndim != 2 or positions.shape[1] != 2 or len(positions) == 0:
                raise ValueError(
                    f"forecast {forecast.agent_id!r} must contain at least one 2D position"
                )
            if not np.isfinite(positions).all():
                raise ValueError(
                    f"forecast {forecast.agent_id!r} has non-finite positions"
                )
            if not np.isfinite(forecast.horizon_s) or forecast.horizon_s < 0:
                raise ValueError(
                    f"forecast {forecast.agent_id!r} has an invalid horizon"
                )
            agent_radius = (
                forecast.footprint_radius_m
                if forecast.footprint_radius_m is not None
                else self.config.dynamic_agent_radius_m
            )
            if not np.isfinite(agent_radius) or agent_radius < 0:
                raise ValueError(
                    f"forecast {forecast.agent_id!r} has an invalid footprint radius"
                )
            if len(positions) > 1 and forecast.horizon_s <= 0:
                raise ValueError(
                    f"forecast {forecast.agent_id!r} needs a positive horizon for multiple positions"
                )

            forecast_times = (
                np.linspace(
                    trajectory_start,
                    trajectory_start + forecast.horizon_s,
                    len(positions),
                )
                if len(positions) > 1
                else np.asarray(
                    (trajectory_start, trajectory_start + forecast.horizon_s),
                    dtype=np.float64,
                )
            )
            forecast_segments.append(
                (forecast, positions, forecast_times, agent_radius)
            )
            for index in range(len(positions) - 1):
                distance = float(
                    np.linalg.norm(positions[index + 1] - positions[index])
                )
                count = max(1, int(np.ceil(distance / step)))
                segment_times = np.linspace(
                    forecast_times[index], forecast_times[index + 1], count + 1
                )
                extra_times.extend(
                    segment_times[
                        (segment_times >= trajectory_start)
                        & (segment_times <= trajectory_end)
                    ].tolist()
                )
            if len(positions) == 1:
                extra_times.extend(
                    value
                    for value in (
                        trajectory_start,
                        trajectory_start + forecast.horizon_s,
                    )
                    if trajectory_start <= value <= trajectory_end
                )

        points, point_times, sampling_error = (
            self.swept_volume_analyzer.sample_trajectory(
                trajectory,
                current_occupancy.resolution_m,
                extra_times,
            )
        )
        times = np.unique(point_times)
        points_by_time = points.reshape(len(times), -1, 3)
        sdf = SignedDistanceField.from_voxel_grid(
            current_occupancy, self.config.occupancy_threshold
        )
        point_clearances = sdf.query_batch_lower_bound(points)
        time_clearances = (
            point_clearances.reshape(len(times), -1).min(axis=1) - sampling_error
        )

        dynamic_sampling_error = sampling_error + step / 2.0
        for forecast, positions, forecast_times, agent_radius in forecast_segments:
            agent_clearances = np.full(len(times), -current_occupancy.resolution_m)
            covered = (times >= forecast_times[0]) & (times <= forecast_times[-1])
            if covered.any():
                centers = np.column_stack(
                    (
                        np.full(covered.sum(), positions[0, 0])
                        if len(positions) == 1
                        else np.interp(times[covered], forecast_times, positions[:, 0]),
                        np.full(covered.sum(), positions[0, 1])
                        if len(positions) == 1
                        else np.interp(times[covered], forecast_times, positions[:, 1]),
                    )
                )
                distances = np.linalg.norm(
                    points_by_time[covered, :, :2] - centers[:, None, :],
                    axis=2,
                )
                agent_clearances[covered] = (
                    distances.min(axis=1) - agent_radius - dynamic_sampling_error
                )
            time_clearances = np.minimum(time_clearances, agent_clearances)

        min_clearance = float(np.min(time_clearances))
        violating = np.flatnonzero(time_clearances <= margin_threshold)
        violation_time = float(times[violating[0]]) if len(violating) else None
        return SafetyVerdict(
            trajectory_id=trajectory.trajectory_id,
            is_safe=not len(violating),
            min_safety_margin_m=min_clearance,
            violation_timestamp_s=violation_time,
            evidence_ref=None,
        )
