"""Shared data types used across every module of the platform.

These dataclasses form the contract boundary between modules (spatial_fusion,
predictive_occupancy, verification_engine, trajectory_arbitration,
simulation_harness, cross_platform, api). Keep this module free of any
module-specific logic.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence, Tuple

import numpy as np


class VehicleClass(str, Enum):
    """Cross-platform generalization categories (see cross_platform module)."""

    ROBOTAXI = "robotaxi"
    TRUCKING = "trucking"
    LAST_MILE_DELIVERY = "last_mile_delivery"


@dataclass(frozen=True)
class Pose2D:
    """Vehicle or agent pose on the ground plane."""

    x: float
    y: float
    heading_rad: float


@dataclass(frozen=True)
class Waypoint:
    """A single timestamped point along a candidate trajectory."""

    pose: Pose2D
    timestamp_s: float
    velocity_mps: float


@dataclass(frozen=True)
class Trajectory:
    """A planner-agnostic candidate trajectory submitted for verification."""

    trajectory_id: str
    waypoints: Sequence[Waypoint]
    vehicle_class: VehicleClass


@dataclass(frozen=True)
class VoxelGrid:
    """Probabilistic 3D occupancy grid produced by the spatial fusion layer.

    `occupancy` and `uncertainty` are arrays of shape (X, Y, Z) with values in
    [0, 1]: occupancy is the probability a voxel is physically occupied,
    uncertainty reflects sensor noise / confidence in that estimate.
    """

    resolution_m: float
    origin: Tuple[float, float, float]
    occupancy: np.ndarray
    uncertainty: np.ndarray


@dataclass(frozen=True)
class DynamicAgentForecast:
    """Predicted future footprint for a single dynamic agent (Phase 2).

    ``footprint_radius_m`` optionally provides a conservative circular bound
    for consumers that need to account for the agent's physical extent.
    """

    agent_id: str
    predicted_positions: Sequence[Pose2D]
    horizon_s: float
    confidence: float
    footprint_radius_m: float | None = None


@dataclass(frozen=True)
class SafetyVerdict:
    """Result returned by the geometric collision-safety verification engine."""

    trajectory_id: str
    is_safe: bool
    min_safety_margin_m: float
    violation_timestamp_s: Optional[float]
    evidence_ref: Optional[str] = None
    """Pointer to a stored SDF/swept-volume evidence artifact, for regulatory
    safety-case packaging (see PROJECT_PLAN.md 6.2 Regulatory Evidence Packaging)."""
