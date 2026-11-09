"""Global configuration for the platform.

Centralizes tunables referenced by multiple modules (safety margins, voxel
resolution, forecast horizon, etc.) so they are not hard-coded per-module.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SafetyConfig:
    min_safety_margin_m: float = 0.5
    max_verification_latency_ms: float = 50.0
    occupancy_threshold: float = 0.5
    dynamic_agent_radius_m: float = 0.5


@dataclass(frozen=True)
class VoxelizationConfig:
    resolution_m: float = 0.1
    max_range_m: float = 100.0


@dataclass(frozen=True)
class ForecastConfig:
    horizon_s: float = 3.0
    time_step_s: float = 0.1


@dataclass(frozen=True)
class ArbitrationConfig:
    max_candidate_trajectories: int = 5
    comfort_weight: float = 0.3
    energy_weight: float = 0.2
    safety_weight: float = 0.5


@dataclass(frozen=True)
class PlatformConfig:
    safety: SafetyConfig = SafetyConfig()
    voxelization: VoxelizationConfig = VoxelizationConfig()
    forecast: ForecastConfig = ForecastConfig()
    arbitration: ArbitrationConfig = ArbitrationConfig()


DEFAULT_CONFIG = PlatformConfig()
