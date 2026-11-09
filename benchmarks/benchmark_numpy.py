"""Measure NumPy/SciPy verifier latency by voxel-grid and agent count."""

from __future__ import annotations

import argparse
import csv
import platform
import sys
import time
from pathlib import Path

import numpy as np
import scipy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

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
from src.verification_engine.swept_volume import SweptVolumeAnalyzer


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--grid-sizes", default="32,64,96")
    parser.add_argument("--agent-counts", default="0,5,20")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.iterations < 2:
        parser.error("--iterations must be at least 2")
    try:
        args.grid_sizes = [int(value) for value in args.grid_sizes.split(",")]
        args.agent_counts = [int(value) for value in args.agent_counts.split(",")]
    except ValueError:
        parser.error("grid sizes and agent counts must be comma-separated integers")
    if any(value <= 0 for value in args.grid_sizes) or any(
        value < 0 for value in args.agent_counts
    ):
        parser.error("grid sizes must be positive and agent counts non-negative")
    return args


def _run_case(
    grid_size: int, agent_count: int, iterations: int
) -> dict[str, float | int]:
    resolution = 0.5
    shape = (grid_size, grid_size, max(8, grid_size // 4))
    origin = (
        -grid_size * resolution / 2.0,
        -grid_size * resolution / 2.0,
        0.0,
    )
    rng = np.random.default_rng(grid_size * 1000 + agent_count)
    occupancy = (rng.random(shape) < 0.0005).astype(np.float32)
    grid = VoxelGrid(
        resolution,
        origin,
        occupancy,
        np.zeros(shape, dtype=np.float32),
    )
    footprint = np.asarray(((-1.0, -0.45), (1.0, -0.45), (1.0, 0.45), (-1.0, 0.45)))
    analyzer = SweptVolumeAnalyzer(footprint)
    certifier = GeometricSafetyCertifier(
        analyzer,
        SafetyConfig(min_safety_margin_m=0.5, dynamic_agent_radius_m=0.5),
    )
    trajectory = Trajectory(
        trajectory_id="numpy-latency",
        waypoints=(
            Waypoint(Pose2D(-1.5, 0.0, 0.0), 0.0, 1.5),
            Waypoint(Pose2D(0.0, 0.0, 0.0), 1.0, 1.5),
            Waypoint(Pose2D(1.5, 0.2, 0.05), 2.0, 1.5),
        ),
        vehicle_class=VehicleClass.ROBOTAXI,
    )
    agents = [
        DynamicAgentForecast(
            agent_id=f"agent-{index}",
            predicted_positions=(
                Pose2D(4.0, 4.0 + index * 0.1, 0.0),
                Pose2D(4.2, 4.0 + index * 0.1, 0.0),
                Pose2D(4.4, 4.0 + index * 0.1, 0.0),
            ),
            horizon_s=2.0,
            confidence=1.0,
        )
        for index in range(agent_count)
    ]

    certifier.verify(trajectory, grid, agents)
    latencies_ms = np.empty(iterations, dtype=np.float64)
    for index in range(iterations):
        started = time.perf_counter()
        certifier.verify(trajectory, grid, agents)
        latencies_ms[index] = (time.perf_counter() - started) * 1000.0
    return {
        "grid_size": grid_size,
        "grid_voxels": int(np.prod(shape)),
        "agent_count": agent_count,
        "p50_ms": float(np.percentile(latencies_ms, 50)),
        "p99_ms": float(np.percentile(latencies_ms, 99)),
        "iterations": iterations,
    }


def main() -> None:
    args = _arguments()
    results = [
        _run_case(grid_size, agent_count, args.iterations)
        for grid_size in args.grid_sizes
        for agent_count in args.agent_counts
    ]
    print(
        f"Python {sys.version.split()[0]} | NumPy {np.__version__} | "
        f"SciPy {scipy.__version__} | {platform.platform()}"
    )
    print("grid_size  grid_voxels  agents  p50_ms  p99_ms")
    for result in results:
        print(
            f"{result['grid_size']:>9}  {result['grid_voxels']:>11}  "
            f"{result['agent_count']:>6}  {result['p50_ms']:>6.2f}  "
            f"{result['p99_ms']:>6.2f}"
        )
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=list(results[0]))
            writer.writeheader()
            writer.writerows(results)
        print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
