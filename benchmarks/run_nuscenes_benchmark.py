"""Run the nuScenes verifier and baseline benchmark."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmarks.nuscenes_eval import (
    NuScenesDataset,
    aggregate_metrics,
    evaluate_dataset,
    write_report,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "dataset_root", type=Path, help="Directory containing nuScenes JSON tables"
    )
    parser.add_argument(
        "--version",
        choices=("v1.0-mini", "v1.0-trainval"),
        default="v1.0-mini",
    )
    parser.add_argument("--horizon-steps", type=int, default=4)
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--max-windows", type=int, default=100)
    parser.add_argument(
        "--offsets", default="-1,0,1", help="Comma-separated lateral offsets in meters"
    )
    parser.add_argument("--margin-m", type=float, default=0.5)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    args = parser.parse_args()
    try:
        offsets = tuple(float(value) for value in args.offsets.split(","))
    except ValueError:
        parser.error("--offsets must be comma-separated numbers")
    if not offsets:
        parser.error("--offsets must not be empty")
    if args.margin_m < 0:
        parser.error("--margin-m must be non-negative")

    dataset = NuScenesDataset(args.dataset_root, version=args.version)
    output = args.output or (
        Path.home()
        / "datasets/nuscenes"
        / f"nuscenes-{args.version.removeprefix('v1.0-')}-results.json"
    )
    records = evaluate_dataset(
        dataset,
        horizon_steps=args.horizon_steps,
        stride=args.stride,
        max_windows=args.max_windows,
        lateral_offsets_m=offsets,
        margin_m=args.margin_m,
    )
    write_report(
        output,
        dataset,
        records,
        args.horizon_steps,
        args.stride,
        args.margin_m,
    )
    print(f"Dataset tables: {dataset.table_root}")
    print(f"Evaluated candidate trajectories: {len(records)}")
    print(f"Scenes represented: {len({record.scene_token for record in records})}")
    print(
        f"Ground-truth collisions: {sum(record.exact_collision for record in records)}"
    )
    print()
    print(
        f"{'system':<24} {'FA rate':>10} {'FR rate':>10} "
        f"{'margin MAE':>12} {'p50 ms':>10} {'p99 ms':>10}"
    )
    for name, result in aggregate_metrics(records).items():
        print(
            f"{name:<24} {result['false_accept_rate']:>10.3f} "
            f"{result['false_reject_rate']:>10.3f} "
            f"{result['margin_mae_m']:>12.3f} "
            f"{result['p50_latency_ms']:>10.3f} "
            f"{result['p99_latency_ms']:>10.3f}"
        )
    print(f"\nWrote JSON results to {output.expanduser().resolve()}")
    print(
        "Note: rss_style is a transparent RSS-inspired baseline, not a run of "
        "the ad-rss-lib implementation."
    )


if __name__ == "__main__":
    main()
