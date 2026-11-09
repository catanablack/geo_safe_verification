from __future__ import annotations

import io
import json
import tarfile
from types import SimpleNamespace

from benchmarks.nuscenes_eval import (
    Box2D,
    NuScenesDataset,
    NuScenesFrame,
    aggregate_metrics,
    evaluate_dataset,
    evaluate_window,
    signed_box_clearance,
)
from benchmarks.prepare_nuscenes_metadata import TABLES, extract_tables
from src.common.types import Pose2D


def _box(x: float, y: float, yaw: float = 0.0) -> Box2D:
    return Box2D(x, y, 0.75, 2.0, 4.0, 1.5, yaw, "agent")


def test_exact_box_clearance_handles_overlap_separation_and_rotation() -> None:
    ego = Box2D(0.0, 0.0, 0.75, 1.9, 4.8, 1.5, 0.0, "ego")
    assert signed_box_clearance(ego, _box(2.0, 0.0)) < 0
    assert signed_box_clearance(ego, _box(10.0, 0.0)) > 0
    assert signed_box_clearance(ego, _box(0.0, 0.0, 0.7)) < 0
    assert signed_box_clearance(ego, _box(0.0, 0.0, 0.0)) < 0


def test_nuscenes_json_loader_follows_linked_scene_samples(tmp_path) -> None:
    table_root = tmp_path / "v1.0-mini"
    table_root.mkdir()
    sample_data = []
    samples = []
    ego_poses = []
    for index in range(2):
        sample_token = f"sample-{index}"
        pose_token = f"pose-{index}"
        lidar_token = f"lidar-{index}"
        samples.append(
            {
                "token": sample_token,
                "timestamp": 1_000_000 + index * 500_000,
                "data": {"LIDAR_TOP": lidar_token},
                "next": "sample-1" if index == 0 else "",
            }
        )
        sample_data.append(
            {
                "token": lidar_token,
                "sample_token": sample_token,
                "ego_pose_token": pose_token,
                "is_key_frame": True,
                "filename": f"samples/LIDAR_TOP/{lidar_token}.pcd",
            }
        )
        ego_poses.append(
            {
                "token": pose_token,
                "translation": [float(index), 0.0, 0.0],
                "rotation": [1.0, 0.0, 0.0, 0.0],
            }
        )
    tables = {
        "sample.json": samples,
        "sample_data.json": sample_data,
        "ego_pose.json": ego_poses,
        "scene.json": [{"token": "scene-0", "first_sample_token": "sample-0"}],
        "sample_annotation.json": [],
    }
    for name, records in tables.items():
        (table_root / name).write_text(json.dumps(records), encoding="utf-8")

    dataset = NuScenesDataset(tmp_path)
    assert dataset.version == "v1.0-mini"
    frames = dataset.scene_frames(dataset.scenes["scene-0"])

    assert len(frames) == 2
    assert frames[1].ego_pose.x == 1.0
    assert frames[1].timestamp_s - frames[0].timestamp_s == 0.5


def test_archive_extractor_writes_only_required_mini_tables(tmp_path) -> None:
    archive_path = tmp_path / "mini.tgz"
    output = tmp_path / "tables"
    with tarfile.open(archive_path, "w:gz") as archive:
        for name in TABLES:
            data = name.encode("utf-8")
            info = tarfile.TarInfo(f"public/v1.0/v1.0-mini/{name}")
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
        unrelated = tarfile.TarInfo("samples/large-sensor-file.bin")
        unrelated.size = 4
        archive.addfile(unrelated, io.BytesIO(b"data"))

    extracted = extract_tables(archive_path, output)

    assert extracted == TABLES
    assert {path.name for path in output.iterdir()} == TABLES


def test_archive_extractor_selects_trainval_tables(tmp_path) -> None:
    archive_path = tmp_path / "trainval.tgz"
    output = tmp_path / "tables"
    with tarfile.open(archive_path, "w:gz") as archive:
        for version in ("v1.0-mini", "v1.0-trainval"):
            for name in TABLES:
                data = f"{version}/{name}".encode()
                info = tarfile.TarInfo(f"public/v1.0/{version}/{name}")
                info.size = len(data)
                archive.addfile(info, io.BytesIO(data))

    extracted = extract_tables(archive_path, output, "v1.0-trainval")

    assert extracted == TABLES
    assert all(
        path.read_text(encoding="utf-8").startswith("v1.0-trainval/")
        for path in output.iterdir()
    )


def test_evaluation_spreads_window_budget_across_scenes(monkeypatch) -> None:
    class Dataset:
        def __init__(self):
            self.scenes = {
                f"scene-{index}": {"token": f"scene-{index}"} for index in range(4)
            }

        def scene_frames(self, scene):
            return list(range(4))

    monkeypatch.setattr(
        "benchmarks.nuscenes_eval.evaluate_window",
        lambda scene_token, *_args: SimpleNamespace(scene_token=scene_token),
    )

    records = evaluate_dataset(
        Dataset(), horizon_steps=1, stride=1, max_windows=2, lateral_offsets_m=(0.0,)
    )

    assert [record.scene_token for record in records] == ["scene-0", "scene-3"]


def test_evaluation_reports_all_baselines_and_exact_collision_label() -> None:
    frames = [
        NuScenesFrame(
            token=f"sample-{index}",
            timestamp_s=index * 0.5,
            ego_pose=Pose2D(float(index) * 0.1, 0.0, 0.0),
            ego_z_m=0.0,
            objects=(_box(1.0 + index * 0.1, 0.0),),
        )
        for index in range(3)
    ]

    record = evaluate_window("scene-0", frames, 0.0)
    metrics = aggregate_metrics([record])

    assert record.exact_collision
    assert set(metrics) == {
        "geometric_verifier",
        "distance_threshold",
        "time_to_collision",
        "rss_style",
    }
    assert metrics["geometric_verifier"]["samples"] == 1
    assert metrics["geometric_verifier"]["p99_latency_ms"] > 0
    assert metrics["distance_threshold"]["p50_latency_ms"] > 0
    assert metrics["time_to_collision"]["p50_latency_ms"] > 0
    assert metrics["rss_style"]["p50_latency_ms"] > 0


def test_evaluation_rejects_empty_metrics() -> None:
    try:
        aggregate_metrics([])
    except ValueError as error:
        assert "empty" in str(error)
    else:
        raise AssertionError("empty metrics should fail")
