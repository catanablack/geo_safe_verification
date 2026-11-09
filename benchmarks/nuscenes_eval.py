"""nuScenes parser, exact box labels, baselines, and verifier comparison."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

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


@dataclass(frozen=True)
class Box2D:
    center_x: float
    center_y: float
    center_z: float
    width_m: float
    length_m: float
    height_m: float
    yaw_rad: float
    instance_token: str

    @property
    def footprint_radius_m(self) -> float:
        return math.hypot(self.length_m / 2.0, self.width_m / 2.0)


@dataclass(frozen=True)
class NuScenesFrame:
    token: str
    timestamp_s: float
    ego_pose: Pose2D
    ego_z_m: float
    objects: tuple[Box2D, ...]


@dataclass(frozen=True)
class EvaluationRecord:
    scene_token: str
    sample_token: str
    lateral_offset_m: float
    exact_collision: bool
    exact_min_clearance_m: float
    systems: dict[str, tuple[bool, float, float]]


def _yaw_from_quaternion(value: list[float]) -> float:
    if len(value) != 4 or not np.isfinite(value).all():
        raise ValueError("nuScenes quaternion must contain four finite values")
    w, x, y, z = value
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class NuScenesDataset:
    """Read nuScenes JSON tables without loading sensor blobs or the SDK."""

    def __init__(self, dataset_root: str | Path, version: str = "v1.0-mini") -> None:
        root = Path(dataset_root).expanduser().resolve()
        table_root = root / version
        if not table_root.is_dir() and (
            root.name == version or (root / "scene.json").is_file()
        ):
            table_root = root
        if not table_root.is_dir():
            raise FileNotFoundError(
                f"nuScenes tables not found at {table_root}; run "
                "benchmarks/prepare_nuscenes_metadata.py on the official archive"
            )
        self.table_root = table_root
        self.version = version
        self.samples = self._load_table("sample")
        self.scenes = self._load_table("scene")
        self.sample_data = self._load_table("sample_data")
        self.ego_poses = self._load_table("ego_pose")
        self.annotations = self._load_table("sample_annotation")
        sample_data_by_sample: dict[str, list[dict[str, Any]]] = {}
        for sample_data in self.sample_data.values():
            sample_data_by_sample.setdefault(sample_data["sample_token"], []).append(
                sample_data
            )
        self.sample_data_by_sample = sample_data_by_sample
        annotations_by_sample: dict[str, list[dict[str, Any]]] = {}
        for annotation in self.annotations.values():
            annotations_by_sample.setdefault(annotation["sample_token"], []).append(
                annotation
            )
        self.annotations_by_sample = annotations_by_sample

    def _load_table(self, name: str) -> dict[str, dict[str, Any]]:
        path = self.table_root / f"{name}.json"
        if not path.is_file():
            raise FileNotFoundError(f"required nuScenes table is missing: {path}")
        records = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(records, list):
            raise TypeError(f"nuScenes table must contain a JSON list: {path}")
        try:
            return {record["token"]: record for record in records}
        except (KeyError, TypeError) as error:
            raise ValueError(f"invalid record in nuScenes table: {path}") from error

    def scene_frames(self, scene: dict[str, Any]) -> list[NuScenesFrame]:
        frames: list[NuScenesFrame] = []
        token = scene["first_sample_token"]
        while token:
            sample = self.samples[token]
            lidar_frames = [
                data
                for data in self.sample_data_by_sample.get(token, ())
                if data["is_key_frame"] and "LIDAR_TOP" in data["filename"]
            ]
            if not lidar_frames:
                raise ValueError(
                    f"sample {token} has no keyframe LIDAR_TOP sample_data"
                )
            ego_pose_token = lidar_frames[0]["ego_pose_token"]
            ego = self.ego_poses[ego_pose_token]
            translation = ego["translation"]
            objects = tuple(
                Box2D(
                    center_x=float(annotation["translation"][0]),
                    center_y=float(annotation["translation"][1]),
                    center_z=float(annotation["translation"][2]),
                    width_m=float(annotation["size"][0]),
                    length_m=float(annotation["size"][1]),
                    height_m=float(annotation["size"][2]),
                    yaw_rad=_yaw_from_quaternion(annotation["rotation"]),
                    instance_token=annotation["instance_token"],
                )
                for annotation in self.annotations_by_sample.get(token, ())
            )
            frames.append(
                NuScenesFrame(
                    token=token,
                    timestamp_s=float(sample["timestamp"]) * 1e-6,
                    ego_pose=Pose2D(
                        float(translation[0]),
                        float(translation[1]),
                        _yaw_from_quaternion(ego["rotation"]),
                    ),
                    ego_z_m=float(translation[2]),
                    objects=objects,
                )
            )
            token = sample["next"]
        return frames


def _rectangle(box: Box2D) -> np.ndarray:
    half_length = box.length_m / 2.0
    half_width = box.width_m / 2.0
    local = np.asarray(
        (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
        )
    )
    cosine, sine = math.cos(box.yaw_rad), math.sin(box.yaw_rad)
    rotation = np.asarray(((cosine, -sine), (sine, cosine)))
    return local @ rotation.T + np.asarray((box.center_x, box.center_y))


def _projection_overlap(
    first: np.ndarray, second: np.ndarray, axis: np.ndarray
) -> float:
    first_projection = first @ axis
    second_projection = second @ axis
    return float(
        min(first_projection.max(), second_projection.max())
        - max(first_projection.min(), second_projection.min())
    )


def _point_segment_distance(
    point: np.ndarray, start: np.ndarray, end: np.ndarray
) -> float:
    direction = end - start
    denominator = float(direction @ direction)
    if denominator == 0:
        return float(np.linalg.norm(point - start))
    fraction = float(np.clip((point - start) @ direction / denominator, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + fraction * direction)))


def _point_in_rectangle(point: np.ndarray, rectangle: np.ndarray) -> bool:
    for start, end in zip(rectangle, np.roll(rectangle, -1, axis=0)):
        edge = end - start
        relative = point - start
        if edge[0] * relative[1] - edge[1] * relative[0] < -1e-10:
            return False
    return True


def _polygon_distance(first: np.ndarray, second: np.ndarray) -> float:
    if _point_in_rectangle(first[0], second) or _point_in_rectangle(second[0], first):
        return 0.0
    distances = [
        _point_segment_distance(point, start, end)
        for polygon, other in ((first, second), (second, first))
        for point in polygon
        for start, end in zip(other, np.roll(other, -1, axis=0))
    ]
    return min(distances)


def signed_box_clearance(ego: Box2D, agent: Box2D) -> float:
    """Return exact signed separation for yaw-oriented rectangular prisms."""
    ego_polygon = _rectangle(ego)
    agent_polygon = _rectangle(agent)
    axes = []
    for polygon in (ego_polygon, agent_polygon):
        for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
            edge = end - start
            axes.append(np.asarray((-edge[1], edge[0])) / np.linalg.norm(edge))

    overlaps = [_projection_overlap(ego_polygon, agent_polygon, axis) for axis in axes]
    ego_z_min, ego_z_max = (
        ego.center_z - ego.height_m / 2,
        ego.center_z + ego.height_m / 2,
    )
    agent_z_min = agent.center_z - agent.height_m / 2
    agent_z_max = agent.center_z + agent.height_m / 2
    z_overlap = min(ego_z_max, agent_z_max) - max(ego_z_min, agent_z_min)
    if min(overlaps) >= 0 and z_overlap >= 0:
        return -min(min(overlaps), z_overlap)

    xy_distance = _polygon_distance(ego_polygon, agent_polygon)
    z_distance = max(ego_z_min - agent_z_max, agent_z_min - ego_z_max, 0.0)
    return math.hypot(xy_distance, z_distance)


def _shifted_pose(pose: Pose2D, offset_m: float) -> Pose2D:
    return Pose2D(
        pose.x - math.sin(pose.heading_rad) * offset_m,
        pose.y + math.cos(pose.heading_rad) * offset_m,
        pose.heading_rad,
    )


def _ego_box(frame: NuScenesFrame, pose: Pose2D) -> Box2D:
    return Box2D(
        pose.x,
        pose.y,
        frame.ego_z_m + 0.75,
        width_m=1.9,
        length_m=4.8,
        height_m=1.5,
        yaw_rad=pose.heading_rad,
        instance_token="ego",
    )


def _sample_trajectory(
    frames: list[NuScenesFrame], offset_m: float, trajectory_id: str
) -> Trajectory:
    times = np.asarray([frame.timestamp_s for frame in frames])
    start = times[0]
    waypoints: list[Waypoint] = []
    shifted = [_shifted_pose(frame.ego_pose, offset_m) for frame in frames]
    for index, (frame, pose) in enumerate(zip(frames, shifted)):
        if index + 1 < len(frames):
            next_pose = shifted[index + 1]
            velocity = math.hypot(next_pose.x - pose.x, next_pose.y - pose.y) / (
                times[index + 1] - times[index]
            )
        elif index > 0:
            prior_pose = shifted[index - 1]
            velocity = math.hypot(pose.x - prior_pose.x, pose.y - prior_pose.y) / (
                times[index] - times[index - 1]
            )
        else:
            velocity = 0.0
        waypoints.append(Waypoint(pose, float(times[index] - start), float(velocity)))
    return Trajectory(trajectory_id, waypoints, VehicleClass.ROBOTAXI)


def _forecasts(frames: list[NuScenesFrame]) -> list[DynamicAgentForecast]:
    tracks: dict[str, list[Box2D | None]] = {}
    for frame_index, frame in enumerate(frames):
        for box in frame.objects:
            tracks.setdefault(box.instance_token, [None] * len(frames))[frame_index] = (
                box
            )

    forecasts = []
    horizon_s = frames[-1].timestamp_s - frames[0].timestamp_s
    for instance_token, boxes in tracks.items():
        known = [index for index, box in enumerate(boxes) if box is not None]
        if not known:
            continue
        positions = np.asarray(
            [
                (
                    np.interp(index, known, [boxes[k].center_x for k in known]),
                    np.interp(index, known, [boxes[k].center_y for k in known]),
                )
                for index in range(len(frames))
            ]
        )
        radius = max(boxes[index].footprint_radius_m for index in known)
        forecasts.append(
            DynamicAgentForecast(
                agent_id=instance_token,
                predicted_positions=tuple(
                    Pose2D(float(x), float(y), 0.0) for x, y in positions
                ),
                horizon_s=horizon_s,
                confidence=1.0,
                footprint_radius_m=radius,
            )
        )
    return forecasts


def _empty_grid_for(trajectory: Trajectory, resolution_m: float = 0.5) -> VoxelGrid:
    xy = np.asarray(
        [(waypoint.pose.x, waypoint.pose.y) for waypoint in trajectory.waypoints]
    )
    padding = 8.0
    lower = xy.min(axis=0) - padding
    upper = xy.max(axis=0) + padding
    shape_xy = np.maximum(1, np.ceil((upper - lower) / resolution_m).astype(int))
    origin = (float(lower[0]), float(lower[1]), -5.0)
    shape = (int(shape_xy[0]), int(shape_xy[1]), 32)
    return VoxelGrid(
        resolution_m,
        origin,
        np.zeros(shape, dtype=np.float32),
        np.zeros(shape, dtype=np.float32),
    )


def _exact_min_clearance(frames: list[NuScenesFrame], offset_m: float) -> float:
    clearances = []
    for frame in frames:
        ego = _ego_box(frame, _shifted_pose(frame.ego_pose, offset_m))
        clearances.extend(signed_box_clearance(ego, box) for box in frame.objects)
    return min(clearances, default=100.0)


def distance_threshold_baseline(
    frames: list[NuScenesFrame], offset_m: float, margin_m: float = 0.5
) -> tuple[bool, float]:
    ego_radius = math.hypot(4.8 / 2.0, 1.9 / 2.0)
    clearances = []
    for frame in frames:
        ego = _shifted_pose(frame.ego_pose, offset_m)
        for box in frame.objects:
            center_distance = math.hypot(ego.x - box.center_x, ego.y - box.center_y)
            clearances.append(center_distance - ego_radius - box.footprint_radius_m)
    clearance = min(clearances, default=100.0)
    return clearance > margin_m, clearance


def ttc_baseline(
    frames: list[NuScenesFrame], offset_m: float, margin_m: float = 0.5
) -> tuple[bool, float]:
    ego_radius = math.hypot(4.8 / 2.0, 1.9 / 2.0)
    ego_poses = [_shifted_pose(frame.ego_pose, offset_m) for frame in frames]
    ego_start = np.asarray((ego_poses[0].x, ego_poses[0].y))
    duration = frames[-1].timestamp_s - frames[0].timestamp_s
    if duration <= 0:
        raise ValueError("TTC baseline needs a positive trajectory duration")
    ego_velocity = (
        np.asarray((ego_poses[-1].x, ego_poses[-1].y)) - ego_start
    ) / duration
    minimum_clearance = math.inf
    collision_predicted = False

    for instance_token in {
        box.instance_token for frame in frames for box in frame.objects
    }:
        boxes = [
            next(
                (box for box in frame.objects if box.instance_token == instance_token),
                None,
            )
            for frame in frames
        ]
        indices = [index for index, box in enumerate(boxes) if box is not None]
        if not indices:
            continue
        first, last = indices[0], indices[-1]
        first_box, last_box = boxes[first], boxes[last]
        assert first_box is not None and last_box is not None
        elapsed = frames[last].timestamp_s - frames[first].timestamp_s
        object_velocity = (
            (
                np.asarray((last_box.center_x, last_box.center_y))
                - np.asarray((first_box.center_x, first_box.center_y))
            )
            / elapsed
            if elapsed > 0
            else np.zeros(2)
        )
        ego_position_at_first = np.asarray((ego_poses[first].x, ego_poses[first].y))
        relative_position = (
            np.asarray((first_box.center_x, first_box.center_y)) - ego_position_at_first
        )
        relative_velocity = object_velocity - ego_velocity
        radius = ego_radius + max(box.footprint_radius_m for box in boxes if box)
        remaining_duration = frames[-1].timestamp_s - frames[first].timestamp_s
        a = float(relative_velocity @ relative_velocity)
        b = 2.0 * float(relative_position @ relative_velocity)
        c = float(relative_position @ relative_position) - radius**2
        if c <= 0:
            collision_predicted = True
            minimum_clearance = min(minimum_clearance, -radius)
        elif a > 0:
            discriminant = b * b - 4.0 * a * c
            if discriminant >= 0:
                contact_time = (-b - math.sqrt(discriminant)) / (2.0 * a)
                if 0 <= contact_time <= remaining_duration:
                    collision_predicted = True
            closest_time = float(np.clip(-b / (2.0 * a), 0.0, remaining_duration))
            closest_distance = np.linalg.norm(
                relative_position + closest_time * relative_velocity
            )
            minimum_clearance = min(minimum_clearance, float(closest_distance - radius))
        else:
            minimum_clearance = min(
                minimum_clearance, float(np.linalg.norm(relative_position) - radius)
            )

    clearance = minimum_clearance if math.isfinite(minimum_clearance) else 100.0
    return not collision_predicted and clearance > margin_m, clearance


def rss_style_baseline(
    frames: list[NuScenesFrame],
    offset_m: float,
    margin_m: float = 0.5,
    response_time_s: float = 1.0,
    max_accel_mps2: float = 2.0,
    ego_brake_mps2: float = 4.0,
    agent_brake_mps2: float = 8.0,
) -> tuple[bool, float]:
    """A transparent RSS-inspired longitudinal safe-distance baseline."""
    ego_poses = [_shifted_pose(frame.ego_pose, offset_m) for frame in frames]
    minimum_slack = math.inf
    for index, (frame, ego_pose) in enumerate(zip(frames, ego_poses)):
        forward = np.asarray(
            (math.cos(ego_pose.heading_rad), math.sin(ego_pose.heading_rad))
        )
        left = np.asarray((-forward[1], forward[0]))
        if index + 1 < len(frames):
            delta_time = frames[index + 1].timestamp_s - frame.timestamp_s
            ego_velocity = (
                np.asarray((ego_poses[index + 1].x, ego_poses[index + 1].y))
                - np.asarray((ego_pose.x, ego_pose.y))
            ) / delta_time
        elif index:
            delta_time = frame.timestamp_s - frames[index - 1].timestamp_s
            ego_velocity = (
                np.asarray((ego_pose.x, ego_pose.y))
                - np.asarray((ego_poses[index - 1].x, ego_poses[index - 1].y))
            ) / delta_time
        else:
            ego_velocity = np.zeros(2)
        ego_speed = max(0.0, float(ego_velocity @ forward))

        for box in frame.objects:
            relative = np.asarray(
                (box.center_x - ego_pose.x, box.center_y - ego_pose.y)
            )
            longitudinal = float(relative @ forward)
            lateral = abs(float(relative @ left))
            if longitudinal < 0 or lateral > 1.9 / 2.0 + box.footprint_radius_m:
                continue
            relative_heading = box.yaw_rad - ego_pose.heading_rad
            agent_half_length = (
                abs(math.cos(relative_heading)) * box.length_m / 2.0
                + abs(math.sin(relative_heading)) * box.width_m / 2.0
            )
            gap = longitudinal - 4.8 / 2.0 - agent_half_length
            agent_index = min(index + 1, len(frames) - 1)
            next_box = next(
                (
                    candidate
                    for candidate in frames[agent_index].objects
                    if candidate.instance_token == box.instance_token
                ),
                box,
            )
            interval = frames[agent_index].timestamp_s - frame.timestamp_s
            agent_speed = (
                max(
                    0.0,
                    float(
                        np.asarray(
                            (
                                next_box.center_x - box.center_x,
                                next_box.center_y - box.center_y,
                            )
                        )
                        @ forward
                        / interval
                    ),
                )
                if interval > 0
                else 0.0
            )
            speed_after_response = ego_speed + response_time_s * max_accel_mps2
            safe_distance = max(
                0.0,
                ego_speed * response_time_s
                + 0.5 * max_accel_mps2 * response_time_s**2
                + speed_after_response**2 / (2.0 * ego_brake_mps2)
                - agent_speed**2 / (2.0 * agent_brake_mps2),
            )
            minimum_slack = min(minimum_slack, gap - safe_distance)
    slack = minimum_slack if math.isfinite(minimum_slack) else 100.0
    return slack > margin_m, slack


def evaluate_window(
    scene_token: str,
    frames: list[NuScenesFrame],
    lateral_offset_m: float,
    margin_m: float = 0.5,
) -> EvaluationRecord:
    if len(frames) < 2:
        raise ValueError("evaluation windows require at least two frames")
    trajectory = _sample_trajectory(
        frames, lateral_offset_m, f"{frames[0].token}-offset-{lateral_offset_m:g}"
    )
    forecasts = _forecasts(frames)
    grid = _empty_grid_for(trajectory)
    analyzer = SweptVolumeAnalyzer(
        np.asarray(((-2.4, -0.95), (2.4, -0.95), (2.4, 0.95), (-2.4, 0.95))),
        z_min_m=-0.75,
        z_max_m=0.75,
    )
    started = time.perf_counter()
    verdict = GeometricSafetyCertifier(
        analyzer,
        SafetyConfig(
            min_safety_margin_m=margin_m,
            dynamic_agent_radius_m=0.5,
        ),
    ).verify(trajectory, grid, forecasts, margin_m)
    verifier_latency_ms = (time.perf_counter() - started) * 1000.0

    exact_margin = _exact_min_clearance(frames, lateral_offset_m)
    started = time.perf_counter()
    distance_result = distance_threshold_baseline(frames, lateral_offset_m, margin_m)
    distance_latency_ms = (time.perf_counter() - started) * 1000.0
    started = time.perf_counter()
    ttc_result = ttc_baseline(frames, lateral_offset_m, margin_m)
    ttc_latency_ms = (time.perf_counter() - started) * 1000.0
    started = time.perf_counter()
    rss_result = rss_style_baseline(frames, lateral_offset_m, margin_m)
    rss_latency_ms = (time.perf_counter() - started) * 1000.0
    systems = {
        "geometric_verifier": (
            verdict.is_safe,
            verdict.min_safety_margin_m,
            verifier_latency_ms,
        ),
        "distance_threshold": (
            *distance_result,
            distance_latency_ms,
        ),
        "time_to_collision": (
            *ttc_result,
            ttc_latency_ms,
        ),
        "rss_style": (*rss_result, rss_latency_ms),
    }
    return EvaluationRecord(
        scene_token=scene_token,
        sample_token=frames[0].token,
        lateral_offset_m=lateral_offset_m,
        exact_collision=exact_margin <= 0.0,
        exact_min_clearance_m=exact_margin,
        systems=systems,
    )


def aggregate_metrics(
    records: list[EvaluationRecord],
) -> dict[str, dict[str, float | int]]:
    if not records:
        raise ValueError("cannot aggregate an empty evaluation set")
    systems = records[0].systems.keys()
    results: dict[str, dict[str, float | int]] = {}
    for name in systems:
        false_accepts = 0
        false_rejects = 0
        latencies = []
        margin_errors = []
        for record in records:
            predicted_safe, reported_margin, latency = record.systems[name]
            false_accepts += int(record.exact_collision and predicted_safe)
            false_rejects += int(not record.exact_collision and not predicted_safe)
            if latency > 0:
                latencies.append(latency)
            margin_errors.append(abs(reported_margin - record.exact_min_clearance_m))
        positives = sum(record.exact_collision for record in records)
        negatives = len(records) - positives
        results[name] = {
            "samples": len(records),
            "ground_truth_collisions": positives,
            "false_accepts": false_accepts,
            "false_accept_rate": false_accepts / positives if positives else 0.0,
            "false_rejects": false_rejects,
            "false_reject_rate": false_rejects / negatives if negatives else 0.0,
            "margin_mae_m": float(np.mean(margin_errors)),
            "p50_latency_ms": float(np.percentile(latencies, 50)) if latencies else 0.0,
            "p99_latency_ms": float(np.percentile(latencies, 99)) if latencies else 0.0,
        }
    return results


def evaluate_dataset(
    dataset: NuScenesDataset,
    horizon_steps: int = 4,
    stride: int = 4,
    max_windows: int = 100,
    lateral_offsets_m: tuple[float, ...] = (-1.0, 0.0, 1.0),
    margin_m: float = 0.5,
) -> list[EvaluationRecord]:
    if horizon_steps < 1 or stride < 1 or max_windows < 1:
        raise ValueError("horizon_steps, stride, and max_windows must be positive")
    records: list[EvaluationRecord] = []
    frames_by_scene: dict[str, list[NuScenesFrame]] = {}
    windows_seen = 0
    window_index = 0
    scenes = list(dataset.scenes.values())
    if max_windows < len(scenes):
        selected_indices = np.linspace(0, len(scenes) - 1, num=max_windows, dtype=int)
        scenes = [scenes[index] for index in selected_indices]
    while windows_seen < max_windows:
        added_window = False
        for scene in scenes:
            scene_token = scene["token"]
            if scene_token not in frames_by_scene:
                frames_by_scene[scene_token] = dataset.scene_frames(scene)
            frames = frames_by_scene[scene_token]
            start = window_index * stride
            if start + horizon_steps >= len(frames):
                continue
            window = frames[start : start + horizon_steps + 1]
            for offset in lateral_offsets_m:
                records.append(evaluate_window(scene_token, window, offset, margin_m))
            windows_seen += 1
            added_window = True
            if windows_seen >= max_windows:
                return records
        if not added_window:
            break
        window_index += 1
    return records


def write_report(
    output_path: str | Path,
    dataset: NuScenesDataset,
    records: list[EvaluationRecord],
    horizon_steps: int,
    stride: int,
    margin_m: float,
) -> None:
    metrics = aggregate_metrics(records)
    report = {
        "dataset": "nuScenes",
        "version": dataset.version,
        "dataset_root": str(dataset.table_root),
        "dataset_license": "non-commercial use only; see nuScenes terms",
        "frames_per_window": horizon_steps + 1,
        "window_stride_frames": stride,
        "requested_margin_m": margin_m,
        "exact_label": "yaw-oriented 3D rectangular-prism intersection at annotated 2 Hz sample times",
        "baseline_notes": {
            "distance_threshold": "minimum center distance minus enclosing-circle radii",
            "time_to_collision": "constant relative velocity and enclosing-circle contact",
            "rss_style": "transparent RSS-inspired longitudinal safe-distance rule; not the ad-rss-lib implementation",
        },
        "metrics": metrics,
        "records": [
            {
                "scene_token": record.scene_token,
                "sample_token": record.sample_token,
                "lateral_offset_m": record.lateral_offset_m,
                "exact_collision": record.exact_collision,
                "exact_min_clearance_m": record.exact_min_clearance_m,
                "systems": {
                    name: {
                        "safe": values[0],
                        "reported_margin_m": values[1],
                        "latency_ms": values[2],
                    }
                    for name, values in record.systems.items()
                },
            }
            for record in records
        ],
    }
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
