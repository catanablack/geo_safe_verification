"""Extract only the required nuScenes JSON tables from an official archive."""

from __future__ import annotations

import argparse
import tarfile
from pathlib import Path

TABLES = {
    "ego_pose.json",
    "sample.json",
    "sample_annotation.json",
    "sample_data.json",
    "scene.json",
}


def extract_tables(archive: Path, output: Path, version: str = "v1.0-mini") -> set[str]:
    output.mkdir(parents=True, exist_ok=True)
    extracted: set[str] = set()
    with tarfile.open(archive, mode="r:gz") as source:
        for member in source:
            parts = Path(member.name).parts
            if (
                not member.isfile()
                or not parts
                or parts[-1] not in TABLES
                or version not in parts[:-1]
            ):
                continue
            if parts[-1] in extracted:
                raise ValueError(f"duplicate nuScenes table in archive: {parts[-1]}")
            stream = source.extractfile(member)
            if stream is None:
                raise OSError(f"could not read archive member: {member.name}")
            destination = output / parts[-1]
            destination.write_bytes(stream.read())
            extracted.add(parts[-1])
            print(f"Extracted {destination.name}")
    return extracted


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path, help="Path to a nuScenes archive")
    parser.add_argument("output", type=Path, help="Directory for JSON tables")
    parser.add_argument(
        "--version",
        choices=("v1.0-mini", "v1.0-trainval"),
        default="v1.0-mini",
        help="Dataset table directory in the archive (default: v1.0-mini)",
    )
    args = parser.parse_args()
    archive = args.archive.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not archive.is_file():
        parser.error(f"nuScenes archive does not exist: {archive}")
    extracted = extract_tables(archive, output, args.version)

    missing = TABLES - extracted
    if missing:
        raise FileNotFoundError(
            "nuScenes archive is missing required tables: " + ", ".join(sorted(missing))
        )
    print(f"Extracted {len(extracted)} metadata tables to {output}")


if __name__ == "__main__":
    main()
