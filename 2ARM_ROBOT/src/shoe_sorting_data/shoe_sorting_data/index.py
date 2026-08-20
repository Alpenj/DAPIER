"""SQLite curation manifest for validated shoe-sorting episodes."""

from __future__ import annotations

from contextlib import closing
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
from typing import Any

from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.quality import validate_episode


SCHEMA = """
CREATE TABLE episodes (
    path TEXT PRIMARY KEY,
    episode_id TEXT NOT NULL,
    schema_version TEXT NOT NULL,
    task_name TEXT NOT NULL,
    skill TEXT NOT NULL,
    shoe_pair_id TEXT NOT NULL,
    outcome_status TEXT NOT NULL,
    success INTEGER,
    failure_reason TEXT,
    source_split TEXT NOT NULL,
    operator_id TEXT NOT NULL,
    session_id TEXT NOT NULL,
    calibration_version TEXT NOT NULL,
    robot_config_version TEXT NOT NULL,
    pipeline_version TEXT NOT NULL,
    data_origin TEXT NOT NULL,
    object_instance_id TEXT NOT NULL,
    background_id TEXT NOT NULL,
    fixture_id TEXT NOT NULL,
    recording_span_id TEXT NOT NULL,
    attempt_id TEXT NOT NULL,
    sample_count INTEGER NOT NULL,
    duration_ns INTEGER NOT NULL,
    error_count INTEGER NOT NULL,
    warning_count INTEGER NOT NULL,
    usable INTEGER NOT NULL,
    issue_codes_json TEXT NOT NULL,
    samples_sha256 TEXT NOT NULL,
    indexed_at_utc TEXT NOT NULL
)
"""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def build_index(root: str | Path, database_path: str | Path) -> dict[str, int]:
    """Rebuild a derived SQLite snapshot for every episode below ``root``."""
    manifest_paths = sorted(Path(root).rglob("episode_manifest.json"))
    if not manifest_paths:
        raise ValueError(f"no episode_manifest.json files found below: {root}")
    database = Path(database_path)
    database.parent.mkdir(parents=True, exist_ok=True)
    indexed = 0
    invalid_manifest = 0
    usable = 0
    with closing(sqlite3.connect(database)) as connection:
        connection.execute("DROP TABLE IF EXISTS episodes")
        connection.execute(SCHEMA)
        for path in manifest_paths:
            report = validate_episode(path)
            try:
                manifest = load_manifest(path)
            except ValueError:
                invalid_manifest += 1
                continue
            success = manifest["outcome"]["success"]
            success_db = None if success is None else int(success)
            provenance = manifest["provenance"]
            record = {
                "path": str(path.resolve()),
                "episode_id": manifest["episode_id"],
                "schema_version": manifest["schema_version"],
                "task_name": manifest["task"]["name"],
                "skill": manifest["task"]["skill"],
                "shoe_pair_id": manifest["task"]["shoe_pair_id"],
                "outcome_status": manifest["outcome"]["status"],
                "success": success_db,
                "failure_reason": manifest["outcome"]["failure_reason"],
                "source_split": provenance["source_split"],
                "operator_id": provenance["operator_id"],
                "session_id": provenance["session_id"],
                "calibration_version": manifest["robot"]["calibration_version"],
                "robot_config_version": manifest["robot"]["robot_config_version"],
                "pipeline_version": provenance["pipeline_version"],
                "data_origin": provenance["data_origin"],
                "object_instance_id": provenance.get("object_instance_id", "object_unknown"),
                "background_id": provenance.get("background_id", "background_unknown"),
                "fixture_id": provenance.get("fixture_id", "fixture_unknown"),
                "recording_span_id": provenance.get("recording_span_id", "span_unknown"),
                "attempt_id": provenance.get("attempt_id", "attempt_unknown"),
                "sample_count": report.sample_count,
                "duration_ns": report.duration_ns,
                "error_count": len(report.errors),
                "warning_count": len(report.warnings),
                "usable": int(report.usable),
                "issue_codes_json": json.dumps(sorted({issue.code for issue in report.issues})),
                "samples_sha256": manifest["checksums"]["samples_sha256"],
                "indexed_at_utc": _utc_now(),
            }
            columns = ", ".join(record)
            placeholders = ", ".join(f":{column}" for column in record)
            connection.execute(f"INSERT INTO episodes ({columns}) VALUES ({placeholders})", record)
            indexed += 1
            usable += int(report.usable)
        connection.commit()
    return {
        "discovered": len(manifest_paths),
        "indexed": indexed,
        "usable": usable,
        "invalid_manifest": invalid_manifest,
    }


def query_index(
    database_path: str | Path,
    *,
    usable: bool | None = None,
    source_split: str | None = None,
    success: bool | None = None,
    shoe_pair_id: str | None = None,
    object_instance_id: str | None = None,
    session_id: str | None = None,
    background_id: str | None = None,
    limit: int = 100,
) -> list[dict[str, Any]]:
    """Return rows using a small safe filter vocabulary."""
    if limit <= 0:
        raise ValueError("limit must be positive")
    database = Path(database_path)
    if not database.is_file():
        raise ValueError(f"manifest database not found: {database}")
    clauses: list[str] = []
    parameters: list[Any] = []
    for column, value in (
        ("usable", None if usable is None else int(usable)),
        ("source_split", source_split),
        ("success", None if success is None else int(success)),
        ("shoe_pair_id", shoe_pair_id),
        ("object_instance_id", object_instance_id),
        ("session_id", session_id),
        ("background_id", background_id),
    ):
        if value is not None:
            clauses.append(f"{column} = ?")
            parameters.append(value)
    where = " WHERE " + " AND ".join(clauses) if clauses else ""
    sql = "SELECT * FROM episodes" + where + " ORDER BY episode_id LIMIT ?"
    parameters.append(limit)
    try:
        with closing(sqlite3.connect(database)) as connection:
            connection.row_factory = sqlite3.Row
            rows = connection.execute(sql, parameters).fetchall()
    except sqlite3.DatabaseError as error:
        raise ValueError(f"invalid manifest database {database}: {error}") from error
    return [dict(row) for row in rows]
