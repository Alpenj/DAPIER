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
CREATE TABLE IF NOT EXISTS episodes (
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
    """Validate and upsert every episode below ``root``."""
    manifest_paths = sorted(Path(root).rglob("episode_manifest.json"))
    if not manifest_paths:
        raise ValueError(f"no episode_manifest.json files found below: {root}")
    database = Path(database_path)
    database.parent.mkdir(parents=True, exist_ok=True)
    indexed = 0
    invalid_manifest = 0
    usable = 0
    with closing(sqlite3.connect(database)) as connection:
        connection.execute(SCHEMA)
        connection.execute("DELETE FROM episodes")
        for path in manifest_paths:
            report = validate_episode(path)
            try:
                manifest = load_manifest(path)
            except ValueError:
                invalid_manifest += 1
                continue
            success = manifest["outcome"]["success"]
            success_db = None if success is None else int(success)
            values = (
                str(path.resolve()),
                manifest["episode_id"],
                manifest["schema_version"],
                manifest["task"]["name"],
                manifest["task"]["skill"],
                manifest["task"]["shoe_pair_id"],
                manifest["outcome"]["status"],
                success_db,
                manifest["outcome"]["failure_reason"],
                manifest["provenance"]["source_split"],
                manifest["provenance"]["operator_id"],
                manifest["provenance"]["session_id"],
                manifest["robot"]["calibration_version"],
                manifest["robot"]["robot_config_version"],
                manifest["provenance"]["pipeline_version"],
                manifest["provenance"]["data_origin"],
                report.sample_count,
                report.duration_ns,
                len(report.errors),
                len(report.warnings),
                int(report.usable),
                json.dumps(sorted({issue.code for issue in report.issues})),
                manifest["checksums"]["samples_sha256"],
                _utc_now(),
            )
            connection.execute(
                """
                INSERT INTO episodes VALUES (
                    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?
                )
                ON CONFLICT(path) DO UPDATE SET
                    episode_id=excluded.episode_id,
                    schema_version=excluded.schema_version,
                    task_name=excluded.task_name,
                    skill=excluded.skill,
                    shoe_pair_id=excluded.shoe_pair_id,
                    outcome_status=excluded.outcome_status,
                    success=excluded.success,
                    failure_reason=excluded.failure_reason,
                    source_split=excluded.source_split,
                    operator_id=excluded.operator_id,
                    session_id=excluded.session_id,
                    calibration_version=excluded.calibration_version,
                    robot_config_version=excluded.robot_config_version,
                    pipeline_version=excluded.pipeline_version,
                    data_origin=excluded.data_origin,
                    sample_count=excluded.sample_count,
                    duration_ns=excluded.duration_ns,
                    error_count=excluded.error_count,
                    warning_count=excluded.warning_count,
                    usable=excluded.usable,
                    issue_codes_json=excluded.issue_codes_json,
                    samples_sha256=excluded.samples_sha256,
                    indexed_at_utc=excluded.indexed_at_utc
                """,
                values,
            )
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
