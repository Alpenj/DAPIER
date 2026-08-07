#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Remove the privileged top image and preserve verified IK teacher provenance."""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path

from lerobot.envs.so101_mujoco import (
    build_wrist_student_dataset_command,
    write_wrist_student_dataset_contract,
)


def run(args: argparse.Namespace) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if args.student_root.exists():
        raise FileExistsError(f"Refusing to overwrite student root: {args.student_root}")
    teacher = LeRobotDataset(args.teacher_repo_id, root=args.teacher_root)
    command = build_wrist_student_dataset_command(
        python_executable=sys.executable,
        teacher_repo_id=args.teacher_repo_id,
        teacher_root=args.teacher_root,
        student_repo_id=args.student_repo_id,
        student_root=args.student_root,
    )
    print(f"edit_command={shlex.join(command)}")
    subprocess.run(command, check=True)

    student = LeRobotDataset(args.student_repo_id, root=args.student_root)
    if student.num_episodes != teacher.num_episodes or student.num_frames != teacher.num_frames:
        raise RuntimeError(
            "Student episode/frame count differs from the IK teacher: "
            f"teacher={teacher.num_episodes}/{teacher.num_frames} "
            f"student={student.num_episodes}/{student.num_frames}"
        )
    contract_path = write_wrist_student_dataset_contract(
        args.teacher_root,
        args.student_root,
        student_features=tuple(student.meta.features),
        episodes=student.num_episodes,
        frames=student.num_frames,
    )
    print(
        f"student_verified episodes={student.num_episodes} frames={student.num_frames} "
        f"features={sorted(student.meta.features)}"
    )
    print(f"student_contract={contract_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher-root", type=Path, required=True)
    parser.add_argument("--teacher-repo-id", required=True)
    parser.add_argument("--student-root", type=Path, required=True)
    parser.add_argument("--student-repo-id", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
