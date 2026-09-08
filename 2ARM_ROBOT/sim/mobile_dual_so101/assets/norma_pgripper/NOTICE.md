# NORMA PGripper source notice

Source: https://github.com/norma-core/norma-core
Pinned revision: `b935fc4b887e576538921b15630a30db22759ec8`.

The unmodified `upstream/elrobot_follower.urdf` and five STL files are selected
from `hardware/elrobot/simulation`. The unmodified `upstream/LICENSE` is the
repository's **hardware Apache-2.0 license**, not its root software MIT license.
All copied file hashes are recorded in `provenance.json`.

The full URDF is retained as a source reference. Only its five gripper-link meshes
are vendored; it is not a self-contained installation of the whole ElRobot.
`hardware/pgripper` supplies printable STL/STEP files, not a standalone SO-101 URDF.

DAPIER adaptation (2026-09-08): `../../pgripper.py` rebases that gripper subtree
onto the existing SO-101 wrist in memory, reverses the motor coordinate to
opening-positive, adds coupled jaw sliders, distal convex collision hulls, a
housing box and TCP datums. Zero source gear inertia is approximated from its
mass and CAD bounding box. The upstream copies themselves are not modified.
This is not an upstream-authorized or physically calibrated SO-101 assembly.
