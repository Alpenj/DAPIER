"""이관 경로·호환 module identity·공개 자산의 SHA를 장치 없이 확인한다."""
import ast
import hashlib
import importlib
import json
import sys
import types
from pathlib import Path

from sweepick.integration.sweepick_resource_paths import PACKAGE, SOURCE_PATHS, source_path

PRODUCT = Path(__file__).resolve().parents[1]
MANIFEST = json.loads((PRODUCT / "docs/sweepick_261009_repository_source_manifest.json").read_text())


def test_every_declared_source_and_resource_exists():
    for old, relative in SOURCE_PATHS.items():
        assert source_path(old) == PACKAGE / relative
        assert source_path(old).is_file(), old
    for path in PACKAGE.rglob("*.py"):
        compile(path.read_text(), str(path), "exec")


def test_old_imports_share_the_canonical_module_and_control_objects():
    # These modules require the declared numpy/MuJoCo dependencies at import time.
    # External-runtime imports are tested separately in the supported runtime.
    for old, new in MANIFEST["independent_imports"].items():
        assert importlib.import_module(old) is importlib.import_module(new), old
    old = importlib.import_module("sweepick_move_a")
    new = importlib.import_module("sweepick.control.sweepick_trajectory_executor")
    assert old.run is new.run and old.grip is new.grip
    jaw = importlib.import_module("sweepick_grasp")
    assert jaw.Jaw is importlib.import_module("sweepick.control.sweepick_gripper_contact_hold").Jaw


def test_profiles_and_small_scene_assets_keep_the_original_bytes():
    for item in MANIFEST["public_assets"]:
        path = PRODUCT.parents[1] / item["path"]
        assert path.is_file(), item["path"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"]


class Semantic(ast.NodeTransformer):
    def visit_Import(self, node):
        return None

    def visit_ImportFrom(self, node):
        return None

    def visit_Expr(self, node):
        if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            return None
        return self.generic_visit(node)


def test_control_bodies_match_the_frozen_source():
    for module, expected in MANIFEST["unchanged_implementations"].items():
        path = PACKAGE.parent / Path(*module.split(".")).with_suffix(".py")
        tree = ast.parse(path.read_text())
        bodies = {n.name: n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
        for node in tree.body:
            if isinstance(node, ast.ClassDef):
                bodies.update({node.name + "." + n.name: n for n in node.body if isinstance(n, ast.FunctionDef)})
        for name, digest in expected.items():
            body = ast.dump(Semantic().visit(bodies[name]), include_attributes=False)
            assert hashlib.sha256(body.encode()).hexdigest() == digest, (module, name)


def test_explicit_receive_override_keeps_the_selected_planner_for_gear(tmp_path, monkeypatch):
    from sweepick.manipulation import sweepick_bimanual_handover as handover

    monkeypatch.setenv("SWEEPICK_RECEIVE_DIR", str(tmp_path))
    monkeypatch.setenv("FACTORY_ROOT", str(handover.RUNTIME))
    monkeypatch.setenv("DAPIER_SO101_MJCF", str(handover.MJCF))
    monkeypatch.setattr(handover, "CODEX", tmp_path)
    monkeypatch.setattr(sys, "path", sys.path.copy())
    modules = {}
    for name in ("receive_real_adapter", "receive_plan", "receive_entry", "receive_session", "full_episode"):
        module = types.ModuleType("sweepick_261008_" + name)
        monkeypatch.setitem(sys.modules, module.__name__, module)
        modules[name] = module
    teacher, q, marker = object(), object(), object()
    calls = []

    def gear(t, measured):
        calls.append((t, measured))
        return marker

    modules["receive_plan"].gear = gear
    assert handover.codex() == modules
    owner = handover.Owner.__new__(handover.Owner)
    owner.teacher = teacher
    assert owner.gear(q) is marker and calls == [(teacher, q)]
