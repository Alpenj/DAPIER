"""The only way a command can be refused or stopped in the real execution path. Every refusal names a gate; the gate must
be in sweepick_HARD_GATE_ALLOWLIST.json. A name outside the list is a programming error and fails loudly (and in the tests)."""
from sweepick.integration.sweepick_resource_paths import source_path
import json
from pathlib import Path

ALLOW = json.loads((source_path("sweepick_HARD_GATE_ALLOWLIST.json")).read_text())


class GateError(RuntimeError):
    """A refusal site used a gate name that is not on the allowlist."""


def hard(name, detail=None, **extra):
    if name not in ALLOW["hard"]:
        raise GateError(f"'{name}' is not an allowed HARD gate")
    return dict(gate=name, level="HARD", detail=detail, **extra)


def provisional(name, detail=None, **extra):
    if name not in ALLOW["provisional"]:
        raise GateError(f"'{name}' is not an allowed PROVISIONAL gate")
    return dict(gate=name, level="PROVISIONAL", detail=detail, **extra)


def contract(name, detail=None, **extra):
    """A gate of the selected execution contract (profile, progress monitoring, reach confirmation, measurement validity)."""
    if name not in ALLOW["contract"]:
        raise GateError(f"'{name}' is not an allowed CONTRACT gate")
    return dict(gate=name, level="CONTRACT", detail=detail, **extra)


def diagnostic(name, detail=None, **extra):
    if name not in ALLOW["diagnostic"]:
        raise GateError(f"'{name}' is not a listed diagnostic")
    return dict(diagnostic=name, level="DIAGNOSTIC", detail=detail, **extra)
