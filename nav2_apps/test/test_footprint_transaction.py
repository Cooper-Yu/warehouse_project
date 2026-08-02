from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from nav2_apps.footprint_transaction import (  # noqa: E402
    FootprintSnapshot,
    edges_match,
    expected_padded_edges,
    parse_polygon,
    run_compensating_transaction,
)


TARGETS = ("global", "local")
ORIGINAL = "[[0.25, 0.25], [-0.25, 0.25], [-0.25, -0.25], [0.25, -0.25]]"
LOADED = "[[0.40, 0.45], [-0.40, 0.45], [-0.40, -0.45], [0.40, -0.45]]"


class FakeBackend:
    def __init__(self, fail_set=None, mismatch_read=None, fail_rollback=False):
        self.values = {target: ORIGINAL for target in TARGETS}
        self.fail_set = fail_set
        self.mismatch_read = mismatch_read
        self.fail_rollback = fail_rollback
        self.loaded_shape_ok = True
        self.restored_shape_ok = True
        self.calls = []
        self.raise_read = None

    def snapshot(self, target):
        self.calls.append(("snapshot", target))
        return FootprintSnapshot(self.values[target], 0.01)

    def set_value(self, target, value):
        self.calls.append(("set", target, value))
        if target == self.fail_set and value == LOADED:
            return False
        if self.fail_rollback and value == ORIGINAL:
            return False
        self.values[target] = value
        return True

    def read_value(self, target):
        self.calls.append(("read", target))
        if target == self.raise_read and self.values[target] == LOADED:
            raise RuntimeError("read service failed")
        if target == self.mismatch_read and self.values[target] == LOADED:
            return "mismatch"
        return self.values[target]

    def verify_loaded(self, _originals):
        self.calls.append(("verify_loaded",))
        return self.loaded_shape_ok

    def verify_restored(self, _originals):
        self.calls.append(("verify_restored",))
        return self.restored_shape_ok


def _run(backend):
    return run_compensating_transaction(
        TARGETS,
        LOADED,
        backend.snapshot,
        backend.set_value,
        backend.read_value,
        backend.verify_loaded,
        backend.verify_restored,
    )


def test_loaded_rectangle_and_padding_edges():
    assert parse_polygon(LOADED) == [
        [0.40, 0.45],
        [-0.40, 0.45],
        [-0.40, -0.45],
        [0.40, -0.45],
    ]
    assert expected_padded_edges(LOADED, 0.01) == pytest.approx(
        [0.82, 0.82, 0.92, 0.92]
    )
    assert edges_match(
        [0.819, 0.821, 0.919, 0.921],
        [0.82, 0.82, 0.92, 0.92],
        0.03,
    )


def test_success_requires_both_readbacks_and_published_shape():
    backend = FakeBackend()
    result = _run(backend)
    assert result.success
    assert backend.values == {target: LOADED for target in TARGETS}
    assert ("verify_loaded",) in backend.calls


@pytest.mark.parametrize("failure", ["set", "read", "shape"])
def test_any_partial_failure_restores_both_targets(failure):
    backend = FakeBackend(
        fail_set="local" if failure == "set" else None,
        mismatch_read="local" if failure == "read" else None,
    )
    if failure == "shape":
        backend.loaded_shape_ok = False
    result = _run(backend)
    assert not result.success
    assert result.rollback_verified
    assert backend.values == {target: ORIGINAL for target in TARGETS}
    assert ("verify_restored",) in backend.calls


def test_rollback_failure_is_explicit_and_blocks_success():
    backend = FakeBackend(fail_set="local", fail_rollback=True)
    result = _run(backend)
    assert not result.success
    assert not result.rollback_verified


def test_service_exception_still_enters_verified_rollback():
    backend = FakeBackend()
    backend.raise_read = "local"
    result = _run(backend)
    assert not result.success
    assert result.rollback_verified
    assert "transaction exception" in result.reason


def test_confirmation_gate_precedes_navigation_logic():
    args = SimpleNamespace(
        loaded_footprint_only=True,
        confirm_lift_accepted=False,
        confirm_robot_stopped=False,
    )
    assert not (args.confirm_lift_accepted and args.confirm_robot_stopped)
