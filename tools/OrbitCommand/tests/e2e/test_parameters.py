"""Parameter refresh, edit, dirty tracking, apply, and restore."""

from __future__ import annotations

import time

import pytest

from helpers import (
    click,
    env_int,
    navigate,
    parse_number,
    read,
    record_metric,
    set_text,
    wait_busy_cycle,
    wait_exists,
    wait_property,
    wait_until,
)

pytestmark = pytest.mark.e2e

FIELD = "oc.parameters.field.actv_led_scale"
EDITOR = "oc.parameters.editor.actv_led_scale"
DIRTY_PILL = "oc.parameters.dirtyPill"


def _wait_idle() -> None:
    assert wait_property("oc.parameters.busyIndicator", "visible", False, timeout=10000)


def test_parameter_refresh_edit_apply_and_restore(app):  # noqa: ARG001
    navigate("parameters", 1)
    click("oc.parameters.tab.1")
    wait_exists(FIELD, timeout=5000)
    _wait_idle()

    start = time.monotonic()
    click("oc.parameters.refreshButton")
    assert wait_busy_cycle(env_int("ORBIT_E2E_MAX_REFRESH_MS", 10000))
    refresh_ms = int((time.monotonic() - start) * 1000)
    record_metric("refresh_ms", refresh_ms)
    assert refresh_ms <= env_int("ORBIT_E2E_MAX_REFRESH_MS", 10000)

    original = parse_number(read(FIELD, "value"))
    assert original is not None, "Parameter value is not numeric"
    target = original + 1.0 if original < 99.0 else original - 1.0

    set_text(EDITOR, f"{target:.3f}")
    assert wait_property(FIELD, "dirty", True, timeout=3000)
    assert wait_until(DIRTY_PILL, "text", lambda value: value.startswith("1"), timeout=3000)

    start = time.monotonic()
    click("oc.parameters.applyButton")
    assert wait_until(DIRTY_PILL, "text", lambda value: value.startswith("0"), timeout=10000)
    apply_ms = int((time.monotonic() - start) * 1000)
    record_metric("parameter_apply_ms", apply_ms)

    assert wait_property(FIELD, "dirty", False, timeout=3000)
    applied = parse_number(read(FIELD, "value"))
    assert applied is not None
    assert abs(applied - target) < 0.001

    set_text(EDITOR, f"{original:.3f}")
    assert wait_property(FIELD, "dirty", True, timeout=3000)
    click("oc.parameters.applyButton")
    assert wait_until(DIRTY_PILL, "text", lambda value: value.startswith("0"), timeout=10000)
    assert wait_property(FIELD, "dirty", False, timeout=3000)
    restored = parse_number(read(FIELD, "value"))
    assert restored is not None
    assert abs(restored - original) < 0.001


def test_apply_is_disabled_without_changes(app):  # noqa: ARG001
    navigate("parameters", 1)
    wait_exists(FIELD, timeout=5000)
    _wait_idle()

    button = wait_exists("oc.parameters.applyButton")
    assert button.enabled is False
