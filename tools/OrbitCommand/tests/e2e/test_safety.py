"""Safety actions: E-STOP, reset confirmation, and simulator restart recovery."""

from __future__ import annotations

import pytest

from helpers import click, navigate, wait_offline, wait_property, wait_ready

pytestmark = pytest.mark.e2e

DIALOG = "oc.dialog"


def test_estop_transitions_to_fault(app):  # noqa: ARG001
    assert wait_ready("oc.header.estopButton").enabled is True

    click("oc.header.estopButton")
    assert wait_property("oc.header.systemMode", "text", "FAULT", timeout=2000)


def test_reset_requires_confirmation(app, simulator):
    navigate("dashboard", 0)

    click("oc.dashboard.resetButton")
    assert wait_property(DIALOG, "visible", True, timeout=3000)

    click("oc.dialog.cancelButton")
    assert wait_property(DIALOG, "visible", False, timeout=3000)

    click("oc.dashboard.resetButton")
    assert wait_property(DIALOG, "visible", True, timeout=3000)
    click("oc.dialog.confirmButton")
    assert wait_property(DIALOG, "visible", False, timeout=3000)

    simulator.stop()
    assert wait_offline(8000)
    simulator.start()
    assert wait_property("oc.header.statusPill", "text", "Online", timeout=20000)
