"""Application launch and page navigation."""

from __future__ import annotations

import pytest

from helpers import navigate, read, record_metric, wait_exists, wait_property, wait_ready

pytestmark = pytest.mark.e2e


def test_main_window_is_visible(app):  # noqa: ARG001
    window = wait_ready("oc.mainWindow")
    assert window.visible is True
    assert window.title == "OrbitCommand"
    assert read("oc.pageStack", "currentIndex") == 0


def test_navigation_switches_pages(app):  # noqa: ARG001
    elapsed = navigate("parameters", 1)
    record_metric("page_switch_ms", elapsed)
    assert elapsed <= 1000

    assert navigate("liveData", 2) <= 1000
    assert navigate("console", 3) <= 1000
    assert navigate("dashboard", 0) <= 1000

    assert wait_property("oc.pageStack", "currentIndex", 0, timeout=2000)


def test_header_controls_are_present(app):  # noqa: ARG001
    assert wait_exists("oc.header.targetCombo").displayText == "Simulator"
    assert wait_ready("oc.header.connectButton").text == "DISCONNECT"
    assert wait_ready("oc.header.estopButton").enabled is True
    assert wait_property("oc.header.statusPill", "text", "Online", timeout=5000)
