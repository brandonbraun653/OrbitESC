"""Console entries, clearing, and auto-scroll."""

from __future__ import annotations

import pytest

from helpers import click, navigate, read, wait_property, wait_until

pytestmark = pytest.mark.e2e

CONSOLE_LIST = "oc.console.list"


def test_console_has_entries_and_clears(app):  # noqa: ARG001
    navigate("console", 3)
    assert wait_until(CONSOLE_LIST, "count", lambda value: value > 0, timeout=3000)

    click("oc.console.clearButton")
    assert wait_property(CONSOLE_LIST, "count", 0, timeout=3000)


def test_console_auto_scroll_toggle(app):  # noqa: ARG001
    navigate("console", 3)
    switch = "oc.console.autoScrollSwitch"
    assert read(switch, "checked") is True

    click(switch)
    assert wait_property(switch, "checked", False, timeout=3000)

    click(switch)
    assert wait_property(switch, "checked", True, timeout=3000)
