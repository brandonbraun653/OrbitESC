"""Connection lifecycle over the simulator transport."""

from __future__ import annotations

import time

import pytest

from helpers import (
    click,
    env_int,
    parse_number,
    read,
    record_metric,
    wait_number,
    wait_property,
    wait_until,
)

pytestmark = pytest.mark.e2e


def test_connect_tick_age_and_reconnect(app):  # noqa: ARG001
    assert wait_property("oc.header.statusPill", "text", "Online", timeout=5000)

    tick_age = wait_number(
        "oc.header.tickAge",
        "text",
        timeout=3000,
        maximum=env_int("ORBIT_E2E_MAX_TICK_AGE_MS", 2000),
    )
    assert tick_age is not None
    record_metric("tick_age_ms", tick_age)

    click("oc.header.connectButton")
    assert wait_property("oc.header.statusPill", "text", "Disconnected", timeout=5000)
    assert wait_until("oc.header.connectButton", "text", lambda value: value == "CONNECT", timeout=3000)

    start = time.monotonic()
    click("oc.header.connectButton")
    assert wait_property(
        "oc.header.statusPill",
        "text",
        "Online",
        timeout=env_int("ORBIT_E2E_MAX_CONNECT_MS", 3000),
    )
    connect_ms = int((time.monotonic() - start) * 1000)
    record_metric("reconnect_ms", connect_ms)

    assert parse_number(read("oc.header.tickAge", "text")) is not None
