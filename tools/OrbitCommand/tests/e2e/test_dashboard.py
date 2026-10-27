"""Dashboard telemetry presence and liveness."""

from __future__ import annotations

import time

import pytest

from helpers import parse_number, read, record_metric, wait_number, wait_property, wait_until

pytestmark = pytest.mark.e2e


def test_dashboard_shows_live_telemetry(app):  # noqa: ARG001
    assert wait_property("oc.dashboard.linkState", "value", "ONLINE", timeout=5000)

    mode = read("oc.dashboard.systemMode", "value")
    assert mode not in ("", "N/A")
    record_metric("system_mode", mode)

    bus_voltage = wait_number("oc.dashboard.busVoltage", "value", timeout=5000)
    temperature = wait_number("oc.dashboard.tempSensor", "value", timeout=5000)
    assert bus_voltage is not None, "DC bus voltage never became numeric"
    assert temperature is not None, "Temperature sensor voltage never became numeric"
    record_metric("bus_voltage", bus_voltage)
    record_metric("temp_sensor_voltage", temperature)

    tick_age = wait_number("oc.header.tickAge", "text", timeout=3000)
    assert tick_age is not None
    assert tick_age < 2000

    time.sleep(0.5)
    later_age = parse_number(read("oc.header.tickAge", "text"))
    assert later_age is not None, "Tick age became unavailable while online"
    record_metric("tick_age_after_wait_ms", later_age)


def test_dashboard_link_state_tracks_connection(app):  # noqa: ARG001
    assert wait_property("oc.dashboard.linkState", "value", "ONLINE", timeout=5000)
    assert wait_until(
        "oc.dashboard.systemMode",
        "value",
        lambda value: value not in ("", "N/A"),
        timeout=3000,
    )
