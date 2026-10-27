"""Phase-current stream toggle, sample rate, and stop behavior."""

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
    wait_number,
    wait_property,
    wait_until,
)

pytestmark = pytest.mark.e2e

SAMPLES = "oc.liveData.samples"
SAMPLE_RATE = "oc.liveData.sampleRate"
STREAM_SWITCH = "oc.liveData.streamSwitch"


def _samples() -> int:
    value = parse_number(read(SAMPLES, "text"))
    return int(value) if value is not None else 0


def test_stream_toggle_and_sample_rate(app):  # noqa: ARG001
    navigate("liveData", 2)
    assert wait_property(STREAM_SWITCH, "enabled", True, timeout=3000)
    assert _samples() == 0

    start = time.monotonic()
    click(STREAM_SWITCH)
    assert wait_until(SAMPLES, "text", lambda value: (parse_number(value) or 0) > 0, timeout=2000)
    stream_start_ms = int((time.monotonic() - start) * 1000)
    record_metric("stream_start_ms", stream_start_ms)
    assert stream_start_ms <= env_int("ORBIT_E2E_MAX_STREAM_START_MS", 2000)

    rate = wait_number(
        SAMPLE_RATE,
        "text",
        timeout=5000,
        minimum=env_int("ORBIT_E2E_MIN_STREAM_HZ", 25),
        maximum=env_int("ORBIT_E2E_MAX_STREAM_HZ", 35),
    )
    assert rate is not None, "Displayed stream rate is outside the expected range"
    record_metric("stream_hz", rate)

    before = _samples()
    time.sleep(0.5)
    after = _samples()
    assert after > before, "Sample count did not increase while streaming"

    click(STREAM_SWITCH)
    assert wait_property(STREAM_SWITCH, "checked", False, timeout=3000)
    time.sleep(1.0)
    stopped = _samples()
    time.sleep(1.0)
    assert _samples() - stopped <= 2, "Samples kept arriving after the stream was disabled"


def test_auto_scale_toggle_and_clear(app):  # noqa: ARG001
    navigate("liveData", 2)
    switch = wait_property("oc.liveData.autoScaleSwitch", "checked", True, timeout=3000)
    assert switch is True

    click("oc.liveData.autoScaleSwitch")
    assert wait_property("oc.liveData.autoScaleSwitch", "checked", False, timeout=3000)
    click("oc.liveData.autoScaleSwitch")
    assert wait_property("oc.liveData.autoScaleSwitch", "checked", True, timeout=3000)

    click("oc.liveData.clearButton")
    assert wait_until(SAMPLES, "text", lambda value: (parse_number(value) or 0) == 0, timeout=3000)
