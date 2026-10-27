"""Shared helpers for the OrbitCommand Qat end-to-end tests."""

from __future__ import annotations

import os
import re
import time
from typing import Any, Callable

import qat

METRICS: dict[str, Any] = {}


def definition(object_name: str) -> dict[str, str]:
    """Return a Qat object definition for an objectName."""
    return {"objectName": object_name}


def object_name(**properties: str) -> dict[str, str]:
    return properties


def wait_exists(object_name: str, timeout: int | None = None):
    """Wait until the named object exists, even if it is hidden or disabled."""
    return qat.wait_for_object_exists(definition(object_name), timeout)


def wait_ready(object_name: str, timeout: int | None = None):
    """Wait until the named object is visible and enabled."""
    return qat.wait_for_object(definition(object_name), timeout)


def wait_property(
    object_name: str,
    property_name: str,
    expected: Any,
    timeout: int | None = None,
) -> bool:
    return qat.wait_for_property_value(definition(object_name), property_name, expected, timeout=timeout)


def wait_until(
    object_name: str,
    property_name: str,
    predicate: Callable[[Any], bool],
    timeout: int | None = None,
) -> bool:
    """Wait until a property satisfies the given predicate."""
    return qat.wait_for_property_value(
        definition(object_name),
        property_name,
        True,
        comparator=lambda value, _reference: predicate(value),
        timeout=timeout,
    )


def read(object_name: str, property_name: str) -> Any:
    return wait_exists(object_name).property(property_name)


def click(object_name: str, require_enabled: bool = True) -> None:
    if require_enabled:
        qat.wait_for_object(definition(object_name))
    else:
        qat.wait_for_object_exists(definition(object_name))
    qat.mouse_click(definition(object_name), check=True)


def set_text(object_name: str, text: str) -> None:
    """Replace the text of an editor and commit it with Enter."""
    qat.wait_for_object(definition(object_name))
    qat.mouse_click(definition(object_name), check=True)
    qat.shortcut(definition(object_name), "Ctrl+A")
    qat.type_in(definition(object_name), f"{text}<Enter>")


def parse_number(value: Any) -> float | None:
    if value is None:
        return None
    match = re.search(r"-?\d+(?:\.\d+)?", str(value))
    return float(match.group(0)) if match else None


def wait_number(
    object_name: str,
    property_name: str,
    timeout: int | None = None,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float | None:
    """Wait for a numeric property, optionally bounded, and return it."""
    found: list[float] = []

    def predicate(value: Any) -> bool:
        number = parse_number(value)
        if number is None:
            return False
        if minimum is not None and number < minimum:
            return False
        if maximum is not None and number > maximum:
            return False
        found.append(number)
        return True

    if not wait_until(object_name, property_name, predicate, timeout):
        return None
    return found[-1] if found else None


def env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def record_metric(name: str, value: float | int | str | None) -> None:
    METRICS[name] = value
    print(f"[ METRIC   ] {name} = {value}")


def wait_online(timeout_ms: int) -> tuple[bool, int]:
    start = time.monotonic()
    reached = wait_property("oc.header.statusPill", "text", "Online", timeout=timeout_ms)
    return reached, int((time.monotonic() - start) * 1000)


def wait_offline(timeout_ms: int) -> bool:
    return qat.wait_for_property_value(
        definition("oc.header.statusPill"),
        "text",
        True,
        comparator=lambda value, _reference: value in ("Offline", "Disconnected", "Connecting"),
        timeout=timeout_ms,
    )


def wait_busy_cycle(timeout_ms: int) -> bool:
    """Wait for the parameter busy indicator to start and finish.

    A refresh can complete between Qat polls, so an already-idle indicator is
    accepted after a short settle.
    """
    started = wait_property("oc.parameters.busyIndicator", "visible", True, timeout=1000)
    if not started:
        time.sleep(0.2)
    return wait_property("oc.parameters.busyIndicator", "visible", False, timeout=timeout_ms)


def navigate(page_key: str, expected_index: int) -> int:
    start = time.monotonic()
    click(f"oc.nav.{page_key}")
    reached = wait_property("oc.pageStack", "currentIndex", expected_index, timeout=5000)
    elapsed = int((time.monotonic() - start) * 1000)
    assert reached, f"{page_key} page did not become current"
    return elapsed
