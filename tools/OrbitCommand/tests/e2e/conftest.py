"""Pytest fixtures for the OrbitCommand Qat end-to-end suite."""

from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import qat
except ImportError as import_error:  # pragma: no cover - environment guard
    qat = None
    QAT_IMPORT_ERROR = import_error
else:
    QAT_IMPORT_ERROR = None

REPO_ROOT = Path(os.environ.get("ORBIT_REPO_ROOT", Path(__file__).resolve().parents[4]))
ARTIFACTS = Path(os.environ.get("ORBIT_E2E_ARTIFACTS", REPO_ROOT / "test-results" / "orbitcommand-e2e"))
APPLICATION_NAME = "OrbitCommandE2E"
LAUNCH_SCRIPT = Path(__file__).resolve().parent / "launch_app.sh"

if qat is not None:
    from helpers import METRICS, definition, record_metric, wait_online, wait_property


def _orbitcommand_binary() -> Path:
    return Path(
        os.environ.get(
            "ORBITCOMMAND_BINARY",
            REPO_ROOT / "tools" / "OrbitCommand" / "build" / "release" / "OrbitCommand",
        )
    )


def _simulator_binary() -> Path:
    override = os.environ.get("ORBIT_SIMULATOR_BINARY")
    candidates = [
        Path(override) if override else None,
        REPO_ROOT / "build" / "sim-release" / "bin" / "OrbitESC",
        REPO_ROOT / "build" / "sim" / "bin" / "OrbitESC",
    ]
    for candidate in candidates:
        if candidate is not None and candidate.exists():
            return candidate
    return candidates[1]


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def pytest_sessionstart(session):  # noqa: ARG001
    if qat is None:
        pytest.exit(f"qat is not importable: {QAT_IMPORT_ERROR}", returncode=77)

    if not os.environ.get("DISPLAY"):
        pytest.exit("No DISPLAY is available; run under xvfb-run", returncode=77)

    simulator = _simulator_binary()
    if not simulator.exists():
        pytest.exit(f"OrbitESC simulator not found at {simulator}", returncode=77)

    binary = _orbitcommand_binary()
    if not binary.exists():
        pytest.exit(f"OrbitCommand binary not found at {binary}; build the release preset", returncode=1)

    ARTIFACTS.mkdir(parents=True, exist_ok=True)


def pytest_sessionfinish(session, exitstatus):  # noqa: ARG001
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    metrics_file = ARTIFACTS / "metrics.json"
    metrics_file.write_text(json.dumps(METRICS, indent=2), encoding="utf-8")
    print(f"Metrics written to {metrics_file}")


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):  # noqa: ARG001
    outcome = yield
    report = outcome.get_result()
    if report.when == "call" and report.failed and qat is not None:
        try:
            qat.take_screenshot(str(ARTIFACTS / f"{item.name}.png"))
        except Exception as error:  # pragma: no cover - best effort
            print(f"Could not capture failure screenshot: {error}")


class SimulatorProcess:
    """Manage the real OrbitESC host simulator for the E2E session."""

    def __init__(self, binary: Path, artifacts: Path) -> None:
        self.binary = binary
        self.artifacts = artifacts
        self.port = _free_port()
        self.log_path = artifacts / "simulator.log"
        self.process: subprocess.Popen | None = None

    def start(self, timeout: float = 20.0) -> None:
        self.stop()
        self.log_path.write_text("", encoding="utf-8")
        environment = dict(os.environ)
        environment["ORBITESC_SIM_PORT"] = str(self.port)
        log_file = self.log_path.open("a", encoding="utf-8")
        self.process = subprocess.Popen(
            [str(self.binary)],
            env=environment,
            stdout=log_file,
            stderr=subprocess.STDOUT,
        )
        marker = f"Created server on port {self.port}"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(f"Simulator exited early:\n{self.log_path.read_text(encoding='utf-8')}")
            if marker in self.log_path.read_text(encoding="utf-8"):
                time.sleep(0.1)
                return
            time.sleep(0.05)
        raise RuntimeError(f"Simulator did not listen on port {self.port}:\n{self.log_path.read_text(encoding='utf-8')}")

    def stop(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.process = None

    def restart(self) -> None:
        self.stop()
        self.start()


@pytest.fixture(scope="session")
def simulator() -> SimulatorProcess:
    process = SimulatorProcess(_simulator_binary(), ARTIFACTS)
    process.start()
    yield process
    process.stop()


@pytest.fixture(scope="session")
def registered_application(simulator) -> str:  # noqa: ARG001
    qat.register_application(APPLICATION_NAME, str(LAUNCH_SCRIPT), "")
    yield APPLICATION_NAME
    try:
        qat.unregister_application(APPLICATION_NAME)
    except Exception:
        pass


class AppHandle:
    def __init__(self, context, log_path: Path, start_ms: int) -> None:
        self.context = context
        self.log_path = log_path
        self.start_ms = start_ms


@pytest.fixture
def app_factory(registered_application, simulator, request):
    """Launch a fresh OrbitCommand process with isolated QSettings."""
    contexts: list[tuple[AppHandle, Path]] = []

    def launch(auto_connect: bool = True, extra_args: str = "", wait_window_ms: int = 20000) -> AppHandle:
        config_dir = Path(tempfile.mkdtemp(prefix="orbitcommand-config-"))
        log_path = ARTIFACTS / f"app-{request.node.name}.log"
        os.environ["XDG_CONFIG_HOME"] = str(config_dir)
        os.environ["ORBITCOMMAND_LOG"] = str(log_path)
        os.environ["ORBITCOMMAND_BINARY"] = str(_orbitcommand_binary())
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
        os.environ.setdefault("QT_QUICK_BACKEND", "software")

        arguments = (
            f"--sim-host 127.0.0.1 --sim-port {simulator.port} --size 1440x900 {extra_args}"
        ).strip()
        if auto_connect:
            arguments = f"--connect simulator {arguments}"

        start = time.monotonic()
        context = qat.start_application(registered_application, args=arguments)
        qat.wait_for_object(definition("oc.mainWindow"), timeout=wait_window_ms)
        handle = AppHandle(context, log_path, int((time.monotonic() - start) * 1000))
        contexts.append((handle, config_dir))
        return handle

    yield launch

    for handle, _config_dir in contexts:
        try:
            qat.close_application(handle.context)
        except Exception:
            pass

    for handle, _config_dir in contexts:
        if handle.log_path.exists():
            issues = [
                line
                for line in handle.log_path.read_text(encoding="utf-8", errors="replace").splitlines()
                if "QML" in line and ("error" in line.lower() or "warning" in line.lower())
            ]
            if issues:
                raise AssertionError("QML warnings or errors in app.log:\n" + "\n".join(issues[:10]))


@pytest.fixture
def app(app_factory):
    """Launch OrbitCommand with automatic simulator connection and wait for Online."""
    handle = app_factory(auto_connect=True)
    record_metric("app_start_ms", handle.start_ms)
    online, connect_ms = wait_online(8000)
    assert online, "OrbitCommand did not reach Online"
    record_metric("app_connect_ms", connect_ms)
    record_metric("app_online_ms", handle.start_ms + connect_ms)
    assert handle.start_ms + connect_ms <= int(os.environ.get("ORBIT_E2E_MAX_APP_START_MS", "8000"))
    return handle


@pytest.fixture
def app_offline(app_factory):
    """Launch OrbitCommand without connecting."""
    handle = app_factory(auto_connect=False)
    record_metric("app_start_ms", handle.start_ms)
    return handle
