# OrbitCommand Qat E2E tests

These tests drive the real Release `OrbitCommand` executable through Qat and assert the V0 operator workflow against the real OrbitESC host simulator.

## Run

```bash
cd tools/OrbitCommand/tests/e2e
./run_e2e.sh
```

`run_e2e.sh`:

- uses the repository `.venv` (fall back `python3`),
- exits `77` when `qat`, `xvfb-run`, or the simulator is missing,
- re-execs under `xvfb-run` when `DISPLAY` is unset,
- sets `QT_QPA_PLATFORM=xcb` and `QT_QUICK_BACKEND=software`,
- writes `junit.xml`, `metrics.json`, `simulator.log`, and `app-<test>.log` under `test-results/orbitcommand-e2e`.

Environment overrides:

- `ORBITCOMMAND_BINARY`
- `ORBIT_SIMULATOR_BINARY`
- `ORBIT_E2E_ARTIFACTS`
- `ORBIT_E2E_PYTHON`

## Authoring

- Select objects by `objectName`; QML `id` values are not visible to Qat.
- Add an `Accessible.name` next to every interactive `objectName`.
- Prefer reading QML properties (`MetricTile.value`, `ParameterField.value`, `Switch.checked`) over parsing rendered text.
- Text helpers tolerate the `—` placeholder and `N/A`.
- Use `wait_for_object` only for visible and enabled objects. Use `wait_for_object_exists` for disabled controls such as the simulator host and port fields while connected.
- Never assert exact physical telemetry values. Without MATLAB the simulator publishes default values.
- Record every measured value with `helpers.record_metric()` so it lands in `metrics.json`.
- Keep tests serial. The simulator accepts one command client, and the MATLAB ports are fixed globals.
- Restore any parameter you write so the simulator's flash-backed store stays clean.

## Fixtures

- `simulator` starts one simulator per pytest session on a free port and sets `ORBITESC_SIM_PORT`.
- `app` launches OrbitCommand with `--connect simulator`, an isolated `XDG_CONFIG_HOME`, and waits for `Online`.
- `app_offline` launches without connecting so a test can exercise the CONNECT button.
- `app_factory` allows custom arguments and closes every launched application at teardown.

The fixtures fail a test when the application log contains QML warnings or errors.

## Debugging

Run a single test with output:

```bash
./run_e2e.sh -k test_dashboard_shows_live_telemetry -s
```

Failure screenshots are written to the artifact directory. The simulator log and per-test application log are captured there as well.
