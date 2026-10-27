# OrbitCommand

Qt 6 / QML desktop console for OrbitESC. OrbitCommand is the C++ replacement path for the legacy PyQt tool in `pyorbit/app`.

## V0 Features

- USB CDC serial connection and TCP simulator connection with COBS-framed nanopb protocol support.
- Auto-detected serial port list plus an always-present Simulator target with editable host and port.
- Automatic TCP reconnect with backoff while the simulator target is selected.
- Dark-neon HUD with a left navigation rail and always-visible telemetry header.
- System dashboard for protocol-backed values:
  - motor control mode
  - link state and tick age
  - DC bus voltage
  - temperature-sensor voltage
  - phase-current RMS and peak while streaming
- Firmware parameter table with four groups, dirty tracking, Apply, Refresh, and JSON save/load.
- Live phase-current strip chart with auto-scale and clear.
- Console for device terminal output and local application events.
- System reset with confirmation and immediate E-STOP.
- Bundled Orbitron and JetBrains Mono fonts (SIL OFL).

## Requirements

- CMake 3.25 or newer
- Ninja
- C++20 compiler
- Qt 6.5 or newer with Core, Gui, Qml, Quick, QuickControls2, SerialPort, and Network modules
- Python 3.11 with `qat` from `uv sync` for the GUI E2E tier
- `xvfb-run` and `python3-tk` for headless GUI E2E runs
- `libzmq3-dev` to build the OrbitESC host simulator

On Debian/Ubuntu, package names are typically:

```bash
sudo apt install build-essential cmake ninja-build gdb libzmq3-dev \
  xvfb python3-tk gcovr \
  qt6-base-dev qt6-base-dev-tools qt6-declarative-dev qt6-declarative-dev-tools \
  qt6-serialport-dev qt6-tools-dev \
  qml6-module-qtquick qml6-module-qtquick-controls qml6-module-qtquick-layouts \
  qml6-module-qtquick-window qml6-module-qtquick-dialogs qml6-module-qtquick-templates \
  qml6-module-qtqml-workerscript
```

Package names vary by distribution and Qt version. The tests use GoogleTest and GoogleMock. CMake finds a system GoogleTest 1.14 when available and otherwise fetches it; set `ORBITCOMMAND_GTEST_SOURCE_DIR` to a local GoogleTest checkout for offline builds.

## Build

```bash
cmake --preset release
cmake --build --preset release
```

Release is the canonical build for every test tier. The debug preset is a developer build and does not register the Qat GUI E2E test.

Or without presets:

```bash
cmake -S . -B build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/debug
```

## Run

```bash
./build/debug/OrbitCommand
```

Select a serial port or the Simulator target in the header and press **CONNECT**. The app requests a ping, then marks the link online when the device starts sending system ticks.

## Simulator Connection

Start the simulator in a separate process before connecting:

```bash
cmake --preset "OrbitESC Simulator Release"
cmake --build --preset "OrbitESC Simulator Release"
./build/sim-release/bin/OrbitESC
```

The `OrbitESC Simulator` debug preset still exists for development.

In OrbitCommand, select **Simulator** from the connection dropdown. The header shows editable Host and Port fields, defaulting to `localhost` and `37218`. Both values persist across runs.

Set `ORBITESC_SIM_PORT` to override the simulator command port. The value must be `1..65535`; invalid values fall back to `37218` with a warning. The test harnesses use this override to give each simulator process a private port.

OrbitCommand does not launch the simulator. If the endpoint is unavailable, the TCP transport retries with backoff (`500 ms`, `1 s`, `2 s`, then every `5 s`) until you press **DISCONNECT**. A dropped simulator link also triggers automatic reconnect. Serial connections stay manual.

The simulator build needs `libzmq`; see the root README for the host simulator prerequisites.

Only one simulator client can connect at a time because the simulator's command serial server accepts a single TCP client. The default port is defined in `src/simulator/sim_usb_serial.cpp`; keep OrbitCommand's default in sync if that value changes.

## Debug In VS Code

Select **OrbitCommand (Debug)** from the Run and Debug view. Its pre-launch task runs:

```bash
cmake --preset debug --fresh
cmake --build --preset debug
```

This configures against the system Qt installation, builds `tools/OrbitCommand/build/debug/OrbitCommand`, and launches it under `gdb`.

Developer flags used for headless validation:

```bash
./build/debug/OrbitCommand --connect ttyACM2 --page 2 --stream \
  --size 1100x700 --screenshot /tmp/orbitcommand.png --screenshot-delay 5000
```

```bash
./build/debug/OrbitCommand --connect simulator --sim-host localhost --sim-port 37218 \
  --page 2 --stream --size 1100x700 --screenshot /tmp/orbitcommand-sim.png --screenshot-delay 5000
```

- `--connect <port>` connects automatically after startup.
- `--connect simulator` connects to the simulator target.
- `--sim-host <host>` and `--sim-port <port>` override the persisted simulator endpoint for this run.
- `--page <0-3>` opens Dashboard, Parameters, Live Data, or Console.
- `--stream` enables the phase-current stream once the link is online.
- `--size <width>x<height>` sets the initial window size.
- `--screenshot <path>` saves a frame and exits.
- `--screenshot-delay <ms>` changes the capture delay.
- `--smoke-test` loads QML offscreen and exits.

## Test

The project has four test tiers:

| Tier | Runner | Scope | Display | Simulator |
| --- | --- | --- | --- | --- |
| Unit | GoogleTest/GMock + CTest | Protocol codecs, registry, models, transports, `DeviceClient`, `AppController`, mocks | No | No |
| System | GoogleTest + CTest | Real simulator over TCP: cadence, latency, control transitions, reconnect | No | Yes |
| GUI E2E | Qat + pytest + CTest | Release OrbitCommand + real simulator: operator workflow and coarse performance | Yes (`xvfb-run`) | Yes |
| Smoke | `OrbitCommand --smoke-test` | Offscreen QML load | No | No |

Build everything and run all tiers:

```bash
# One-time Python setup (qat is a core dependency)
uv sync

# Release simulator
cmake --preset "OrbitESC Simulator Release"
cmake --build --preset "OrbitESC Simulator Release"

# OrbitCommand
cmake --preset release
cmake --build --preset release

ctest --preset release --output-on-failure
```

Focused runs:

```bash
ctest --preset release -L unit
ctest --preset release -L system
ctest --preset release -L e2e
```

The local wrapper builds the required pieces and writes JUnit output under `test-results/orbitcommand`:

```bash
scripts/test_orbitcommand.sh              # all tiers
scripts/test_orbitcommand.sh --unit-only
scripts/test_orbitcommand.sh --e2e-only
scripts/test_orbitcommand.sh --no-simulator
scripts/test_orbitcommand.sh --coverage
scripts/test_orbitcommand.sh --asan
scripts/test_orbitcommand.sh --tsan
```

System and Qat tests skip with a clear message when the simulator, `qat`, or a display is unavailable. `run_e2e.sh` re-execs under `xvfb-run` when `DISPLAY` is unset.

### Performance bounds

System and GUI tests assert coarse bounds and write every measurement to `test-results/orbitcommand-e2e/metrics.json`. Override the bounds with environment variables:

- `ORBIT_E2E_MAX_CONNECT_MS`
- `ORBIT_E2E_MIN_TICK_HZ` / `ORBIT_E2E_MAX_TICK_HZ`
- `ORBIT_E2E_MAX_TICK_GAP_MS`
- `ORBIT_E2E_MIN_STATUS_HZ` / `ORBIT_E2E_MAX_STATUS_HZ`
- `ORBIT_E2E_MIN_VOLTAGE_HZ` / `ORBIT_E2E_MAX_VOLTAGE_HZ`
- `ORBIT_E2E_MIN_CURRENT_HZ` / `ORBIT_E2E_MAX_CURRENT_HZ`
- `ORBIT_E2E_MAX_PARAM_RTT_MS`
- `ORBIT_E2E_MAX_CONTROL_MS`
- `ORBIT_E2E_MAX_APP_START_MS`
- `ORBIT_E2E_MAX_TICK_AGE_MS`
- `ORBIT_E2E_MAX_STREAM_START_MS`
- `ORBIT_E2E_MIN_STREAM_HZ` / `ORBIT_E2E_MAX_STREAM_HZ`
- `ORBIT_E2E_MAX_REFRESH_MS`
- `ORBIT_E2E_MAX_PAGE_SWITCH_MS`

Without MATLAB the simulator publishes default physical values. The tests assert cadence, numeric presence, and UI liveness, never exact current or voltage values.

### Qat object-name contract

Qat selects QML objects by `objectName`, so these names are an external test API. Keep them stable when you change the QML layout.

| Object name | Item |
| --- | --- |
| `oc.mainWindow` | `ApplicationWindow` |
| `oc.pageStack` | root `StackLayout` |
| `oc.nav.dashboard`, `oc.nav.parameters`, `oc.nav.liveData`, `oc.nav.console` | navigation delegates |
| `oc.header.targetCombo` | target `ComboBox` |
| `oc.header.simHost`, `oc.header.simPort` | simulator fields |
| `oc.header.connectButton`, `oc.header.estopButton` | header actions |
| `oc.header.statusPill`, `oc.header.systemMode`, `oc.header.tickAge` | link state |
| `oc.dashboard.systemMode`, `linkState`, `busVoltage`, `tempSensor`, `phaseRms`, `phasePeak` | dashboard tiles (`oc.dashboard.<name>`) |
| `oc.dashboard.resetButton` | reset action |
| `oc.parameters.tabs`, `oc.parameters.tab.<index>` | parameter tabs |
| `oc.parameters.refreshButton`, `applyButton`, `loadButton`, `saveButton` | parameter actions |
| `oc.parameters.dirtyPill`, `oc.parameters.busyIndicator` | parameter state |
| `oc.parameters.field.<key>`, `oc.parameters.editor.<key>` | parameter field and editor |
| `oc.liveData.streamSwitch`, `autoScaleSwitch`, `sampleRate`, `samples`, `clearButton`, `plot` | live data |
| `oc.console.list`, `clearButton`, `autoScrollSwitch` | console |
| `oc.dialog.cancelButton`, `oc.dialog.confirmButton` | confirmation dialog |

See `tests/e2e/README.md` for authoring guidance.

### Coverage and sanitizers

```bash
cmake --preset coverage
cmake --build --preset coverage
ctest --preset coverage
cmake --build --preset coverage --target coverage   # requires gcovr

cmake --preset asan && cmake --build --preset asan
ctest --preset asan -L unit && ctest --preset asan -L system

cmake --preset tsan && cmake --build --preset tsan
ctest --preset tsan -L unit && ctest --preset tsan -L system
```

Coverage and sanitizer presets do not register Qat E2E. TSan is advisory: `tests/tsan.supp` suppresses known Qt/glib thread-registry noise, and the QThread-heavy `AppController` suite is excluded from the TSan test preset because TSan's thread registry fails when that suite creates and destroys many QThreads. Treat new TSan findings in project code as defects and triage Qt internals separately.

## Protocol Notes

OrbitCommand reuses the checked-in firmware protocol assets:

- `src/core/com/proto/*.proto` and generated `*.pb.c` / `*.pb.h`
- `lib/Aurora/lib/nanopb/nanopb`
- `lib/Aurora/lib/cobs/cobs`

No protoc C++ runtime is used. If the protocol schema changes, regenerate the checked-in C bindings:

```bash
cd ../../scripts
./build_proto.sh
```

Wire format is `COBS(payload) + 0x00`. Requests use nonzero 16-bit UUIDs and are processed one at a time. The link goes offline when no system tick arrives for three seconds. The same wire format runs over USB serial and the simulator TCP socket.

Firmware GET replies currently omit the optional `ParamIOMessage.id` field. OrbitCommand correlates each reply with the in-flight request UUID and requested parameter ID, so parameter values still resolve correctly.

## Parameter Source Of Truth

Parameter metadata is defined in `src/model/ParameterRegistry.cpp` and follows the firmware table in `src/core/data/volatile/orbit_parameter_decl.hpp`.

It includes firmware parameters that the Python mapping omitted:

- `PARAM_PWM_DRIVE_MAX_DUTY`
- `PARAM_MIN_ARM_VOLTAGE`
- `PARAM_MAX_ARM_VOLTAGE`

It intentionally omits `PARAM_BOOT_MODE`, which does not exist in the firmware parameter table.

JSON format:

```json
{
  "schema": "orbitcommand.parameters",
  "version": 1,
  "parameters": [
    { "id": 20, "key": "stator_pwm_freq", "type": "FLOAT", "value": 10000.0 }
  ]
}
```

Loading a file updates the UI and marks values dirty. Press **APPLY** to write them to the device.

## Dashboard Limitations

The current protocol does not provide dedicated fault-code, average-current, or RPM dashboard messages. OrbitCommand displays `N/A` for those fields instead of inventing values. Temperature is displayed as the raw sensor voltage.

## Safety Behavior

- **E-STOP** is always visible, sends `FAULT`, does not ask for confirmation, and is only enabled while the link is online.
- **COMMAND RESET** asks for confirmation and sends `RESET`. The simulator ACKs the command but ChimeraSim leaves the reset callback unset, so the simulated device does not actually reboot. The E2E test verifies the confirmation dialog and command dispatch, then restarts the simulator externally and asserts that OrbitCommand reconnects.
- Controls that require a link are disabled while offline.

## Out Of Scope For V0

- Inner-loop manual control page
- Speed/position and custom multi-field plots
- CSV telemetry logging
- CAN transport
- Windows packaging
- Firmware or protocol changes
- Removing the legacy Python GUI

## Fonts

- Orbitron: SIL Open Font License, see `assets/fonts/OFL-Orbitron.txt`
- JetBrains Mono: SIL Open Font License, see `assets/fonts/OFL-JetBrainsMono.txt`
