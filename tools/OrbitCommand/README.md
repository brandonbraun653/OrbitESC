# OrbitCommand

Qt 6 / QML desktop console for OrbitESC. OrbitCommand is the C++ replacement path for the legacy PyQt tool in `pyorbit/app`.

## V0 Features

- USB CDC serial connection with COBS-framed nanopb protocol support.
- Auto-detected serial port list with last-port persistence.
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
- Qt 6.5 or newer with Core, Gui, Qml, Quick, QuickControls2, SerialPort, and Test modules

On Debian/Ubuntu, package names are typically:

```bash
sudo apt install build-essential cmake ninja-build gdb \
  qt6-base-dev qt6-base-dev-tools qt6-declarative-dev qt6-declarative-dev-tools \
  qt6-serialport-dev qt6-tools-dev \
  qml6-module-qtquick qml6-module-qtquick-controls qml6-module-qtquick-layouts \
  qml6-module-qtquick-window qml6-module-qtquick-dialogs qml6-module-qtquick-templates \
  qml6-module-qtqml-workerscript
```

Package names vary by distribution and Qt version.

## Build

```bash
cmake --preset debug
cmake --build --preset debug
```

Or without presets:

```bash
cmake -S . -B build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/debug
```

## Run

```bash
./build/debug/OrbitCommand
```

Select a serial port in the header and press **CONNECT**. The app requests a ping, then marks the link online when the device starts sending system ticks.

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

- `--connect <port>` connects automatically after startup.
- `--page <0-3>` opens Dashboard, Parameters, Live Data, or Console.
- `--stream` enables the phase-current stream once the link is online.
- `--size <width>x<height>` sets the initial window size.
- `--screenshot <path>` saves a frame and exits.
- `--screenshot-delay <ms>` changes the capture delay.
- `--smoke-test` loads QML offscreen and exits.

## Test

```bash
ctest --test-dir build/debug --output-on-failure
QT_QPA_PLATFORM=offscreen QT_QUICK_BACKEND=software ./build/debug/OrbitCommand --smoke-test
```

The test suite covers:

- COBS framing, partial frames, invalid frames, and overflow.
- Message encoding/decoding and parameter value codecs.
- Firmware parameter registry contents and dirty tracking.
- Device client connect, ping, tick watchdog, parameter correlation, stream commands, and offline rejection.
- Offscreen QML load smoke test.

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

Wire format is `COBS(payload) + 0x00`. Requests use nonzero 16-bit UUIDs and are processed one at a time. The link goes offline when no system tick arrives for three seconds.

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
- **COMMAND RESET** asks for confirmation, sends `RESET`, and expects a brief link drop while the device reboots.
- Controls that require a link are disabled while offline.

## Out Of Scope For V0

- Inner-loop manual control page
- Speed/position and custom multi-field plots
- CSV telemetry logging
- CAN and TCP/simulator transports
- Windows packaging
- Firmware or protocol changes
- Removing the legacy Python GUI

## Fonts

- Orbitron: SIL Open Font License, see `assets/fonts/OFL-Orbitron.txt`
- JetBrains Mono: SIL Open Font License, see `assets/fonts/OFL-JetBrainsMono.txt`
