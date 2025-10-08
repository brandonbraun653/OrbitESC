# OrbitESC

Brushless motor controller development environment for STM32-based ESC firmware, a host-side simulator, and a Python GUI/tooling suite.

## Features

- Field-oriented control (FOC)-centric ESC stack with FreeRTOS integration
- STM32F446RE target (embedded) and host simulator build (ChimeraSim)
- Protobuf/Nanopb command and telemetry schema for C/C++ and Python
- Python tools and PyQt5 GUI for monitoring, configuration, and testing
- CMake presets for embedded and simulator builds

## Project Structure

| Path | Description |
| --- | --- |
| `src/` | C++ sources for firmware, simulator integration, startup, tasks, and libraries. |
| `lib/` | Third-party and in-tree libraries: Aurora, Chimera, ChimeraSim, CMSIS, FreeRTOS, Thor, tinyusb, db. |
| `pyorbit/` | Python package for host tools, GUI (`pyorbit/app`), serial/CAN clients, protobuf stubs. |
| `scripts/` | Helper scripts: build embedded, generate protobufs, CAN interface setup. |
| `model/` | MATLAB/Simulink models and notebooks related to control algorithms. |
| `CMakePresets.json` | Configure/build presets for embedded and simulator targets. |

---

## Quickstart (Python tooling & GUI)

This repo uses `uv` for Python dependency management. Python 3.11 is required and auto-detected from `pyproject.toml`.

1) Create and sync the environment
```bash
uv venv
uv sync
```

2) Option A: Activate the environment
```bash
source .venv/bin/activate
```

2) Option B: Run one-off commands without activation
```bash
uv run <command>
```

3) Launch the GUI
```bash
uv run python -m pyorbit.app.main
```

Common tasks
```bash
# Run unit tests
uv run pytest

# Type check
uv run mypy

# Lint / Format
uv run flake8
uv run black .
```

Development workflow
- **Add dependencies**: `uv add <package>`
- **Sync/update**: `uv sync`
- **Update lockfile**: `uv lock`

---

## Protobuf/Nanopb workflow

Protobuf definitions live under `src/core/com/proto`. C bindings are generated with Nanopb for the firmware, and Python stubs for `pyorbit`.

Prerequisites
- `protoc` available in PATH
- The repo includes the Nanopb generator under `lib/Aurora/lib/nanopb/...`

Generate all bindings
```bash
cd scripts
./build_proto.sh
```

What it does
- Runs Nanopb to generate C/C++ descriptors into `src/core/com/proto`
- Runs `protoc` to generate Python modules and `.pyi` into `pyorbit/nanopb`

---

## Embedded firmware build (STM32F446RE)

The firmware uses CMake presets and GCC ARM toolchain.

Prerequisites
- CMake ≥ 3.16, build-essential toolchain
- Arm GNU Toolchain `arm-none-eabi-*` (see `scripts/install_arm_gcc.sh` or install from Arm)

Configure & build (Debug)
```bash
cmake --preset "OrbitESC_V3 Debug"
cmake --build --preset "OrbitESC_V3 Debug"
```

Configure & build (Release)
```bash
cmake --preset "OrbitESC_V3 Release"
cmake --build --preset "OrbitESC_V3 Release"
```

Artifacts
- Build directories under `build/debug` and `build/release`
- The `OrbitESC` image and map file are produced; a copy of the image is placed under a `sys_image` folder in the preset's runtime output directory

Flashing
- Flashing is board/debugger dependent (ST-Link, J-Link, OpenOCD). Provide your preferred tool or script; a generic OpenOCD flow is recommended.

---

## Simulator build (host)

Build a host-executable with simulated hardware using ChimeraSim.

Host prerequisites (Ubuntu/Debian)
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config libzmq3-dev
```

Configure & build
```bash
cmake --preset "OrbitESC Simulator"
cmake --build --preset "OrbitESC Simulator"
```

Run
```bash
# Example path; adjust if your CMake generator places binaries differently
./build/sim/OrbitESC
```

---

## CAN bus setup (Linux)

Scripts are provided to bring up a `can0` interface at 1 Mbit/s. Root privileges are required.

Open the CAN port
```bash
sudo ./scripts/open_can_port.sh
```

Close the CAN port
```bash
sudo ./scripts/close_can_port.sh
```

Note: Ensure a compatible CAN adapter/driver is present. The Python side uses `python-can`.

---

## Testing & quality checks

All commands can be run with `uv run ...` without activating the venv.

```bash
# Python tests
uv run pytest -q

# Type checks
uv run mypy

# Linting / formatting
uv run flake8
uv run black --check .
```

Pytest is configured to discover tests under `pyorbit/tests`.

---

## Troubleshooting

- CMake cannot find `arm-none-eabi-gcc`: install the Arm GNU Toolchain and ensure it is in PATH.
- Simulator link fails on `zmq`: install `libzmq3-dev` (or the platform equivalent) and reconfigure.
- `protoc: command not found`: install protobuf compiler (`sudo apt-get install -y protobuf-compiler`).
- GUI fails to start: ensure `PyQt5` is installed (it is declared in `pyproject.toml`), then `uv sync` again.

---

## License

MIT. See `LICENSE`.
