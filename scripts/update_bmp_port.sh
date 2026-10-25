#!/usr/bin/env bash
# Resolve the BMP port and inject it into .vscode/launch.json as BMPGDBSerialPort.
# Run as a preLaunchTask so launch.json always has the current port before Cortex-Debug starts.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LAUNCH_JSON="$PROJECT_ROOT/.vscode/launch.json"

PORT=$("$SCRIPT_DIR/find_bmp_port.sh")

# Replace the BMPGDBSerialPort value in the "BMP OrbitESC V3" config.
# Uses python for safe JSON edit (jq not guaranteed on all systems).
python3 - "$LAUNCH_JSON" "$PORT" <<'PY'
import json, sys

path, port = sys.argv[1], sys.argv[2]
with open(path) as f:
    data = json.load(f)

for cfg in data.get("configurations", []):
    if cfg.get("name") == "BMP OrbitESC V3":
        cfg["BMPGDBSerialPort"] = port
        break

with open(path, "w") as f:
    json.dump(data, f, indent=2)
    f.write("\n")

print(f"BMPGDBSerialPort -> {port}")
PY