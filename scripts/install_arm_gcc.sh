#!/usr/bin/env bash

set -euo pipefail

DEFAULT_URL="https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v14.2.1-1.1/xpack-arm-none-eabi-gcc-14.2.1-1.1-linux-x64.tar.gz"
DEFAULT_INSTALL_DIR="/opt/xpack"
DEFAULT_PRIORITY=60

DOWNLOAD_URL="$DEFAULT_URL"
INSTALL_ROOT="$DEFAULT_INSTALL_DIR"
PRIORITY="$DEFAULT_PRIORITY"
SET_DEFAULT=0

TOOLS=(
  "arm-none-eabi-gcc"
  "arm-none-eabi-g++"
  "arm-none-eabi-ar"
  "arm-none-eabi-ranlib"
  "arm-none-eabi-ld"
  "arm-none-eabi-nm"
  "arm-none-eabi-objcopy"
  "arm-none-eabi-objdump"
  "arm-none-eabi-strip"
)

print_usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Install the xPack GNU Arm Embedded GCC toolchain and register it with update-alternatives.

Options:
  -u, --url <url>            Download URL for the toolchain archive
  -d, --install-dir <path>   Installation directory (default: ${DEFAULT_INSTALL_DIR})
  -p, --priority <value>     update-alternatives priority (default: ${DEFAULT_PRIORITY})
      --set-default          Set the installed toolchain as the system default
  -h, --help                 Show this help message

Examples:
  sudo $(basename "$0")
  sudo $(basename "$0") --install-dir /usr/local/xpack --set-default
  sudo $(basename "$0") --url "https://example.com/toolchain.tar.gz"
EOF
}

require_command() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Error: Required command '$cmd' not found in PATH." >&2
    exit 1
  fi
}

cleanup() {
  if [[ -n "${WORK_DIR:-}" && -d "$WORK_DIR" ]]; then
    rm -rf "$WORK_DIR"
  fi
}

trap cleanup EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    -u|--url)
      shift
      DOWNLOAD_URL="${1:-}"
      ;;
    -d|--install-dir)
      shift
      INSTALL_ROOT="${1:-}"
      ;;
    -p|--priority)
      shift
      PRIORITY="${1:-}"
      ;;
    --set-default)
      SET_DEFAULT=1
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      print_usage
      exit 1
      ;;
  esac
  shift || true
done

if [[ -z "$DOWNLOAD_URL" ]]; then
  echo "Error: Download URL must not be empty." >&2
  exit 1
fi

if [[ -z "$INSTALL_ROOT" ]]; then
  echo "Error: Installation directory must not be empty." >&2
  exit 1
fi

if ! [[ "$PRIORITY" =~ ^[0-9]+$ ]]; then
  echo "Error: Priority must be a positive integer." >&2
  exit 1
fi

require_command curl
require_command tar
require_command sudo
require_command update-alternatives

WORK_DIR=$(mktemp -d)
echo "Using temporary workspace: ${WORK_DIR}"

ARCHIVE_NAME=$(basename "$DOWNLOAD_URL")
ARCHIVE_PATH="${WORK_DIR}/${ARCHIVE_NAME}"

echo "Downloading toolchain archive from ${DOWNLOAD_URL} ..."
curl -L -o "$ARCHIVE_PATH" "$DOWNLOAD_URL"

if [[ ! -s "$ARCHIVE_PATH" ]]; then
  echo "Error: Download failed or archive is empty." >&2
  exit 1
fi

echo "Inspecting archive contents..."

# Test if tar can read the archive
if ! tar -tzf "$ARCHIVE_PATH" >/dev/null 2>&1; then
  echo "Error: Unable to inspect archive contents." >&2
  echo "Archive path: $ARCHIVE_PATH" >&2
  echo "Archive exists: $([[ -f "$ARCHIVE_PATH" ]] && echo "yes" || echo "no")" >&2
  exit 1
fi

# Extract archive root directory
FIRST_LINE=""
read -r FIRST_LINE < <(tar -tzf "$ARCHIVE_PATH")
ARCHIVE_ROOT=$(echo "$FIRST_LINE" | cut -d/ -f1)

if [[ -z "$ARCHIVE_ROOT" ]]; then
  echo "Error: Unable to determine archive root directory." >&2
  exit 1
fi

INSTALL_ROOT=$(readlink -f "$INSTALL_ROOT")
mkdir -p "$INSTALL_ROOT"

INSTALL_PATH="${INSTALL_ROOT}/${ARCHIVE_ROOT}"

echo "Resolved install path: ${INSTALL_PATH}"

if [[ -d "$INSTALL_PATH" ]]; then
  echo "Existing installation detected at ${INSTALL_PATH}. Skipping extraction."
else
  echo "Extracting archive to ${INSTALL_ROOT} ..."
  tar -xzf "$ARCHIVE_PATH" -C "$INSTALL_ROOT"
fi

BIN_DIR="${INSTALL_PATH}/bin"

if [[ ! -d "$BIN_DIR" ]]; then
  echo "Error: Bin directory not found at ${BIN_DIR}." >&2
  exit 1
fi

echo "Registering toolchain executables with update-alternatives (priority ${PRIORITY}) ..."

for tool in "${TOOLS[@]}"; do
  TOOL_PATH="${BIN_DIR}/${tool}"
  echo "Registering ${tool} -> ${TOOL_PATH}"
  if [[ -x "$TOOL_PATH" ]]; then
    sudo update-alternatives --install "/usr/bin/${tool}" "$tool" "$TOOL_PATH" "$PRIORITY"
    if [[ "$SET_DEFAULT" -eq 1 ]]; then
      sudo update-alternatives --set "$tool" "$TOOL_PATH"
    fi
  else
    echo "Warning: Expected executable not found: ${TOOL_PATH}" >&2
  fi
done

echo "Toolchain installed at ${INSTALL_PATH}."
echo "Bin directory added to update-alternatives: ${BIN_DIR}."

if [[ "$SET_DEFAULT" -eq 1 ]]; then
  echo "The toolchain has been set as the default for the registered executables."
else
  echo "The toolchain has been registered. Use 'sudo update-alternatives --config <tool>' to set defaults if desired."
fi


