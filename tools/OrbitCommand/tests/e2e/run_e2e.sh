#!/usr/bin/env bash
set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="${ORBIT_REPO_ROOT:-$(cd "${script_dir}/../../../.." && pwd)}"
python_bin="${ORBIT_E2E_PYTHON:-${repo_root}/.venv/bin/python}"

if [[ ! -x "${python_bin}" ]]; then
  python_bin="$(command -v python3 || true)"
fi
if [[ -z "${python_bin}" ]]; then
  echo "SKIP: python3 is not available" >&2
  exit 77
fi

if ! "${python_bin}" -c "import qat" >/dev/null 2>&1; then
  echo "SKIP: the qat package is not installed (run 'uv sync')" >&2
  exit 77
fi

if [[ -z "${DISPLAY:-}" && -z "${ORBIT_E2E_XVFB_ACTIVE:-}" ]]; then
  if command -v xvfb-run >/dev/null 2>&1; then
    export ORBIT_E2E_XVFB_ACTIVE=1
    exec xvfb-run -a -s "-screen 0 1600x1000x24" "${BASH_SOURCE[0]}" "$@"
  fi
  echo "SKIP: no DISPLAY and xvfb-run is not available" >&2
  exit 77
fi

simulator_binary="${ORBIT_SIMULATOR_BINARY:-${repo_root}/build/sim-release/bin/OrbitESC}"
if [[ ! -x "${simulator_binary}" ]]; then
  echo "SKIP: OrbitESC simulator not found at ${simulator_binary}" >&2
  exit 77
fi

orbitcommand_binary="${ORBITCOMMAND_BINARY:-${repo_root}/tools/OrbitCommand/build/release/OrbitCommand}"
if [[ ! -x "${orbitcommand_binary}" ]]; then
  echo "OrbitCommand binary not found at ${orbitcommand_binary}; build the release preset" >&2
  exit 1
fi

artifacts_dir="${ORBIT_E2E_ARTIFACTS:-${repo_root}/test-results/orbitcommand-e2e}"
mkdir -p "${artifacts_dir}"

export ORBIT_REPO_ROOT="${repo_root}"
export ORBIT_SIMULATOR_BINARY="${simulator_binary}"
export ORBITCOMMAND_BINARY="${orbitcommand_binary}"
export ORBIT_E2E_ARTIFACTS="${artifacts_dir}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"

cd "${script_dir}"
exec "${python_bin}" -m pytest -c pytest.ini --junitxml="${artifacts_dir}/junit.xml" "$@"
