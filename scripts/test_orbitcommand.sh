#!/usr/bin/env bash
# Build and run the OrbitCommand test tiers.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
orbit_dir="${repo_root}/tools/OrbitCommand"
results_dir="${ORBIT_TEST_RESULTS:-${repo_root}/test-results/orbitcommand}"
preset="release"
label=""
build_simulator=1
coverage=0

usage() {
  cat <<'EOF'
Usage: scripts/test_orbitcommand.sh [options]

Options:
  --unit-only       Run only the GoogleTest unit tier
  --e2e-only        Run only the Qat GUI E2E tier
  --no-simulator    Do not build or require the OrbitESC simulator
  --coverage        Build the coverage preset and run the unit tier with gcovr
  --asan            Build and run the ASan + UBSan preset
  --tsan            Build and run the advisory TSan preset
  -h, --help        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --unit-only)
      label="unit"
      ;;
    --e2e-only)
      label="e2e"
      ;;
    --no-simulator)
      build_simulator=0
      ;;
    --coverage)
      preset="coverage"
      label="unit"
      coverage=1
      build_simulator=0
      ;;
    --asan)
      preset="asan"
      ;;
    --tsan)
      preset="tsan"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

mkdir -p "${results_dir}"

if [[ "${build_simulator}" -eq 1 ]]; then
  echo "== Building the OrbitESC simulator (${preset} tests) =="
  # shellcheck disable=SC2086
  cmake --preset "OrbitESC Simulator Release" ${ORBIT_SIM_CMAKE_ARGS:-}
  cmake --build --preset "OrbitESC Simulator Release"
fi

echo "== Building OrbitCommand (${preset}) =="
cd "${orbit_dir}"
cmake --preset "${preset}"
cmake --build --preset "${preset}"

ctest_args=(--preset "${preset}" --output-on-failure --output-junit "${results_dir}/ctest-${preset}.xml")
if [[ -n "${label}" ]]; then
  ctest_args+=(-L "${label}")
fi

echo "== Running CTest (${preset}${label:+ label=${label}}) =="
ctest "${ctest_args[@]}"

if [[ "${coverage}" -eq 1 ]]; then
  echo "== Generating the coverage report =="
  cmake --build --preset coverage --target coverage
  echo "Coverage report: ${orbit_dir}/build/coverage/coverage/index.html"
fi

echo "Results: ${results_dir}"
