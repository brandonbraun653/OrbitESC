#!/usr/bin/env bash
set -u

: "${ORBITCOMMAND_BINARY:?ORBITCOMMAND_BINARY must point at the release OrbitCommand binary}"
log_file="${ORBITCOMMAND_LOG:-/dev/null}"

exec "${ORBITCOMMAND_BINARY}" "$@" >>"${log_file}" 2>&1
