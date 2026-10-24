#!/bin/bash

# Script to profile the OrbitESC simulator using Linux perf tools
# Usage: ./profile_simulator.sh [profile_duration_seconds] [perf_options]

set -euo pipefail

# Default values
DURATION="${1:-30}"  # Default 30 seconds if not specified
PERF_OPTIONS="${2:--F 99 --call-graph=dwarf}"  # Default high frequency sampling with call graph

# Build paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SIMULATOR_PATH="$PROJECT_ROOT/build/artifacts/last_build_version/OrbitESC"
OUTPUT_DIR="$PROJECT_ROOT/profile_data"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $*"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $*"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $*"
}

# Check if simulator exists
if [[ ! -f "$SIMULATOR_PATH" ]]; then
    print_error "Simulator executable not found at: $SIMULATOR_PATH"
    print_error "Make sure to build the project first with: ./scripts/build_embedded.sh"
    exit 1
fi

# Check if perf is available
if ! command -v perf >/dev/null 2>&1; then
    print_error "perf command not found. Please install linux-tools-common:"
    print_error "sudo apt-get install linux-tools-common linux-tools-generic"
    exit 1
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Generate timestamp for unique filenames
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
PERF_DATA="$OUTPUT_DIR/perf_${TIMESTAMP}.data"
PERF_SCRIPT="$OUTPUT_DIR/perf_${TIMESTAMP}.script"
PERF_REPORT="$OUTPUT_DIR/perf_${TIMESTAMP}.report"

print_info "Starting OrbitESC simulator profiler"
print_info "Profile duration: ${DURATION} seconds"
print_info "Perf options: $PERF_OPTIONS"
print_info "Output directory: $OUTPUT_DIR"

# Start the simulator in the background
print_info "Starting simulator..."
"$SIMULATOR_PATH" &
SIM_PID=$!

# Wait a moment for the simulator to initialize
sleep 2

# Check if simulator is still running
if ! kill -0 "$SIM_PID" 2>/dev/null; then
    print_error "Simulator failed to start or crashed immediately"
    exit 1
fi

print_info "Simulator started with PID: $SIM_PID"

# Function to cleanup on exit
cleanup() {
    print_info "Cleaning up..."

    if kill -0 "$SIM_PID" 2>/dev/null; then
        print_info "Stopping simulator (PID: $SIM_PID)..."
        kill -TERM "$SIM_PID" 2>/dev/null || true

        # Wait for graceful shutdown
        for i in {1..5}; do
            if ! kill -0 "$SIM_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done

        # Force kill if still running
        if kill -0 "$SIM_PID" 2>/dev/null; then
            print_warning "Force killing simulator..."
            kill -KILL "$SIM_PID" 2>/dev/null || true
        fi
    fi

    print_info "Cleanup complete"
}

# Set trap to cleanup on script exit
trap cleanup EXIT

# Record performance profile
print_info "Recording performance profile for ${DURATION} seconds..."
perf record -p "$SIM_PID" -o "$PERF_DATA" $PERF_OPTIONS -- sleep "$DURATION"

# Generate script output for detailed analysis
print_info "Generating script output..."
perf script -i "$PERF_DATA" > "$PERF_SCRIPT"

# Generate human-readable report
print_info "Generating report..."
perf report -i "$PERF_DATA" --stdio > "$PERF_REPORT"

print_info "Profiling complete!"
print_info "Generated files:"
echo "  - $PERF_DATA (raw perf data)"
echo "  - $PERF_SCRIPT (script output for detailed analysis)"
echo "  - $PERF_REPORT (human-readable report)"

echo ""
echo "To view results:"
echo "  perf report -i $PERF_DATA"
echo "  perf script -i $PERF_DATA | less"
echo "  cat $PERF_REPORT | less"
echo ""
echo "For interactive exploration:"
echo "  perf report --gtk -i $PERF_DATA"
