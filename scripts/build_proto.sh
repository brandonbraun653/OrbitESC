#!/bin/bash

# Expects to be called from this directory
cd ../
_cwd=$(pwd)

# Ensure the current shell can run our project python interpreter
# https://stackoverflow.com/a/65183109/8341975
echo "Activating project environment"
source "$_cwd"/.venv/bin/activate

SRC_DIR=$_cwd/src/core/com/proto
NPB_DIR=$_cwd/lib/Aurora/lib/nanopb/nanopb/generator/proto
DST_DIR=$_cwd/pyorbit/nanopb
GENERATOR_DIR=$_cwd/lib/Aurora/lib/nanopb/nanopb/generator

# Ensure nanopb generator modules are discoverable
export PYTHONPATH="$GENERATOR_DIR${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$GENERATOR_DIR${PATH:+:$PATH}"

# Build the C bindings
echo "Building Nanopb C-Bindings"
python lib/Aurora/lib/nanopb/nanopb/generator/nanopb_generator.py --cpp-descriptors --output-dir="$SRC_DIR" --proto-path="$SRC_DIR" motor_control.proto
python lib/Aurora/lib/nanopb/nanopb/generator/nanopb_generator.py --cpp-descriptors --output-dir="$SRC_DIR" --proto-path="$SRC_DIR" serial_interface.proto
python lib/Aurora/lib/nanopb/nanopb/generator/nanopb_generator.py --cpp-descriptors --output-dir="$SRC_DIR" --proto-path="$SRC_DIR" system_config.proto
python lib/Aurora/lib/nanopb/nanopb/generator/nanopb_generator.py --cpp-descriptors --output-dir="$SRC_DIR" --proto-path="$SRC_DIR" system_control.proto
python lib/Aurora/lib/nanopb/nanopb/generator/nanopb_generator.py --cpp-descriptors --output-dir="$SRC_DIR" --proto-path="$SRC_DIR" system_data.proto


# Build the Python bindings
# Uses the protoc bundled with grpcio-tools. nanopb.proto must be listed as an
# input (not just an include path) so that nanopb_pb2.py gets generated — the
# generated *_pb2.py modules import it at load time.
echo "Building Python Bindings"
python -m grpc_tools.protoc -I="$SRC_DIR" -I="$NPB_DIR" \
  --python_out="$DST_DIR" --mypy_out="$DST_DIR" \
  "$SRC_DIR"/*.proto "$NPB_DIR/nanopb.proto"

# protoc emits flat imports (e.g. "import serial_interface_pb2 as ...") that
# assume the output directory is on sys.path. Rewrite them to package-qualified
# imports so the modules load correctly as pyorbit.nanopb.* submodules.
echo "Fixing generated import paths"
sed -i -E 's/^import ([a-z0-9_]+_pb2) as /from pyorbit.nanopb import \1 as /' "$DST_DIR"/*_pb2.py
