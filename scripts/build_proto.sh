#!/bin/bash

# Expects to be called from this directory
cd ../
_cwd=$(pwd)

# Ensure the current shell can run conda activate
# https://stackoverflow.com/a/65183109/8341975
echo "Activating project environment"
source "$_cwd"/.venv/bin/activate

# Build the C bindings
echo "Building Nanopb C-Bindings"
python lib/Aurora/lib/nanopb/nanopb/generator/nanopb_generator.py src/core/com/proto/serial_interface.proto

# Build the Python bindings
SRC_DIR=$_cwd/src/core/com/proto
NPB_DIR=$_cwd/lib/Aurora/lib/nanopb/nanopb/generator/proto
DST_DIR=$_cwd/pyorbit/nanopb

echo "Building Python Bindings"
protoc -I="$SRC_DIR" -I="$NPB_DIR" --python_out="$DST_DIR" "$SRC_DIR"/serial_interface.proto