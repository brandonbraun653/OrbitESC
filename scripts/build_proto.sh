#!/bin/bash

# Expects to be called from this directory
cd ../
_cwd=$(pwd)
_conda=$(which conda)

# Figure out where anacaonda was installed on this machine
if [ ! -f "$_conda" ]
then
    echo "Conda application not found!"
    exit -1
fi

_conda_install_dir=$(readlink -f $(dirname $_conda)/..)


# Ensure the current shell can run conda activate
# https://stackoverflow.com/a/65183109/8341975
echo "Activating project environment"
source $_conda_install_dir/etc/profile.d/conda.sh
conda activate $_cwd/.conda

# Build the C bindings
echo "Building Nanopb C-Bindings"
python lib/Aurora/lib/nanopb/nanopb/generator/nanopb_generator.py src/core/com/serial/interface.proto

# Build the Python bindings
SRC_DIR=$_cwd/src/core/com/serial
NPB_DIR=$_cwd/lib/Aurora/lib/nanopb/nanopb/generator/proto
DST_DIR=$_cwd/pyorbit/nanopb

echo "Building Python Bindings"
protoc -I=$SRC_DIR -I=$NPB_DIR --python_out=$DST_DIR $SRC_DIR/interface.proto