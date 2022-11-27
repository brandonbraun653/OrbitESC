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


# Remove the existing conda environment
if [ -d "$_cwd/.conda" ]
then
    echo "Removing existing virtual environment"
    rm -rf $_cwd/.conda/
fi

# Use the project's environment file to initialize this instance
conda env create --file lib/Conda/project/QuadcopterDrone/environment.yml --prefix ./.conda

# Ensure the current shell can run conda activate
# https://stackoverflow.com/a/65183109/8341975
source $_conda_install_dir/etc/profile.d/conda.sh
conda activate $_cwd/.conda

# Add local python tooling
conda develop $_cwd
conda develop $_cwd/src/core/com/serial
conda develop $_cwd/lib/Aurora/lib/nanopb/nanopb/generator
