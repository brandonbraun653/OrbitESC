#!/bin/bash

ncpu=$(nproc)
echo "Building OrbitESC"
cmake --build ./../build --target OrbitESC -j$ncpu