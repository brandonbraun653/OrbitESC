#!/bin/bash

ncpu=$(grep ^cpu\\scores /proc/cpuinfo | uniq |  awk '{print $4}')
echo "Building OrbitESC with $ncpu threads"
cmake --build ./../build --target OrbitESC -j$ncpu