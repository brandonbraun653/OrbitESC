ncpu=$(nproc)
echo "Building OrbitESC with $ncpu threads"
cmake --build ./../build --target OrbitESC -j$ncpu