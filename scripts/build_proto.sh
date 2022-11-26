# Expects to be called from this directory
cd ../
_cwd=$(pwd)

# Ensure the current shell can run conda activate
# https://stackoverflow.com/a/65183109/8341975
echo "Activating project environment"
source ~/anaconda3/etc/profile.d/conda.sh
conda activate $_cwd/.conda

# Build the C bindings
echo "Building Nanopb C-Bindings"
python lib/Aurora/lib/nanopb/nanopb/generator/nanopb_generator.py src/core/com/serial/interface.proto

# Build the Python bindings
SRC_DIR=$_cwd/src/core/com/serial
DST_DIR=$_cwd/src/core/com/serial

echo "Building Python Bindings"
protoc -I=$SRC_DIR --python_out=$DST_DIR $SRC_DIR/interface.proto