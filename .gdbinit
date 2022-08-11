# NOTE:
# When using this script on a new system, it expects the system Python version to match the version used when compiling
# the arm-none-eabi-gdb-py3 executable. As of writing, this project's compiler is the XPack GCC 11.2.1 arm-none-eabi
# variant, which was built with Python 3.10.4. Installing the system Python to default to a 3.10.x variant is sufficient
# to allow this module to load properly.

python
# Update GDB's Python paths with the `sys.path` values of the local
#  Python installation, whether that is brew'ed Python, a virtualenv,
#  or another system python.

# Convert GDB to interpret in Python
import os,subprocess,sys

# Uncomment to figure out which python version your GDB is using
print(sys.version)

# Execute a Python using the user's shell and pull out the sys.path (for site-packages)
paths = subprocess.check_output('python3 -c "import os,sys;print(os.linesep.join(sys.path).strip())"',shell=True).decode("utf-8").split()
# Extend GDB's Python's search path
sys.path.extend(paths)
end

# Import PyCortexMDebug
# source lib/CommonTools/PyCortexMDebug/scripts/gdb.py
# svd lib/CommonTools/svd/STM32L4x2.svd
