# NOTE:
# The xpack arm-none-eabi-gdb is built --without-python, so the Python block below
# is disabled. Re-enable only if you switch to a Python-enabled GDB build (e.g.
# arm-none-eabi-gdb-py3) and need PyCortexMDebug's sys.path injection.

# python
# # Update GDB's Python paths with the `sys.path` values of the local
# #  Python installation, whether that is brew'ed Python, a virtualenv,
# #  or another system python.

# # Convert GDB to interpret in Python
# import os,subprocess,sys

# # Uncomment to figure out which python version your GDB is using
# print(sys.version)

# # Execute a Python using the user's shell and pull out the sys.path (for site-packages)
# paths = subprocess.check_output('python3 -c "import os,sys;print(os.linesep.join(sys.path).strip())"',shell=True).decode("utf-8").split()
# # Extend GDB's Python's search path
# sys.path.extend(paths)
# end

# Import PyCortexMDebug
# source lib/CommonTools/PyCortexMDebug/scripts/gdb.py
# svd lib/CommonTools/svd/STM32L4x2.svd
