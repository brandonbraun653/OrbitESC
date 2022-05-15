# ====================================================
# Override the toolchain used. This must be done here
# else CMake won't allow the override.
# ====================================================
set(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "")
set(COMMON_TOOL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/CommonTools" CACHE STRING "")
# set(ARM_NONE_EABI_ROOT "$ENV{HOME}/toolchain/gcc-arm-none-eabi-9-2020-q2-update/bin" CACHE STRING "")
set(ARM_NONE_EABI_ROOT "$ENV{HOME}/toolchain/gcc-arm-none-eabi-10.3-2021.10/bin" CACHE STRING "")

include("${COMMON_TOOL_ROOT}/cmake/options/toolchain.cmake")
