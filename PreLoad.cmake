# ====================================================
# Override the toolchain used. This must be done here
# else CMake won't allow the override.
# ====================================================
set(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "")
set(COMMON_TOOL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/CommonTools" CACHE STRING "")
set(ARM_NONE_EABI_ROOT "$ENV{HOME}/toolchain/xpack-arm-none-eabi-gcc-11.2.1-1.2/bin" CACHE STRING "")

include("${COMMON_TOOL_ROOT}/cmake/options/toolchain.cmake")
