include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

add_subdirectory(com)
add_subdirectory(data)
add_subdirectory(events)
add_subdirectory(hw)
add_subdirectory(runtime)
add_subdirectory(tasks)

gen_static_lib_variants(
  TARGET
    lib_orbit_core
  SOURCES
    bootup.cpp
    syscalls.cpp
    system.cpp
    tasks.cpp
    utility.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    project_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
