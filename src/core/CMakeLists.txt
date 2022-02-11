include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

add_subdirectory(tasks)

gen_static_lib_variants(
  TARGET
    lib_orbit_core
  SOURCES
    bootup.cpp
    tasks.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    project_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
