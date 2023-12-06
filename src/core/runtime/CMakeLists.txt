include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    lib_orbit_runtime
  SOURCES
    can_runtime.cpp
    serial_runtime.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    project_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
