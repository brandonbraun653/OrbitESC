include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    lib_orbit_monitor
  SOURCES
    orbit_current_monitor.cpp
    orbit_monitors.cpp
    orbit_voltage_monitor.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    project_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
