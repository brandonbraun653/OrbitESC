include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    lib_orbit_tasks
  SOURCES
    tsk_ctrl_sys.cpp
    tsk_idle.cpp
    tsk_hwm.cpp
    tsk_hwm_dio.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    project_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
