include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    lib_orbit_com
  SOURCES
    can_message.cpp
    can_server.cpp
    routers/can_arm_router.cpp
    routers/can_ping_router.cpp
    routers/can_speed_router.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    project_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
