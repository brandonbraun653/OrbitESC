include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    lib_orbit_hw
  SOURCES
    orbit_adc.cpp
    orbit_can.cpp
    orbit_gpio.cpp
    orbit_i2c.cpp
    orbit_timer.cpp
    orbit_usart.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    project_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
