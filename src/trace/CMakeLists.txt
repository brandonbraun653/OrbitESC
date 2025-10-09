include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# Define sources based on build type
set(TRACE_SOURCES
  trace_registry.cpp
  trace_interface.cpp
  trace_examples.cpp
)

# Add environment-specific sources
if(NOT SIMULATOR)
  list(APPEND TRACE_SOURCES trace_embedded.cpp)
else()
  list(APPEND TRACE_SOURCES trace_simulator.cpp)
endif()

gen_static_lib_variants(
  TARGET
    lib_orbit_trace
  SOURCES
    ${TRACE_SOURCES}
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    project_intf_inc
  PRV_DEFINES
    ORBIT_TRACE_MODULE
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
