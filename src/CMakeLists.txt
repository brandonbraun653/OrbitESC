include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# -----------------------------------------------------------------------------
# Project Libraries
# -----------------------------------------------------------------------------
add_subdirectory(config/bsp)
add_subdirectory(core)

# -----------------------------------------------------------------------------
# Project Libraries
# -----------------------------------------------------------------------------
gen_intf_lib_variants(
  TARGET
    embedded_core_libs
  LIBRARIES
    thor_cmn_cm4
    thor_hld_core_dbg
    thor_lld_intf_dbg
    thor_lld_stm32l4_dbg
    thor_cfg_freertos
  EXPORT_DIR
    ${PROJECT_BINARY_DIR}/OrbitESC
)

gen_intf_lib_variants(
  TARGET
    aurora_libs
  LIBRARIES
    aurora_core
    aurora_database
    aurora_datastore
    aurora_logging
    aurora_memory
  EXPORT_DIR
    ${PROJECT_BINARY_DIR}/OrbitESC
)


# -----------------------------------------------------------------------------
# Hardware Config:
# cmake -DTOOLCHAIN=arm-none-eabi -DFREERTOS_THREADS=ON -DDEVICE_TARGET=stm32l432kb -DCMAKE_BUILD_TYPE=Debug ./../
#
# Simulator Config:
# cmake -DTOOLCHAIN=gcc -DNATIVE_THREADS=ON -DDEVICE_TARGET=sim -DCMAKE_BUILD_TYPE=Debug ./../
# -----------------------------------------------------------------------------
add_executable(OrbitESC startup.cpp)

target_link_options(OrbitESC PRIVATE "-Wl,-Map=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/OrbitESC.map")

# Include Directories
target_include_directories(OrbitESC PRIVATE ".")

# Dependent Modules
target_link_libraries(OrbitESC PRIVATE
  # Public Includes
  aurora_intf_inc
  chimera_intf_inc
  freertos_intf_inc_all
  thor_intf_inc

  # Project Libraries
  lib_orbit_hw
  lib_orbit_cfg_bsp
  lib_orbit_tasks
  lib_orbit_core

  # Submodule Libraries
  chimera_core_rel
  aurora_libs_dbg

  # Target Properties
  prj_device_target
  prj_build_target
)

# Hardware Driver Model
if(${DEVICE_TARGET} MATCHES "^stm32l4")
  target_link_libraries(OrbitESC PRIVATE embedded_core_libs)
elseif(${DEVICE_TARGET} MATCHES "^sim")
  target_link_libraries(OrbitESC PRIVATE chimera_sim_core)
else()
  message(FATAL_ERROR "Unsupported device target")
endif()


if(Toolchain::HAS_GCC_ARM_NONE_EABI)
  target_link_libraries(OrbitESC PRIVATE
    # Aurora

    # Threading Library
    freertos_intf_inc_all
    freertos_core_rel
    freertos_port
    freertos_heap
    freertos_heap_4_rel
    freertos_cm4f_port_rel

    # Segger System View
    segger_sys_view_rel
    segger_sys_view_intf
    segger_sys_view_cortex_m4_freertos_rel
  )
endif()

target_include_directories(OrbitESC PRIVATE ${PROJECT_ROOT})
add_custom_command(TARGET OrbitESC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:OrbitESC> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sys_image
)
