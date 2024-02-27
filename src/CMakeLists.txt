include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# -----------------------------------------------------------------------------
# Project Libraries
# -----------------------------------------------------------------------------
add_subdirectory(core)
add_subdirectory(control)
add_subdirectory(monitor)

# -----------------------------------------------------------------------------
# Project Libraries
# -----------------------------------------------------------------------------
gen_intf_lib_variants(
  TARGET
    embedded_core_libs
  LIBRARIES
    thor_cfg_freertos
    thor_cmn_cm4
    thor_hld_adc
    thor_hld_can
    thor_hld_clock
    thor_hld_dma
    thor_hld_exti
    thor_hld_gpio
    thor_hld_i2c
    thor_hld_interrupt
    thor_hld_sdio
    thor_hld_serial
    thor_hld_spi
    thor_hld_system
    thor_hld_timer
    thor_hld_uart
    thor_hld_usart
    thor_hld_watchdog
    thor_lld_cmn
    thor_lld_intf_adc
    thor_lld_intf_can
    thor_lld_intf_dma
    thor_lld_intf_exti
    thor_lld_intf_gpio
    thor_lld_intf_i2c
    thor_lld_intf_interrupt
    thor_lld_intf_rcc
    thor_lld_intf_sdio
    thor_lld_intf_serial
    thor_lld_intf_spi
    thor_lld_intf_sys
    thor_lld_intf_timer
    thor_lld_intf_usart
    thor_lld_intf_watchdog
  EXPORT_DIR
    ${PROJECT_BINARY_DIR}/OrbitESC
)

gen_intf_lib_variants(
  TARGET
    stm32l4_libs
  LIBRARIES
    thor_lld_stm32l4_adc
    thor_lld_stm32l4_can
    thor_lld_stm32l4_crs
    thor_lld_stm32l4_des
    thor_lld_stm32l4_dma
    thor_lld_stm32l4_exti
    thor_lld_stm32l4_flash
    thor_lld_stm32l4_gpio
    thor_lld_stm32l4_i2c
    thor_lld_stm32l4_interrupt
    thor_lld_stm32l4_iwdg
    thor_lld_stm32l4_power
    thor_lld_stm32l4_rcc
    thor_lld_stm32l4_spi
    thor_lld_stm32l4_startup
    thor_lld_stm32l4_system
    thor_lld_stm32l4_usart
    thor_lld_stm32l4_wwdg
  EXPORT_DIR
    ${PROJECT_BINARY_DIR}/OrbitESC
)

gen_intf_lib_variants(
  TARGET
    stm32f4_libs
  LIBRARIES
    thor_lld_stm32f4_adc
    thor_lld_stm32f4_can
    thor_lld_stm32f4_des
    thor_lld_stm32f4_dma
    thor_lld_stm32f4_exti
    thor_lld_stm32f4_flash
    thor_lld_stm32f4_gpio
    thor_lld_stm32f4_i2c
    thor_lld_stm32f4_interrupt
    thor_lld_stm32f4_iwdg
    thor_lld_stm32f4_power
    thor_lld_stm32f4_rcc
    thor_lld_stm32f4_sdio
    thor_lld_stm32f4_spi
    thor_lld_stm32f4_startup
    thor_lld_stm32f4_system
    thor_lld_stm32f4_usart
    thor_lld_stm32f4_wwdg
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
    aurora_filesystem_core
    aurora_filesystem_fatfs_driver
    aurora_logging
    aurora_memory
  EXPORT_DIR
    ${PROJECT_BINARY_DIR}/OrbitESC
)

gen_intf_lib_variants(
  TARGET
    orbit_libs
  LIBRARIES
    lib_orbit_com
    lib_orbit_core
    lib_orbit_ctrl
    lib_orbit_data_rel_dbg
    lib_orbit_events
    lib_orbit_hw
    lib_orbit_monitor
    lib_orbit_runtime
    lib_orbit_tasks
    lib_orbit_testing
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
  aurora_libs_rel
  chimera_core_rel
  lib_cobs_rel
  lib_flash_db_rel
  lib_nanopb_rel

  # Orbit Libraries
  lib_orbit_com_rel
  lib_orbit_core_rel
  lib_orbit_ctrl_rel_dbg
  lib_orbit_data_rel
  lib_orbit_events_rel
  lib_orbit_hw_rel
  lib_orbit_monitor_rel
  lib_orbit_runtime_rel
  lib_orbit_tasks_rel

  # Target Properties
  prj_device_target
  prj_build_target
)

# Hardware Driver Model
if(${DEVICE_TARGET} MATCHES "^stm32[lf]4")
  target_link_libraries(OrbitESC PRIVATE
    embedded_core_libs_rel
    lib_tiny_usb_rel
    )

  if(${DEVICE_TARGET} MATCHES "^stm32l4")
    target_link_libraries(OrbitESC PRIVATE stm32l4_libs)
  elseif(${DEVICE_TARGET} MATCHES "^stm32f4")
    target_link_libraries(OrbitESC PRIVATE stm32f4_libs_rel)
  endif()

elseif(${DEVICE_TARGET} MATCHES "^sim")
  target_link_libraries(OrbitESC PRIVATE
    chimera_sim_core
    gtest_src
    gmock_src
    zmq
    )
else()
  message(FATAL_ERROR "Unsupported device target")
endif()


if(Toolchain::HAS_GCC_ARM_NONE_EABI)
  target_link_libraries(OrbitESC PRIVATE
    # Threading Library
    freertos_intf_inc_all
    freertos_core_rel
    freertos_port
    freertos_heap
    freertos_heap_4
    freertos_cm4f_port

    # Segger System View
    segger_sys_view
    segger_sys_view_intf
    segger_sys_view_cortex_m4_freertos
  )
endif()

target_include_directories(OrbitESC PRIVATE ${PROJECT_ROOT})
add_custom_command(TARGET OrbitESC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:OrbitESC> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sys_image
)
