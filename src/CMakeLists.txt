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
    thor_cfg_freertos_rel_dbg
    thor_cmn_cm4_dbg
    thor_hld_adc_rel_dbg
    thor_hld_can_rel_dbg
    thor_hld_clock_rel_dbg
    thor_hld_dma_rel_dbg
    thor_hld_exti_rel_dbg
    thor_hld_gpio_rel_dbg
    thor_hld_i2c_rel_dbg
    thor_hld_interrupt_rel_dbg
    thor_hld_serial_rel_dbg
    thor_hld_spi_rel_dbg
    thor_hld_system_rel_dbg
    thor_hld_timer_rel_dbg
    thor_hld_uart_rel_dbg
    thor_hld_usart_rel_dbg
    thor_hld_watchdog_rel_dbg
    thor_lld_cmn_rel_dbg
    thor_lld_intf_adc_rel_dbg
    thor_lld_intf_can_rel_dbg
    thor_lld_intf_dma_rel_dbg
    thor_lld_intf_exti_rel_dbg
    thor_lld_intf_gpio_rel_dbg
    thor_lld_intf_i2c_rel_dbg
    thor_lld_intf_interrupt_rel_dbg
    thor_lld_intf_rcc_rel_dbg
    thor_lld_intf_serial_rel_dbg
    thor_lld_intf_spi_rel_dbg
    thor_lld_intf_sys_rel_dbg
    thor_lld_intf_timer_rel_dbg
    thor_lld_intf_usart_rel_dbg
    thor_lld_intf_watchdog_rel_dbg
  EXPORT_DIR
    ${PROJECT_BINARY_DIR}/OrbitESC
)

gen_intf_lib_variants(
  TARGET
    stm32l4_libs
  LIBRARIES
    thor_lld_stm32l4_adc_rel_dbg
    thor_lld_stm32l4_can_rel_dbg
    thor_lld_stm32l4_crs_rel_dbg
    thor_lld_stm32l4_des_rel_dbg
    thor_lld_stm32l4_dma_rel_dbg
    thor_lld_stm32l4_exti_rel_dbg
    thor_lld_stm32l4_flash_rel_dbg
    thor_lld_stm32l4_gpio_rel_dbg
    thor_lld_stm32l4_i2c_rel_dbg
    thor_lld_stm32l4_interrupt_rel_dbg
    thor_lld_stm32l4_iwdg_rel_dbg
    thor_lld_stm32l4_power_rel_dbg
    thor_lld_stm32l4_rcc_rel_dbg
    thor_lld_stm32l4_spi_rel_dbg
    thor_lld_stm32l4_startup_rel_dbg
    thor_lld_stm32l4_system_rel_dbg
    thor_lld_stm32l4_usart_rel_dbg
    thor_lld_stm32l4_wwdg_rel_dbg
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
    aurora_filesystem_lfs_driver
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
    lib_orbit_data
    lib_orbit_hw
    lib_orbit_monitor
    lib_orbit_runtime
    lib_orbit_tasks
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
  aurora_libs_rel_dbg
  chimera_core_rel_dbg
  lib_cobs_rel_dbg
  lib_nanopb_rel_dbg
  orbit_libs

  # Target Properties
  prj_device_target
  prj_build_target
)

# Hardware Driver Model
if(${DEVICE_TARGET} MATCHES "^stm32[lf]4")
  target_link_libraries(OrbitESC PRIVATE embedded_core_libs)

  if(${DEVICE_TARGET} MATCHES "^stm32l4")
    target_link_libraries(OrbitESC PRIVATE stm32l4_libs)
  elseif(${DEVICE_TARGET} MATCHES "^stm32f4")
    target_link_libraries(OrbitESC PRIVATE stm32f4_libs)
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
    freertos_core_rel_dbg
    freertos_port
    freertos_heap
    freertos_heap_4_rel_dbg
    freertos_cm4f_port_rel_dbg

    # Segger System View
    # segger_sys_view_rel_dbg
    # segger_sys_view_intf
    # segger_sys_view_cortex_m4_freertos_rel_dbg
  )
endif()

target_include_directories(OrbitESC PRIVATE ${PROJECT_ROOT})
add_custom_command(TARGET OrbitESC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:OrbitESC> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sys_image
)
