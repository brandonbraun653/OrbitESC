cmake_minimum_required(VERSION 3.25.0)
project(OrbitESC)
enable_language(C CXX ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# ====================================================
# Perform very high level configuration
# ====================================================
# Set some vars common to both development environments
include("lib/CommonTools/cmake/bootstrap.cmake")
include(CTEmbedded)

# Select a global target flag
if(Toolchain::HAS_GCC_ARM_NONE_EABI)
  set(EMBEDDED ON)
  add_compile_definitions(EMBEDDED)
  include("lib/CommonTools/cmake/device/stm32f4x.cmake")
  include("lib/CommonTools/cmake/toolchains/linker_whole_archive.cmake")
elseif(Toolchain::HAS_GCC)
  set(SIMULATOR ON)
  add_compile_definitions(
    SIMULATOR 
    CHIMERA_SIMULATOR)
  include("lib/CommonTools/cmake/device/host.cmake")
endif()

set(CPP_PLATFORM "freertos")

# ====================================================
# Project Exports
# ====================================================

# FreeRTOS Configuration
if(Toolchain::HAS_GCC_ARM_NONE_EABI)
gen_intf_lib(
  TARGET
    freertos_cfg
  INTF_INCLUDES
    "${CMAKE_SOURCE_DIR}/src/config"
    "${CMAKE_SOURCE_DIR}/src/config/freertos"
    "${CMAKE_SOURCE_DIR}/lib/Aurora/lib/segger/contrib/cortex_m4_freertos"
  INTF_LIBRARIES
    segger_sys_view_intf
    segger_sys_view_cortex_m4_freertos_intf
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Project"
)

gen_intf_lib(
  TARGET
    freertos_port
  INTF_INCLUDES
    "${CMAKE_SOURCE_DIR}/lib/FreeRTOS/portable/GCC/ARM_CM4F"
  INTF_DEFINES
    FREERTOS_CFG_PORT_ARM_CM4F=1
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Project"
)
elseif(Toolchain::HAS_GCC)
gen_intf_lib(
  TARGET
    freertos_cfg
  INTF_INCLUDES
    "${CMAKE_SOURCE_DIR}/src/config"
    "${CMAKE_SOURCE_DIR}/src/config/freertos"
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Project"
)

gen_intf_lib(
  TARGET
    freertos_port
  INTF_INCLUDES
    "${CMAKE_SOURCE_DIR}/lib/FreeRTOS/portable/GCC/Posix"
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Project"
)
endif()

gen_intf_lib(
  TARGET
    freertos_heap
  INTF_DEFINES
    FREERTOS_CFG_MEM_MANG_HEAP4=1
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Project"
)

# Project Headers
gen_intf_lib(
  TARGET
    project_intf_inc
  INTF_DEFINES
    THOR_USER_CONFIG
  INTF_INCLUDES
    "${CMAKE_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/src/config"
    "${CMAKE_SOURCE_DIR}/src/config/fatfs"
    "${CMAKE_SOURCE_DIR}/src/config/flashdb"
    "${CMAKE_SOURCE_DIR}/src/config/freertos"
    "${CMAKE_SOURCE_DIR}/src/config/segger"
    "${CMAKE_SOURCE_DIR}/src/config/thor"
    "${CMAKE_SOURCE_DIR}/src/config/tinyusb"
  INTF_LIBRARIES
    cmsis_intf_inc
    freertos_intf_inc_all
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Project"
)

# ====================================================
# Import common packages and projects
# ====================================================
add_subdirectory("lib/Aurora")
add_subdirectory("lib/Chimera")
add_subdirectory("lib/ChimeraSim")
add_subdirectory("lib/CommonTools/googletest")
add_subdirectory("lib/db")
add_subdirectory("lib/cmsis")
add_subdirectory("lib/FreeRTOS")
add_subdirectory("lib/Thor")
add_subdirectory("lib/usb")

# ====================================================
# Project Imports
# ====================================================
add_subdirectory("src")

# ====================================================
# Test Projects
# ====================================================
enable_testing()
add_subdirectory("test")