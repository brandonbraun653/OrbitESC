include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_intf_lib(
  TARGET
    cmsis_intf_inc
  INTF_INCLUDES
    core/Core/Include
    core/DSP/Include
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/cmsis"
)

gen_static_lib_variants(
  TARGET
    lib_cmsis
  SOURCES
    core/DSP/Source/FilteringFunctions/arm_fir_f32.c
    core/DSP/Source/FilteringFunctions/arm_fir_init_f32.c
    core/DSP/Source/FilteringFunctions/arm_iir_lattice_f32.c
    core/DSP/Source/FilteringFunctions/arm_iir_lattice_init_f32.c
  PRV_LIBRARIES
    cmsis_intf_inc
    project_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/cmsis"
)
