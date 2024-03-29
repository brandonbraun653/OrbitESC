include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    lib_orbit_ctrl
  SOURCES
    estimator/system_observer.cpp
    foc_data.cpp
    foc_driver.cpp
    foc_math.cpp
    hardware/current_control.cpp
    hardware/speed_control.cpp
    math/pid.cpp
    modes/sys_mode_armed.cpp
    modes/sys_mode_base.cpp
    modes/sys_mode_fault.cpp
    modes/sys_mode_idle.cpp
    modes/sys_mode_run.cpp
    subroutines/subroutines.cpp
    subroutines/idle.cpp
    subroutines/optimize_sample_point.cpp
    subroutines/rotor_detector.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    project_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/OrbitESC"
)
