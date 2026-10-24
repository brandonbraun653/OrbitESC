include(CTTesting)

gen_gtest_executable(
  TARGET
    foc_math_tests
  SOURCES
    test_main.cpp
    aurora_stubs.cpp
    control/test_foc_math.cpp
    "${CMAKE_SOURCE_DIR}/src/control/foc_math.cpp"
  LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    chimera_core
    project_intf_inc
    prj_device_target
    prj_build_target
)

