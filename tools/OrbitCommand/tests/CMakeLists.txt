orbit_add_gtest(orbit_unit_tests
  unit/test_frame_codec.cpp
  unit/test_message_codec.cpp
  unit/test_parameter_registry.cpp
  unit/test_telemetry.cpp
  unit/test_transport_registry.cpp
  unit/test_tcp_transport.cpp
  unit/test_device_client.cpp
  unit/test_app_controller.cpp
  LABELS "unit"
  TIMEOUT 120
)

if(ORBITCOMMAND_ENABLE_SYSTEM_TESTS)
  orbit_add_gtest(orbit_system_tests
    system/test_simulator_e2e.cpp
    LABELS "system"
    TIMEOUT 300
    RESOURCE_LOCK orbit_e2e
  )
endif()

add_test(NAME qml_smoke COMMAND OrbitCommand --smoke-test)
set_tests_properties(qml_smoke PROPERTIES
  LABELS "smoke"
  ENVIRONMENT "QT_QPA_PLATFORM=offscreen;QT_QUICK_BACKEND=software"
  TIMEOUT 15
)

if(ORBITCOMMAND_ENABLE_E2E)
  add_test(NAME qat_e2e COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/e2e/run_e2e.sh")
  set_tests_properties(qat_e2e PROPERTIES
    LABELS "e2e;gui;system"
    RESOURCE_LOCK orbit_e2e
    TIMEOUT 600
    SKIP_RETURN_CODE 77
    ENVIRONMENT "ORBITCOMMAND_BINARY=$<TARGET_FILE:OrbitCommand>;ORBIT_REPO_ROOT=${ORBIT_REPO_ROOT}"
  )
endif()
