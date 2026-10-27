include_guard(GLOBAL)

option(ORBITCOMMAND_BUILD_TESTS "Build the OrbitCommand test suites" ON)
option(ORBITCOMMAND_ENABLE_SYSTEM_TESTS "Register the real-simulator system tests" ON)
option(ORBITCOMMAND_ENABLE_E2E "Register the Qat GUI E2E test" ON)
option(ORBITCOMMAND_COVERAGE "Instrument OrbitCommand libraries and tests for coverage" OFF)
set(ORBITCOMMAND_SANITIZERS "" CACHE STRING "Comma-separated sanitizers, for example address,undefined or thread")
set(ORBITCOMMAND_GTEST_SOURCE_DIR "" CACHE PATH "Optional local GoogleTest source tree for offline builds")

# ---------------------------------------------------------------------------
# Quality build flags
# ---------------------------------------------------------------------------
if(ORBITCOMMAND_SANITIZERS)
  string(REPLACE "," ";" ORBITCOMMAND_SANITIZER_LIST "${ORBITCOMMAND_SANITIZERS}")
  string(REPLACE ";" "," ORBITCOMMAND_SANITIZER_FLAGS "${ORBITCOMMAND_SANITIZER_LIST}")
  add_compile_options(-fsanitize=${ORBITCOMMAND_SANITIZER_FLAGS} -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${ORBITCOMMAND_SANITIZER_FLAGS})
endif()

if(ORBITCOMMAND_COVERAGE)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

if(NOT ORBITCOMMAND_BUILD_TESTS)
  return()
endif()

# ---------------------------------------------------------------------------
# GoogleTest / GoogleMock
# ---------------------------------------------------------------------------
find_package(GTest 1.14 QUIET)
if(NOT TARGET GTest::gtest OR NOT TARGET GTest::gmock)
  include(FetchContent)

  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
  set(BUILD_GTEST ON CACHE BOOL "" FORCE)

  if(ORBITCOMMAND_GTEST_SOURCE_DIR)
    set(FETCHCONTENT_SOURCE_DIR_GOOGLETEST "${ORBITCOMMAND_GTEST_SOURCE_DIR}" CACHE PATH "" FORCE)
  endif()

  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(googletest)
endif()

include(GoogleTest)

# ---------------------------------------------------------------------------
# Test support library
# ---------------------------------------------------------------------------
add_library(orbit_test_support STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/support/TestMain.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/support/MessageFactory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/support/FakeTransport.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/support/FakeDeviceServer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/support/SimulatorProcess.cpp
)
target_include_directories(orbit_test_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/tests")
target_compile_definitions(orbit_test_support PUBLIC ORBIT_REPO_ROOT_PATH="${ORBIT_REPO_ROOT}")
target_link_libraries(orbit_test_support PUBLIC
  orbit_client
  GTest::gtest
  GTest::gmock
  Qt6::Core
  Qt6::Network
  Qt6::SerialPort
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
function(orbit_add_gtest target)
  cmake_parse_arguments(ARG "" "LABELS;TIMEOUT;RESOURCE_LOCK" "" ${ARGN})
  if(NOT ARG_LABELS)
    set(ARG_LABELS "unit")
  endif()

  add_executable(${target} ${ARG_UNPARSED_ARGUMENTS})
  target_link_libraries(${target} PRIVATE orbit_test_support)

  set(_properties LABELS "${ARG_LABELS}")
  if(ARG_TIMEOUT)
    list(APPEND _properties TIMEOUT "${ARG_TIMEOUT}")
  endif()
  if(ARG_RESOURCE_LOCK)
    list(APPEND _properties RESOURCE_LOCK "${ARG_RESOURCE_LOCK}")
  endif()

  gtest_discover_tests(${target}
    DISCOVERY_MODE PRE_TEST
    PROPERTIES ${_properties}
    SKIP_REGULAR_EXPRESSION "\\[  SKIPPED \\]"
  )
endfunction()

function(orbit_add_coverage_target)
  find_program(GCOVR_EXECUTABLE gcovr)
  if(NOT GCOVR_EXECUTABLE)
    message(STATUS "gcovr not found; the coverage target is unavailable")
    return()
  endif()

  add_custom_target(coverage
    COMMAND ${CMAKE_CTEST_COMMAND} --test-dir "${CMAKE_BINARY_DIR}" -L unit --output-on-failure
    COMMAND ${GCOVR_EXECUTABLE}
      --root "${ORBIT_REPO_ROOT}"
      --object-directory "${CMAKE_BINARY_DIR}"
      --exclude "${ORBIT_REPO_ROOT}/lib/"
      --exclude ".*/proto/.*\\.pb\\.c"
      --exclude ".*/tests/"
      --exclude ".*/build/"
      --html-details "${CMAKE_BINARY_DIR}/coverage/index.html"
      --xml-pretty --output "${CMAKE_BINARY_DIR}/coverage/coverage.xml"
    COMMENT "Running unit tests and generating the coverage report"
    USES_TERMINAL
  )
endfunction()
