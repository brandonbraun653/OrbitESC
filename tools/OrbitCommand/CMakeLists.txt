cmake_minimum_required(VERSION 3.25)
project(OrbitCommand VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(ORBIT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "OrbitESC repository root")
set(ORBIT_PROTO_DIR "${ORBIT_REPO_ROOT}/src/core/com/proto")
set(ORBIT_NANOPB_DIR "${ORBIT_REPO_ROOT}/lib/Aurora/lib/nanopb/nanopb")
set(ORBIT_COBS_DIR "${ORBIT_REPO_ROOT}/lib/Aurora/lib/cobs/cobs")

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2 SerialPort Network)
qt_standard_project_setup(REQUIRES 6.5)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/OrbitTesting.cmake")

add_library(orbit_protocol STATIC
  src/protocol/FrameCodec.cpp
  src/protocol/MessageCodec.cpp
  src/protocol/ParameterCodec.cpp
  src/protocol/ConsoleAssembler.cpp
  ${ORBIT_PROTO_DIR}/motor_control.pb.c
  ${ORBIT_PROTO_DIR}/serial_interface.pb.c
  ${ORBIT_PROTO_DIR}/system_config.pb.c
  ${ORBIT_PROTO_DIR}/system_control.pb.c
  ${ORBIT_PROTO_DIR}/system_data.pb.c
  ${ORBIT_NANOPB_DIR}/pb_common.c
  ${ORBIT_NANOPB_DIR}/pb_decode.c
  ${ORBIT_NANOPB_DIR}/pb_encode.c
  ${ORBIT_COBS_DIR}/cobs.c
)
target_include_directories(orbit_protocol PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${ORBIT_PROTO_DIR}"
  "${ORBIT_NANOPB_DIR}"
  "${ORBIT_COBS_DIR}"
)
target_link_libraries(orbit_protocol PUBLIC Qt6::Core)

add_library(orbit_client STATIC
  src/device/Transport.cpp
  src/device/SerialTransport.cpp
  src/device/SerialTransportProvider.cpp
  src/device/TcpTransport.cpp
  src/device/SimulatorTransportProvider.cpp
  src/device/TransportRegistry.cpp
  src/device/DeviceClient.cpp
  src/model/ParameterRegistry.cpp
  src/model/ParameterModel.cpp
  src/model/TelemetryBuffer.cpp
  src/model/LogModel.cpp
  src/app/AppController.cpp
)
target_include_directories(orbit_client PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(orbit_client PUBLIC orbit_protocol Qt6::Core Qt6::SerialPort Qt6::Network)

set(ORBIT_QML_FILES
  qml/Main.qml
  qml/theme/Theme.qml
  qml/components/NeonButton.qml
  qml/components/NeonCard.qml
  qml/components/StatusPill.qml
  qml/components/NeonSwitch.qml
  qml/components/NavRail.qml
  qml/components/TelemetryHeader.qml
  qml/components/MetricTile.qml
  qml/components/ParameterField.qml
  qml/components/ParameterGroupView.qml
  qml/components/PlotCanvas.qml
  qml/components/ConsolePane.qml
  qml/components/ToastHost.qml
  qml/components/ConfirmDialog.qml
  qml/pages/DashboardPage.qml
  qml/pages/ParametersPage.qml
  qml/pages/LiveDataPage.qml
  qml/pages/ConsolePage.qml
)

foreach(qml_file IN LISTS ORBIT_QML_FILES)
  file(RELATIVE_PATH qml_alias "${CMAKE_CURRENT_SOURCE_DIR}/qml" "${CMAKE_CURRENT_SOURCE_DIR}/${qml_file}")
  set_source_files_properties("${qml_file}" PROPERTIES QT_RESOURCE_ALIAS "${qml_alias}")
endforeach()
set_source_files_properties(qml/theme/Theme.qml PROPERTIES QT_QML_SINGLETON_TYPE TRUE)

qt_add_executable(OrbitCommand src/main.cpp)
qt_add_qml_module(OrbitCommand
  URI OrbitCommand
  VERSION 1.0
  OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/qml/OrbitCommand"
  QML_FILES ${ORBIT_QML_FILES}
)
target_link_libraries(OrbitCommand PRIVATE
  orbit_client
  Qt6::Quick
  Qt6::QuickControls2
  Qt6::SerialPort
  Qt6::Network
)

set(ORBIT_FONT_FILES
  assets/fonts/Orbitron-VariableFont_wght.ttf
  assets/fonts/JetBrainsMono-Regular.ttf
  assets/fonts/JetBrainsMono-Bold.ttf
)
set(ORBIT_FONT_FILES_PRESENT TRUE)
foreach(font_file IN LISTS ORBIT_FONT_FILES)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${font_file}")
    set(ORBIT_FONT_FILES_PRESENT FALSE)
  endif()
endforeach()
if(ORBIT_FONT_FILES_PRESENT)
  qt_add_resources(OrbitCommand "orbitcommand_fonts"
    PREFIX "/"
    FILES ${ORBIT_FONT_FILES}
  )
endif()

enable_testing()

if(ORBITCOMMAND_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(ORBITCOMMAND_COVERAGE)
  orbit_add_coverage_target()
endif()
