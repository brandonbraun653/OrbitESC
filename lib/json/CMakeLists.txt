include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_intf_lib(
  TARGET
    json_intf_inc
  INTF_INCLUDES
    "./ArduinoJson"
    "./ArduinoJson/src"
  INTF_LIBRARIES
    # None
  EXPORT_DIR
  "${PROJECT_BINARY_DIR}/OrbitESC/lib/json"
)
