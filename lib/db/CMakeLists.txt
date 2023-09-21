include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_intf_lib(
  TARGET
    flash_db_intf_inc
  INTF_INCLUDES
    ./flashdb/inc
  INTF_LIBRARIES
    project_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/flasdb"
)

gen_static_lib_variants(
  TARGET
    lib_flash_db
  SOURCES
    flashdb/src/fdb_file.c
    flashdb/src/fdb_kvdb.c
    flashdb/src/fdb_tsdb.c
    flashdb/src/fdb_utils.c
    flashdb/src/fdb.c
  PRV_LIBRARIES
    flash_db_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/flashdb"
)