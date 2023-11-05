/******************************************************************************
 *  File Name:
 *    orbit_data.cpp
 *
 *  Description:
 *    Supporting software for enabling data storage on OrbitESC
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/datastruct>
#include <Aurora/filesystem>
#include <Aurora/logging>
#include <Aurora/memory>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/system.hpp>
#include <src/core/data/volatile/orbit_parameter.hpp>
#include <src/core/data/persistent/orbit_database.hpp>
#include <src/core/data/persistent/orbit_filesystem.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  Identity      SysIdentity;
  Calibration   SysCalibration;
  Controls      SysControl;
  Information   SysInfo;
  Configuration SysConfig;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Ensure that fixed system identity data is stored in persistent memory
   * @return void
   */
  static void init_system_identity()
  {
    Param::write( ParamId_PARAM_BOARD_NAME, DFLT_BOARD_NAME.cbegin(), DFLT_BOARD_NAME.size() );
    Param::write( ParamId_PARAM_DESCRIPTION, DFLT_DESCRIPTION.cbegin(), DFLT_DESCRIPTION.size() );
    Param::write( ParamId_PARAM_SW_VERSION, DFLT_FIRMWARE_VERSION.cbegin(), DFLT_FIRMWARE_VERSION.size() );
    Param::write( ParamId_PARAM_HW_VERSION, &DFLT_HARDWARE_VERSION, sizeof( DFLT_HARDWARE_VERSION ) );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool initialize()
  {
    /*-------------------------------------------------------------------------
    Initialize persistent data storage
    -------------------------------------------------------------------------*/
    FileSystem::init();     // Attach and load the file system
    Persistent::db_init();  // Initialize the persistent database built on the filesystem

    /*-------------------------------------------------------------------------
    Initialize volatile data storage
    -------------------------------------------------------------------------*/
    Param::init();  // Initialize the parameter system
    Param::load();  // Load the parameters from persistent storage

    /*-------------------------------------------------------------------------
    Initialize various parameters
    -------------------------------------------------------------------------*/
    init_system_identity();

    return true;
  }


  void printSystemInfo()
  {
    LOG_INFO( "OrbitESC -- Boot#: %d, Mode: %s, HW: %d, SW:%s, SN:%s\r\n", SysInfo.bootCount,
              System::modeString( SysInfo.bootMode ), SysIdentity.hwVersion, SysIdentity.swVersion, SysIdentity.serialNumber );
  }

}    // namespace Orbit::Data
