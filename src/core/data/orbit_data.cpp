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
  Public Functions
  ---------------------------------------------------------------------------*/
  bool initialize()
  {
    /*-------------------------------------------------------------------------
    Initialize persistent data storage
    -------------------------------------------------------------------------*/
    //File::init();               // Attach and load the file system
    Persistent::db_init();

    /*-------------------------------------------------------------------------
    Initialize volatile data storage
    -------------------------------------------------------------------------*/
    Param::init();  // Initialize the parameter system
    Param::load();  // Load the parameters from persistent storage

    return true;
  }


  void printSystemInfo()
  {
    LOG_INFO( "OrbitESC -- Boot#: %d, Mode: %s, HW: %d, SW:%s, SN:%s\r\n", SysInfo.bootCount,
              System::modeString( SysInfo.bootMode ), SysIdentity.hwVersion, SysIdentity.swVersion, SysIdentity.serialNumber );
  }

}    // namespace Orbit::Data
