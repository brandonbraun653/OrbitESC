/******************************************************************************
 *  File Name:
 *    orbit_data_types.cpp
 *
 *  Description:
 *    Functional details of initializing the data storage layer
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_storage.hpp>
#include <src/core/data/orbit_data_types.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Identity Class Implementation
  ---------------------------------------------------------------------------*/
  void Identity::clear()
  {
    boardName.clear();
    description.clear();
    serialNumber.clear();
    swVersion.clear();
    hwVersion = 0;
    deviceId  = 0;
  }

  void Identity::setDefaults()
  {
    boardName.assign( DFLT_BOARD_NAME.data(), DFLT_BOARD_NAME.size() );
    description.assign( DFLT_DESCRIPTION.data(), DFLT_DESCRIPTION.size() );
    serialNumber.assign( DFLT_SERIAL_NUMBER.data(), DFLT_SERIAL_NUMBER.size() );
    swVersion.assign( DFLT_FIRMWARE_VERSION.data(), DFLT_FIRMWARE_VERSION.size() );
    hwVersion = DFLT_HARDWARE_VERSION;

    /*-----------------------------------------------------------------------
    Update the device ID with the unique ID from the system
    -----------------------------------------------------------------------*/
    deviceId = DFLT_DEVICE_ID;

    Chimera::System::Information *sys_info = nullptr;
    if ( Chimera::System::getSystemInformation( sys_info ); sys_info != nullptr )
    {
      memcpy( &deviceId, sys_info->uniqueId.data(), sizeof( deviceId ) );
    }
  }

  /*---------------------------------------------------------------------------
  Calibration Class Implementation
  ---------------------------------------------------------------------------*/
  void Calibration::clear()
  {
  }

  void Calibration::setDefaults()
  {
  }

  /*---------------------------------------------------------------------------
  Controls Class Implementation
  ---------------------------------------------------------------------------*/
  void Controls::clear()
  {
  }

  void Controls::setDefaults()
  {
  }

  /*---------------------------------------------------------------------------
  Information Class Implementation
  ---------------------------------------------------------------------------*/
  void Information::clear()
  {
    bootCount = 0;
    bootMode  = System::Mode::NORMAL;
  }

  void Information::setDefaults()
  {
    bootCount = 0;
    bootMode = System::Mode::NORMAL;
  }

  /*---------------------------------------------------------------------------
  Configuration Class Implementation
  ---------------------------------------------------------------------------*/
  void Configuration::clear()
  {
    activityLedScaler = 0.0f;
    canNodeId         = Orbit::CAN::NodeId::INVALID;
    diskUpdateRateMs  = 0;
  }

  void Configuration::setDefaults()
  {
    activityLedScaler = DFLT_ACTIVITY_LED_SCALER;
    canNodeId         = DFLT_CAN_NODE_ID;
    diskUpdateRateMs  = DFLT_DISK_SYNC_PERIOD_MS;
  }
}    // namespace Orbit::Data
