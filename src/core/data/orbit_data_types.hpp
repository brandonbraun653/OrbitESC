/******************************************************************************
 *  File Name:
 *    orbit_data_types.hpp
 *
 *  Description:
 *    Type declarations for system data
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SYSTEM_DATA_TYPES_HPP
#define ORBIT_ESC_SYSTEM_DATA_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/datastruct>
#include <Chimera/common>
#include <Chimera/system>
#include <cstdint>
#include <etl/string.h>
#include <src/core/com/serial/serial_interface.pb.h>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_can.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class Identity
  {
  public:
    etl::string<16> boardName;    /**< PARAM_BOARD_NAME */
    etl::string<32> description;  /**< PARAM_DESCRIPTION */
    etl::string<8>  serialNumber; /**< PARAM_SERIAL_NUMBER */
    etl::string<8>  swVersion;    /**< PARAM_SW_VERSION */
    uint8_t         hwVersion;    /**< PARAM_HW_VERSION */
    uint32_t        deviceId;     /**< PARAM_DEVICE_ID */

    void clear();
    void setDefaults();
  };


  class Calibration
  {
  public:
    void clear();
    void setDefaults();
  };

  class Controls
  {
  public:
    void clear();
    void setDefaults();
  };


  class Information
  {
  public:
    uint32_t bootCount; /**< PARAM_BOOT_COUNT */

    void clear();
    void setDefaults();
  };

  class Configuration
  {
  public:
    size_t             diskUpdateRateMs;  /**< PARAM_DISK_UPDATE_RATE_MS */
    float              activityLedScaler; /**< PARAM_ACTIVITY_LED_SCALER */
    Orbit::CAN::NodeId canNodeId;

    void clear();
    void setDefaults();
  };
}    // namespace Orbit::Data

#endif /* !ORBIT_ESC_SYSTEM_DATA_TYPES_HPP */
