/******************************************************************************
 *  File Name:
 *    orbit_data_params.hpp
 *
 *  Description:
 *    Parameter listings for the whole orbit data system
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_PARAMETERS_HPP
#define ORBIT_DATA_PARAMETERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <array>
#include <algorithm>
#include <cstddef>
#include <src/core/com/serial/serial_interface.pb.h>
#include <src/core/data/orbit_data_validators.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using ParameterList = std::array<ParameterNode, 11>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr const char *FMT_UINT8  = "%u";
  static constexpr const char *FMT_UINT16 = FMT_UINT8;
  static constexpr const char *FMT_UINT32 = "%lu";
  static constexpr const char *FMT_FLOAT  = "%4.9f";
  static constexpr const char *FMT_DOUBLE = "%4.17f";
  static constexpr const char *FMT_STRING = "%s";
  static constexpr const char *FMT_BOOL   = "%d";

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  namespace Internal
  {
    static constexpr ParameterList _compile_time_sort( const ParameterList &list )
    {
      auto result = list;
      std::sort( result.begin(), result.end(),
                 []( const ParameterNode &a, const ParameterNode &b ) -> bool { return a.id < b.id; } );
      return result;
    }

    static constexpr ParameterList _unsorted_parameters = {
      /*-----------------------------------------------------------------------
      Read Only Parameters
      -----------------------------------------------------------------------*/
      ParameterNode{ .id        = ParamId_PARAM_BOOT_COUNT,
                     .type      = ParamType_UINT32,
                     .key       = "pwr_cnt",
                     .address   = &SysInfo.bootCount,
                     .maxSize   = sizeof( SysInfo.bootCount ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_HW_VERSION,
                     .type      = ParamType_UINT8,
                     .key       = "hw_ver",
                     .address   = &SysIdentity.hwVersion,
                     .maxSize   = sizeof( SysIdentity.hwVersion ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_SW_VERSION,
                     .type      = ParamType_STRING,
                     .key       = "sw_ver",
                     .address   = &SysIdentity.swVersion,
                     .maxSize   = SysIdentity.swVersion.MAX_SIZE,
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_DEVICE_ID,
                     .type      = ParamType_UINT32,
                     .key       = "dev_id",
                     .address   = &SysIdentity.deviceId,
                     .maxSize   = sizeof( SysIdentity.deviceId ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_BOARD_NAME,
                     .type      = ParamType_STRING,
                     .key       = "name",
                     .address   = &SysIdentity.boardName,
                     .maxSize   = SysIdentity.boardName.MAX_SIZE,
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_DESCRIPTION,
                     .type      = ParamType_STRING,
                     .key       = "desc",
                     .address   = &SysIdentity.description,
                     .maxSize   = SysIdentity.description.MAX_SIZE,
                     .validator = nullptr },

      /*-----------------------------------------------------------------------
      Read/Write Parameters
      -----------------------------------------------------------------------*/
      ParameterNode{ .id        = ParamId_PARAM_SERIAL_NUMBER,
                     .type      = ParamType_STRING,
                     .key       = "ser_num",
                     .address   = &SysIdentity.serialNumber,
                     .maxSize   = SysIdentity.serialNumber.MAX_SIZE,
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_DISK_UPDATE_RATE_MS,
                     .type      = ParamType_UINT32,
                     .key       = "dsk_updt",
                     .address   = &SysConfig.diskUpdateRateMs,
                     .maxSize   = sizeof( SysConfig.diskUpdateRateMs ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_ACTIVITY_LED_SCALER,
                     .type      = ParamType_FLOAT,
                     .key       = "actv_led_scale",
                     .address   = &SysConfig.activityLedScaler,
                     .maxSize   = sizeof( SysConfig.activityLedScaler ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_BOOT_MODE,
                     .type      = ParamType_UINT8,
                     .key       = "boot_mode",
                     .address   = &SysInfo.bootMode,
                     .maxSize   = sizeof( SysInfo.bootMode ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_CAN_NODE_ID,
                     .type      = ParamType_UINT8,
                     .key       = "can_id",
                     .address   = &SysConfig.canNodeId,
                     .maxSize   = sizeof( SysConfig.canNodeId ),
                     .validator = ValidateParamId_PARAM_CAN_NODE_ID },
      /***** Add new entries above here *****/
    };
  }    // namespace Internal

}    // namespace Orbit::Data

#endif /* !ORBIT_DATA_PARAMETERS_HPP */
