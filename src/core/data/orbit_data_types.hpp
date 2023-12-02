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
#include <src/control/filter.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/system_types.hpp>

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
    using FIRFilter = Control::Math::FIR<float, 15>;

    /*-------------------------------------------------------------------------
    Motor Control Parameters
    -------------------------------------------------------------------------*/
    float statorPWMFreq;                                  /**< PARAM_STATOR_PWM_FREQ */
    float speedCtrlUpdateFreq;                            /**< PARAM_SPEED_CTRL_FREQ */
    float speedCtrlKp;                                    /**< PARAM_SPEED_CTRL_KP */
    float speedCtrlKi;                                    /**< PARAM_SPEED_CTRL_KI */
    float speedCtrlKd;                                    /**< PARAM_SPEED_CTRL_KD */
    float targetIdleRPM;                                  /**< PARAM_TARGET_IDLE_RPM */
    float currentCtrl_Q_Kp;                               /**< PARAM_CURRENT_CTRL_Q_AXIS_KP */
    float currentCtrl_Q_Ki;                               /**< PARAM_CURRENT_CTRL_Q_AXIS_KI */
    float currentCtrl_Q_Kd;                               /**< PARAM_CURRENT_CTRL_Q_AXIS_KD */
    float currentCtrl_D_Kp;                               /**< PARAM_CURRENT_CTRL_D_AXIS_KP */
    float currentCtrl_D_Ki;                               /**< PARAM_CURRENT_CTRL_D_AXIS_KI */
    float currentCtrl_D_Kd;                               /**< PARAM_CURRENT_CTRL_D_AXIS_KD */
    float currentCtrl_Q_FIR[ FIRFilter::CoefData::SIZE ]; /**< PARAM_CURRENT_CTRL_Q_AXIS_FIR */
    float currentCtrl_D_FIR[ FIRFilter::CoefData::SIZE ]; /**< PARAM_CURRENT_CTRL_D_AXIS_FIR */
    float currentObserver_KSlide;                         /**< PARAM_CURRENT_OBSERVER_KSLIDE */
    float currentObserver_MaxError;                       /**< PARAM_CURRENT_OBSERVER_MAX_ERROR */
    float rampCtrlFirstOrderTerm;                         /**< PARAM_RAMP_CTRL_FIRST_ORDER_TERM */
    float rampCtrlSecondOrderTerm;                        /**< PARAM_RAMP_CTRL_SECOND_ORDER_TERM */
    float rampCtrlRampTimeSec;                            /**< PARAM_RAMP_CTRL_RAMP_TIME_SEC */

    void clear();
    void setDefaults();
  };


  class Information
  {
  public:
    uint32_t     bootCount; /**< PARAM_BOOT_COUNT */

    float v_mcu;      /**< System chip voltage */
    float v_dc_link;  /**< Main power rail for the motor */
    float v_temp;     /**< Temperature sensor voltage */
    float v_isense;   /**< Current sense comparator reference voltage */

    void clear();
    void setDefaults();
  };

  class Configuration
  {
  public:
    /*-------------------------------------------------------------------------
    System Configuration
    -------------------------------------------------------------------------*/
    size_t             diskUpdateRateMs;  /**< PARAM_DISK_UPDATE_RATE_MS */
    float              activityLedScaler; /**< PARAM_ACTIVITY_LED_SCALER */
    Orbit::CAN::NodeId canNodeId;         /**< PARAM_CAN_NODE_ID */

    /*-------------------------------------------------------------------------
    Motor Description
    -------------------------------------------------------------------------*/
    uint8_t rotorPoles;       /**< PARAM_ROTOR_POLES */
    uint8_t statorSlots;      /**< PARAM_STATOR_SLOTS */
    float   statorResistance; /**< PARAM_STATOR_RESISTANCE */
    float   statorInductance; /**< PARAM_STATOR_INDUCTANCE */

    /*-------------------------------------------------------------------------
    Monitor Thresholds
    -------------------------------------------------------------------------*/
    float peakCurrentThreshold; /**< PARAM_PEAK_CURRENT_THRESHOLD */
    float peakVoltageThreshold; /**< PARAM_PEAK_VOLTAGE_THRESHOLD */

    void clear();
    void setDefaults();
  };
}    // namespace Orbit::Data

#endif /* !ORBIT_ESC_SYSTEM_DATA_TYPES_HPP */
