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
    statorPWMFreq            = 0.0f;
    speedCtrlUpdateFreq      = 0.0f;
    speedCtrlKp              = 0.0f;
    speedCtrlKi              = 0.0f;
    speedCtrlKd              = 0.0f;
    targetIdleRPM            = 0.0f;
    currentCtrl_Q_Kp         = 0.0f;
    currentCtrl_Q_Ki         = 0.0f;
    currentCtrl_Q_Kd         = 0.0f;
    currentCtrl_D_Kp         = 0.0f;
    currentCtrl_D_Ki         = 0.0f;
    currentCtrl_D_Kd         = 0.0f;
    currentObserver_KSlide   = 0.0f;
    currentObserver_MaxError = 0.0f;
    rampCtrlFirstOrderTerm   = 0.0f;
    rampCtrlSecondOrderTerm  = 0.0f;
    rampCtrlRampTimeSec      = 0.0f;
    memset( currentCtrl_Q_FIR, 0, sizeof( currentCtrl_Q_FIR ) );
    memset( currentCtrl_D_FIR, 0, sizeof( currentCtrl_D_FIR ) );
  }

  void Controls::setDefaults()
  {
    statorPWMFreq            = DFLT_STATOR_PWM_FREQ_HZ;
    speedCtrlUpdateFreq      = DFLT_SPEED_CTL_UPDT_FREQ_HZ;
    speedCtrlKp              = DFLT_SPEED_PID_KP;
    speedCtrlKi              = DFLT_SPEED_PID_KI;
    speedCtrlKd              = DFLT_SPEED_PID_KD;
    targetIdleRPM            = DFLT_TARGET_IDLE_RPM;
    currentCtrl_Q_Kp         = DFLT_ICTRL_Q_PID_KP;
    currentCtrl_Q_Ki         = DFLT_ICTRL_Q_PID_KI;
    currentCtrl_Q_Kd         = DFLT_ICTRL_Q_PID_KD;
    currentCtrl_D_Kp         = DFLT_ICTRL_D_PID_KP;
    currentCtrl_D_Ki         = DFLT_ICTRL_D_PID_KI;
    currentCtrl_D_Kd         = DFLT_ICTRL_D_PID_KD;
    currentObserver_KSlide   = DFLT_CURRENT_OBSERVER_KSLIDE;
    currentObserver_MaxError = DFLT_CURRENT_OBSERVER_MAX_ERROR;
    rampCtrlFirstOrderTerm   = DFLT_RAMP_CTRL_FIRST_ORDER_TERM;
    rampCtrlSecondOrderTerm  = DFLT_RAMP_CTRL_SECOND_ORDER_TERM;
    rampCtrlRampTimeSec      = DFLT_RAMP_CTRL_RAMP_TIME_SEC;
    memcpy( currentCtrl_Q_FIR, DFLT_ICTRL_DQ_FIR_FILTER, sizeof( currentCtrl_Q_FIR ) );
    memcpy( currentCtrl_D_FIR, DFLT_ICTRL_DQ_FIR_FILTER, sizeof( currentCtrl_D_FIR ) );
  }

  /*---------------------------------------------------------------------------
  Information Class Implementation
  ---------------------------------------------------------------------------*/
  void Information::clear()
  {
    bootCount = 0;
    bootMode  = System::Mode::NORMAL;
    v_dc_link = 0.0f;
    v_isense  = 0.0f;
    v_mcu     = 0.0f;
    v_temp    = 0.0f;
  }

  void Information::setDefaults()
  {
    clear();
  }

  /*---------------------------------------------------------------------------
  Configuration Class Implementation
  ---------------------------------------------------------------------------*/
  void Configuration::clear()
  {
    /* System Configuration */
    activityLedScaler = 0.0f;
    canNodeId         = Orbit::CAN::NodeId::INVALID;
    diskUpdateRateMs  = 0;

    /* Motor Descriptions */
    rotorPoles       = 0;
    statorSlots      = 0;
    statorResistance = 0.0f;
    statorInductance = 0.0f;

    /* Monitor Thresholds */
    peakCurrentThreshold = 0.0f;
    peakVoltageThreshold = 0.0f;
  }

  void Configuration::setDefaults()
  {
    /* System Configuration */
    activityLedScaler = DFLT_ACTIVITY_LED_SCALER;
    canNodeId         = DFLT_CAN_NODE_ID;
    diskUpdateRateMs  = DFLT_DISK_SYNC_PERIOD_MS;

    /* Motor Descriptions */
    rotorPoles       = DFLT_ROTOR_NUM_POLES;
    statorSlots      = DFLT_STATOR_NUM_SLOTS;
    statorResistance = DFLT_STATOR_RESISTANCE;
    statorInductance = DFLT_STATOR_INDUCTANCE;

    /* Monitor Thresholds */
    peakCurrentThreshold = DFLT_PEAK_PHASE_CURRENT;
    peakVoltageThreshold = DFLT_PEAK_VOLTAGE;
  }
}    // namespace Orbit::Data
