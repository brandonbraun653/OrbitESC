/******************************************************************************
 *  File Name:
 *    com_app_tx.cpp
 *
 *  Description:
 *    Transport agnostic interface for publishing data to a remote host
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <cstdint>
#include <src/core/com/com_app_tx.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/runtime/serial_runtime.hpp>
#include <src/control/foc_data.hpp>

/*-----------------------------------------------------------------------------
Local Macros
-----------------------------------------------------------------------------*/

/**
 * @brief Convert a frequency in Hz to a period in microseconds
 */
static constexpr uint32_t hz_to_us( const uint32_t x )
{
  return static_cast<uint32_t>( 1000000.0f / static_cast<float>( x ) );
}

namespace Orbit::Com
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void publishPhaseCurrents()
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr uint32_t publish_delta_us = hz_to_us( 250 );

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    static uint32_t last_publish = 0;
    uint32_t        time_us      = Chimera::micros();

    /*-------------------------------------------------------------------------
    Wait until it's time to publish the next message
    -------------------------------------------------------------------------*/
    if ( !Data::SysConfig.streamPhaseCurrents || ( ( time_us - last_publish ) < publish_delta_us ) )
    {
      return;
    }

    last_publish = time_us;

    /*-------------------------------------------------------------------------
    Populate the message
    -------------------------------------------------------------------------*/
    SystemDataMessage_ADCPhaseCurrents msg;
    msg.timestamp = time_us;
    msg.ia        = Control::foc_ireg_state.ima;
    msg.ib        = Control::foc_ireg_state.imb;
    msg.ic        = Control::foc_ireg_state.imc;

    /*-------------------------------------------------------------------------
    Pack the message data and publish it
    -------------------------------------------------------------------------*/
    Serial::Message::SysData sysDataMsg;
    sysDataMsg.reset();
    sysDataMsg.payload.header.msgId = MsgId_MSG_SYS_DATA;
    sysDataMsg.payload.header.subId = 0;
    sysDataMsg.payload.header.uuid  = Serial::Message::getNextUUID();
    sysDataMsg.payload.id           = SystemDataId_ADC_PHASE_CURRENTS;
    sysDataMsg.payload.has_data     = true;
    sysDataMsg.payload.data.size    = sizeof( SystemDataMessage_ADCPhaseCurrents );
    memcpy( &sysDataMsg.payload.data.bytes, &msg, sizeof( SystemDataMessage_ADCPhaseCurrents ) );

    Serial::publishDataMessage( sysDataMsg );
  }


  void publishPhaseVoltages()
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr uint32_t publish_delta_us = hz_to_us( 40 );

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    static uint32_t last_publish = 0;
    const uint32_t  time_us      = Chimera::micros();

    /*-------------------------------------------------------------------------
    Wait until it's time to publish the next message
    -------------------------------------------------------------------------*/
    if ( !Data::SysConfig.streamPwmCommands || ( ( time_us - last_publish ) < publish_delta_us ) )
    {
      return;
    }

    last_publish = time_us;

    /*-------------------------------------------------------------------------
    Populate the message
    -------------------------------------------------------------------------*/
    SystemDataMessage_PWMCommands msg;
    msg.timestamp = time_us;
    msg.va        = Control::foc_ireg_state.vma;
    msg.vb        = Control::foc_ireg_state.vmb;
    msg.vc        = Control::foc_ireg_state.vmc;

    /*-------------------------------------------------------------------------
    Pack the message data and publish it
    -------------------------------------------------------------------------*/
    Serial::Message::SysData sysDataMsg;
    sysDataMsg.reset();
    sysDataMsg.payload.header.msgId = MsgId_MSG_SYS_DATA;
    sysDataMsg.payload.header.subId = 0;
    sysDataMsg.payload.header.uuid  = Serial::Message::getNextUUID();
    sysDataMsg.payload.id           = SystemDataId_PWM_COMMANDS;
    sysDataMsg.payload.has_data     = true;
    sysDataMsg.payload.data.size    = sizeof( msg );
    memcpy( &sysDataMsg.payload.data.bytes, &msg, sizeof( msg ) );

    Serial::publishDataMessage( sysDataMsg );
  }


  void publishStateEstimates()
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr uint32_t publish_delta_us = hz_to_us( 100 );

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    static uint32_t last_publish = 0;
    const uint32_t  time_us      = Chimera::micros();

    /*-------------------------------------------------------------------------
    Wait until it's time to publish the next message
    -------------------------------------------------------------------------*/
    if ( !Data::SysConfig.streamStateEstimates || ( ( time_us - last_publish ) < publish_delta_us ) )
    {
      return;
    }

    last_publish = time_us;

    /*-------------------------------------------------------------------------
    Populate the message
    -------------------------------------------------------------------------*/
    SystemDataMessage_StateEstimates msg;
    msg.timestamp = time_us;
    //msg.omega_est = static_cast<float>( s_state.iLoop.activeSector );
    msg.omega_est = Control::foc_ireg_state.vd;
    //msg.omega_est = s_state.motor.m_phase_now_observer;
    // msg.theta_est = s_state.sObserve.theta_comp;

    /*-------------------------------------------------------------------------
    Pack the message data and publish it
    -------------------------------------------------------------------------*/
    Serial::Message::SysData sysDataMsg;
    sysDataMsg.reset();
    sysDataMsg.payload.header.msgId = MsgId_MSG_SYS_DATA;
    sysDataMsg.payload.header.subId = 0;
    sysDataMsg.payload.header.uuid  = Serial::Message::getNextUUID();
    sysDataMsg.payload.id           = SystemDataId_STATE_ESTIMATES;
    sysDataMsg.payload.has_data     = true;
    sysDataMsg.payload.data.size    = sizeof( msg );
    memcpy( &sysDataMsg.payload.data.bytes, &msg, sizeof( msg ) );

    Serial::publishDataMessage( sysDataMsg );
  }


  void publishSystemADCMeasurements()
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr uint32_t publish_delta_us = hz_to_us( 10 );

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    static uint32_t last_publish = 0;
    const uint32_t  time_us      = Chimera::micros();

    /*-------------------------------------------------------------------------
    Wait until it's time to publish the next message
    -------------------------------------------------------------------------*/
    if ( !Data::SysConfig.streamPwmCommands || ( ( time_us - last_publish ) < publish_delta_us ) )
    {
      return;
    }

    last_publish = time_us;

    /*-------------------------------------------------------------------------
    Populate the message
    -------------------------------------------------------------------------*/
    SystemDataMessage_ADCSystemMeasurments msg;
    msg.timestamp = time_us;

    /*-------------------------------------------------------------------------
    Pack the message data and publish it
    -------------------------------------------------------------------------*/
    Serial::Message::SysData sysDataMsg;
    sysDataMsg.reset();
    sysDataMsg.payload.header.msgId = MsgId_MSG_SYS_DATA;
    sysDataMsg.payload.header.subId = 0;
    sysDataMsg.payload.header.uuid  = Serial::Message::getNextUUID();
    sysDataMsg.payload.id           = SystemDataId_ADC_SYSTEM_VOLTAGES;
    sysDataMsg.payload.has_data     = true;
    sysDataMsg.payload.data.size    = sizeof( msg );
    memcpy( &sysDataMsg.payload.data.bytes, &msg, sizeof( msg ) );

    Serial::publishDataMessage( sysDataMsg );

  }

}    // namespace Orbit::Com
