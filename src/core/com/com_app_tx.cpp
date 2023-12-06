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
#include <src/control/foc_data.hpp>
#include <src/control/foc_driver.hpp>
#include <src/core/com/com_app_tx.hpp>
#include <src/core/com/com_scheduler.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/runtime/serial_runtime.hpp>

/*-----------------------------------------------------------------------------
Local Macros
-----------------------------------------------------------------------------*/

/**
 * @brief Convert a frequency in Hz to a period in microseconds
 */
static constexpr uint32_t hz_to_ms( const uint32_t x )
{
  return static_cast<uint32_t>( 1000.0f / static_cast<float>( x ) );
}

namespace Orbit::COM
{
  /*---------------------------------------------------------------------------
  Private Data
  ---------------------------------------------------------------------------*/
  static Scheduler::TaskId             s_stream_ids[ STREAM_ID_NUM_OPTIONS ];
  static Serial::Message::SystemData   s_phase_currents;
  static Serial::Message::SystemData   s_phase_voltages;
  static Serial::Message::SystemData   s_system_voltages;
  static Serial::Message::SystemTick   s_system_tick;
  static Serial::Message::SystemStatus s_system_status;

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Update the phase current measurements periodic data
   *
   * @param task  The task to update
   * @return void
   */
  static void update_phase_currents( Scheduler::Task *task )
  {
    /*-------------------------------------------------------------------------
    Pack the message data
    -------------------------------------------------------------------------*/
    s_phase_currents.raw.header.msgId = MsgId_MSG_SYS_DATA;
    s_phase_currents.raw.header.subId = 0;
    s_phase_currents.raw.header.uuid  = Serial::Message::getNextUUID();
    s_phase_currents.raw.id           = SystemDataId_ADC_PHASE_CURRENTS;
    s_phase_currents.raw.timestamp    = Chimera::micros();
    s_phase_currents.raw.has_payload  = true;

    /*-------------------------------------------------------------------------
    Pack the payload data
    -------------------------------------------------------------------------*/
    Serial::Message::Payload::ADCPhaseCurrents payload;
    payload.raw.ia = Control::foc_ireg_state.ima;
    payload.raw.ib = Control::foc_ireg_state.imb;
    payload.raw.ic = Control::foc_ireg_state.imc;

    Serial::Message::encode( &payload.state, Serial::Message::ENCODE_NO_COBS );
    memcpy( s_phase_currents.raw.payload.bytes, payload.data(), payload.size() );
    s_phase_currents.raw.payload.size = payload.size();

    /*-------------------------------------------------------------------------
    Update the task data
    -------------------------------------------------------------------------*/
    if( Serial::Message::encode( &s_phase_currents.state ) == Chimera::Status::OK )
    {
      task->data = s_phase_currents.data();
      task->size = s_phase_currents.size();
    }
  }


  /**
   * @brief Update the system voltage measurements periodic data
   *
   * @param task  The task to update
   * @return void
   */
  static void update_system_voltages( Scheduler::Task *task )
  {
    /*-------------------------------------------------------------------------
    Pack the message data
    -------------------------------------------------------------------------*/
    s_system_voltages.raw.header.msgId = MsgId_MSG_SYS_DATA;
    s_system_voltages.raw.header.subId = 0;
    s_system_voltages.raw.header.uuid  = Serial::Message::getNextUUID();
    s_system_voltages.raw.id           = SystemDataId_ADC_SYSTEM_VOLTAGES;
    s_system_voltages.raw.timestamp    = Chimera::micros();
    s_system_voltages.raw.has_payload  = true;
    s_system_voltages.raw.payload.size = sizeof( ADCSystemVoltagesPayload );

    /*-------------------------------------------------------------------------
    Pack the payload data
    -------------------------------------------------------------------------*/
    Serial::Message::Payload::ADCSystemVoltages payload;
    payload.raw.v_mcu     = Instrumentation::getMCUVoltage();
    payload.raw.v_dc_link = Instrumentation::getSupplyVoltage();
    payload.raw.v_temp    = Instrumentation::getTemperatureVoltage();
    payload.raw.v_isense  = Instrumentation::getCurrentSenseReferenceVoltage();

    Serial::Message::encode( &payload.state, Serial::Message::ENCODE_NO_COBS );
    memcpy( s_system_voltages.raw.payload.bytes, payload.data(), payload.size() );
    s_system_voltages.raw.payload.size = payload.size();

    /*-------------------------------------------------------------------------
    Update the task data
    -------------------------------------------------------------------------*/
    if( Serial::Message::encode( &s_system_voltages.state ) == Chimera::Status::OK )
    {
      task->data = s_system_voltages.data();
      task->size = s_system_voltages.size();
    }
  }


  /**
   * @brief Update the phase voltage measurements periodic data
   *
   * @param task  The task to update
   * @return void
   */
  static void update_phase_voltages( Scheduler::Task *task )
  {
    /*-------------------------------------------------------------------------
    Pack the message data
    -------------------------------------------------------------------------*/
    s_phase_voltages.raw.header.msgId = MsgId_MSG_SYS_DATA;
    s_phase_voltages.raw.header.subId = 0;
    s_phase_voltages.raw.header.uuid  = Serial::Message::getNextUUID();
    s_phase_voltages.raw.id           = SystemDataId_ADC_PHASE_VOLTAGES;
    s_phase_voltages.raw.timestamp    = Chimera::micros();
    s_phase_voltages.raw.has_payload  = true;

    /*-------------------------------------------------------------------------
    Pack the payload data
    -------------------------------------------------------------------------*/
    Serial::Message::Payload::ADCPhaseVoltages payload;
    payload.raw.va = Control::foc_ireg_state.vma;
    payload.raw.vb = Control::foc_ireg_state.vmb;
    payload.raw.vc = Control::foc_ireg_state.vmc;

    Serial::Message::encode( &payload.state, Serial::Message::ENCODE_NO_COBS );
    memcpy( s_phase_voltages.raw.payload.bytes, payload.data(), payload.size() );
    s_phase_voltages.raw.payload.size = payload.size();

    /*-------------------------------------------------------------------------
    Update the task data
    -------------------------------------------------------------------------*/
    if( Serial::Message::encode( &s_phase_voltages.state ) == Chimera::Status::OK )
    {
      task->data = s_phase_voltages.data();
      task->size = s_phase_voltages.size();
    }
  }


  /**
   * @brief Update the system tick periodic data
   *
   * @param task  The task to update
   * @return void
   */
  static void update_system_tick( Scheduler::Task *task )
  {
    /*-------------------------------------------------------------------------
    Pack the message data
    -------------------------------------------------------------------------*/
    s_system_tick.raw.header.msgId = MsgId_MSG_SYS_TICK;
    s_system_tick.raw.header.subId = 0;
    s_system_tick.raw.header.uuid  = Serial::Message::getNextUUID();
    s_system_tick.raw.tick         = Chimera::millis();

    /*-------------------------------------------------------------------------
    Update the task data
    -------------------------------------------------------------------------*/
    if( Serial::Message::encode( &s_system_tick.state ) == Chimera::Status::OK )
    {
      task->data = s_system_tick.data();
      task->size = s_system_tick.size();
    }
  }


  /**
   * @brief Updates the system status periodic data
   *
   * @param task  The task to update
   * @return void
   */
  static void update_system_status( Scheduler::Task *task )
  {
    /*-------------------------------------------------------------------------
    Pack the message data
    -------------------------------------------------------------------------*/
    s_system_status.raw.header.msgId   = MsgId_MSG_SYS_STATUS;
    s_system_status.raw.header.subId   = 0;
    s_system_status.raw.header.uuid    = Serial::Message::getNextUUID();
    s_system_status.raw.systemTick     = Chimera::millis();
    s_system_status.raw.motorCtrlState = static_cast<MotorCtrlState>( Control::FOC::currentMode() );

    /*-------------------------------------------------------------------------
    Update the task data
    -------------------------------------------------------------------------*/
    if( Serial::Message::encode( &s_system_status.state ) == Chimera::Status::OK )
    {
      task->data = s_system_status.data();
      task->size = s_system_status.size();
    }
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initPeriodicData()
  {
    Scheduler::Task tsk;

    /*-------------------------------------------------------------------------
    Initialize module data
    -------------------------------------------------------------------------*/
    memset( s_stream_ids, Scheduler::INVALID_TASK_ID, sizeof( s_stream_ids ) );

    /*-------------------------------------------------------------------------
    Register the phase current publishing task
    -------------------------------------------------------------------------*/
    tsk.clear();
    tsk.updater  = update_phase_currents;
    tsk.period   = hz_to_ms( 30 );
    tsk.endpoint = Scheduler::Endpoint::USB;
    tsk.priority = Scheduler::Priority::HIGH;
    tsk.ttl      = Scheduler::TTL_INFINITE;
    tsk.data     = s_phase_currents.data();
    tsk.size     = 0;

    s_stream_ids[ STREAM_ID_PHASE_CURRENTS ] = Scheduler::add( tsk );
    RT_DBG_ASSERT( s_stream_ids[ STREAM_ID_PHASE_CURRENTS ] != Scheduler::INVALID_TASK_ID );

    /*-------------------------------------------------------------------------
    Register the phase voltage publishing task
    -------------------------------------------------------------------------*/
    tsk.clear();
    tsk.updater  = update_phase_voltages;
    tsk.period   = hz_to_ms( 30 );
    tsk.endpoint = Scheduler::Endpoint::USB;
    tsk.priority = Scheduler::Priority::HIGH;
    tsk.ttl      = Scheduler::TTL_INFINITE;
    tsk.data     = s_phase_voltages.data();
    tsk.size     = 0;

    s_stream_ids[ STREAM_ID_PHASE_VOLTAGES ] = Scheduler::add( tsk );
    RT_DBG_ASSERT( s_stream_ids[ STREAM_ID_PHASE_VOLTAGES ] != Scheduler::INVALID_TASK_ID );

    /*-------------------------------------------------------------------------
    Register the system voltage publishing task
    -------------------------------------------------------------------------*/
    tsk.clear();
    tsk.updater  = update_system_voltages;
    tsk.period   = hz_to_ms( 5 );
    tsk.endpoint = Scheduler::Endpoint::USB;
    tsk.priority = Scheduler::Priority::LOW;
    tsk.ttl      = Scheduler::TTL_INFINITE;
    tsk.data     = s_system_voltages.data();
    tsk.size     = 0;

    s_stream_ids[ STREAM_ID_SYSTEM_VOLTAGES ] = Scheduler::add( tsk, true );
    RT_DBG_ASSERT( s_stream_ids[ STREAM_ID_SYSTEM_VOLTAGES ] != Scheduler::INVALID_TASK_ID );

    /*-------------------------------------------------------------------------
    Register the system tick publishing task
    -------------------------------------------------------------------------*/
    tsk.clear();
    tsk.updater  = update_system_tick;
    tsk.period   = hz_to_ms( 10 );
    tsk.endpoint = Scheduler::Endpoint::USB;
    tsk.priority = Scheduler::Priority::LOW;
    tsk.ttl      = Scheduler::TTL_INFINITE;
    tsk.data     = s_system_tick.data();
    tsk.size     = 0;

    s_stream_ids[ STREAM_ID_SYSTEM_TICK ] = Scheduler::add( tsk, true );
    RT_DBG_ASSERT( s_stream_ids[ STREAM_ID_SYSTEM_TICK ] != Scheduler::INVALID_TASK_ID );

    /*-------------------------------------------------------------------------
    Register the system status publishing task
    -------------------------------------------------------------------------*/
    tsk.clear();
    tsk.updater  = update_system_status;
    tsk.period   = hz_to_ms( 5 );
    tsk.endpoint = Scheduler::Endpoint::USB;
    tsk.priority = Scheduler::Priority::LOW;
    tsk.ttl      = Scheduler::TTL_INFINITE;
    tsk.data     = s_system_status.data();
    tsk.size     = 0;

    s_stream_ids[ STREAM_ID_SYSTEM_STATUS ] = Scheduler::add( tsk, true );
    RT_DBG_ASSERT( s_stream_ids[ STREAM_ID_SYSTEM_STATUS ] != Scheduler::INVALID_TASK_ID );
  }


  void enableStream( const StreamId id, const bool enable )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( id >= STREAM_ID_NUM_OPTIONS )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Enable/disable the stream
    -------------------------------------------------------------------------*/
    if( enable )
    {
      Scheduler::enable( s_stream_ids[ id ] );
    }
    else
    {
      Scheduler::disable( s_stream_ids[ id ] );
    }
  }

}    // namespace Orbit::COM
