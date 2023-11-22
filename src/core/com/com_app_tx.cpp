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
#include <src/core/com/com_app_tx.hpp>
#include <src/core/com/com_scheduler.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/data/orbit_data.hpp>
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
  static Scheduler::TaskId        s_stream_ids[ STREAM_ID_NUM_OPTIONS ];
  static Serial::Message::SysData s_phase_currents;
  static Serial::Message::SysData s_phase_voltages;
  static Serial::Message::SysData s_system_voltages;

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
    s_phase_currents.payload.header.msgId = MsgId_MSG_SYS_DATA;
    s_phase_currents.payload.header.subId = 0;
    s_phase_currents.payload.header.uuid  = Serial::Message::getNextUUID();
    s_phase_currents.payload.id           = SystemDataId_ADC_PHASE_CURRENTS;
    s_phase_currents.payload.has_data     = true;
    s_phase_currents.payload.data.size    = sizeof( SystemDataMessage_ADCPhaseCurrents );

    auto data = reinterpret_cast<SystemDataMessage_ADCPhaseCurrents *>( s_phase_currents.payload.data.bytes );
    memset( data, 0, sizeof( s_phase_currents.payload.data.bytes ) );

    data->timestamp = Chimera::micros();
    data->ia        = Control::foc_ireg_state.ima;
    data->ib        = Control::foc_ireg_state.imb;
    data->ic        = Control::foc_ireg_state.imc;

    /*-------------------------------------------------------------------------
    Update the task data
    -------------------------------------------------------------------------*/
    if( Serial::Message::encode( &s_phase_currents.state ) == Chimera::Status::OK )
    {
      task->data = s_phase_currents.state.IOBuffer;
      task->size = s_phase_currents.state.EncodedSize;
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
    s_system_voltages.payload.header.msgId = MsgId_MSG_SYS_DATA;
    s_system_voltages.payload.header.subId = 0;
    s_system_voltages.payload.header.uuid  = Serial::Message::getNextUUID();
    s_system_voltages.payload.id           = SystemDataId_ADC_SYSTEM_VOLTAGES;
    s_system_voltages.payload.has_data     = true;
    s_system_voltages.payload.data.size    = sizeof( SystemDataMessage_ADCSystemMeasurements );

    auto data = reinterpret_cast<SystemDataMessage_ADCSystemMeasurements *>( s_system_voltages.payload.data.bytes );
    memset( data, 0, sizeof( s_system_voltages.payload.data.bytes ) );

    data->timestamp = Chimera::micros();
    data->v_mcu     = 0.0f;
    data->v_dc_link = 0.0f;
    data->v_temp    = 0.0f;
    data->v_isense  = 0.0f;

    /*-------------------------------------------------------------------------
    Update the task data
    -------------------------------------------------------------------------*/
    if( Serial::Message::encode( &s_system_voltages.state ) == Chimera::Status::OK )
    {
      task->data = s_system_voltages.state.IOBuffer;
      task->size = s_system_voltages.state.EncodedSize;
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
    s_phase_voltages.payload.header.msgId = MsgId_MSG_SYS_DATA;
    s_phase_voltages.payload.header.subId = 0;
    s_phase_voltages.payload.header.uuid  = Serial::Message::getNextUUID();
    s_phase_voltages.payload.id           = SystemDataId_ADC_PHASE_VOLTAGES;
    s_phase_voltages.payload.has_data     = true;
    s_phase_voltages.payload.data.size    = sizeof( SystemDataMessage_ADCPhaseVoltages );

    auto data = reinterpret_cast<SystemDataMessage_ADCPhaseVoltages *>( s_phase_voltages.payload.data.bytes );
    memset( data, 0, sizeof( s_phase_voltages.payload.data.bytes ) );

    data->timestamp = Chimera::micros();
    data->va        = Control::foc_ireg_state.vma;
    data->vb        = Control::foc_ireg_state.vmb;
    data->vc        = Control::foc_ireg_state.vmc;

    /*-------------------------------------------------------------------------
    Update the task data
    -------------------------------------------------------------------------*/
    if( Serial::Message::encode( &s_phase_voltages.state ) == Chimera::Status::OK )
    {
      task->data = s_phase_voltages.state.IOBuffer;
      task->size = s_phase_voltages.state.EncodedSize;
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
    tsk.data     = s_phase_currents.state.IOBuffer;
    tsk.size     = 0;

    s_stream_ids[ STREAM_ID_PHASE_CURRENTS ] = Scheduler::add( tsk, true );
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
    tsk.data     = s_phase_voltages.state.IOBuffer;
    tsk.size     = 0;

    s_stream_ids[ STREAM_ID_PHASE_VOLTAGES ] = Scheduler::add( tsk, true );
    RT_DBG_ASSERT( s_stream_ids[ STREAM_ID_PHASE_VOLTAGES ] != Scheduler::INVALID_TASK_ID );

    /*-------------------------------------------------------------------------
    Register the system voltage publishing task
    -------------------------------------------------------------------------*/
    tsk.clear();
    tsk.updater  = update_system_voltages;
    tsk.period   = hz_to_ms( 10 );
    tsk.endpoint = Scheduler::Endpoint::USB;
    tsk.priority = Scheduler::Priority::LOW;
    tsk.ttl      = Scheduler::TTL_INFINITE;
    tsk.data     = s_system_voltages.state.IOBuffer;
    tsk.size     = 0;

    s_stream_ids[ STREAM_ID_SYSTEM_VOLTAGES ] = Scheduler::add( tsk, true );
    RT_DBG_ASSERT( s_stream_ids[ STREAM_ID_SYSTEM_VOLTAGES ] != Scheduler::INVALID_TASK_ID );

    Scheduler::enable( s_stream_ids[ STREAM_ID_PHASE_CURRENTS ] );
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
