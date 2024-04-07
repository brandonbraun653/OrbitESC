/******************************************************************************
 *  File Name:
 *    com_stream_tasks.hpp
 *
 *  Description:
 *    Periodic tasks to publish data for the COM stream.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_COM_STREAM_TASKS_HPP
#define ORBIT_COM_STREAM_TASKS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Orbit::COM
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum StreamId : uint8_t
  {
    STREAM_ID_PHASE_CURRENTS,
    STREAM_ID_PHASE_VOLTAGES,
    STREAM_ID_SYSTEM_VOLTAGES,
    STREAM_ID_STATE_ESTIMATES,
    STREAM_ID_SYSTEM_TICK,
    STREAM_ID_SYSTEM_STATUS,

    STREAM_ID_NUM_OPTIONS
  };


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Register periodic data to be transmitted
   * @return void
   */
  void initStreamTasks();

  /**
   * @brief Enable/disable a data stream for transmission
   *
   * @param id    Which stream to enable/disable
   * @param enable  True to enable, false to disable
   * @return void
   */
  void enableStream( const StreamId id, const bool enable );

}  // namespace Orbit::COM

#endif  /* !ORBIT_COM_STREAM_TASKS_HPP */
