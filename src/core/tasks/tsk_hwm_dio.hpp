/******************************************************************************
 *  File Name:
 *    tsk_hwm_dio.hpp
 *
 *  Description:
 *    Task for handling delayed/slow IO operations
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TSK_HWM_DELAYED_IO_HPP
#define ORBIT_TSK_HWM_DELAYED_IO_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>

namespace Orbit::Tasks::DIO
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t                        PERIOD_MS = 25;
  static constexpr size_t                        STACK     = STACK_BYTES( 4096 );
  static constexpr std::string_view              NAME      = "dio";
  static constexpr Chimera::Thread::TaskPriority PRIORITY  = 5;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Main thread for DIO task
   *
   * @param arg   Unused
   */
  void DIOThread( void *arg );

}  // namespace Orbit::Task

#endif  /* !ORBIT_TSK_HWM_DELAYED_IO_HPP */
