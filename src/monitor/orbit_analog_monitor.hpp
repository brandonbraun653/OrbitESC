/******************************************************************************
 *  File Name:
 *    orbit_analog_monitor.hpp
 *
 *  Description:
 *    Analog monitoring interface specification
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ANALOG_MONITOR_HPP
#define ORBIT_ANALOG_MONITOR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>

namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class EngageState : uint8_t
  {
    ACTIVE,
    INACTIVE
  };

  enum class TripState : uint8_t
  {
    NOT_TRIPPED,
    EXCEED_UPPER,
    EXCEED_LOWER
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief A simple interface to abstract an analog monitor solution
   */
  class IAnalogMonitor
  {
  public:
    virtual ~IAnalogMonitor() = default;

    /**
     * @brief Set the engagement state of the monitor
     *
     * @param state   State being set
     */
    virtual void setEngageState( const EngageState state ) = 0;

    /**
     * @brief Set basic min/max thresholds for the monitor before tripping
     *
     * @param min   Lower trip threshold
     * @param max   Upper trip threshold
     */
    virtual void setThresholds( const float min, const float max ) = 0;

    /**
     * @brief Get the current trip state of the monitor
     *
     * @return TripState
     */
    virtual TripState tripped() = 0;

    /**
     * @brief Push a new value into the monitor
     *
     * @param val   Value being monitored
     * @param time  Time the value was acquired in microseconds
     */
    virtual void update( const float val, const size_t time_us ) = 0;
  };
}  // namespace Orbit::Monitor

#endif  /* !ORBIT_ANALOG_MONITOR_HPP */
