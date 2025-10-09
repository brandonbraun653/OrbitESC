/******************************************************************************
 *  File Name:
 *    trace_registry.hpp
 *
 *  Description:
 *    Trace callback registration and management system
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TRACE_REGISTRY_HPP
#define ORBIT_TRACE_REGISTRY_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_types.hpp>
#include <etl/algorithm.h>
#include <etl/atomic.h>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Registry Class
  ---------------------------------------------------------------------------*/
  /**
   * @brief Manages trace callback registrations and execution
   *
   * This class provides a thread-safe registry for trace callbacks using ETL
   * containers for embedded compatibility. It supports:
   * - Callback registration/unregistration
   * - Rate limiting for trace messages
   * - Thread-safe operations
   * - Minimal memory footprint
   */
  class Registry
  {
  public:
    Registry();
    ~Registry() = default;

    /**
     * @brief Register a callback for a specific trace type
     *
     * @param type Trace type to register for
     * @param callback Function to call when trace is sent
     * @param sample_rate_us Sample rate in microseconds (0 = every call)
     * @return true if registration successful, false if type already registered or registry full
     */
    bool registerCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us = 0 );

    /**
     * @brief Unregister a callback for a specific trace type
     *
     * @param type Trace type to unregister
     * @return true if unregistration successful, false if type not found
     */
    bool unregisterCallback( TraceType type );

    /**
     * @brief Enable or disable a specific trace type
     *
     * @param type Trace type to enable/disable
     * @param enabled true to enable, false to disable
     * @return true if operation successful, false if type not found
     */
    bool setEnabled( TraceType type, bool enabled );

    /**
     * @brief Check if a trace type is enabled
     *
     * @param type Trace type to check
     * @return true if enabled, false if disabled or not found
     */
    bool isEnabled( TraceType type ) const;

    /**
     * @brief Execute callbacks for a trace type
     *
     * @param type Trace type to send
     * @param data Pointer to serialized data
     * @param size Size of serialized data
     * @param timestamp_us Timestamp when trace was generated
     * @return true if at least one callback executed successfully
     */
    bool executeCallbacks( TraceType type, const void *data, size_t size, uint32_t timestamp_us );

    /**
     * @brief Get the number of registered callbacks
     *
     * @return Number of registered callbacks
     */
    size_t getRegistrationCount() const;

    /**
     * @brief Clear all registrations
     */
    void clear();

  private:
    mutable etl::atomic<bool> m_lock;                // Simple spinlock for thread safety
    TraceRegistry             m_registry;            // Registered callbacks
    uint32_t                  m_sequence_counter;    // Global sequence counter

    /**
     * @brief Find registration entry for a trace type
     *
     * @param type Trace type to find
     * @return Pointer to registration entry, nullptr if not found
     */
    TraceRegistration *findRegistration( TraceType type );

    /**
     * @brief Find registration entry for a trace type (const version)
     *
     * @param type Trace type to find
     * @return Pointer to registration entry, nullptr if not found
     */
    const TraceRegistration *findRegistration( TraceType type ) const;

    /**
     * @brief Check if enough time has passed for rate limiting
     *
     * @param registration Registration entry to check
     * @param current_time_us Current time in microseconds
     * @return true if enough time has passed, false otherwise
     */
    bool shouldSample( const TraceRegistration &registration, uint32_t current_time_us ) const;

    /**
     * @brief Acquire the registry lock
     */
    void acquireLock() const;

    /**
     * @brief Release the registry lock
     */
    void releaseLock() const;
  };

}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_REGISTRY_HPP */
