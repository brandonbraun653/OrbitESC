/******************************************************************************
 *  File Name:
 *    trace_interface.hpp
 *
 *  Description:
 *    Unified trace interface providing simple API for sending trace data
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TRACE_INTERFACE_HPP
#define ORBIT_TRACE_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_types.hpp>
#include <src/trace/trace_registry.hpp>
#include <src/trace/trace_embedded.hpp>
#include <src/trace/trace_simulator.hpp>
#include <Chimera/thread>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Trace Interface
  ---------------------------------------------------------------------------*/
  /**
   * @brief Unified trace interface for both embedded and simulator environments
   *
   * This class provides a simple, unified API for sending trace data regardless
   * of the underlying serialization format. It automatically selects the
   * appropriate serializer based on the build configuration.
   */
  class Interface
  {
  public:
    Interface();
    ~Interface() = default;

    /**
     * @brief Initialize the trace interface
     *
     * @return true if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * @brief Shutdown the trace interface
     */
    void shutdown();

    /**
     * @brief Register a callback for a specific trace type
     *
     * @param type Trace type to register for
     * @param callback Function to call when trace is sent
     * @param sample_rate_us Sample rate in microseconds (0 = every call)
     * @return true if registration successful, false otherwise
     */
    bool registerCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us = 0 );

    /**
     * @brief Unregister a callback for a specific trace type
     *
     * @param type Trace type to unregister
     * @return true if unregistration successful, false otherwise
     */
    bool unregisterCallback( TraceType type );

    /**
     * @brief Enable or disable a specific trace type
     *
     * @param type Trace type to enable/disable
     * @param enabled true to enable, false to disable
     * @return true if operation successful, false otherwise
     */
    bool setEnabled( TraceType type, bool enabled );

    /**
     * @brief Send trace data
     *
     * This is the main function for sending trace data. It automatically
     * serializes the data using the appropriate format and calls registered
     * callbacks.
     *
     * @param type Trace type identifier
     * @param data Pointer to data to trace
     * @param size Size of data in bytes
     * @return true if trace was sent successfully, false otherwise
     */
    bool sendTrace( TraceType type, const void *data, size_t size );

    /**
     * @brief Send trace data with custom timestamp
     *
     * @param type Trace type identifier
     * @param data Pointer to data to trace
     * @param size Size of data in bytes
     * @param timestamp_us Custom timestamp in microseconds
     * @return true if trace was sent successfully, false otherwise
     */
    bool sendTrace( TraceType type, const void *data, size_t size, uint32_t timestamp_us );

    /**
     * @brief Get the number of registered callbacks
     *
     * @return Number of registered callbacks
     */
    size_t getRegistrationCount() const;

    /**
     * @brief Check if a trace type is enabled
     *
     * @param type Trace type to check
     * @return true if enabled, false if disabled or not found
     */
    bool isEnabled( TraceType type ) const;

    /**
     * @brief Get the current serialization format
     *
     * @return Current serialization format
     */
    SerializationFormat getSerializationFormat() const;

  private:
    bool     m_initialized;
    Registry m_registry;

#if defined( SIMULATOR )
    SimulatorSerializer m_serializer;
#else
    EmbeddedSerializer m_serializer;
#endif

    /**
     * @brief Get current timestamp in microseconds
     *
     * @return Current timestamp
     */
    uint32_t getCurrentTimestamp() const;

    /**
     * @brief Serialize trace data using the appropriate format
     *
     * @param type Trace type
     * @param data Pointer to data
     * @param size Size of data
     * @param timestamp_us Timestamp
     * @param output Buffer to write serialized data
     * @param output_size Size of output buffer
     * @return Number of bytes written, 0 on error
     */
    size_t serializeData( TraceType type, const void *data, size_t size, uint32_t timestamp_us, uint8_t *output,
                          size_t output_size );
  };

  /*---------------------------------------------------------------------------
  Global Interface
  ---------------------------------------------------------------------------*/
  /**
   * @brief Get the global trace interface instance
   *
   * @return Reference to global trace interface
   */
  Interface &getTraceInterface();

  /*---------------------------------------------------------------------------
  Convenience Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Send trace data using the global interface
   *
   * @param type Trace type identifier
   * @param data Pointer to data to trace
   * @param size Size of data in bytes
   * @return true if trace was sent successfully, false otherwise
   */
  bool sendTrace( TraceType type, const void *data, size_t size );

  /**
   * @brief Register a callback using the global interface
   *
   * @param type Trace type to register for
   * @param callback Function to call when trace is sent
   * @param sample_rate_us Sample rate in microseconds (0 = every call)
   * @return true if registration successful, false otherwise
   */
  bool registerTraceCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us = 0 );

  /**
   * @brief Enable or disable a trace type using the global interface
   *
   * @param type Trace type to enable/disable
   * @param enabled true to enable, false to disable
   * @return true if operation successful, false otherwise
   */
  bool setTraceEnabled( TraceType type, bool enabled );

}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_INTERFACE_HPP */
