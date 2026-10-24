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
#include <Chimera/common>
#include <Chimera/thread>
#include <cstddef>

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
     * @brief Register a callback for serialized trace traffic.
     *
     * @param type Trace type identifier to bind against.
     * @param callback Function invoked when the trace is dispatched.
     * @param sample_rate_us Minimum microseconds between callback executions (0 = every trace).
     * @return true if registration succeeds.
     */
    bool registerCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us = 0 );

    /**
     * @brief Remove a previously registered callback.
     *
     * @param type Trace type to unhook.
     * @return true if the callback existed and was removed.
     */
    bool unregisterCallback( TraceType type );

    /**
     * @brief Enable or disable callback execution for a trace type.
     *
     * @param type Trace type to modify.
     * @param enabled true to allow callbacks, false to suppress them.
     * @return true if the registration was found and updated.
     */
    bool setEnabled( TraceType type, bool enabled );

    /**
     * @brief Serialize and dispatch trace data using the current timestamp.
     *
     * @param type Trace type identifier.
     * @param data Pointer to trace payload.
     * @param size Size of payload in bytes.
     * @return true if at least one callback handled the trace.
     */
    bool sendTrace( TraceType type, const void *data, size_t size );

    /**
     * @brief Serialize and dispatch trace data with a caller-supplied timestamp.
     *
     * @param type Trace type identifier.
     * @param data Pointer to trace payload.
     * @param size Size of payload in bytes.
     * @param timestamp_us Timestamp in microseconds applied to the trace.
     * @return true if at least one callback handled the trace.
     */
    bool sendTrace( TraceType type, const void *data, size_t size, uint32_t timestamp_us );

  private:
    bool     m_initialized;
    Registry m_registry;

#if defined( SIMULATOR )
    SimulatorSerializer m_serializer;
#else
    EmbeddedSerializer m_serializer;
#endif
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
   * @brief Serialize and dispatch trace data through the global interface.
   *
   * @param type Trace type identifier.
   * @param data Pointer to trace payload.
   * @param size Size of payload in bytes.
   * @return true if at least one callback handled the trace.
   */
  bool sendTrace( TraceType type, const void *data, size_t size );
  bool registerTraceCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us = 0 );
}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_INTERFACE_HPP */
