/******************************************************************************
 *  File Name:
 *    trace_types.hpp
 *
 *  Description:
 *    Core type definitions for the generalized trace module
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TRACE_TYPES_HPP
#define ORBIT_TRACE_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <etl/span.h>
#include <etl/vector.h>
#include <etl/array.h>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t MAX_TRACE_DATA_SIZE     = 256;    // Maximum serialized data size
  static constexpr size_t MAX_TRACE_REGISTRY_SIZE = 32;     // Maximum registered trace types
  static constexpr size_t MAX_TRACE_CALLBACK_ARGS = 8;      // Maximum callback arguments

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   * @brief Trace data type identifiers
   *
   * Each enum value maps to a specific data structure that can be traced.
   * Add new trace types here as needed.
   */
  enum class TraceType : uint8_t
  {
    INVALID = 0,

    // Motor control traces
    MOTOR_CURRENT_MEASUREMENTS,
    MOTOR_VOLTAGE_COMMANDS,
    MOTOR_SPEED_ESTIMATE,
    MOTOR_POSITION_ESTIMATE,

    // Control system traces
    CONTROL_ERROR_SIGNALS,
    CONTROL_OUTPUT_SIGNALS,
    CONTROL_REFERENCE_SIGNALS,

    // System monitoring traces
    SYSTEM_TEMPERATURE,
    SYSTEM_VOLTAGE,
    SYSTEM_CURRENT,
    SYSTEM_STATUS,

    // Custom user traces
    USER_CUSTOM_1,
    USER_CUSTOM_2,
    USER_CUSTOM_3,

    // Sentinel
    COUNT
  };

  /**
   * @brief Trace serialization format
   */
  enum class SerializationFormat : uint8_t
  {
    INVALID = 0,
    BINARY_PACKED,    // Raw binary serialization (simulator)
    PROTOBUF_COBS,    // Protocol buffer + COBS encoding (embedded)
    COUNT
  };

  /*---------------------------------------------------------------------------
  Data Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Generic trace data container
   *
   * This structure holds the raw serialized data and metadata for any trace type.
   * The actual data format depends on the serialization method used.
   */
  struct TraceData
  {
    TraceType                type;               // Type of trace data
    uint32_t                 timestamp_us;       // Timestamp in microseconds
    SerializationFormat      format;             // Serialization format used
    etl::span<const uint8_t> data;               // Serialized data payload
    uint16_t                 sequence_number;    // Optional sequence number

    TraceData() : type( TraceType::INVALID ), timestamp_us( 0 ), format( SerializationFormat::INVALID ), sequence_number( 0 )
    {
    }
  };

  /**
   * @brief Trace callback function signature
   *
   * @param type Trace type being sent
   * @param data Pointer to serialized data
   * @param size Size of serialized data in bytes
   * @param timestamp_us Timestamp when trace was generated
   * @return true if trace was successfully sent, false otherwise
   */
  using TraceCallback = bool ( * )( TraceType type, const void *data, size_t size, uint32_t timestamp_us );

  /**
   * @brief Trace registration entry
   */
  struct TraceRegistration
  {
    TraceType     type;                   // Trace type identifier
    TraceCallback callback;               // Callback function to invoke
    bool          enabled;                // Whether this trace is enabled
    uint32_t      sample_rate_us;         // Sample rate in microseconds (0 = every call)
    uint32_t      last_sample_time_us;    // Last time this trace was sampled

    TraceRegistration() :
        type( TraceType::INVALID ), callback( nullptr ), enabled( false ), sample_rate_us( 0 ), last_sample_time_us( 0 )
    {
    }
  };

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using TraceRegistry = etl::vector<TraceRegistration, MAX_TRACE_REGISTRY_SIZE>;
  using TraceBuffer   = etl::array<uint8_t, MAX_TRACE_DATA_SIZE>;

}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_TYPES_HPP */
