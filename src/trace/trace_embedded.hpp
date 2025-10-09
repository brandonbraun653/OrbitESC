/******************************************************************************
 *  File Name:
 *    trace_embedded.hpp
 *
 *  Description:
 *    Embedded trace implementation using COBS + nanopb serialization
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TRACE_EMBEDDED_HPP
#define ORBIT_TRACE_EMBEDDED_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_types.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <cobs.h>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Embedded Trace Serializer
  ---------------------------------------------------------------------------*/
  /**
   * @brief Serializes trace data using nanopb + COBS for embedded systems
   *
   * This class provides serialization of trace data using protocol buffers
   * with COBS encoding for reliable transmission over serial links.
   */
  class EmbeddedSerializer
  {
  public:
    EmbeddedSerializer();
    ~EmbeddedSerializer() = default;

    /**
     * @brief Serialize trace data using nanopb + COBS
     *
     * @param type Trace type identifier
     * @param data Pointer to data to serialize
     * @param size Size of data in bytes
     * @param timestamp_us Timestamp in microseconds
     * @param output Buffer to write serialized data to
     * @param output_size Size of output buffer
     * @return Number of bytes written to output buffer, 0 on error
     */
    size_t serialize( TraceType type, const void *data, size_t size, uint32_t timestamp_us, uint8_t *output,
                      size_t output_size );

    /**
     * @brief Deserialize trace data from nanopb + COBS
     *
     * @param input Pointer to serialized data
     * @param input_size Size of input data in bytes
     * @param trace_data Output structure to populate
     * @return true if deserialization successful, false otherwise
     */
    bool deserialize( const uint8_t *input, size_t input_size, TraceData &trace_data );

    /**
     * @brief Get the maximum serialized size for a given data size
     *
     * @param data_size Size of input data
     * @return Maximum serialized size including COBS overhead
     */
    size_t getMaxSerializedSize( size_t data_size ) const;

  private:
    static constexpr size_t PB_BUFFER_SIZE   = 256;
    static constexpr size_t COBS_BUFFER_SIZE = 512;

    uint8_t m_pb_buffer[ PB_BUFFER_SIZE ];
    uint8_t m_cobs_buffer[ COBS_BUFFER_SIZE ];

    /**
     * @brief Create a nanopb message structure for trace data
     *
     * @param type Trace type
     * @param data Pointer to data
     * @param size Size of data
     * @param timestamp_us Timestamp
     * @return Protocol buffer message structure
     */
    SystemDataMessage createPbMessage( TraceType type, const void *data, size_t size, uint32_t timestamp_us );

    /**
     * @brief Convert trace type to system data ID
     *
     * @param type Trace type to convert
     * @return Corresponding SystemDataId
     */
    SystemDataId traceTypeToSystemDataId( TraceType type );

    /**
     * @brief Convert system data ID to trace type
     *
     * @param id SystemDataId to convert
     * @return Corresponding TraceType
     */
    TraceType systemDataIdToTraceType( SystemDataId id );
  };

}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_EMBEDDED_HPP */
