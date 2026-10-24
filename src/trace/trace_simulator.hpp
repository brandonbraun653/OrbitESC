/******************************************************************************
 *  File Name:
 *    trace_simulator.hpp
 *
 *  Description:
 *    Simulator trace implementation that forwards raw packed structures
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TRACE_SIMULATOR_HPP
#define ORBIT_TRACE_SIMULATOR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_types.hpp>
#include <cstddef>
#include <cstdint>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Simulator Trace Serializer
  ---------------------------------------------------------------------------*/
  /**
   * @brief Pass-through serialization for simulator traces
   *
   * Matlab expects raw packed trace structures with no additional metadata.
   * The simulator serializer therefore forwards buffers unchanged while
   * retaining a consistent interface with other serializer implementations.
   */
  class SimulatorSerializer
  {
  public:
    SimulatorSerializer();
    ~SimulatorSerializer() = default;

    /**
     * @brief Serialize trace data using raw binary pass-through
     *
     * @param type Trace type identifier (ignored)
     * @param data Pointer to data to serialize
     * @param size Size of data in bytes
     * @param timestamp_us Timestamp in microseconds (ignored)
     * @param output Buffer to write serialized data to
     * @param output_size Size of output buffer
     * @return Number of bytes written to output buffer, 0 on error
     */
    size_t serialize( TraceType type, const void *data, size_t size, uint32_t timestamp_us, uint8_t *output,
                      size_t output_size );

    /**
     * @brief Deserialize raw trace data
     *
     * @param input Pointer to serialized data
     * @param input_size Size of input data in bytes
     * @param trace_data Output structure to populate
     * @return true if deserialization successful, false otherwise
     */
    bool deserialize( const uint8_t *input, size_t input_size, TraceData &trace_data );

    /**
     * @brief Get the serialized size for a given data size
     *
     * @param data_size Size of input data
     * @return Serialized size (equal to data_size)
     */
    size_t getSerializedSize( size_t data_size ) const;
  };

}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_SIMULATOR_HPP */
