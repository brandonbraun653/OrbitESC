/******************************************************************************
 *  File Name:
 *    trace_simulator.hpp
 *
 *  Description:
 *    Simulator trace implementation using binary packed structures
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
#include <cstring>
#include <cstdint>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Simulator Trace Serializer
  ---------------------------------------------------------------------------*/
  /**
   * @brief Serializes trace data using binary packed structures for simulator
   *
   * This class provides serialization of trace data using simple binary
   * packed structures suitable for Matlab communication.
   */
  class SimulatorSerializer
  {
  public:
    SimulatorSerializer();
    ~SimulatorSerializer() = default;

    /**
     * @brief Serialize trace data using binary packed format
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
     * @brief Deserialize trace data from binary packed format
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
     * @return Serialized size including header
     */
    size_t getSerializedSize( size_t data_size ) const;

  private:
    /*---------------------------------------------------------------------------
    Binary Packed Structure Format
    ---------------------------------------------------------------------------*/
    /**
     * @brief Binary trace header structure
     *
     * This structure is packed to ensure consistent binary layout
     * for Matlab communication.
     */
    struct TraceHeader
    {
      uint8_t  magic[ 4 ];      // Magic bytes: "TRCE"
      uint8_t  type;            // Trace type identifier
      uint8_t  format;          // Serialization format
      uint16_t sequence;        // Sequence number
      uint32_t timestamp_us;    // Timestamp in microseconds
      uint32_t data_size;       // Size of payload data
      uint32_t checksum;        // Simple checksum of header + data
    } __attribute__( ( packed ) );

    static constexpr uint8_t MAGIC_BYTES[ 4 ] = { 'T', 'R', 'C', 'E' };
    static constexpr size_t  HEADER_SIZE      = sizeof( TraceHeader );

    /**
     * @brief Calculate simple checksum
     *
     * @param data Pointer to data
     * @param size Size of data
     * @return Calculated checksum
     */
    uint32_t calculateChecksum( const uint8_t *data, size_t size ) const;

    /**
     * @brief Verify checksum
     *
     * @param data Pointer to data
     * @param size Size of data
     * @param expected_checksum Expected checksum value
     * @return true if checksum matches, false otherwise
     */
    bool verifyChecksum( const uint8_t *data, size_t size, uint32_t expected_checksum ) const;
  };

}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_SIMULATOR_HPP */
