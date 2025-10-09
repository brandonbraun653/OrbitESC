/******************************************************************************
 *  File Name:
 *    trace_simulator.cpp
 *
 *  Description:
 *    Implementation of simulator trace serialization using binary packed structures
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_simulator.hpp>
#include <Aurora/logging>
#include <cstring>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Simulator Serializer Implementation
  ---------------------------------------------------------------------------*/
  SimulatorSerializer::SimulatorSerializer()
  {
    // Constructor - no initialization needed
  }

  size_t SimulatorSerializer::serialize( TraceType type, const void *data, size_t size, uint32_t timestamp_us, uint8_t *output,
                                         size_t output_size )
  {
    if( !data || !size || !output || !output_size )
    {
      return 0;
    }

    size_t total_size = HEADER_SIZE + size;
    if( total_size > output_size )
    {
      LOG_ERROR( "Output buffer too small: need %zu bytes, have %zu", total_size, output_size );
      return 0;
    }

    // Create header
    TraceHeader header;
    memcpy( header.magic, MAGIC_BYTES, sizeof( MAGIC_BYTES ) );
    header.type         = static_cast<uint8_t>( type );
    header.format       = static_cast<uint8_t>( SerializationFormat::BINARY_PACKED );
    header.sequence     = 0;    // Could be incremented for sequence tracking
    header.timestamp_us = timestamp_us;
    header.data_size    = static_cast<uint32_t>( size );

    // Copy header to output
    memcpy( output, &header, HEADER_SIZE );

    // Copy data to output
    memcpy( output + HEADER_SIZE, data, size );

    // Calculate and store checksum
    uint32_t checksum = calculateChecksum( output, total_size );
    memcpy( output + offsetof( TraceHeader, checksum ), &checksum, sizeof( checksum ) );

    return total_size;
  }

  bool SimulatorSerializer::deserialize( const uint8_t *input, size_t input_size, TraceData &trace_data )
  {
    if( !input || input_size < HEADER_SIZE )
    {
      return false;
    }

    // Extract header
    TraceHeader header;
    memcpy( &header, input, HEADER_SIZE );

    // Verify magic bytes
    if( memcmp( header.magic, MAGIC_BYTES, sizeof( MAGIC_BYTES ) ) != 0 )
    {
      LOG_ERROR( "Invalid magic bytes in trace data" );
      return false;
    }

    // Verify data size
    if( input_size != HEADER_SIZE + header.data_size )
    {
      LOG_ERROR( "Invalid data size: expected %zu, got %zu", HEADER_SIZE + header.data_size, input_size );
      return false;
    }

    // Verify checksum
    if( !verifyChecksum( input, input_size, header.checksum ) )
    {
      LOG_ERROR( "Checksum verification failed" );
      return false;
    }

    // Populate trace data structure
    trace_data.type            = static_cast<TraceType>( header.type );
    trace_data.timestamp_us    = header.timestamp_us;
    trace_data.format          = static_cast<SerializationFormat>( header.format );
    trace_data.data            = etl::span<const uint8_t>( input + HEADER_SIZE, header.data_size );
    trace_data.sequence_number = header.sequence;

    return true;
  }

  size_t SimulatorSerializer::getSerializedSize( size_t data_size ) const
  {
    return HEADER_SIZE + data_size;
  }

  uint32_t SimulatorSerializer::calculateChecksum( const uint8_t *data, size_t size ) const
  {
    uint32_t checksum = 0;

    // Simple checksum calculation (exclude the checksum field itself)
    size_t checksum_offset = offsetof( TraceHeader, checksum );

    for( size_t i = 0; i < size; ++i )
    {
      if( i < checksum_offset || i >= checksum_offset + sizeof( uint32_t ) )
      {
        checksum += data[ i ];
      }
    }

    return checksum;
  }

  bool SimulatorSerializer::verifyChecksum( const uint8_t *data, size_t size, uint32_t expected_checksum ) const
  {
    uint32_t calculated_checksum = calculateChecksum( data, size );
    return calculated_checksum == expected_checksum;
  }

}    // namespace Orbit::Trace
