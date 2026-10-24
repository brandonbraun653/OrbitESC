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
      LOG_ERROR( "Invalid data or output buffer for simulator serialization" );
      return 0;
    }

    if( size > output_size )
    {
      LOG_ERROR( "Output buffer too small: need %zu bytes, have %zu", size, output_size );
      return 0;
    }

    memcpy( output, data, size );
    return size;
  }

  bool SimulatorSerializer::deserialize( const uint8_t *input, size_t input_size, TraceData &trace_data )
  {
    // Matlab sends raw packed structures with no metadata. Consumers must
    // already know the trace type they expect, so deserialization simply
    // exposes the provided buffer as-is.
    if( !input || input_size == 0 )
    {
      return false;
    }

    trace_data.type            = TraceType::INVALID;
    trace_data.timestamp_us    = 0;
    trace_data.format          = SerializationFormat::BINARY_PACKED;
    trace_data.data            = etl::span<const uint8_t>( input, input_size );
    trace_data.sequence_number = 0;

    return true;
  }

  size_t SimulatorSerializer::getSerializedSize( size_t data_size ) const
  {
    return data_size;
  }

}    // namespace Orbit::Trace
