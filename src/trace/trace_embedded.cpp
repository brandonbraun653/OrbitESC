/******************************************************************************
 *  File Name:
 *    trace_embedded.cpp
 *
 *  Description:
 *    Implementation of embedded trace serialization using COBS + nanopb
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_embedded.hpp>
#include <Aurora/logging>
#include <cstring>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Embedded Serializer Implementation
  ---------------------------------------------------------------------------*/
  EmbeddedSerializer::EmbeddedSerializer()
  {
    memset( m_pb_buffer, 0, sizeof( m_pb_buffer ) );
    memset( m_cobs_buffer, 0, sizeof( m_cobs_buffer ) );
  }

  size_t EmbeddedSerializer::serialize( TraceType type, const void *data, size_t size, uint32_t timestamp_us, uint8_t *output,
                                        size_t output_size )
  {
    if( !data || !size || !output || !output_size )
    {
      return 0;
    }

    // Create protocol buffer message
    SystemDataMessage pb_msg = createPbMessage( type, data, size, timestamp_us );

    // Encode using nanopb
    pb_ostream_t stream     = pb_ostream_from_buffer( m_pb_buffer, sizeof( m_pb_buffer ) );
    bool         pb_success = pb_encode( &stream, SystemDataMessage_fields, &pb_msg );

    if( !pb_success )
    {
      LOG_ERROR( "Failed to encode trace data with nanopb" );
      return 0;
    }

    // Encode with COBS
    cobs_encode_result cobs_result = cobs_encode( output, output_size, m_pb_buffer, stream.bytes_written );

    if( cobs_result.status != COBS_ENCODE_OK )
    {
      LOG_ERROR( "Failed to encode trace data with COBS: %d", cobs_result.status );
      return 0;
    }

    // Add null terminator for COBS
    if( cobs_result.out_len + 1 <= output_size )
    {
      output[ cobs_result.out_len ] = 0x00;
      return cobs_result.out_len + 1;
    }

    return 0;
  }

  bool EmbeddedSerializer::deserialize( const uint8_t *input, size_t input_size, TraceData &trace_data )
  {
    if( !input || !input_size )
    {
      return false;
    }

    // Decode COBS (remove null terminator if present)
    size_t cobs_input_size = input_size;
    if( input[ input_size - 1 ] == 0x00 )
    {
      cobs_input_size = input_size - 1;
    }

    cobs_decode_result cobs_result = cobs_decode( m_cobs_buffer, sizeof( m_cobs_buffer ), input, cobs_input_size );

    if( cobs_result.status != COBS_DECODE_OK )
    {
      LOG_ERROR( "Failed to decode COBS data: %d", cobs_result.status );
      return false;
    }

    // Decode nanopb
    SystemDataMessage pb_msg;
    pb_istream_t      stream     = pb_istream_from_buffer( m_cobs_buffer, cobs_result.out_len );
    bool              pb_success = pb_decode( &stream, SystemDataMessage_fields, &pb_msg );

    if( !pb_success )
    {
      LOG_ERROR( "Failed to decode nanopb data" );
      return false;
    }

    // Populate trace data structure
    trace_data.type            = systemDataIdToTraceType( pb_msg.id );
    trace_data.timestamp_us    = pb_msg.timestamp;
    trace_data.format          = SerializationFormat::PROTOBUF_COBS;
    trace_data.data            = etl::span<const uint8_t>( pb_msg.payload.bytes, pb_msg.payload.size );
    trace_data.sequence_number = 0;    // Could be extracted from header if needed

    return true;
  }

  size_t EmbeddedSerializer::getMaxSerializedSize( size_t data_size ) const
  {
    // nanopb overhead + COBS overhead + null terminator
    size_t pb_overhead   = 32;                                       // Approximate nanopb overhead
    size_t cobs_overhead = ( data_size + pb_overhead ) / 254 + 1;    // COBS worst case
    return data_size + pb_overhead + cobs_overhead + 1;              // +1 for null terminator
  }

  SystemDataMessage EmbeddedSerializer::createPbMessage( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
    SystemDataMessage msg = SystemDataMessage_init_default;

    // Set header
    msg.header.msgId = MsgId_MSG_SYS_DATA;
    msg.header.subId = SubId_SUB_MSG_NONE;
    msg.header.uuid  = 0;    // Could be incremented for sequence tracking

    // Set trace-specific fields
    msg.id        = traceTypeToSystemDataId( type );
    msg.timestamp = timestamp_us;

    // Copy payload data
    if( size <= sizeof( msg.payload.bytes ) )
    {
      memcpy( msg.payload.bytes, data, size );
      msg.payload.size = size;
    }
    else
    {
      LOG_ERROR( "Trace data size %zu exceeds maximum payload size %zu", size, sizeof( msg.payload.bytes ) );
      msg.payload.size = 0;
    }

    return msg;
  }

  SystemDataId EmbeddedSerializer::traceTypeToSystemDataId( TraceType type )
  {
    switch( type )
    {
      case TraceType::MOTOR_CURRENT_MEASUREMENTS:
        return SystemDataId_ADC_PHASE_CURRENTS;

      case TraceType::MOTOR_VOLTAGE_COMMANDS:
        return SystemDataId_ADC_PHASE_VOLTAGES;

      case TraceType::MOTOR_SPEED_ESTIMATE:
        return SystemDataId_SYSTEM_OBSERVER_MONITOR;

      case TraceType::MOTOR_POSITION_ESTIMATE:
        return SystemDataId_SYSTEM_OBSERVER_MONITOR;

      case TraceType::CONTROL_ERROR_SIGNALS:
        return SystemDataId_CURRENT_CONTROL_MONITOR;

      case TraceType::CONTROL_OUTPUT_SIGNALS:
        return SystemDataId_CURRENT_CONTROL_MONITOR;

      case TraceType::CONTROL_REFERENCE_SIGNALS:
        return SystemDataId_CURRENT_CONTROL_MONITOR;

      case TraceType::SYSTEM_TEMPERATURE:
        return SystemDataId_ADC_SYSTEM_VOLTAGES;

      case TraceType::SYSTEM_VOLTAGE:
        return SystemDataId_ADC_SYSTEM_VOLTAGES;

      case TraceType::SYSTEM_CURRENT:
        return SystemDataId_ADC_BUS_VOLTAGE;

      case TraceType::SYSTEM_STATUS:
        return SystemDataId_ADC_BUS_VOLTAGE;

      case TraceType::USER_CUSTOM_1:
        return SystemDataId_ADC_BUS_VOLTAGE;

      case TraceType::USER_CUSTOM_2:
        return SystemDataId_ADC_BUS_VOLTAGE;

      case TraceType::USER_CUSTOM_3:
        return SystemDataId_ADC_BUS_VOLTAGE;

      default:
        return SystemDataId_SYS_DATA_INVALID;
    }
  }

  TraceType EmbeddedSerializer::systemDataIdToTraceType( SystemDataId id )
  {
    switch( id )
    {
      case SystemDataId_ADC_PHASE_CURRENTS:
        return TraceType::MOTOR_CURRENT_MEASUREMENTS;

      case SystemDataId_ADC_PHASE_VOLTAGES:
        return TraceType::MOTOR_VOLTAGE_COMMANDS;

      case SystemDataId_SYSTEM_OBSERVER_MONITOR:
        return TraceType::MOTOR_SPEED_ESTIMATE;

      case SystemDataId_CURRENT_CONTROL_MONITOR:
        return TraceType::CONTROL_ERROR_SIGNALS;

      case SystemDataId_ADC_SYSTEM_VOLTAGES:
        return TraceType::SYSTEM_TEMPERATURE;

      case SystemDataId_ADC_BUS_VOLTAGE:
        return TraceType::SYSTEM_CURRENT;

      default:
        return TraceType::INVALID;
    }
  }

}    // namespace Orbit::Trace
