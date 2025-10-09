/******************************************************************************
 *  File Name:
 *    trace_interface.cpp
 *
 *  Description:
 *    Implementation of unified trace interface
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_interface.hpp>
#include <Aurora/logging>
#include <Chimera/thread>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Interface Implementation
  ---------------------------------------------------------------------------*/
  Interface::Interface() : m_initialized( false )
  {
  }

  bool Interface::initialize()
  {
    if( m_initialized )
    {
      return true;
    }

    // Initialize registry
    m_registry.clear();

    m_initialized = true;
    LOG_INFO( "Trace interface initialized with %s serialization",
              getSerializationFormat() == SerializationFormat::BINARY_PACKED ? "binary" : "protobuf" );

    return true;
  }

  void Interface::shutdown()
  {
    if( !m_initialized )
    {
      return;
    }

    m_registry.clear();
    m_initialized = false;
    LOG_INFO( "Trace interface shutdown" );
  }

  bool Interface::registerCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us )
  {
    if( !m_initialized )
    {
      return false;
    }

    return m_registry.registerCallback( type, callback, sample_rate_us );
  }

  bool Interface::unregisterCallback( TraceType type )
  {
    if( !m_initialized )
    {
      return false;
    }

    return m_registry.unregisterCallback( type );
  }

  bool Interface::setEnabled( TraceType type, bool enabled )
  {
    if( !m_initialized )
    {
      return false;
    }

    return m_registry.setEnabled( type, enabled );
  }

  bool Interface::sendTrace( TraceType type, const void *data, size_t size )
  {
    return sendTrace( type, data, size, getCurrentTimestamp() );
  }

  bool Interface::sendTrace( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
    if( !m_initialized || !data || !size )
    {
      return false;
    }

    // Serialize the data
    uint8_t serialized_data[ MAX_TRACE_DATA_SIZE ];
    size_t  serialized_size = serializeData( type, data, size, timestamp_us, serialized_data, sizeof( serialized_data ) );

    if( serialized_size == 0 )
    {
      LOG_ERROR( "Failed to serialize trace data for type %d", static_cast<int>( type ) );
      return false;
    }

    // Execute registered callbacks
    bool success = m_registry.executeCallbacks( type, serialized_data, serialized_size, timestamp_us );

    if( !success )
    {
      LOG_WARN( "No callbacks executed for trace type %d", static_cast<int>( type ) );
    }

    return success;
  }

  size_t Interface::getRegistrationCount() const
  {
    return m_registry.getRegistrationCount();
  }

  bool Interface::isEnabled( TraceType type ) const
  {
    return m_registry.isEnabled( type );
  }

  SerializationFormat Interface::getSerializationFormat() const
  {
#if defined( SIMULATOR )
    return SerializationFormat::BINARY_PACKED;
#else
    return SerializationFormat::PROTOBUF_COBS;
#endif
  }

  uint32_t Interface::getCurrentTimestamp() const
  {
    // Use Chimera's time functions for consistent timing
    return static_cast<uint32_t>( Chimera::millis() * 1000 );    // Convert to microseconds
  }

  size_t Interface::serializeData( TraceType type, const void *data, size_t size, uint32_t timestamp_us, uint8_t *output,
                                   size_t output_size )
  {
#if defined( SIMULATOR )
    return m_serializer.serialize( type, data, size, timestamp_us, output, output_size );
#else
    return m_serializer.serialize( type, data, size, timestamp_us, output, output_size );
#endif
  }

  /*---------------------------------------------------------------------------
  Global Interface
  ---------------------------------------------------------------------------*/
  Interface &getTraceInterface()
  {
    static Interface instance;
    return instance;
  }

  /*---------------------------------------------------------------------------
  Convenience Functions
  ---------------------------------------------------------------------------*/
  bool sendTrace( TraceType type, const void *data, size_t size )
  {
    return getTraceInterface().sendTrace( type, data, size );
  }

  bool registerTraceCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us )
  {
    return getTraceInterface().registerCallback( type, callback, sample_rate_us );
  }

  bool setTraceEnabled( TraceType type, bool enabled )
  {
    return getTraceInterface().setEnabled( type, enabled );
  }

}    // namespace Orbit::Trace
