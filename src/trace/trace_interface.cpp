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
    LOG_INFO( "Trace interface initialized" );

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
    return sendTrace( type, data, size, Chimera::micros() );
  }

  bool Interface::sendTrace( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
    if( !m_initialized || !data || !size )
    {
      return false;
    }

    // Serialize the data
    uint8_t serialized_data[ MAX_TRACE_DATA_SIZE ];
    size_t  serialized_size =
        m_serializer.serialize( type, data, size, timestamp_us, serialized_data, sizeof( serialized_data ) );

    if( serialized_size == 0 )
    {
      LOG_ERROR( "Failed to serialize trace data for type %d", static_cast<int>( type ) );
      return false;
    }

    // Execute registered callbacks
    return m_registry.executeCallbacks( type, serialized_data, serialized_size, timestamp_us );
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
  bool registerTraceCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us )
  {
    return getTraceInterface().registerCallback( type, callback, sample_rate_us );
  }

}    // namespace Orbit::Trace
