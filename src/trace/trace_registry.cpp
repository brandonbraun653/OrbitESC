/******************************************************************************
 *  File Name:
 *    trace_registry.cpp
 *
 *  Description:
 *    Implementation of trace callback registration and management system
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_registry.hpp>
#include <Aurora/logging>
#include <Chimera/thread>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Registry Implementation
  ---------------------------------------------------------------------------*/
  Registry::Registry() : m_lock( false ), m_sequence_counter( 0 )
  {
    m_registry.clear();
  }

  bool Registry::registerCallback( TraceType type, TraceCallback callback, uint32_t sample_rate_us )
  {
    if( !callback || type == TraceType::INVALID || type >= TraceType::COUNT )
    {
      return false;
    }

    acquireLock();

    // Check if already registered
    if( findRegistration( type ) != nullptr )
    {
      releaseLock();
      return false;
    }

    // Add new registration
    TraceRegistration reg;
    reg.type                = type;
    reg.callback            = callback;
    reg.enabled             = true;
    reg.sample_rate_us      = sample_rate_us;
    reg.last_sample_time_us = 0;

    if( m_registry.full() )
    {
      releaseLock();
      return false;
    }

    m_registry.push_back( reg );
    releaseLock();

    LOG_DEBUG( "Registered trace callback for type %d with sample rate %u us", static_cast<int>( type ), sample_rate_us );
    return true;
  }

  bool Registry::unregisterCallback( TraceType type )
  {
    if( type == TraceType::INVALID || type >= TraceType::COUNT )
    {
      return false;
    }

    acquireLock();

    auto it = etl::find_if( m_registry.begin(), m_registry.end(),
                            [ type ]( const TraceRegistration &reg ) { return reg.type == type; } );

    if( it != m_registry.end() )
    {
      m_registry.erase( it );
      releaseLock();
      LOG_DEBUG( "Unregistered trace callback for type %d", static_cast<int>( type ) );
      return true;
    }

    releaseLock();
    return false;
  }

  bool Registry::setEnabled( TraceType type, bool enabled )
  {
    acquireLock();

    TraceRegistration *reg = findRegistration( type );
    if( reg != nullptr )
    {
      reg->enabled = enabled;
      releaseLock();
      return true;
    }

    releaseLock();
    return false;
  }

  bool Registry::isEnabled( TraceType type ) const
  {
    acquireLock();

    const TraceRegistration *reg     = findRegistration( type );
    bool                     enabled = ( reg != nullptr ) && reg->enabled;

    releaseLock();
    return enabled;
  }

  bool Registry::executeCallbacks( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
    if( !data || !size || type == TraceType::INVALID || type >= TraceType::COUNT )
    {
      return false;
    }

    acquireLock();

    TraceRegistration *reg = findRegistration( type );
    if( reg == nullptr || !reg->enabled )
    {
      releaseLock();
      return false;
    }

    // Check rate limiting
    if( !shouldSample( *reg, timestamp_us ) )
    {
      releaseLock();
      return true;    // Rate limited, but not an error
    }

    // Update last sample time
    reg->last_sample_time_us = timestamp_us;

    // Execute callback
    bool success = reg->callback( type, data, size, timestamp_us );

    releaseLock();
    return success;
  }

  size_t Registry::getRegistrationCount() const
  {
    acquireLock();
    size_t count = m_registry.size();
    releaseLock();
    return count;
  }

  void Registry::clear()
  {
    acquireLock();
    m_registry.clear();
    m_sequence_counter = 0;
    releaseLock();
    LOG_DEBUG( "Cleared all trace registrations" );
  }

  TraceRegistration *Registry::findRegistration( TraceType type )
  {
    auto it = etl::find_if( m_registry.begin(), m_registry.end(),
                            [ type ]( const TraceRegistration &reg ) { return reg.type == type; } );
    return ( it != m_registry.end() ) ? &( *it ) : nullptr;
  }

  const TraceRegistration *Registry::findRegistration( TraceType type ) const
  {
    auto it = etl::find_if( m_registry.begin(), m_registry.end(),
                            [ type ]( const TraceRegistration &reg ) { return reg.type == type; } );
    return ( it != m_registry.end() ) ? &( *it ) : nullptr;
  }

  bool Registry::shouldSample( const TraceRegistration &registration, uint32_t current_time_us ) const
  {
    // No rate limiting if sample_rate_us is 0
    if( registration.sample_rate_us == 0 )
    {
      return true;
    }

    // Check if enough time has passed
    uint32_t time_diff = current_time_us - registration.last_sample_time_us;
    return time_diff >= registration.sample_rate_us;
  }

  void Registry::acquireLock() const
  {
    // Simple spinlock implementation
    while( m_lock.exchange( true, etl::memory_order_acquire ) )
    {
      Chimera::delayMicroseconds( 1 );
    }
  }

  void Registry::releaseLock() const
  {
    m_lock.store( false, etl::memory_order_release );
  }

}    // namespace Orbit::Trace
