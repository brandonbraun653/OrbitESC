/******************************************************************************
 *  File Name:
 *    utility.cpp
 *
 *  Description:
 *    Utility function implementation
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/assert>
#include <Chimera/thread>
#include <Chimera/system>
#include <src/core/utility.hpp>
#include <src/control/foc_math.hpp>

namespace Orbit::Utility
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t LOG_BUF_SIZE = 512;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static char                   s_log_buffer[ LOG_BUF_SIZE ];
  static Chimera::Thread::Mutex s_format_lock;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  size_t comCycleCount( const float freqSwitch, const size_t poles, const size_t rpm )
  {
    RT_DBG_ASSERT( Control::Math::is_nearly_equal( freqSwitch, 0.0f ) == false );

    /*-------------------------------------------------------------------------
    Pre-calculate some data
    -------------------------------------------------------------------------*/
    const float  period = 1.0f / freqSwitch;               /**< PWM switching period */
    const size_t cpm    = eComEventsPerRev( poles ) * rpm; /**< Commutation events per minute */
    const float  cps    = static_cast<float>( cpm / 60 );  /**< Commutation events per second */

    /*-------------------------------------------------------------------------
    Total commutation width is distributed through the switching period minus
    one cycle to account for any aliasing issues.
    -------------------------------------------------------------------------*/
    return static_cast<size_t>( freqSwitch / cps ) - 1u;
  }
}    // namespace Orbit::Utility


/**
 * @brief Logs a message from the USB driver to the system logger
 *
 * @param format  The format string
 * @param ...     Variable arguments
 * @return int    Number of bytes written
 */
extern "C" int orbit_esc_printf( const char *format, ... )
{
  using namespace Orbit::Utility;
  using namespace Aurora::Logging;

  if ( Chimera::System::inISR() )
  {
    return 0;
  }

  Chimera::Thread::LockGuard _lock( s_format_lock );

  memset( s_log_buffer, 0, LOG_BUF_SIZE );
  va_list argptr;
  va_start( argptr, format );
  const int write_size = npf_vsnprintf( s_log_buffer, LOG_BUF_SIZE, format, argptr );
  va_end( argptr );

  if ( write_size > 0 )
  {
    getRootSink()->log( Level::LVL_DEBUG, s_log_buffer, static_cast<size_t>( write_size ) );
    return write_size;
  }
  else
  {
    return 0;
  }
}
