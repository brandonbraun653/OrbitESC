/******************************************************************************
 *  File Name:
 *    utility.cpp
 *
 *  Description:
 *    Utility function implementation
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/assert>
#include <src/core/utility.hpp>
#include <src/control/foc_math.hpp>

namespace Orbit::Utility
{
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
}  // namespace Orbit::Utility
