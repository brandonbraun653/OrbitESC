/******************************************************************************
 *  File Name:
 *    aurora_stubs.cpp
 *
 *  Description:
 *    Stub implementations for Aurora library functions used during testing
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 ******************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/source/logging/logging_types.hpp>
#include <cstdarg>


namespace Aurora::Logging
{
  /*---------------------------------------------------------------------------
  Stub Implementations
  ---------------------------------------------------------------------------*/
  Result flog( const Level lvl, const char *const file, const size_t line,
               const char *fmt, ... )
  {
    /* Stub implementation - just return success without doing anything */
    ( void )lvl;
    ( void )file;
    ( void )line;
    ( void )fmt;

    return Result::RESULT_SUCCESS;
  }

}    // namespace Aurora::Logging
