/******************************************************************************
 *  File Name:
 *    orbit_log_sink.cpp
 *
 *  Description:
 *    Sink driver for the project's on-board NOR filesystem
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/thread>
#include <src/core/data/orbit_log_sink.hpp>

namespace Orbit::Log
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  FileLogger::FileLogger() : numBufferOverruns( 0 ), mFileName( "" ), mFileDesc( -1 ), mBuffer( {} )
  {
  }


  FileLogger::~FileLogger()
  {
  }


  Aurora::Logging::Result FileLogger::open()
  {
    Chimera::Thread::LockGuard _lck( *this );
  }


  Aurora::Logging::Result FileLogger::close()
  {
    Chimera::Thread::LockGuard _lck( *this );
  }


  Aurora::Logging::Result FileLogger::flush()
  {
    Chimera::Thread::LockGuard _lck( *this );
  }


  Aurora::Logging::IOType FileLogger::getIOType()
  {
    return Aurora::Logging::IOType::FILE_SINK;
  }


  Aurora::Logging::Result FileLogger::log( const Aurora::Logging::Level level, const void *const message, const size_t length )
  {
    if( !enabled || ( level < logLevel ) )
    {
      return Aurora::Logging::Result::RESULT_IGNORE;
    }

    RT_DBG_ASSERT( message && length );

    // Increment buffer overruns

    // Policy:
    //  - Cache if possible, for speed
    //  - If cache is full, flush it, then fill the cache with whatever data is left.
    //  - Repeat until cache is partially full
    //  - Protect against reentrancy...does this happen in the main logger framework?
  }


  void FileLogger::setLogFile( const std::string_view &file )
  {
    mFileName = file;
  }

}    // namespace Orbit::Log
