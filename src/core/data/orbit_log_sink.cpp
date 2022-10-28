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
  Aliases
  ---------------------------------------------------------------------------*/
  namespace FS = Aurora::FileSystem;
  namespace LG = Aurora::Logging;

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
    /*-------------------------------------------------------------------------
    Entrancy Checks
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( *this );
    if ( !enabled || ( mFileDesc < 0 ) )
    {
      return LG::Result::RESULT_FAIL_BAD_SINK;
    }

    /*-------------------------------------------------------------------------
    Copy out the data from the buffer
    -------------------------------------------------------------------------*/
    const size_t cache_size = mBuffer.size();

    etl::array<uint8_t, CACHE_SIZE> stack_cache;
    etl::copy( mBuffer.begin(), mBuffer.end(), stack_cache );

    /*-------------------------------------------------------------------------
    Flush the cache to disk
    -------------------------------------------------------------------------*/
    size_t written = FS::fwrite( stack_cache.data(), 1, cache_size, mFileDesc );
    if ( cache_size == written )
    {
      mBuffer.clear();
      return LG::Result::RESULT_SUCCESS;
    }
    else
    {
      while( written > 0)
      {
        written--;
        mBuffer.pop();
      }

      return LG::Result::RESULT_FAIL;
    }
  }


  Aurora::Logging::IOType FileLogger::getIOType()
  {
    return Aurora::Logging::IOType::FILE_SINK;
  }


  Aurora::Logging::Result FileLogger::log( const Aurora::Logging::Level level, const void *const message, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Entrancy Checks
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( *this );
    if( !enabled || ( level < logLevel ) || ( mFileDesc < 0 ) )
    {
      return LG::Result::RESULT_IGNORE;
    }

    RT_DBG_ASSERT( message && length );

    /*-------------------------------------------------------------------------
    Fill the cache first. If full flush to disk immediately.
    -------------------------------------------------------------------------*/
    size_t byte_idx = 0;
    auto   pData    = reinterpret_cast<const uint8_t *const>( message );

    while( byte_idx < length )
    {
      if( !mBuffer.available() )
      {
        numBufferOverruns++;
        auto err = this->flush();

        if( err == LG::Result::RESULT_SUCCESS )
        {
          continue;
        }

        return err;
      }

      mBuffer.push( pData[ byte_idx++ ] );
    }

    return LG::Result::RESULT_SUCCESS;
  }


  void FileLogger::setLogFile( const std::string_view &file )
  {
    mFileName = file;
  }

}    // namespace Orbit::Log
