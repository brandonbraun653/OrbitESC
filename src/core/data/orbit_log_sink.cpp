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

    /*-------------------------------------------------------------------------
    Don't attempt to reopen
    -------------------------------------------------------------------------*/
    if( enabled ) //&& ( mFileDesc > 0 ) )
    {
      return LG::Result::RESULT_SUCCESS;
    }

    auto flags = FS::AccessFlags::O_APPEND | FS::AccessFlags::O_CREAT | FS::AccessFlags::O_RDWR;
    if ( FS::fopen( mFileName.data(), flags, mFileDesc ) == 0 )
    {
      FS::fclose( mFileDesc );
    }

    /*-------------------------------------------------------------------------
    Attempt to open the file, creating one if it doesn't exist yet.
    -------------------------------------------------------------------------*/
    enabled = true;
    return LG::Result::RESULT_SUCCESS;

    // auto result = LG::Result::RESULT_FAIL;
    // auto flags  = FS::AccessFlags::O_APPEND | FS::AccessFlags::O_CREAT | FS::AccessFlags::O_WRONLY;

    // if ( FS::fopen( mFileName.data(), flags, mFileDesc ) == 0 )
    // {
    //   enabled = true;
    //   result  = LG::Result::RESULT_SUCCESS;
    // }

    // return result;
  }


  Aurora::Logging::Result FileLogger::close()
  {
    Chimera::Thread::LockGuard _lck( *this );

    auto result = LG::Result::RESULT_SUCCESS;

    /*-------------------------------------------------------------------------
    Flush the buffer first before closing the file
    -------------------------------------------------------------------------*/
    // if ( enabled && ( mFileDesc >= 0 ) )
    // {
    //   this->flush();
    //   if ( FS::fclose( mFileDesc ) != 0 )
    //   {
    //     result = LG::Result::RESULT_FAIL;
    //   }

    //   mFileDesc = -1;
    //   mBuffer.clear();
    // }

    enabled = false;
    return result;
  }


  Aurora::Logging::Result FileLogger::flush()
  {
    /*-------------------------------------------------------------------------
    Entrancy Checks
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( *this );
    if ( !enabled )//|| ( mFileDesc < 0 ) )
    {
      return LG::Result::RESULT_FAIL_BAD_SINK;
    }

    const size_t cache_size = mBuffer.size();
    if( cache_size == 0 )
    {
      return LG::Result::RESULT_SUCCESS;
    }

    auto flags  = FS::AccessFlags::O_APPEND | FS::AccessFlags::O_EXCL | FS::AccessFlags::O_RDWR;
    if ( FS::fopen( mFileName.data(), flags, mFileDesc ) == 0 )
    {
      /*-------------------------------------------------------------------------
      Copy out the data from the buffer
      -------------------------------------------------------------------------*/

      etl::array<uint8_t, CACHE_SIZE> stack_cache;
      stack_cache.fill( 0 );
      etl::copy( mBuffer.begin(), mBuffer.end(), stack_cache.begin() );

      /*-------------------------------------------------------------------------
      Flush the cache to disk
      -------------------------------------------------------------------------*/
      size_t written = FS::fwrite( stack_cache.data(), 1, cache_size, mFileDesc );
      // FS::fflush( mFileDesc );

      if ( cache_size == written )
      {
        mBuffer.clear();
      }
      else
      {
        while( written > 0)
        {
          written--;
          mBuffer.pop();
        }
      }

      auto err = FS::fclose( mFileDesc );
    }

    return LG::Result::RESULT_SUCCESS;
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
    if( !enabled || ( level < logLevel ) )//|| ( mFileDesc < 0 ) )
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
