/******************************************************************************
 *  File Name:
 *    orbit_log_sink.cpp
 *
 *  Description:
 *    Sink driver for the project's on-board NOR filesystem
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/thread>
#include <src/core/data/orbit_log_sink.hpp>
#include <src/core/data/persistent/orbit_filesystem.hpp>

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
  FileLogger::FileLogger() : mFileName( "" ), mBuffer( nullptr ), mBufferOverruns( 0 )
  {
  }


  FileLogger::~FileLogger()
  {
  }


  Aurora::Logging::Result FileLogger::open()
  {
    Chimera::Thread::LockGuard _lck( *this );
    Aurora::FileSystem::FileId file;

    if ( !Data::FileSystem::isMounted() )
    {
      return LG::Result::RESULT_FAIL_BAD_SINK;
    }

    /*-------------------------------------------------------------------------
    Don't attempt to reopen
    -------------------------------------------------------------------------*/
    if ( enabled )
    {
      return LG::Result::RESULT_SUCCESS;
    }

    /*-------------------------------------------------------------------------
    Effectively perform a "touch" operation
    -------------------------------------------------------------------------*/
    auto flags = FS::AccessFlags::O_APPEND | FS::AccessFlags::O_CREAT | FS::AccessFlags::O_RDWR;
    if ( FS::fopen( mFileName.data(), flags, file ) == 0 )
    {
      FS::fclose( file );
      enabled = true;
      return LG::Result::RESULT_SUCCESS;
    }
    else
    {
      enabled = false;
      return LG::Result::RESULT_FAIL_BAD_SINK;
    }
  }


  Aurora::Logging::Result FileLogger::close()
  {
    Chimera::Thread::LockGuard _lck( *this );
    enabled = false;
    return LG::Result::RESULT_SUCCESS;
  }


  Aurora::Logging::Result FileLogger::flush()
  {
    constexpr size_t HEAP_ALIGN = 128;

    Aurora::FileSystem::FileId file;

    /*-------------------------------------------------------------------------
    Entrancy Checks
    -------------------------------------------------------------------------*/
    if ( !Data::FileSystem::isMounted() )
    {
      return LG::Result::RESULT_FAIL_BAD_SINK;
    }

    if ( !enabled )
    {
      return LG::Result::RESULT_FAIL_BAD_SINK;
    }

    if ( mBuffer->size() == 0 )
    {
      return LG::Result::RESULT_SUCCESS;
    }

    /*-------------------------------------------------------------------------
    Flush the cache to disk. Cache the current size of the buffer to allow
    other threads to continue logging while this one is flushing.
    -------------------------------------------------------------------------*/
    const size_t buff_size = mBuffer->size();

    auto flags = FS::AccessFlags::O_APPEND | FS::AccessFlags::O_RDWR;
    if ( FS::fopen( mFileName.data(), flags, file ) == 0 )
    {
      /*-------------------------------------------------------------------------
      Copy out the data from the buffer. Try not to finely fragment the heap by
      using a decently sized buffer.
      -------------------------------------------------------------------------*/
      const size_t heap_size  = HEAP_ALIGN * ( buff_size / HEAP_ALIGN ) + 1u;
      uint8_t     *heap_cache = new uint8_t[ heap_size ];
      RT_HARD_ASSERT( heap_cache );
      memset( heap_cache, 0, heap_size );

      etl::copy( mBuffer->begin(), mBuffer->end(), heap_cache );

      /*-------------------------------------------------------------------------
      Flush the cache to disk
      -------------------------------------------------------------------------*/
      const size_t written = FS::fwrite( heap_cache, 1, buff_size, file );

      size_t cleared = 0;
      while ( cleared < written )
      {
        cleared++;
        mBuffer->pop();
      }

      FS::fclose( file );
      delete[] heap_cache;
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
    if ( !message || !length )
    {
      return LG::Result::RESULT_FAIL_BAD_ARG;
    }

    if ( !Data::FileSystem::isMounted() || !enabled || ( level < logLevel ) )
    {
      return LG::Result::RESULT_IGNORE;
    }

    /*-------------------------------------------------------------------------
    Fill the cache first. If full flush to disk immediately.
    -------------------------------------------------------------------------*/
    size_t byte_idx = 0;
    auto   pData    = reinterpret_cast<const uint8_t *const>( message );

    while ( byte_idx < length )
    {
      if ( !mBuffer->available() )
      {
        mBufferOverruns++;
        return LG::Result::RESULT_NO_MEM;
      }

      mBuffer->push( pData[ byte_idx++ ] );
    }

    return LG::Result::RESULT_SUCCESS;
  }


  void FileLogger::setLogFile( const std::string_view &file )
  {
    mFileName = file;
  }


  void FileLogger::setLogCache( etl::icircular_buffer<uint8_t> *const cache )
  {
    RT_HARD_ASSERT( cache );

    mBuffer = cache;
    mBuffer->fill( 0 );
    mBuffer->clear();
  }


  size_t FileLogger::getBufferOverruns() const
  {
    return mBufferOverruns;
  }

}    // namespace Orbit::Log
