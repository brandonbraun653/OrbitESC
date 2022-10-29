/******************************************************************************
 *  File Name:
 *    orbit_log_io.cpp
 *
 *  Description:
 *    Logging utilities
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/filesystem>
#include <Aurora/logging>
#include <Chimera/serial>
#include <Chimera/usart>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_log_io.hpp>
#include <src/core/data/orbit_log_sink.hpp>

namespace Orbit::Log
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace FS = Aurora::FileSystem;
  namespace LG = Aurora::Logging;


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static FileLogger          s_file_sink;
  static LG::SinkHandle_rPtr s_file_sink_hndl;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  int initialize()
  {
    auto result = LG::Result::RESULT_SUCCESS;

    s_file_sink.setLogFile( LogFile );
    s_file_sink.logLevel = LG::Level::LVL_WARN;
    s_file_sink.enabled  = false;
    s_file_sink.name     = "FileLog";

    if ( !s_file_sink_hndl )
    {
      s_file_sink_hndl = LG::SinkHandle_rPtr( &s_file_sink );
      result           = LG::registerSink( s_file_sink_hndl );
    }

    return ( result == LG::Result::RESULT_SUCCESS ) ? 0 : -1;
  }


  int enable()
  {
    if( s_file_sink.open() == LG::Result::RESULT_SUCCESS )
    {
      return 0;
    }

    return -1;
  }


  int disable()
  {
    if( s_file_sink.close() == LG::Result::RESULT_SUCCESS )
    {
      return 0;
    }

    return -1;
  }


  void flushCache()
  {
    s_file_sink.flush();
  }


  void dumpToConsole()
  {
    static constexpr size_t CHUNK_SIZE = 64;

    /*-------------------------------------------------------------------------
    Ensure basic file access
    -------------------------------------------------------------------------*/
    auto ser = Chimera::USART::getDriver( IO::USART::serialChannel );
    RT_DBG_ASSERT( ser );

    if( s_file_sink.close() != LG::Result::RESULT_SUCCESS )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Open the file, then read out chunks of data
    -------------------------------------------------------------------------*/
    uint8_t dbuff[ 2 ][ CHUNK_SIZE ];
    uint8_t *pRead  = dbuff[ 0 ];
    uint8_t *pWrite = dbuff[ 1 ];


    auto flags = FS::AccessFlags::O_RDONLY;
    auto file  = static_cast<FS::FileId>( 0 );
    int  err   = 0;
    size_t toRead = 0;
    size_t bytesRead = 0;


    if( FS::fopen( LogFile.data(), flags, file ) == 0 )
    {
      FS::frewind( file );
      size_t size = FS::fsize( file );
      memset( dbuff, 0, sizeof( dbuff ) );

      while ( ( size > 0 ) && ( err == 0 ) )
      {
        /*---------------------------------------------------------------------
        Read in some data from the file
        ---------------------------------------------------------------------*/
        toRead    = ( size >= CHUNK_SIZE ) ? CHUNK_SIZE : size;
        bytesRead = FS::fread( pRead, 1, toRead, file );

        if( bytesRead == 0 )
        {
          err = -1;
          continue;
        }

        /*---------------------------------------------------------------------
        Wait on the last serial transaction to complete if it hasn't yet
        ---------------------------------------------------------------------*/
        ser->await( Chimera::Event::Trigger::TRIGGER_WRITE_COMPLETE, Chimera::Thread::TIMEOUT_10MS );

        /*---------------------------------------------------------------------
        Swap the working buffers, then push the data on the wire
        ---------------------------------------------------------------------*/
        uint8_t *tmp = pRead;
        pRead        = pWrite;
        pWrite       = tmp;

        ser->write( pWrite, bytesRead );
        size -= bytesRead;
      }
    }
  }

}    // namespace Orbit::Log
