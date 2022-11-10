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
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t           CHUNK_SIZE  = 64;
  static constexpr std::string_view s_start_msg = "***** Dumping File Log *****\r\n";
  static constexpr std::string_view s_error_msg = "Error opening file\r\n";
  static constexpr std::string_view s_end_msg   = "******* End File Log *******\r\n";

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
    if ( s_file_sink.open() == LG::Result::RESULT_SUCCESS )
    {
      return 0;
    }

    return -1;
  }


  int disable()
  {
    if ( s_file_sink.close() == LG::Result::RESULT_SUCCESS )
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
    using namespace Chimera::Event;
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Close the file sink to prevent any conflicting accesses
    -------------------------------------------------------------------------*/
    if ( s_file_sink.close() != LG::Result::RESULT_SUCCESS )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Acquire serial output access
    -------------------------------------------------------------------------*/
    auto ser = Chimera::USART::getDriver( IO::USART::serialChannel );
    RT_DBG_ASSERT( ser );
    Chimera::Thread::LockGuard _serialLock( *ser );

    ser->write( s_start_msg.data(), s_start_msg.size() );
    ser->await( Trigger::TRIGGER_WRITE_COMPLETE, TIMEOUT_BLOCK );

    /*-------------------------------------------------------------------------
    Open the file, then read out chunks of data
    -------------------------------------------------------------------------*/
    uint8_t  buff[ CHUNK_SIZE + 1 ];
    auto     flags     = FS::AccessFlags::O_APPEND | FS::AccessFlags::O_RDWR;
    auto     file      = static_cast<FS::FileId>( 0 );
    int      err       = 0;
    size_t   toRead    = 0;
    size_t   bytesRead = 0;

    if ( FS::fopen( LogFile.data(), flags, file ) == 0 )
    {
      /*-----------------------------------------------------------------------
      Move the file back to the beginning and reset the memory buffers
      -----------------------------------------------------------------------*/
      FS::frewind( file );
      size_t size = FS::fsize( file );
      memset( buff, 0, sizeof( buff ) );

      /*-----------------------------------------------------------------------
      Pull out chunks of data and dump it to the terminal
      -----------------------------------------------------------------------*/
      while ( ( size > 0 ) && ( err == 0 ) )
      {
        /*---------------------------------------------------------------------
        Read in some data from the file
        ---------------------------------------------------------------------*/
        memset( buff, 0, sizeof( buff ) );
        toRead    = ( size >= CHUNK_SIZE ) ? CHUNK_SIZE : size;
        bytesRead = FS::fread( buff, 1, toRead, file );

        if ( bytesRead == 0 )
        {
          err = -1;
          continue;
        }

        /*---------------------------------------------------------------------
        Push the data on the wire
        ---------------------------------------------------------------------*/
        ser->write( buff, bytesRead );
        ser->await( Trigger::TRIGGER_WRITE_COMPLETE, TIMEOUT_BLOCK );
        size -= bytesRead;
      }

      /*-----------------------------------------------------------------------
      Give control back to the logging sink
      -----------------------------------------------------------------------*/
      FS::fclose( file );
    }
    else
    {
      ser->write( s_error_msg.data(), s_error_msg.size() );
      ser->await( Trigger::TRIGGER_WRITE_COMPLETE, TIMEOUT_BLOCK );
    }

    /*-------------------------------------------------------------------------
    Dump the completion message to console
    -------------------------------------------------------------------------*/
    ser->write( s_end_msg.data(), s_end_msg.size() );
    ser->await( Trigger::TRIGGER_WRITE_COMPLETE, TIMEOUT_BLOCK );
    s_file_sink.enabled = true;
  }

}    // namespace Orbit::Log
