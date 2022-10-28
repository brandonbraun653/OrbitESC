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
    /*-------------------------------------------------------------------------
    Ensure basic file access
    -------------------------------------------------------------------------*/
    auto ser = Chimera::Serial::getDriver( IO::USART::serialChannel );
    RT_DBG_ASSERT( ser );

    if( s_file_sink.close() != LG::Result::RESULT_SUCCESS )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Open the file, then read out chunks of data
    -------------------------------------------------------------------------*/
    auto flags = FS::AccessFlags::O_RDONLY;
    auto file  = static_cast<FS::FileId>( 0 );
    int  err   = 0;

    if( FS::fopen( LogFile.data(), flags, file ) == 0 )
    {
      // Get the file size
      // Allocate a chunk double buffer on the stack, maybe 128 bytes or so.
      // Copy chunks to working buffer, while the serial channel is writing
      // Wait on the serial transfer to complete
      // Swap buffers
      // Transmit the old buffer and start reading out file data to the new buffer
    }
  }

}    // namespace Orbit::Log
