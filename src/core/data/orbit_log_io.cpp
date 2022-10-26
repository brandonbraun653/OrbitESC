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
#include <Aurora/logging>
#include <src/core/data/orbit_log_io.hpp>
#include <src/core/data/orbit_log_sink.hpp>

namespace Orbit::Log
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace ALog = Aurora::Logging;


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static FileLogger            s_file_sink;
  static ALog::SinkHandle_rPtr s_file_sink_hndl;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  int initialize()
  {
    auto result = ALog::Result::RESULT_SUCCESS;

    s_file_sink.setLogFile( LogFile );
    s_file_sink.logLevel = ALog::Level::LVL_WARN;
    s_file_sink.enabled = true;
    s_file_sink.name = "FileLog";

    if( !s_file_sink_hndl )
    {
      s_file_sink_hndl = ALog::SinkHandle_rPtr( &s_file_sink );
      result           = ALog::registerSink( s_file_sink_hndl );
    }

    return ( result == ALog::Result::RESULT_SUCCESS ) ? 0 : -1;
  }


  int enable()
  {
    // Open the file
  }


  int disable()
  {
    // Close the file
  }


  void flushCache()
  {
    // Lock the file?
  }


  void dumpToConsole()
  {
  }

}    // namespace Orbit::Log
