/******************************************************************************
 *  File Name:
 *    orbit_log_sink.hpp
 *
 *  Description:
 *    Logging sink for Orbit
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_LOG_SINK_HPP
#define ORBIT_LOG_SINK_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Aurora/filesystem>
#include <etl/circular_buffer.h>

namespace Orbit::Log
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Simple sink for the Aurora logging infrastructure
   *
   * Not too many fancy things going on here. It's built on top of the
   * Aurora filesystem module and will log to the configured file upon
   * a call to "open" and stop logging on "close".
   */
  class FileLogger : public Aurora::Logging::SinkInterface
  {
  public:
    /**
     * How many times the cache had to be flushed directly during a log
     * event because it was too full. Use this as an indication of the
     * buffer size needing to be increased.
     */
    size_t numBufferOverruns;

    FileLogger();
    ~FileLogger();

    Aurora::Logging::Result open() final override;
    Aurora::Logging::Result close() final override;
    Aurora::Logging::Result flush() final override;
    Aurora::Logging::IOType getIOType() final override;
    Aurora::Logging::Result log( const Aurora::Logging::Level level, const void *const message,
                                 const size_t length ) final override;

    /**
     * @brief Assigns the file for logging into
     *
     * @param file  Name of the file to log to
     */
    void setLogFile( const std::string_view &file );

  private:
    static constexpr size_t CACHE_SIZE = 128;

    std::string_view                          mFileName; /**< File being logged against */
    etl::circular_buffer<uint8_t, CACHE_SIZE> mBuffer;   /**< Cache for buffering frequent writes */
  };
}    // namespace Orbit::Log

#endif /* !ORBIT_LOG_SINK_HPP */
