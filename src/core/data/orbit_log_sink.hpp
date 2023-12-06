/******************************************************************************
 *  File Name:
 *    orbit_log_sink.hpp
 *
 *  Description:
 *    Logging sink for Orbit
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_LOG_SINK_HPP
#define ORBIT_LOG_SINK_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/filesystem>
#include <Aurora/logging>
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
     * @return void
     */
    void setLogFile( const std::string_view &file );

    /**
     * @brief Sets the backing memory for caching log information
     *
     * @param cache  Memory to use for caching log information
     * @return void
     */
    void setLogCache( etl::icircular_buffer<uint8_t> *const cache );

    /**
     * @brief Get the number of times the cache was too full to accept a log
     *
     * This is a good indication of the buffer size needing to be increased.
     *
     * @return size_t
     */
    size_t getBufferOverruns() const;

  private:
    std::string_view                mFileName;       /**< File being logged against */
    etl::icircular_buffer<uint8_t> *mBuffer;         /**< Cache for buffering frequent writes */
    size_t                          mBufferOverruns; /**< How many overrun events occurred */
  };
}    // namespace Orbit::Log

#endif /* !ORBIT_LOG_SINK_HPP */
