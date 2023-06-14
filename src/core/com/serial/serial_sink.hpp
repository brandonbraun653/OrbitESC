/******************************************************************************
 *  File Name:
 *    serial_sink.hpp
 *
 *  Description:
 *    Serial sink for logging infrastructure that uses COBS & nanopb encoding
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SERIAL_SINK_HPP
#define ORBIT_SERIAL_SINK_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/serial>
#include <cstdlib>

namespace Orbit::Serial
{
  class EncodedLogSink : public Aurora::Logging::SinkInterface
  {
  public:
    EncodedLogSink();
    ~EncodedLogSink();

    /**
     * @brief Assign the serial channel to use
     *
     * @param channel   Which channel to send data on
     */
    void assignChannel( Chimera::Serial::Channel channel );

    Aurora::Logging::Result open() final override;
    Aurora::Logging::Result close() final override;
    Aurora::Logging::Result flush() final override;
    Aurora::Logging::IOType getIOType() final override;
    Aurora::Logging::Result log( const Aurora::Logging::Level level, const void *const message, const size_t length ) final override;

  private:
    Chimera::Serial::Driver_rPtr mSerial;
  };
}    // namespace Orbit::Serial

#endif /* !ORBIT_SERIAL_SINK_HPP */
