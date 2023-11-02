/******************************************************************************
 *  File Name:
 *    serial_usb.hpp
 *
 *  Description:
 *    Serial driver for the USB CDC interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_COM_SERIAL_USB_HPP
#define ORBIT_COM_SERIAL_USB_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/serial>
#include <etl/circular_buffer.h>


namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using CircularBuffer = etl::icircular_buffer<uint8_t>*;

  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class USBSerial;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Gets the USB serial driver instance
   * @return USBSerial*
   */
  USBSerial *getUSBSerialDriver();

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class USBSerial : public Chimera::Serial::Driver
  {
  public:
    USBSerial();
    ~USBSerial();

    /*-------------------------------------------------------------------------
    Custom Interface
    -------------------------------------------------------------------------*/

    /**
     * @brief Map buffers to the USB driver
     *
     * @param endpoint Which CDC endpoint to push/pull from
     * @param prx RX buffer to use
     * @param ptx TX buffer to use
     * @return Chimera::Status_t
     */
    Chimera::Status_t init( const size_t endpoint, CircularBuffer prx, CircularBuffer ptx );

    /**
     * @brief Periodic processing to flush IO buffers as data arrives.
     *
     * Needs to be called rapidly to ensure data is not lost.
     *
     * @return void
     */
    void process();

    /*-------------------------------------------------------------------------
    Chimera::Serial::Driver Implementation
    -------------------------------------------------------------------------*/
    Chimera::Status_t open( const Chimera::Serial::Config &config ) final override;
    Chimera::Status_t close() final override;
    int               write( const void *const buffer, const size_t length, const size_t timeout = Chimera::Thread::TIMEOUT_DONT_WAIT ) final override;
    int               read( void *const buffer, const size_t length, const size_t timeout = Chimera::Thread::TIMEOUT_DONT_WAIT ) final override;

  private:
    size_t         mEndpoint;
    CircularBuffer mRXBuffer;
    CircularBuffer mTXBuffer;
  };
}  // namespace Orbit::Serial

#endif  /* !ORBIT_COM_SERIAL_USB_HPP */
