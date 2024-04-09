/******************************************************************************
 *  File Name:
 *    serial_usb.hpp
 *
 *  Description:
 *    Serial driver for the Tiny USB CDC interface
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_COM_SERIAL_USB_HPP
#define ORBIT_COM_SERIAL_USB_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/serial>
#include <etl/circular_buffer.h>
#include <etl/queue_spsc_locked.h>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using CircularBuffer = etl::icircular_buffer<uint8_t> *;

  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/

  class USBSerial;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum Endpoint : uint8_t
  {
    COM_ENDPOINT = 0,  /**< Default communication endpoint */

    NUM_ENDPOINTS
  };


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Gets the USB serial driver instance
   * @return USBSerial*
   */
  USBSerial *getUSBSerialDriver();

  /**
   * @brief Checks if the USB serial driver is connected to a host
   * @return bool  True if connected, false otherwise
   */
  bool isConnected();


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Serial over USB driver.
   *
   * This class is a thin wrapper around the TinyUSB CDC driver. It provides
   * a serial interface to the host PC over USB. All read/write operations are
   * done in user space and are thread safe.
   */
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
     * @param prx RX buffer to use for normal multi-threaded operation
     * @param ptx TX buffer to use for normal multi-threaded operation
     * @return Chimera::Status_t
     */
    Chimera::Status_t init( const Endpoint endpoint, CircularBuffer prx, CircularBuffer ptx );

    /**
     * @brief Periodic processing to flush IO buffers as data arrives.
     *
     * Normally most data is transferred via the userspace interrupt handlers
     * for the USB peripheral. However, it's possible to stall data transfer
     * in some scenarios. This function is used to ensure that data is still
     * being processed even if the interrupt handlers are not being called.
     *
     * @return void
     */
    void process();

    /*-------------------------------------------------------------------------
    Chimera::Serial::Driver Implementation
    -------------------------------------------------------------------------*/
    Chimera::Status_t open( const Chimera::Serial::Config &config ) final override;
    Chimera::Status_t close() final override;
    int               write( const void *const buffer, const size_t length,
                             const size_t timeout = Chimera::Thread::TIMEOUT_DONT_WAIT ) final override;
    int               read( void *const buffer, const size_t length,
                            const size_t timeout = Chimera::Thread::TIMEOUT_DONT_WAIT ) final override;

  protected:
    friend etl::delegate<void( void )>;

    void on_rx_complete();
    void on_tx_complete();

  private:
    size_t         mEndpoint;
    CircularBuffer mRXBuffer;
    CircularBuffer mTXBuffer;
  };
}    // namespace Orbit::Serial

#endif /* !ORBIT_COM_SERIAL_USB_HPP */
