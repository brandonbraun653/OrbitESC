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
  using ISRLockedQueue = etl::iqueue_spsc_locked<uint8_t> *;


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

  /**
   * @brief Checks if the USB serial driver is connected to a host
   *
   * @return bool  True if connected, false otherwise
   */
  bool isConnected();


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Serial over USB driver
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
     * @param ptx_isr TX buffer to use for ISR generated data
     * @return Chimera::Status_t
     */
    Chimera::Status_t init( const size_t endpoint, CircularBuffer prx, CircularBuffer ptx, ISRLockedQueue ptx_isr );

    /**
     * @brief Periodic processing to flush IO buffers as data arrives.
     *
     * Needs to be called rapidly to ensure data is not lost.
     *
     * @return void
     */
    void process();

    /**
     * @brief Write to the serial endpoint from an ISR context.
     *
     * This operation assumes a single producer, single consumer model. Calling from
     * multiple ISRs that can preempt each other will result in data corruption.
     *
     * @param buffer Data to write
     * @param length Number of bytes to write
     * @return int Number of bytes written
     */
    int writeFromISR( const void *const buffer, const size_t length );

    /*-------------------------------------------------------------------------
    Chimera::Serial::Driver Implementation
    -------------------------------------------------------------------------*/
    Chimera::Status_t open( const Chimera::Serial::Config &config ) final override;
    Chimera::Status_t close() final override;
    int               write( const void *const buffer, const size_t length,
                             const size_t timeout = Chimera::Thread::TIMEOUT_DONT_WAIT ) final override;
    int               read( void *const buffer, const size_t length,
                            const size_t timeout = Chimera::Thread::TIMEOUT_DONT_WAIT ) final override;

  private:
    size_t         mEndpoint;
    CircularBuffer mRXBuffer;
    CircularBuffer mTXBuffer;
    ISRLockedQueue mTXBufferISR;
  };
}    // namespace Orbit::Serial

#endif /* !ORBIT_COM_SERIAL_USB_HPP */
