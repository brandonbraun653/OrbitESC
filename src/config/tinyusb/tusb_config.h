/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif

//--------------------------------------------------------------------
// Board Specific Configuration
//--------------------------------------------------------------------

// Root hub port. Use the High Speed port on the board.
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT 1
#endif

// Use Full Speed configuration
#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_STM32F4
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_FREERTOS
#endif

// Set the logging level to info an higher
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

// Map printf to custom logger
#ifndef CFG_TUSB_DEBUG_PRINTF
#define CFG_TUSB_DEBUG_PRINTF orbit_esc_printf
#endif


// Enable Device stack
#define CFG_TUD_ENABLED 1

// Default is max speed that hardware controller could support with on-chip PHY
#define CFG_TUD_MAX_SPEED BOARD_TUD_MAX_SPEED

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__( ( aligned( 4 ) ) )
#endif

#ifndef CFG_TUSB_RHPORT1_MODE
#define CFG_TUSB_RHPORT1_MODE ( OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED )
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0 // TODO: Might turn this on later for reading SD card.

// CDC FIFO size of TX and RX
/*-------------------------------------------------------------------
Buffer size was set to accommodate a few milliseconds of high speed
ISR data. There is an unfortunate side-effect where the USB stack
won't work properly if the RX FIFO size is less than the EP buffer
size. So until that's fixed, we get to waste some memory.

https://github.com/hathach/tinyusb/issues/1924
-------------------------------------------------------------------*/
#define _RTX_BUFFER_SIZE ( 4096 )

#define CFG_TUD_CDC_RX_BUFSIZE ( _RTX_BUFFER_SIZE )
#define CFG_TUD_CDC_TX_BUFSIZE ( _RTX_BUFFER_SIZE )
#define CFG_TUD_CDC_EP_BUFSIZE ( _RTX_BUFFER_SIZE )

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
