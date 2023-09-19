/******************************************************************************
 *  File Name:
 *    orbit_tusb.c
 *
 *  Description:
 *    C implementation of various requirements of the TinyUSB stack. Some of
 *    this is ported software from the TinyUSB examples, some of it are drivers
 *    that use the STM32HAL out of convenience. I can't use those definitions
 *    directly because they'll conflict with my custom HW drivers in Thor.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <msc.h>
#include <tusb.h>
#include <stm32f446xx.h>
#include <src/core/hw/orbit_tusb.h>


/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/

void tusb_configure_clocks()
{
  RCC_TypeDef           *rcc     = ( ( RCC_TypeDef * )RCC_BASE );
  USB_OTG_GlobalTypeDef *usb_otg = ( ( USB_OTG_GlobalTypeDef * )USB_OTG_HS_PERIPH_BASE );
  USB_OTG_DeviceTypeDef *usb_dev = ( USB_OTG_DeviceTypeDef * )( USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE );

  rcc->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;
}


/*-----------------------------------------------------------------------------
Interrupt Handlers: Overrides declarations in hw_startup_stm32f446xx.c
-----------------------------------------------------------------------------*/

void OTG_HS_EP1_OUT_IRQHandler()
{
  tud_int_handler( TUD_OPT_RHPORT );
}

void OTG_HS_EP1_IN_IRQHandler()
{
  tud_int_handler( TUD_OPT_RHPORT );
}

void OTG_HS_WKUP_IRQHandler()
{
  tud_int_handler( TUD_OPT_RHPORT );
}

void OTG_HS_IRQHandler()
{
  tud_int_handler( TUD_OPT_RHPORT );
}


/*-----------------------------------------------------------------------------
TinyUSB Descriptor Callbacks
-----------------------------------------------------------------------------*/

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]         HID | MSC | CDC          [LSB]
 */
#define _PID_MAP( itf, n ) ( ( CFG_TUD_##itf ) << ( n ) )
#define USB_PID \
  ( 0x4000 | _PID_MAP( CDC, 0 ) | _PID_MAP( MSC, 1 ) | _PID_MAP( HID, 2 ) | _PID_MAP( MIDI, 3 ) | _PID_MAP( VENDOR, 4 ) )

#define USB_VID 0xCafe
#define USB_BCD 0x0200

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
  .bLength         = sizeof( tusb_desc_device_t ),
  .bDescriptorType = TUSB_DESC_DEVICE,
  .bcdUSB          = USB_BCD,

  // Use Interface Association Descriptor (IAD) for CDC
  // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
  .bDeviceClass    = TUSB_CLASS_MISC,
  .bDeviceSubClass = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol = MISC_PROTOCOL_IAD,

  .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

  .idVendor  = USB_VID,
  .idProduct = USB_PID,
  .bcdDevice = 0x0100,

  .iManufacturer = 0x01,
  .iProduct      = 0x02,
  .iSerialNumber = 0x03,

  .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb( void )
{
  return ( uint8_t const * )&desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_MSC,
  ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82

#define EPNUM_MSC_OUT 0x03
#define EPNUM_MSC_IN 0x83

#define CONFIG_TOTAL_LEN ( TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN )

uint8_t const desc_fs_configuration[] = {
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR( 1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100 ),

  // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR( ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64 ),

  // Interface number, string index, EP Out & EP In address, EP size
  TUD_MSC_DESCRIPTOR( ITF_NUM_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64 ),
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb( uint8_t index )
{
  ( void )index;    // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return ( tud_speed_get() == TUSB_SPEED_HIGH ) ? desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// String Descriptor Index
enum
{
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

// array of pointer to string descriptors
char const *string_desc_arr[] = {
  ( const char[] ){ 0x09, 0x04 },    // 0: is supported language is English (0x0409)
  "TinyUSB",                         // 1: Manufacturer
  "TinyUSB Device",                  // 2: Product
  NULL,                              // 3: Serials will use unique ID if possible
  "TinyUSB CDC",                     // 4: CDC Interface
  "TinyUSB MSC",                     // 5: MSC Interface
};

static uint16_t _desc_str[ 32 + 1 ];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb( uint8_t index, uint16_t langid )
{
  ( void )langid;
  size_t chr_count;

  switch ( index )
  {
    case STRID_LANGID:
      memcpy( &_desc_str[ 1 ], string_desc_arr[ 0 ], 2 );
      chr_count = 1;
      break;

    case STRID_SERIAL:
      // chr_count = board_usb_get_serial( _desc_str + 1, 32 );
      chr_count = 5;
      memcpy( &_desc_str[ 1 ], "12345", 5 );
      break;

    default:
      // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
      // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

      if ( !( index < sizeof( string_desc_arr ) / sizeof( string_desc_arr[ 0 ] ) ) )
        return NULL;

      const char *str = string_desc_arr[ index ];

      // Cap at max char
      chr_count              = strlen( str );
      size_t const max_count = sizeof( _desc_str ) / sizeof( _desc_str[ 0 ] ) - 1;    // -1 for string type
      if ( chr_count > max_count )
        chr_count = max_count;

      // Convert ASCII string into UTF-16
      for ( size_t i = 0; i < chr_count; i++ )
      {
        _desc_str[ 1 + i ] = str[ i ];
      }
      break;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[ 0 ] = ( uint16_t )( ( TUSB_DESC_STRING << 8 ) | ( 2 * chr_count + 2 ) );

  return _desc_str;
}
