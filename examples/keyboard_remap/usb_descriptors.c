/*
 * USB descriptors for keyboard_remap — HID boot keyboard device.
 */

#include <string.h>

#include "pico/unique_id.h"
#include "tusb.h"

#define USB_VID   0xCafe
#define USB_PID   0x4004
#define USB_BCD   0x0200

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------+
// HID Report Descriptor (boot keyboard)
//--------------------------------------------------------------------+
uint8_t const desc_hid_keyboard_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  (void)instance;
  return desc_hid_keyboard_report;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
  ITF_NUM_KEYBOARD = 0,
  ITF_NUM_TOTAL,
};

#define EPNUM_KEYBOARD 0x81

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_HID_DESCRIPTOR(ITF_NUM_KEYBOARD, 0, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(desc_hid_keyboard_report), EPNUM_KEYBOARD,
                       CFG_TUD_HID_EP_BUFSIZE, 10),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;

  uint8_t chr_count = 0;
  const char *str = NULL;
  char serial[3 * 8 + 1];

  switch (index) {
    case STRID_LANGID:
      _desc_str[1] = 0x0409;
      chr_count = 1;
      break;

    case STRID_MANUFACTURER:
      str = "Pico";
      break;

    case STRID_PRODUCT:
      str = "Pico Cap Fn";
      break;

    case STRID_SERIAL:
      pico_get_unique_board_id_string(serial, sizeof(serial));
      str = serial;
      break;

    default:
      return NULL;
  }

  if (str != NULL) {
    chr_count = (uint8_t)strlen(str);
    if (chr_count > 31) chr_count = 31;
    for (uint8_t i = 0; i < chr_count; i++) {
      _desc_str[1 + i] = (uint16_t)str[i];
    }
  }

  _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}
