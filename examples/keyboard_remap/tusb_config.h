/*
 * TinyUSB config for keyboard_remap:
 * - Device: HID keyboard on native USB (roothub port0)
 *           Optional CDC debug serial (KEYBOARD_REMAP_WITH_CDC=1)
 * - Host: HID on PIO USB (roothub port1)
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

// Set by CMake: 0 = HID-only (restricted PCs), 1 = CDC+HID debug
#ifndef KEYBOARD_REMAP_WITH_CDC
#define KEYBOARD_REMAP_WITH_CDC 0
#endif

#define CFG_TUSB_OS               OPT_OS_PICO

#define CFG_TUD_ENABLED           1

#define CFG_TUH_ENABLED           1
#define CFG_TUH_RPI_PIO_USB       1

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

#define CFG_TUD_HID               1
#define CFG_TUD_CDC               KEYBOARD_REMAP_WITH_CDC
#define CFG_TUD_MSC               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

#define CFG_TUD_HID_EP_BUFSIZE    8
#if CFG_TUD_CDC
#define CFG_TUD_CDC_RX_BUFSIZE    256
#define CFG_TUD_CDC_TX_BUFSIZE    256
#define CFG_TUD_CDC_EP_BUFSIZE    64
#endif

//--------------------------------------------------------------------
// HOST CONFIGURATION
//--------------------------------------------------------------------

// Match host_hid_to_device_cdc build that successfully enumerated the keyboard
#define CFG_TUH_ENUMERATION_BUFSIZE 512

#define CFG_TUH_HUB                 0
#define CFG_TUH_DEVICE_MAX          (CFG_TUH_HUB ? 4 : 1)

#define CFG_TUH_HID                  4
#define CFG_TUH_HID_EPIN_BUFSIZE    64
#define CFG_TUH_HID_EPOUT_BUFSIZE   64

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
