/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *                    sekigon-gonnoc
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

// This example runs both host and device concurrently. The USB host receive
// reports from HID device and print it out over USB Device CDC interface.
// For TinyUSB roothub port0 is native usb controller, roothub port1 is
// pico-pio-usb.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

#include "pio_usb.h"
#include "tusb.h"

// pio_usb_ll.h: extern only (root port array); avoid pulling PIO SM internals
extern root_port_t pio_usb_root_port[PIO_USB_ROOT_PORT_CNT];
#ifndef PIO_USB_ROOT_PORT
#define PIO_USB_ROOT_PORT(_idx) (pio_usb_root_port + (_idx))
#endif

// E4: try 240 MHz (PIO USB examples allow 120 or 240 MHz only — not 125).
// #define USE_SYS_CLK_240MHZ 1

// PIO USB root interrupt bits (pio_usb_ll.h)
#define HOST_INT_CONNECT      (1u << 0)
#define HOST_INT_DISCONNECT   (1u << 1)
#define HOST_INT_RESET_END    (1u << 2)
#define HOST_INT_SETUP_REQ    (1u << 3)
#define HOST_INT_SOF          (1u << 4)
#define HOST_INT_EP_COMPLETE  (1u << 5)
#define HOST_INT_EP_ERROR     (1u << 6)
#define HOST_INT_EP_STALLED   (1u << 7)
#define HOST_INT_EP_CONTINUE  (1u << 8)

#define ENUM_TIMEOUT_MS 3000

static volatile uint8_t host_mount_count;
static volatile uint8_t host_hid_mount_count;

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// uncomment if you are using colemak layout
// #define KEYBOARD_COLEMAK

#ifdef KEYBOARD_COLEMAK
const uint8_t colemak[128] = {
  0  ,  0,  0,  0,  0,  0,  0, 22,
  9  , 23,  7,  0, 24, 17,  8, 12,
  0  , 14, 28, 51,  0, 19, 21, 10,
  15 ,  0,  0,  0, 13,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0,
  0  ,  0,  0, 18,  0,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0
};
#endif

static uint8_t const keycode2ascii[128][2] =  { HID_KEYCODE_TO_ASCII };

// Debug: GPIO switches GP6-GP9 (active low to GND, internal pull-up)
#define SW_PIN_BASE 6
#define SW_COUNT    4
static const char *sw_names[SW_COUNT] = { "SW6", "SW7", "SW8", "SW9" };
static bool sw_state[SW_COUNT];
static absolute_time_t sw_debounce_at[SW_COUNT];

static void switches_init(void) {
  for (int i = 0; i < SW_COUNT; i++) {
    uint pin = SW_PIN_BASE + i;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    sw_state[i] = true; // pull-up = not pressed
    sw_debounce_at[i] = get_absolute_time();
  }
}

static void poll_switches(void) {
  for (int i = 0; i < SW_COUNT; i++) {
    uint pin = SW_PIN_BASE + i;
    bool raw = gpio_get(pin); // true = released (pull-up)
    if (raw == sw_state[i]) continue;
    if (absolute_time_diff_us(sw_debounce_at[i], get_absolute_time()) < 10000) continue;
    sw_debounce_at[i] = get_absolute_time();
    sw_state[i] = raw;
    if (tud_cdc_connected()) {
      char buf[32];
      int n = sprintf(buf, "%s %s\r\n", sw_names[i], raw ? "up" : "down");
      tud_cdc_write(buf, n);
      tud_cdc_write_flush();
    }
  }
}

// Line state after PIO invert override: raw gpio 1 -> bit 0
static const char *line_state_str(uint8_t s) {
  switch (s) {
    case 0b00: return "SE0";
    case 0b01: return "FS_IDLE";
    case 0b10: return "LS_IDLE";
    default:   return "SE1";
  }
}

// Poll PIO USB host root port (roothub port1 = index 0 for PIO)
// Print on structural status change only (not per-line flaps during traffic)
static void poll_host_port(bool force) {
  static bool first = true;
  static uint32_t last_bits = 0xFFFFFFFF;

  root_port_t *root = PIO_USB_ROOT_PORT(0);

  // Match pio_usb_bus_get_line_state(): DP/DM are inverted at GPIO input
  uint8_t dp = gpio_get(root->pin_dp) ? 0 : 1;
  uint8_t dm = gpio_get(root->pin_dm) ? 0 : 1;
  uint8_t line = (uint8_t)((dm << 1) | dp);

  // Structural only: init/conn/fs/susp/mode — ignore line flaps during SOF/reset
  uint32_t bits = ((uint32_t)root->initialized << 0) |
                  ((uint32_t)root->connected   << 1) |
                  ((uint32_t)root->is_fullspeed<< 2) |
                  ((uint32_t)root->suspended   << 3) |
                  ((uint32_t)root->mode        << 5);

  if (!force && !first && bits == last_bits) return;
  first = false;
  last_bits = bits;

  if (!tud_cdc_connected()) return;

  char buf[128];
  int n = sprintf(buf,
      "host: init=%u conn=%u fs=%u susp=%u mode=%u dp=%u dm=%u line=%s\r\n",
      (unsigned)root->initialized,
      (unsigned)root->connected,
      (unsigned)root->is_fullspeed,
      (unsigned)root->suspended,
      (unsigned)root->mode,
      (unsigned)gpio_get(root->pin_dp),
      (unsigned)gpio_get(root->pin_dm),
      line_state_str(line));
  tud_cdc_write(buf, n);
  tud_cdc_write_flush();
}

// E3: print PIO host IRQ bits when they fire (race with host core; best-effort)
static void poll_host_ints(void) {
  static uint32_t last_ints;
  root_port_t *root = PIO_USB_ROOT_PORT(0);
  uint32_t ints = root->ints;
  if (ints == last_ints) return;
  last_ints = ints;
  if (!tud_cdc_connected() || ints == 0) return;

  char buf[128];
  int n = sprintf(buf, "host ints: 0x%03x%s%s%s%s%s%s%s%s%s\r\n", (unsigned)ints,
      (ints & HOST_INT_CONNECT)     ? " CONNECT" : "",
      (ints & HOST_INT_DISCONNECT)  ? " DISCONNECT" : "",
      (ints & HOST_INT_RESET_END)   ? " RESET_END" : "",
      (ints & HOST_INT_SETUP_REQ)   ? " SETUP_REQ" : "",
      (ints & HOST_INT_SOF)         ? " SOF" : "",
      (ints & HOST_INT_EP_COMPLETE) ? " EP_OK" : "",
      (ints & HOST_INT_EP_ERROR)    ? " EP_ERR" : "",
      (ints & HOST_INT_EP_STALLED)  ? " EP_STALL" : "",
      (ints & HOST_INT_EP_CONTINUE) ? " EP_CONT" : "");
  tud_cdc_write(buf, n);
  tud_cdc_write_flush();
}

// E1+E2: if connected but TinyUSB never mounts, force PIO re-detect so
// CONNECT irq fires again and TinyUSB starts a fresh enumeration.
// Do NOT call pio_usb_host_restart() here — it busy-waits on a flag that
// is never cleared and hangs core0.
static void poll_enum_timeout(void) {
  static absolute_time_t conn_since;
  static bool conn_tracking;
  static uint8_t restart_count;
  static uint8_t last_fail_count;

  root_port_t *root = PIO_USB_ROOT_PORT(0);

  if (!root->connected) {
    conn_tracking = false;
    return;
  }

  if (!conn_tracking) {
    conn_tracking = true;
    conn_since = get_absolute_time();
    return;
  }

  if (host_mount_count > 0 || host_hid_mount_count > 0) return;

  int64_t ms = absolute_time_diff_us(conn_since, get_absolute_time()) / 1000;
  if (ms < ENUM_TIMEOUT_MS) return;

  restart_count++;
  char buf[160];
  int n = sprintf(buf,
      "ENUM FAIL #%u conn=%lldms mounts=%u hid=%u — force re-attach\r\n",
      (unsigned)restart_count, (long long)ms,
      (unsigned)host_mount_count, (unsigned)host_hid_mount_count);
  tud_cdc_write(buf, n);
  tud_cdc_write_flush();

  // Clear connected so next SOF frame re-sees idle line and raises CONNECT.
  // TinyUSB then gets HCD_EVENT_DEVICE_ATTACH and retries enumeration.
  root->connected = false;
  root->suspended = true;

  last_fail_count = restart_count;
  (void)last_fail_count;
  conn_tracking = false; // wait for re-detect, start a new timeout window
}

/*------------- MAIN -------------*/

// core1: handle host events
void core1_main() {
  sleep_ms(10);

  // Use tuh_configure() to pass pio configuration to the host stack
  // Note: tuh_configure() must be called before
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

  // To run USB SOF interrupt in core1, init host stack for pio_usb (roothub
  // port1) on core1
  tuh_init(1);

  while (true) {
    tuh_task(); // tinyusb host task
  }
}

// core0: handle device events
int main(void) {
  // PIO USB requires sysclock 120 or 240 MHz (examples reject others).
#ifdef USE_SYS_CLK_240MHZ
  set_sys_clock_khz(240000, true);
#else
  set_sys_clock_khz(120000, true);
#endif

  sleep_ms(10);

  multicore_reset_core1();
  // all USB task run in core1
  multicore_launch_core1(core1_main);

  // init device stack on native usb (roothub port0)
  tud_init(0);

  switches_init();

  // May be lost if Mac opens CDC later; connected/alive prints cover that
#ifdef USE_SYS_CLK_240MHZ
  printf("boot host_hid_to_device_cdc clk=240MHz\r\n");
#else
  printf("boot host_hid_to_device_cdc clk=120MHz\r\n");
#endif

  bool was_cdc_connected = false;
  absolute_time_t next_alive = get_absolute_time();

  while (true) {
    tud_task(); // tinyusb device task
    tud_cdc_write_flush();

    poll_switches();
    poll_host_port(false);
    poll_host_ints();
    poll_enum_timeout();

    bool cdc_connected = tud_cdc_connected();
    if (cdc_connected && !was_cdc_connected) {
      tud_cdc_write_str("cdc connected\r\n");
      tud_cdc_write_flush();
      poll_host_port(true); // initial host status once CDC is up
      next_alive = delayed_by_ms(get_absolute_time(), 2000);
    }
    was_cdc_connected = cdc_connected;

    if (cdc_connected && absolute_time_diff_us(get_absolute_time(), next_alive) <= 0) {
      char abuf[80];
      int an = sprintf(abuf, "alive mounts=%u hid=%u\r\n",
                       (unsigned)host_mount_count, (unsigned)host_hid_mount_count);
      tud_cdc_write(abuf, an);
      tud_cdc_write_flush();
      poll_host_port(true); // re-assert host line every alive tick
      next_alive = delayed_by_ms(get_absolute_time(), 2000);
    }
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device CDC
//--------------------------------------------------------------------+

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;

  char buf[64];
  uint32_t count = tud_cdc_read(buf, sizeof(buf));

  // TODO control LED on keyboard of host stack
  (void) count;
}

//--------------------------------------------------------------------+
// Host HID
//--------------------------------------------------------------------+

// Invoked when a device is mounted (configured) - enumeration finished
void tuh_mount_cb(uint8_t daddr)
{
  host_mount_count++;

  uint16_t vid = 0, pid = 0;
  tuh_vid_pid_get(daddr, &vid, &pid);

  char tempbuf[80];
  int count = sprintf(tempbuf, "USBH mount addr=%u [%04x:%04x]\r\n", daddr, vid, pid);
  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();
}

// Invoked when a device is unmounted (detached)
void tuh_umount_cb(uint8_t daddr)
{
  char tempbuf[48];
  int count = sprintf(tempbuf, "USBH umount addr=%u\r\n", daddr);
  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();
}

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  (void)desc_report;
  (void)desc_len;

  host_hid_mount_count++;

  // Interface protocol (hid_interface_protocol_enum_t)
  const char* protocol_str[] = { "None", "Keyboard", "Mouse" };
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  char tempbuf[256];
  int count = sprintf(tempbuf, "[%04x:%04x][%u] HID Interface%u, Protocol = %s\r\n", vid, pid, dev_addr, instance, protocol_str[itf_protocol]);

  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();

  // Receive report from boot keyboard & mouse only
  // tuh_hid_report_received_cb() will be invoked when report is available
  if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD || itf_protocol == HID_ITF_PROTOCOL_MOUSE)
  {
    if ( !tuh_hid_receive_report(dev_addr, instance) )
    {
      tud_cdc_write_str("Error: cannot request report\r\n");
    }
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  char tempbuf[256];
  int count = sprintf(tempbuf, "[%u] HID Interface%u is unmounted\r\n", dev_addr, instance);
  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();
}

// look up new key in previous keys
static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}


// convert hid keycode to ascii and print via usb device CDC (ignore non-printable)
static void process_kbd_report(uint8_t dev_addr, hid_keyboard_report_t const *report)
{
  (void) dev_addr;
  static hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
  bool flush = false;

  for(uint8_t i=0; i<6; i++)
  {
    uint8_t keycode = report->keycode[i];
    if ( keycode )
    {
      if ( find_key_in_report(&prev_report, keycode) )
      {
        // exist in previous report means the current key is holding
      }else
      {
        // not existed in previous report means the current key is pressed

        // remap the key code for Colemak layout
        #ifdef KEYBOARD_COLEMAK
        uint8_t colemak_key_code = colemak[keycode];
        if (colemak_key_code != 0) keycode = colemak_key_code;
        #endif

        bool const is_shift = report->modifier & (KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTSHIFT);
        uint8_t ch = keycode2ascii[keycode][is_shift ? 1 : 0];

        if (ch)
        {
          if (ch == '\n') tud_cdc_write("\r", 1);
          tud_cdc_write(&ch, 1);
          flush = true;
        }
        else
        {
          // non-printable (arrows, F-keys, etc.)
          char kbuf[32];
          int kn = sprintf(kbuf, "[key 0x%02x]\r\n", keycode);
          tud_cdc_write(kbuf, kn);
          flush = true;
        }
      }
    }
    // TODO example skips key released
  }

  if (flush) tud_cdc_write_flush();

  prev_report = *report;
}

// send mouse report to usb device CDC
static void process_mouse_report(uint8_t dev_addr, hid_mouse_report_t const * report)
{
  //------------- button state  -------------//
  //uint8_t button_changed_mask = report->buttons ^ prev_report.buttons;
  char l = report->buttons & MOUSE_BUTTON_LEFT   ? 'L' : '-';
  char m = report->buttons & MOUSE_BUTTON_MIDDLE ? 'M' : '-';
  char r = report->buttons & MOUSE_BUTTON_RIGHT  ? 'R' : '-';

  char tempbuf[32];
  int count = sprintf(tempbuf, "[%u] %c%c%c %d %d %d\r\n", dev_addr, l, m, r, report->x, report->y, report->wheel);

  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) len;
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  switch(itf_protocol)
  {
    case HID_ITF_PROTOCOL_KEYBOARD:
      process_kbd_report(dev_addr, (hid_keyboard_report_t const*) report );
    break;

    case HID_ITF_PROTOCOL_MOUSE:
      process_mouse_report(dev_addr, (hid_mouse_report_t const*) report );
    break;

    default: break;
  }

  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, instance) )
  {
    tud_cdc_write_str("Error: cannot request report\r\n");
  }
}
