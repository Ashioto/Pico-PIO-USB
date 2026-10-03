/*
 * Caps-as-fn keyboard remapper + GPIO arrow switches + CDC debug.
 *
 * Physical USB keyboard -> Pico PIO USB host (GP0/GP1)
 * GPIO switches GP6-GP9 (to GND) -> arrow keys
 * Pico native USB composite -> computer:
 *   - HID keyboard (remapped keys)
 *   - CDC serial (debug logs)
 *
 * Caps alone: nothing
 * Caps+Space: CapsLock
 * Caps+H/J/K/L: Left/Down/Up/Right
 * GP6/GP7/GP8/GP9: Left/Down/Up/Right (always active)
 * Other keys: pass through
 * Switches + Caps layer merge into one HID report.
 *
 * Debug: LED SET_REPORT disabled; CDC logs only from core0.
 */

#define DEBUG_DISABLE_LED_FORWARD 1

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "pio_usb.h"
#include "tusb.h"

// PIO host root port (for enum-fail re-attach); avoid full pio_usb_ll.h
extern root_port_t pio_usb_root_port[PIO_USB_ROOT_PORT_CNT];
#ifndef PIO_USB_ROOT_PORT
#define PIO_USB_ROOT_PORT(_idx) (pio_usb_root_port + (_idx))
#endif

#define HOST_INT_CONNECT      (1u << 0)
#define HOST_INT_DISCONNECT   (1u << 1)
#define HOST_INT_RESET_END    (1u << 2)
#define HOST_INT_SETUP_REQ    (1u << 3)
#define HOST_INT_SOF          (1u << 4)
#define HOST_INT_EP_COMPLETE  (1u << 5)
#define HOST_INT_EP_ERROR     (1u << 6)
#define HOST_INT_EP_STALLED   (1u << 7)
#define HOST_INT_EP_CONTINUE  (1u << 8)
#define HOST_INT_EP_ANY       (HOST_INT_EP_COMPLETE | HOST_INT_EP_ERROR | HOST_INT_EP_STALLED)

// PIO USB examples allow 120 or 240 MHz only — not 125.
// 240 MHz = more PIO cycles per USB bit (timing margin if 120 is marginal).
#define USE_SYS_CLK_240MHZ 1

// Match host_hid_to_device_cdc: force re-attach every 3s when no mount.
// Do NOT call pio_usb_host_restart() — it busy-waits forever and hangs.
#define HOST_ENUM_TIMEOUT_MS 3000

#define KEY_NONE        0x00
#define KEY_H           0x0B
#define KEY_J           0x0D
#define KEY_K           0x0E
#define KEY_L           0x0F
#define KEY_SPACE       0x2C
#define KEY_CAPS_LOCK   0x39
#define KEY_ARROW_RIGHT 0x4F
#define KEY_ARROW_LEFT  0x50
#define KEY_ARROW_DOWN  0x51
#define KEY_ARROW_UP    0x52

#define SW_COUNT        4
#define SW_LEFT_PIN     6
#define SW_DOWN_PIN     7
#define SW_UP_PIN       8
#define SW_RIGHT_PIN    9
#define SW_DEBOUNCE_US  10000

static volatile uint8_t kbd_mod;
static volatile uint8_t kbd_keys[6];
static volatile uint8_t kbd_dirty;

static volatile uint8_t kbd_dev_addr;
static volatile uint8_t kbd_instance;
static volatile uint8_t led_state;
static volatile uint8_t led_dirty;

static volatile uint32_t host_rx_count;
static volatile uint32_t host_tx_ok_count;
static volatile uint32_t host_tx_fail_count;

// Host callbacks (core1) publish these; core0 formats CDC lines.
static volatile uint8_t  usbh_mount_seq;     // device-level enum finished
static volatile uint8_t  usbh_umount_seq;
static volatile uint8_t  usbh_mount_addr;
static volatile uint16_t usbh_mount_vid;
static volatile uint16_t usbh_mount_pid;
static volatile uint8_t  usbh_umount_addr;

static volatile uint8_t  hid_mount_seq;      // increments on HID mount
static volatile uint8_t  hid_umount_seq;     // increments on umount
static volatile uint8_t  hid_mount_addr;
static volatile uint8_t  hid_mount_inst;
static volatile uint16_t hid_mount_vid;
static volatile uint16_t hid_mount_pid;
static volatile uint8_t  hid_mount_proto;
static volatile uint8_t  hid_umount_addr;
static volatile uint8_t  hid_umount_inst;

// Last host report snapshot for core0 RX log (core1 writes, core0 reads)
static volatile uint8_t  rx_mod;
static volatile uint8_t  rx_phys[6];
static volatile uint8_t  rx_caps;
static volatile uint8_t  rx_out[6];
static volatile uint8_t  rx_seq;

static const uint8_t sw_pins[SW_COUNT] = {
    SW_LEFT_PIN, SW_DOWN_PIN, SW_UP_PIN, SW_RIGHT_PIN,
};
static const uint8_t sw_keycodes[SW_COUNT] = {
    KEY_ARROW_LEFT, KEY_ARROW_DOWN, KEY_ARROW_UP, KEY_ARROW_RIGHT,
};

static bool sw_stable[SW_COUNT];
static uint64_t sw_last_us[SW_COUNT];
static volatile uint8_t sw_keys[SW_COUNT];
static volatile uint8_t sw_dirty;

// Track mapping of keys currently held so layer stays sticky until release.
static uint8_t prev_phys[6];
static uint8_t prev_mapped[6];

// CDC helpers — call from core0 only
static void cdc_log(const char *s) {
  if (tud_cdc_connected()) {
    tud_cdc_write_str(s);
    tud_cdc_write_flush();
  }
}

static void cdc_logf(const char *fmt, ...) {
  if (!tud_cdc_connected()) return;
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) {
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    tud_cdc_write(buf, (uint32_t)n);
    tud_cdc_write_flush();
  }
}

static const char *line_state_str(uint8_t s) {
  switch (s) {
    case 0b00: return "SE0";
    case 0b01: return "FS_IDLE";
    case 0b10: return "LS_IDLE";
    default:   return "SE1";
  }
}

static void poll_host_port(bool force) {
  static bool first = true;
  static uint32_t last_bits = 0xFFFFFFFF;

  root_port_t *root = PIO_USB_ROOT_PORT(0);
  uint8_t dp = gpio_get(root->pin_dp) ? 0 : 1;
  uint8_t dm = gpio_get(root->pin_dm) ? 0 : 1;
  uint8_t line = (uint8_t)((dm << 1) | dp);

  uint32_t bits = ((uint32_t)root->initialized << 0) |
                  ((uint32_t)root->connected   << 1) |
                  ((uint32_t)root->is_fullspeed<< 2) |
                  ((uint32_t)root->suspended   << 3) |
                  ((uint32_t)root->mode        << 5);

  if (!force && !first && bits == last_bits) return;
  first = false;
  last_bits = bits;

  cdc_logf("host: init=%u conn=%u fs=%u susp=%u mode=%u dp=%u dm=%u line=%s\r\n",
           (unsigned)root->initialized,
           (unsigned)root->connected,
           (unsigned)root->is_fullspeed,
           (unsigned)root->suspended,
           (unsigned)root->mode,
           (unsigned)gpio_get(root->pin_dp),
           (unsigned)gpio_get(root->pin_dm),
           line_state_str(line));
}

static void poll_host_ints(void) {
  static uint32_t last_ints;
  root_port_t *root = PIO_USB_ROOT_PORT(0);
  uint32_t ints = root->ints;
  if (ints == last_ints) return;
  last_ints = ints;
  if (ints == 0) return;

  cdc_logf("host ints: 0x%03x%s%s%s%s%s%s%s%s%s\r\n", (unsigned)ints,
           (ints & HOST_INT_CONNECT)     ? " CONNECT" : "",
           (ints & HOST_INT_DISCONNECT)  ? " DISCONNECT" : "",
           (ints & HOST_INT_RESET_END)   ? " RESET_END" : "",
           (ints & HOST_INT_SETUP_REQ)   ? " SETUP_REQ" : "",
           (ints & HOST_INT_SOF)         ? " SOF" : "",
           (ints & HOST_INT_EP_COMPLETE) ? " EP_OK" : "",
           (ints & HOST_INT_EP_ERROR)    ? " EP_ERR" : "",
           (ints & HOST_INT_EP_STALLED)  ? " EP_STALL" : "",
           (ints & HOST_INT_EP_CONTINUE) ? " EP_CONT" : "");
}

// Re-attach every 3s when connected but nothing mounted — same as working build.
static void poll_host_enum_timeout(void) {
  static absolute_time_t conn_since;
  static bool conn_tracking;
  static uint8_t restart_count;

  root_port_t *root = PIO_USB_ROOT_PORT(0);

  if (!root->connected || kbd_dev_addr != 0 || usbh_mount_seq != 0 ||
      hid_mount_seq != 0) {
    conn_tracking = false;
    return;
  }

  if (!conn_tracking) {
    conn_tracking = true;
    conn_since = get_absolute_time();
    return;
  }

  int64_t ms = absolute_time_diff_us(conn_since, get_absolute_time()) / 1000;
  if (ms < HOST_ENUM_TIMEOUT_MS) return;

  restart_count++;
  cdc_logf("ENUM FAIL #%u conn=%lldms usbh=%u hid=%u kbd=0 — force re-attach\r\n",
           (unsigned)restart_count, (long long)ms,
           (unsigned)usbh_mount_seq, (unsigned)hid_mount_seq);
  // Clear connected so next SOF re-sees idle line → CONNECT → fresh enum.
  root->connected = false;
  root->suspended = true;
  conn_tracking = false;
}

// Drain host-callback events into CDC lines (core0 only)
static void drain_host_events(void) {
  static uint8_t last_usbh_mount_seq;
  static uint8_t last_usbh_umount_seq;
  static uint8_t last_hid_mount_seq;
  static uint8_t last_hid_umount_seq;
  static uint8_t last_rx_seq;

  if (usbh_mount_seq != last_usbh_mount_seq) {
    last_usbh_mount_seq = usbh_mount_seq;
    cdc_logf("USBH mount addr=%u [%04x:%04x]\r\n",
             usbh_mount_addr, usbh_mount_vid, usbh_mount_pid);
  }

  if (usbh_umount_seq != last_usbh_umount_seq) {
    last_usbh_umount_seq = usbh_umount_seq;
    cdc_logf("USBH umount addr=%u\r\n", usbh_umount_addr);
  }

  if (hid_mount_seq != last_hid_mount_seq) {
    last_hid_mount_seq = hid_mount_seq;
    cdc_logf("HID mount addr=%u inst=%u [%04x:%04x] proto=%u\r\n",
             hid_mount_addr, hid_mount_inst, hid_mount_vid, hid_mount_pid,
             hid_mount_proto);
  }

  if (hid_umount_seq != last_hid_umount_seq) {
    last_hid_umount_seq = hid_umount_seq;
    cdc_logf("HID umount addr=%u inst=%u\r\n",
             hid_umount_addr, hid_umount_inst);
  }

  if (rx_seq != last_rx_seq) {
    last_rx_seq = rx_seq;
    cdc_logf("RX mod=%02x caps=%d phys=%02x%02x%02x%02x%02x%02x out=%02x%02x%02x%02x%02x%02x\r\n",
             rx_mod, rx_caps,
             rx_phys[0], rx_phys[1], rx_phys[2], rx_phys[3], rx_phys[4], rx_phys[5],
             rx_out[0], rx_out[1], rx_out[2], rx_out[3], rx_out[4], rx_out[5]);
  }
}

static uint8_t remap_key(uint8_t keycode, bool caps_held) {
  if (!caps_held) {
    if (keycode == KEY_CAPS_LOCK) return KEY_NONE;
    return keycode;
  }

  switch (keycode) {
    case KEY_CAPS_LOCK: return KEY_NONE;
    case KEY_SPACE:     return KEY_CAPS_LOCK;
    case KEY_H:         return KEY_ARROW_LEFT;
    case KEY_J:         return KEY_ARROW_DOWN;
    case KEY_K:         return KEY_ARROW_UP;
    case KEY_L:         return KEY_ARROW_RIGHT;
    default:            return keycode;
  }
}

// Runs on core1 (tuh_task) — no CDC writes here
static void process_kbd_report(hid_keyboard_report_t const *report) {
  bool caps_held = false;
  for (int i = 0; i < 6; i++) {
    if (report->keycode[i] == KEY_CAPS_LOCK) {
      caps_held = true;
      break;
    }
  }

  uint8_t cur_phys[6] = {0};
  uint8_t cur_mapped[6] = {0};
  int cur_n = 0;

  for (int i = 0; i < 6; i++) {
    uint8_t phys = report->keycode[i];
    if (phys == KEY_NONE || phys == KEY_CAPS_LOCK) continue;

    uint8_t mapped = KEY_NONE;
    bool held_before = false;
    for (int j = 0; j < 6; j++) {
      if (prev_phys[j] == phys) {
        mapped = prev_mapped[j];
        held_before = true;
        break;
      }
    }

    if (!held_before) {
      mapped = remap_key(phys, caps_held);
    }

    if (mapped != KEY_NONE && cur_n < 6) {
      cur_phys[cur_n] = phys;
      cur_mapped[cur_n] = mapped;
      cur_n++;
    }
  }

  memcpy(prev_phys, cur_phys, sizeof(prev_phys));
  memcpy(prev_mapped, cur_mapped, sizeof(prev_mapped));

  kbd_mod = report->modifier;
  for (int i = 0; i < 6; i++) {
    kbd_keys[i] = (i < cur_n) ? cur_mapped[i] : KEY_NONE;
  }
  kbd_dirty = 1;
  host_rx_count++;

  rx_mod = report->modifier;
  rx_caps = caps_held ? 1 : 0;
  for (int i = 0; i < 6; i++) {
    rx_phys[i] = report->keycode[i];
    rx_out[i] = kbd_keys[i];
  }
  rx_seq++;
}

static void switches_init(void) {
  uint64_t now = time_us_64();
  for (int i = 0; i < SW_COUNT; i++) {
    gpio_init(sw_pins[i]);
    gpio_set_dir(sw_pins[i], GPIO_IN);
    gpio_pull_up(sw_pins[i]);
    sw_stable[i] = false;
    sw_last_us[i] = now;
    sw_keys[i] = KEY_NONE;
  }
  sw_dirty = 1;
}

static void switches_poll(void) {
  uint64_t now = time_us_64();
  bool changed = false;

  for (int i = 0; i < SW_COUNT; i++) {
    bool raw_pressed = (gpio_get(sw_pins[i]) == 0);
    if (raw_pressed == sw_stable[i]) {
      sw_last_us[i] = now;
      continue;
    }
    if ((now - sw_last_us[i]) < SW_DEBOUNCE_US) {
      continue;
    }
    sw_stable[i] = raw_pressed;
    sw_last_us[i] = now;
    changed = true;
    cdc_logf("SW%u %s\r\n", (unsigned)(sw_pins[i]), raw_pressed ? "down" : "up");
  }

  if (changed) {
    for (int i = 0; i < SW_COUNT; i++) {
      sw_keys[i] = sw_stable[i] ? sw_keycodes[i] : KEY_NONE;
    }
    sw_dirty = 1;
  }
}

// core1: PIO USB host — read physical keyboard
void core1_main(void) {
  sleep_ms(10);

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
  tuh_init(1);

  while (true) {
    tuh_task();

#if !DEBUG_DISABLE_LED_FORWARD
    if (led_dirty && kbd_dev_addr) {
      uint8_t leds = led_state;
      led_dirty = 0;
      tuh_hid_set_report(kbd_dev_addr, kbd_instance, 0, HID_REPORT_TYPE_OUTPUT,
                         &leds, 1);
    }
#else
    if (led_dirty) led_dirty = 0;
#endif
  }
}

// core0: native USB composite — HID + CDC
int main(void) {
  // PIO USB requires sysclock 120 or 240 MHz (examples reject others).
#ifdef USE_SYS_CLK_240MHZ
  set_sys_clock_khz(240000, true);
#else
  set_sys_clock_khz(120000, true);
#endif

  sleep_ms(10);

  switches_init();

  multicore_reset_core1();
  multicore_launch_core1(core1_main);

  tud_init(0);

  bool was_cdc_connected = false;
  absolute_time_t next_alive = get_absolute_time();

#ifdef USE_SYS_CLK_240MHZ
  cdc_log("boot keyboard_remap debug clk=240MHz\r\n");
#else
  cdc_log("boot keyboard_remap debug clk=120MHz\r\n");
#endif

  while (true) {
    tud_task();
    switches_poll();
    poll_host_port(false);
    poll_host_ints();
    poll_host_enum_timeout();
    drain_host_events();

    if ((kbd_dirty || sw_dirty) && tud_hid_ready()) {
      uint8_t keys[6] = {0};
      int n = 0;

      for (int i = 0; i < 6 && n < 6; i++) {
        if (kbd_keys[i] != KEY_NONE) {
          keys[n++] = kbd_keys[i];
        }
      }
      for (int i = 0; i < SW_COUNT && n < 6; i++) {
        if (sw_keys[i] != KEY_NONE) {
          keys[n++] = sw_keys[i];
        }
      }

      kbd_dirty = 0;
      sw_dirty = 0;
      bool ok = tud_hid_keyboard_report(0, kbd_mod, keys);
      if (ok) {
        host_tx_ok_count++;
        cdc_logf("TX ok n=%d\r\n", n);
      } else {
        host_tx_fail_count++;
        cdc_logf("TX fail n=%d\r\n", n);
      }
    }

    bool cdc_connected = tud_cdc_connected();
    if (cdc_connected && !was_cdc_connected) {
      cdc_log("cdc connected\r\n");
      poll_host_port(true);
      next_alive = delayed_by_ms(get_absolute_time(), 2000);
    }
    was_cdc_connected = cdc_connected;

    if (cdc_connected && absolute_time_diff_us(get_absolute_time(), next_alive) <= 0) {
      root_port_t *root = PIO_USB_ROOT_PORT(0);
      cdc_logf("alive conn=%u usbh=%u hid=%u kbd=%u rx=%lu tx_ok=%lu tx_fail=%lu hid_ready=%u\r\n",
               (unsigned)root->connected,
               (unsigned)usbh_mount_seq,
               (unsigned)hid_mount_seq,
               (unsigned)kbd_dev_addr,
               (unsigned long)host_rx_count,
               (unsigned long)host_tx_ok_count,
               (unsigned long)host_tx_fail_count,
               (unsigned)tud_hid_ready());
      poll_host_port(true);
      next_alive = delayed_by_ms(get_absolute_time(), 2000);
    }
  }

  return 0;
}

//--------------------------------------------------------------------
// Host device-level + HID callbacks (core1) — publish events only
//--------------------------------------------------------------------

// Invoked when a device is mounted (configured) — enumeration finished
void tuh_mount_cb(uint8_t daddr) {
  uint16_t vid = 0, pid = 0;
  tuh_vid_pid_get(daddr, &vid, &pid);
  usbh_mount_addr = daddr;
  usbh_mount_vid = vid;
  usbh_mount_pid = pid;
  usbh_mount_seq++;
}

// Invoked when a device is unmounted (detached)
void tuh_umount_cb(uint8_t daddr) {
  usbh_umount_addr = daddr;
  usbh_umount_seq++;
}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;

  uint16_t vid = 0, pid = 0;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
    kbd_dev_addr = dev_addr;
    kbd_instance = instance;
  }

  hid_mount_addr = dev_addr;
  hid_mount_inst = instance;
  hid_mount_vid = vid;
  hid_mount_pid = pid;
  hid_mount_proto = itf_protocol;
  hid_mount_seq++;

  tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  hid_umount_addr = dev_addr;
  hid_umount_inst = instance;
  hid_umount_seq++;

  if (kbd_dev_addr == dev_addr && kbd_instance == instance) {
    kbd_dev_addr = 0;
    kbd_instance = 0;
  }
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                uint8_t const *report, uint16_t len) {
  (void)len;
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
    process_kbd_report((hid_keyboard_report_t const *)report);
  }

  tuh_hid_receive_report(dev_addr, instance);
}

//--------------------------------------------------------------------
// Device HID + CDC callbacks (core0)
//--------------------------------------------------------------------

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;
  return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
  (void)instance;
  (void)report_id;

  if (report_type == HID_REPORT_TYPE_OUTPUT && bufsize >= 1) {
    led_state = buffer[0];
    led_dirty = 1;
  }
}

bool tud_hid_set_idle_cb(uint8_t instance, uint8_t idle_rate) {
  (void)instance;
  (void)idle_rate;
  return true;
}

void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol) {
  (void)instance;
  (void)protocol;
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report,
                                uint16_t len) {
  (void)instance;
  (void)report;
  (void)len;
}

void tud_cdc_rx_cb(uint8_t itf) {
  (void)itf;
  char buf[64];
  (void)tud_cdc_read(buf, sizeof(buf));
}
