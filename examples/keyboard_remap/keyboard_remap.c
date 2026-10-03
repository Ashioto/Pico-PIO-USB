/*
 * Caps-as-fn keyboard remapper + GPIO arrow switches.
 *
 * Physical USB keyboard -> Pico PIO USB host (GP0/GP1)
 * GPIO switches GP6-GP9 (to GND) -> arrow keys
 * Pico native USB HID keyboard -> computer
 *
 * Caps alone: nothing
 * Caps+Space: CapsLock
 * Caps+H/J/K/L: Left/Down/Up/Right
 * GP6/GP7/GP8/GP9: Left/Down/Up/Right (always active)
 * Other keys: pass through
 * Switches + Caps layer merge into one HID report.
 */

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "pio_usb.h"
#include "tusb.h"

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

typedef struct {
  uint8_t modifier;
  uint8_t keycode[6];
} kbd_report_t;

static volatile uint8_t kbd_mod;
static volatile uint8_t kbd_keys[6];
static volatile uint8_t kbd_dirty;

static volatile uint8_t kbd_dev_addr;
static volatile uint8_t kbd_instance;
static volatile uint8_t led_state;
static volatile uint8_t led_dirty;

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
}

static void switches_init(void) {
  uint64_t now = time_us_64();
  for (int i = 0; i < SW_COUNT; i++) {
    gpio_init(sw_pins[i]);
    gpio_set_dir(sw_pins[i], GPIO_IN);
    gpio_pull_up(sw_pins[i]);
    // Treat as not pressed at boot (pull-up reads high).
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
    // Active low: pressed == connected to GND == reads 0.
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

    // Forward LED state from computer to physical keyboard.
    // Host stack runs on this core, so tuh_hid_set_report is safe here.
    if (led_dirty && kbd_dev_addr) {
      uint8_t leds = led_state;
      led_dirty = 0;
      tuh_hid_set_report(kbd_dev_addr, kbd_instance, 0, HID_REPORT_TYPE_OUTPUT,
                         &leds, 1);
    }
  }
}

// core0: native USB device — HID keyboard to computer
int main(void) {
  // PIO USB requires sysclock multiple of 12 MHz.
  set_sys_clock_khz(120000, true);

  sleep_ms(10);

  switches_init();

  multicore_reset_core1();
  multicore_launch_core1(core1_main);

  tud_init(0);

  while (true) {
    tud_task();
    switches_poll();

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
      tud_hid_keyboard_report(0, kbd_mod, keys);
    }
  }

  return 0;
}

//--------------------------------------------------------------------+
// Host HID callbacks (core1)
//--------------------------------------------------------------------+

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;

  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
    kbd_dev_addr = dev_addr;
    kbd_instance = instance;
  }
  tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
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

//--------------------------------------------------------------------+
// Device HID callbacks (core0)
//--------------------------------------------------------------------+

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
