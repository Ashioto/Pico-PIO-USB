# keyboard_remap firmware

Prebuilt UF2s for the Caps-as-fn keyboard remapper (RP2040 Pico).

| File | Use when | USB device |
|------|----------|------------|
| `keyboard_remap_hid_only.uf2` | Restricted PCs — no CDC/serial | `Pico Cap Fn` |
| `keyboard_remap_debug.uf2` | Debug / serial logs | `Pico Cap Fn Debug` |

## Remap
- **Caps** alone: no-op
- **Caps + Space**: CapsLock
- **Caps + H/J/K/L**: arrows
- **Caps + 1..9,0,-,=**: F1..F12
- **GP6–GP9**: always arrows

## 240 MHz is important
Run the PIO USB host at **240 MHz** (`USE_SYS_CLK_240MHZ`). That is the fix when the keyboard fails to enumerate (host shows no HID device). At 120 MHz this hardware often never completes USB enumeration; 240 MHz makes it reliable.

## Rebuild
Source: `examples/keyboard_remap/`
CMake targets: `keyboard_remap` (HID-only) and `keyboard_remap_debug` (CDC+HID).
These UF2s are snapshots — rebuild after source changes if you want the folder updated.
