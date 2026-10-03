# Pico-PIO-USB

USB host/device implementation using PIO of raspberry pi pico (RP2040).

You can add additional USB port to RP2040.

## Demo

https://user-images.githubusercontent.com/43873124/146642806-bdf34af6-4342-4a95-bfca-229cdc4bdca2.mp4

## Project status

|Planned Features|Status|
|-|-|
|FS Host|✔|
|LS Host|✔|
|Hub support|✔|
|Multi port|✔|
|FS Device|✔|

## Examples

- [host_hid_to_device_cdc.c](examples/host_hid_to_device_cdc/host_hid_to_device_cdc.c) which print mouse/keyboard report from host port to device port's cdc. TinyUSB is used to manage both device (native usb) and host (pio usb) stack.
- [usb_device.c](examples/usb_device/usb_device.c) is a HID USB FS device sample which moves mouse cursor every 0.5s. External 1.5kohm pull-up register is necessary to D+ pin (Default is gp0).

```bash
cd examples
mkdir build
cd build
cmake ..
make
# Copy UF2 file in capture_hid_report/ or usbdevice/ to RPiPico
```

Another sample program for split keyboard with QMK

[https://github.com/sekigon-gonnoc/qmk_firmware/tree/rp2040/keyboards/pico_pico_usb](https://github.com/sekigon-gonnoc/qmk_firmware/tree/rp2040/keyboards/pico_pico_usb)

## Resource Usage


- 1 PIO, 3 state machines, 32 instructions
- Two GPIO for D+/D- (Series 22ohm resitors are better)
- 15KB ROM and RAM
- (For Host) One 1ms repeating timer
- (For Device) One PIO IRQ for receiver

## keyboard_remap: Caps-as-fn keyboard remapper

A local example that turns an existing USB keyboard into a Caps-as-fn keyboard via a Raspberry Pi Pico.

**Behavior**

| Input | Output |
|-------|--------|
| Caps alone | nothing |
| Caps+Space | CapsLock |
| Caps+H / J / K / L | Left / Down / Up / Right |
| GP6 / GP7 / GP8 / GP9 switch | Left / Down / Up / Right |
| Other keys | pass through |

GPIO switches are always active (not Caps-gated). Switch arrows and Caps-layer arrows merge into one HID report (up to 6 keys). Switches work even if no USB keyboard is connected.

**Architecture**

```
[USB keyboard] -> Pico PIO USB host (GP0/GP1) -> remap --\
                                                          +-> Pico native USB HID -> computer
[GP6-GP9 switches to GND] -> debounced GPIO ------------/
```

**Build (macOS)**

Homebrew `arm-none-eabi-gcc` does not ship newlib (`nosys.specs` missing). Install the official ARM toolchain instead:

```bash
brew install --cask gcc-arm-embedded
# installs to /Applications/ArmGNUToolchain/<version>/arm-none-eabi/bin/
```

```bash
export PICO_SDK_PATH=/path/to/pico-sdk   # with lib/tinyusb initialized
export PATH="/Applications/ArmGNUToolchain/15.3.rel1/arm-none-eabi/bin:$PATH"

cd examples
rm -rf build && mkdir build && cd build
cmake ..
make keyboard_remap -j4
# Output: build/keyboard_remap/keyboard_remap.uf2
```

**Flash (macOS)**

1. Hold **BOOTSEL** on the Pico, plug USB into the computer
2. A drive named **RPI-RP2** appears
3. Copy `keyboard_remap.uf2` onto it
4. Pico reboots and appears as HID keyboard **“Pico Cap Fn”** (no drivers needed on macOS)

**Wiring (physical keyboard -> Pico host port)**

| Pico pin | USB |
|----------|-----|
| GP0 | D+ |
| GP1 | D- |
| VBUS (5V) | Keyboard VCC |
| GND | Keyboard GND |

Series 22Ω resistors on D+/D- are recommended. Use a USB host breakout; do not plug the keyboard into the Pico’s own micro-USB (that port is the device side to the computer).

**Wiring (GPIO arrow switches)**

Each switch connects the pin to **GND** when pressed (active low). Internal pull-ups are enabled in firmware; no external resistors required.

| Pico pin | Arrow |
|----------|-------|
| GP6 | Left |
| GP7 | Down |
| GP8 | Up |
| GP9 | Right |
