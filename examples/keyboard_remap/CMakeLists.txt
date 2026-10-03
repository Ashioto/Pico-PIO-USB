# Two targets:
#   keyboard_remap        -> HID only (restricted PCs)     KEYBOARD_REMAP_WITH_CDC=0
#   keyboard_remap_debug  -> CDC + HID debug serial        KEYBOARD_REMAP_WITH_CDC=1

set(KEYBOARD_REMAP_SOURCES
 keyboard_remap.c
 usb_descriptors.c
 # can use 'tinyusb_pico_pio_usb' library later when pico-sdk is updated
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
)

set(KEYBOARD_REMAP_LIBS pico_stdlib pico_unique_id pico_pio_usb tinyusb_device tinyusb_host)

function(keyboard_remap_add_target target_name with_cdc)
 add_executable(${target_name} ${KEYBOARD_REMAP_SOURCES})
 target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
 target_compile_options(${target_name} PRIVATE -Wall -Wextra)
 target_compile_definitions(${target_name} PRIVATE
   PIO_USB_USE_TINYUSB
   KEYBOARD_REMAP_WITH_CDC=${with_cdc}
 )
 target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
 target_link_libraries(${target_name} PRIVATE ${KEYBOARD_REMAP_LIBS})
 pico_add_extra_outputs(${target_name})
endfunction()

keyboard_remap_add_target(keyboard_remap 0)
keyboard_remap_add_target(keyboard_remap_debug 1)
