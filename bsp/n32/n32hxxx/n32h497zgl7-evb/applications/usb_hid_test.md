# N32H497ZGL7-EVB CherryUSB (FSDEV_NATION) HID Keyboard Test Guide

## 1. System Clock (HSE)

`RCC_Configuration()` in `board/Cube_Config/USER/src/n32h49x_cfg.c` uses **HSE as the PLL source**:

| Item | Value |
| --- | --- |
| Clock source | HSE (8 MHz crystal on the EVB) |
| PLL | HSE/2 x 60 = **240 MHz** system clock |
| USBFS clock | PLL/5 = **48 MHz** (`USBFS_SYSCLK_MHZ 240` -> DIV5 in `usb_glue_nation.c`) |
| APB1/APB2 | HCLK/2 = 120 MHz |

> Note: the N32H49x part macro is **`N32H49X`** (capital X, defined by board/SConscript).
> CherryUSB's `usb_glue_nation.c` / `usb_fsdev_reg.h` rely on it to detect the N32H4x series.

## 2. USB Hardware Connection

USBFS pins (default EVAL board mapping, PA11/PA12, alternate function AF10):

| Signal | Pin |
| --- | --- |
| USB_DM | PA11 |
| USB_DP | PA12 |

- Connect the board's USB port to the PC with a USB cable (this port acts as a USB device, it is not the debug port).
- The debug console is USART1: PA9 (TX), PA10 (RX), 115200-8-1-N.

## 3. Test Features

Current configuration template: `RT_CHERRYUSB_DEVICE_TEMPLATE_HID_KEYBOARD`
(application implementation: [usb_hid_keyboard.c](usb_hid_keyboard.c), standard HID Boot Keyboard protocol with both IN and OUT endpoints).

Device information: VID/PID = `0x19F5/0x5560`, product name "N32H497 EVB HID Keyboard".

| Endpoint | Address | Description |
| --- | --- | --- |
| IN | 0x81 | Keyboard input report, 8 bytes/packet: `[modifier, reserved, key0..key5]` |
| OUT | 0x02 | LED output report, 1 byte: bit0=NumLock, bit1=CapsLock, bit2=ScrollLock... |

### 3.1 On-board Key -> Host Typed Character

| On-board key | Pin | Typed character | HID keycode |
| --- | --- | --- | --- |
| WKUP | PA0 | `A` | 0x04 |
| KEY1 | PC13 | `B` | 0x05 |
| KEY2 | PA15 | `C` | 0x06 |
| KEY3 | PC8 | `D` | 0x07 |

- When a key is pressed (pull-up input, active low), the `key_scan` thread (10 ms polling + 10 ms debounce) detects the edge transition and sends a keyboard report; multi-key combinations are supported.
- An empty report (all zeros) is sent on release.
- Note: KEY3 (PC8) follows the EVB series convention; if it does not match your hardware, change `KEY_3_PIN` in `usb_hid_keyboard.c`.

### 3.2 Host Keyboard LED -> On-board LED

| Host key | On-board LED | Pin | LED report bit |
| --- | --- | --- | --- |
| CapsLock | LED2 | PB3 | bit1 |
| NumLock | LED3 | PA8 | bit0 |

- When CapsLock/NumLock is toggled on the host, Windows and other systems send a 1-byte LED output report over the OUT endpoint.
- The device parses bit0/bit1 in `usbd_hid_out_callback` and turns the corresponding LED on/off (active high).

## 4. Test Procedure

1. Build and flash (GCC: `scons`, or the Keil project), then reset the device.
2. Connect the board's USB port to the PC with a USB cable. The PC should enumerate a HID keyboard device
   ("N32H497 EVB HID Keyboard", listed under the "Keyboards" category in Device Manager).
3. Open Notepad or any text input window and press WKUP, KEY1, KEY2, KEY3 in turn:
   - Expected: `A`, `B`, `C`, `D` are typed in order at the cursor; holding multiple keys produces a combined input.
4. Press CapsLock on the host keyboard:
   - Expected: on-board LED2 (PB3) lights up; press CapsLock again -> LED2 turns off.
5. Press NumLock on the host keyboard:
   - Expected: on-board LED3 (PA8) lights up; press NumLock again -> LED3 turns off.
   - Note: on some laptops NumLock is triggered by an Fn combination; on desktop keyboards press NumLock directly.
6. (Optional) Serial console: the device side msh console prints `cherryusb fsdev hid keyboard test started.`.

## 5. Troubleshooting

| Symptom | Check |
| --- | --- |
| PC does not enumerate the device | Check the PA11/PA12 wiring; verify the USBFS 48 MHz clock (PLL/5 when HSE=8 MHz); verify that board/SConscript defines `N32H49X`; check the serial boot log |
| Key press produces no input | Check that the keys are pulled up (rt_pin_mode INPUT_PULLUP); PA15 requires the SWJ remap to SWD (the board already configures `GPIO_RMP_SWJ_SWD`) |
| LED does not respond | Verify that the host CapsLock/NumLock state actually changes (watch the host indicator); the LED is active high |

## 6. Code Locations

- Application: `applications/usb_hid_keyboard.c` (key scanning + bidirectional HID keyboard)
- Board configuration: `board/ports/usb_config.h` (CherryUSB configuration, including `CONFIG_USBDEV_FSDEV_PMA_ACCESS 2`)
- Driver: `components/drivers/usb/cherryusb/port/fsdev/usb_dc_fsdev.c` (DCD),
  `usb_glue_nation.c` (N32H4x clock/GPIO/interrupt), `usb_fsdev_reg.h` (register remapping)
