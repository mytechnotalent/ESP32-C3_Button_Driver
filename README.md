<img src="https://github.com/mytechnotalent/ESP32-C3_Button_Driver/blob/main/ESP32-C3_Button_Driver.png?raw=true">

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)
### VIDEO PROMO [HERE](https://www.youtube.com/watch?v=aD7X9sXirF8)

<br>

# ESP32-C3 Button Driver
An ESP32-C3 button driver written entirely in RISC-V Assembler.

<br>

# Install ESP Toolchain
## Windows Installer [HERE](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/windows-setup.html)
## Linux and macOS Installer [HERE](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/linux-macos-setup.html)

<br>

# Hardware
## ESP32-C3 Super Mini [BUY](https://www.amazon.com/Teyleten-Robot-Development-Supermini-Bluetooth/dp/B0D47G24W3)
## USB-C to USB Cable [BUY](https://www.amazon.com/USB-Cable-10Gbps-Transfer-Controller/dp/B09WKCT26M)
## Complete Component Kit for Raspberry Pi [BUY](https://www.pishop.us/product/complete-component-kit-for-raspberry-pi)
## 10pc 25v 1000uF Capacitor [BUY](https://www.amazon.com/Cionyce-Capacitor-Electrolytic-CapacitorsMicrowave/dp/B0B63CCQ2N?th=1)
### 10% PiShop DISCOUNT CODE - KVPE_HS320548_10PC

<br>

# Circuit Diagram
```
3.3V -------- [Button] -------- GPIO1 -------- [1kΩ] -------- GND
                                                                  
GPIO0 -------- [220Ω] -------- [LED+] -------- [LED-] -------- GND
```

## Circuit Details
- **Button**: Connects GPIO1 to 3.3V when pressed (active-high)
- **1kΩ Pull-down Resistor**: Keeps GPIO1 LOW when button not pressed
- **LED**: Mirrors button state - ON when pressed, OFF when released
- **220Ω Resistor**: Current limiting resistor for LED protection

<br>

# Functionality
- LED mirrors button state in real-time
- Button pressed → LED ON (GPIO1 HIGH → GPIO0 HIGH)
- Button released → LED OFF (GPIO1 LOW → GPIO0 LOW)
- Simple direct state transfer without debouncing or toggling

<br>

# main.s Code
```
/*
 * FILE: main.s
 *
 * DESCRIPTION:
 * ESP32-C3 Bare-Metal Button-Controlled LED Application.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 14, 2025
 * UPDATE DATE: November 29, 2025
 */

.include "inc/registers.inc"

/**
 * Initialize the .text.init section.
 * The .text.init section contains executable code.
 */
.section .text

/**
 * @brief   Main application entry point.
 *
 * @details Button-controlled LED using gpio_read and gpio_write functions.
 *          GPIO1 input (button) controls GPIO0 output (LED).
 *          Active-high: button pressed = LED ON.
 *
 * @param   None
 * @retval  None
 */
.global main
.type main, %function
main:
  # Initialize GPIO0 as output
  li    a0, 0                                    # pin number 0
  jal   gpio_output_enable                       # enable GPIO0 output
  
  # Initialize GPIO1 as input
  li    a0, 1                                    # pin number 1
  jal   gpio_input_enable                        # enable GPIO1 input
  
.loop:
  # Read GPIO1 (button)
  li    a0, 1                                    # pin number 1
  jal   gpio_read                                # read button state (0 or 1)
  
  # Write result to GPIO0 (LED)
  mv    a1, a0                                   # move result to a1
  li    a0, 0                                    # pin number 0
  jal   gpio_write                               # write to LED
  
  j     .loop                                    # loop forever
.size main, .-main
```

<br>

# License
[Apache License 2.0](https://github.com/mytechnotalent/ESP32-C3_Button_Driver/blob/main/LICENSE)
