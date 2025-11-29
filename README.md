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

# startup Code
```
/*
 * FILE: startup.s
 *
 * DESCRIPTION:
 * Minimal reset/startup stub for ESP32-C3.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 14, 2025
 * UPDATE DATE: November 14, 2025
 */

.include "inc/registers.inc"

/**
 * Initialize the .text.init section.
 * The .text.init section contains init executable code.
 */
.section .text.init

/**
 * @brief   Reset / startup entry point.
 *
 * @details Minimal reset/startup handler used after 2nd stage
 *          bootloader. This stub sets up the stack, disables the
 *          watchdogs, and transfers control to the `main` application. 
 *          It intentionally remains small to minimize boot-time overhead.
 *
 * @param   None
 * @retval  None
 */
.global _start
.type _start, %function
_start:
  jal   wdt_disable                              # call wtd_disable
  jal   main                                     # call main
  j     .                                        # jump infinite loop if main returns
.size _start, .-_start
```

# systimer Code 
```
/*
 * FILE: systimer.s
 *
 * DESCRIPTION:
 * ESP32-C3 Systimer Delay Functions.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 14, 2025
 * UPDATE DATE: November 14, 2025
 */

.include "inc/registers.inc"

/**
 * Initialize the .text.init section.
 * The .text.init section contains executable code.
 */
.section .text

/**
 * @brief   Get the current systimer systick value.
 *
 * @details Reads the systimer unit 0 value register after updating.
 *
 * @param   None
 * @retval  a0: current systick value
 */
.type systimer_systick_get, %function
systimer_systick_get:
  li    t0, SYSTIMER_UNIT0_OP_REG                # read UNIT0 value to registers 
  li    t1, (1<<30)                              # SYSTIMER_TIMER_UNIT0_UPDATE
  sw    t1, 0(t0)                                # write update to SYSTIMER_UNIT0_OP_REG
  li    t0, SYSTIMER_UNIT0_VALUE_LO_REG          # UNIT0 value, low 32 bits 
  lw    a0, 0(t0)                                # load systick value
  ret                                            # return
.size systimer_systick_get, .-systimer_systick_get

/**
 * @brief   Delay for a specified number of systicks.
 *
 * @details Implements a busy-wait delay using systimer.
 *
 * @param   a0: delay in systicks
 * @retval  None
 */
.type delay_systicks, %function
delay_systicks:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  sw    s0, 8(sp)                                # save s0 (callee-saved)
  sw    a0, 4(sp)                                # save delay value
  jal   systimer_systick_get                     # get current systick
  mv    s0, a0                                   # store time #1 in s0
  lw    t1, 4(sp)                                # load delay value
  add   s0, s0, t1                               # compute expiry = time#1 + delay
.delay_systicks_delay_loop:
  jal   systimer_systick_get                     # get current systick
  blt   a0, s0, .delay_systicks_delay_loop       # loop if not elapsed
  lw    s0, 8(sp)                                # restore s0
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size delay_systicks, .-delay_systicks

/**
 * @brief   Delay for a specified number of microseconds.
 *
 * @details Converts microseconds to systicks and calls delay_systicks.
 *
 * @param   a0: delay in µs
 * @retval  None
 */
.global delay_us
.type delay_us, %function
delay_us:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  li    t0, 16                                   # 16MHz clock, 16 ticks per µs
  mul   a0, a0, t0                               # convert µs to systicks
  jal   delay_systicks                           # call delay function
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size delay_us, .-delay_us

/**
 * @brief   Delay for a specified number of milliseconds.
 *
 * @details Converts milliseconds to systicks and calls delay_systicks.
 *
 * @param   a0: delay in ms
 * @retval  None
 */
.global delay_ms
.type delay_ms, %function
delay_ms:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  li    t0, 16000                                # 16MHz clock, 16000 ticks per ms
  mul   a0, a0, t0                               # convert µs to systicks
  jal   delay_systicks                           # call delay function
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size delay_ms, .-delay_ms
```

# wdt Code
```
/*
 * FILE: wdt.s
 *
 * DESCRIPTION:
 * ESP32-C3 Bare-Metal Watchdog Timer Utilities.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 15, 2025
 * UPDATE DATE: November 15, 2025
 */

.include "inc/registers.inc"

.equ WDT_WRITE_PROTECT, 0x50D83AA1
.equ SWD_WRITE_PROTECT, 0x8F1D312A

/**
 * Initialize the .text.init section.
 * The .text.init section contains executable code.
 */
.section .text

/**
 * @brief   Feed the watchdog timer.
 *
 * @param   None
 * @retval  None
 */
.type wdt_feed, %function
wdt_feed:
  li    t0, TIMG0_WDTFEED_REG                    # load wdt feed register address
  addi  t1, t1, 1                                # increment feed counter
  sw    t1, 0(t0)                                # write feed value
  ret                                            # return
.size wdt_feed, .-wdt_feed

/**
 * @brief   Disable all watchdog timers.
 *
 * @param   None
 * @retval  None
 */
.global wdt_disable
.type wdt_disable, %function
wdt_disable:
  li    t0, TIMG0_WDTWPROTECT_REG                # timg0 write protect register
  li    t1, WDT_WRITE_PROTECT                    # load write protect key
  sw    t1, (t0)                                 # unlock write protection
  li    t0, TIMG0_WDTCONFIG0_REG                 # timg0 config register
  li    t1, 0                                    # load disable value
  sw    t1, (t0)                                 # disable timg0 watchdog
  li    t0, TIMG1_WDTWPROTECT_REG                # timg1 write protect register
  li    t1, WDT_WRITE_PROTECT                    # load write protect key
  sw    t1, (t0)                                 # unlock write protection
  li    t0, TIMG1_WDTCONFIG0_REG                 # timg1 config register
  li    t1, 0                                    # load disable value
  sw    t1, (t0)                                 # disable timg1 watchdog
  li    t0, RTC_CNTL_WDTWPROTECT_REG             # rtc write protect register
  li    t1, WDT_WRITE_PROTECT                    # load write protect key
  sw    t1, (t0)                                 # unlock write protection
  li    t0, RTC_CNTL_WDTCONFIG0_REG              # rtc config register
  li    t1, 0                                    # load disable value
  sw    t1, (t0)                                 # disable rtc watchdog
  li    t0, RTC_CNTL_SWD_WPROTECT_REG            # swd write protect register
  li    t1, SWD_WRITE_PROTECT                    # load write protect key
  sw    t1, (t0)                                 # unlock write protection
  li    t0, RTC_CNTL_SWD_CONF_REG                # swd config register
  li    t1, ((1<<31) | 0x4B00000)                # enable with auto feed
  sw    t1, (t0)                                 # write swd config
  ret                                            # return
.size wdt_disable, .-wdt_disable
```

# gpio Code
```
/*
 * FILE: gpio.s
 *
 * DESCRIPTION:
 * ESP32-C3 Bare-Metal GPIO Utilities.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 14, 2025
 * UPDATE DATE: November 14, 2025
 */

.include "inc/registers.inc"

/**
 * Initialize the .text.init section.
 * The .text.init section contains executable code.
 */
.section .text

/**
 * @brief   Enable GPIO pin input by configuring IO_MUX.
 *
 * @param   a0: pin number
 * @retval  None
 */
.global gpio_input_enable
.type gpio_input_enable, %function
gpio_input_enable:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  sw    s0, 8(sp)                                # save s0
  mv    s0, a0                                   # save pin number in s0
  li    t0, GPIO_ENABLE_REG                      # load GPIO_ENABLE register addr
  lw    t1, 0(t0)                                # read current enable bits
  li    t2, 1                                    # constant 1
  sll   t2, t2, s0                               # shift to pin bit
  not   t2, t2                                   # invert mask
  and   t1, t1, t2                               # clear the pin bit
  sw    t1, 0(t0)                                # write back enable register
  li    t0, IO_MUX_GPIO0_REG                     # load IO_MUX base address
  slli  t1, s0, 2                                # compute pin offset
  add   t0, t0, t1                               # addr = base + offset
  lw    t1, 0(t0)                                # read current IO_MUX value
  li    t2, ~(0x7 << 12)                         # mask to clear MCU_SEL field
  and   t1, t1, t2                               # clear MCU_SEL bits 12-14
  li    t2, (1 << 12)                            # MCU_SEL = 1 (GPIO function)
  or    t1, t1, t2                               # set MCU_SEL = 1
  li    t2, (1 << 9)                             # FUN_IE bit 9 (input enable)
  or    t1, t1, t2                               # set FUN_IE = 1
  li    t2, (1 << 8)                             # FUN_WPU bit 8 (weak pull-up)
  or    t1, t1, t2                               # set FUN_WPU = 1
  li    t2, ~(1 << 7)                            # mask to clear FUN_WPD bit 7
  and   t1, t1, t2                               # clear FUN_WPD (disable pull-down)
  sw    t1, 0(t0)                                # write to IO_MUX register
  lw    s0, 8(sp)                                # restore s0
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size gpio_input_enable, .-gpio_input_enable

/**
 * @brief   Enable GPIO pin output in GPIO_ENABLE_REG.
 *
 * @param   a0: pin number
 * @retval  None
 */
.global gpio_output_enable
.type gpio_output_enable, %function
gpio_output_enable:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  sw    s0, 8(sp)                                # save s0
  mv    s0, a0                                   # save pin number in s0
  li    t0, IO_MUX_GPIO0_REG                     # load IO_MUX base address
  slli  t1, s0, 2                                # compute pin offset
  add   t0, t0, t1                               # addr = base + offset
  lw    t1, 0(t0)                                # read current IO_MUX value
  li    t2, ~(0x7 << 12)                         # mask to clear MCU_SEL field
  and   t1, t1, t2                               # clear MCU_SEL bits 12-14
  li    t2, (1 << 12)                            # MCU_SEL = 1 (GPIO function)
  or    t1, t1, t2                               # set MCU_SEL = 1
  sw    t1, 0(t0)                                # write to IO_MUX register
  li    t0, GPIO_FUNC0_OUT_SEL_CFG_REG           # load GPIO matrix output func
  slli  t1, s0, 2                                # compute pin offset  
  add   t0, t0, t1                               # addr = base + offset
  li    t1, 0x80                                 # simple GPIO output signal
  sw    t1, 0(t0)                                # write func sel register
  li    t0, GPIO_ENABLE_REG                      # load GPIO_ENABLE register addr
  lw    t1, 0(t0)                                # read current enable bits
  li    t2, 1                                    # constant 1
  sll   t2, t2, s0                               # shift to pin bit
  or    t1, t1, t2                               # set the pin bit
  sw    t1, 0(t0)                                # write back enable register
  lw    s0, 8(sp)                                # restore s0
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size gpio_output_enable, .-gpio_output_enable

/**
 * @brief   Select output function for a GPIO pin.
 *
 * @param   a0: pin number
 * @param   a1: function value
 * @retval  None
 */
.global gpio_output_func_select
.type gpio_output_func_select, %function
gpio_output_func_select:
  li    t0, GPIO_FUNC0_OUT_SEL_CFG_REG           # load function select base
  slli  t1, a0, 2                                # compute pin offset
  add   t0, t0, t1                               # addr = base + offset
  sw    a1, 0(t0)                                # write function selection
  ret                                            # return
.size gpio_output_func_select, .-gpio_output_func_select

/**
 * @brief   Read GPIO pin level.
 *
 * @param   a0: pin number
 * @retval  a0: 0 or 1
 */
.global gpio_read
.type gpio_read, %function
gpio_read:
  mv    t2, a0                                   # save pin number
  li    t0, GPIO_IN_REG                          # load GPIO input register
  lw    t1, 0(t0)                                # read all inputs
  li    t3, 1                                    # constant 1
  sll   t3, t3, t2                               # mask = 1 << pin
  and   t1, t1, t3                               # isolate pin bit
  li    a0, 0                                    # assume low
  beq   t1, zero, .gpio_read_ret                 # if zero, return 0
  li    a0, 1                                    # else return 1
.gpio_read_ret:
  ret                                            # return
.size gpio_read, .-gpio_read

/**
 * @brief   Write GPIO pin (set or clear).
 *
 * @param   a0: pin number
 * @param   a1: value (0 clear, non-zero set)
 * @retval  None
 */
.global gpio_write
.type gpio_write, %function
gpio_write:
  mv    t2, a0                                   # save pin number
  li    t0, GPIO_OUT_W1TC_REG                    # assume clear
  beq   a1, zero, .gpio_write_exec               # if a1 == 0, use clear
  li    t0, GPIO_OUT_W1TS_REG                    # else use set
.gpio_write_exec:
  li    t1, 1                                    # constant 1
  sll   t1, t1, t2                               # shift to pin position
  sw    t1, 0(t0)                                # write to register
  ret                                            # return
.size gpio_write, .-gpio_write

/**
 * @brief   Toggle GPIO pin (read, then write inverted).
 *
 * @param   a0: pin number
 * @retval  None
 */
.global gpio_toggle
.type gpio_toggle, %function
gpio_toggle:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  li    t0, GPIO_OUT_REG                         # load GPIO output register addr
  lw    t1, 0(t0)                                # read current output bits
  li    t2, 1                                    # constant 1
  sll   t2, t2, a0                               # mask = 1 << pin
  and   t3, t1, t2                               # test bit
  li    a1, 0                                    # default write value
  bne   zero, t3, .gpio_toggle_write             # if bit set branch
  li    a1, 1                                    # set write value to 1
.gpio_toggle_write:
  call  gpio_write                               # call gpio_write to update pin
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size gpio_toggle, .-gpio_toggle
```

# button Code
```
/*
 * FILE: button.s
 *
 * DESCRIPTION:
 * ESP32-C3 Bare-Metal Button Driver for GPIO1 with Debouncing.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 29, 2025
 * UPDATE DATE: November 29, 2025
 */

.include "inc/registers.inc"

.equ DEBOUNCE_DELAY_MS, 50                       # debounce delay in milliseconds

/**
 * Initialize the .text.init section.
 * The .text.init section contains executable code.
 */
.section .text

/**
 * @brief   Initialize button on GPIO1 as input.
 *
 * @details Configures GPIO1 as input with pull-down resistor (active-high).
 *          Button connects GPIO1 to 3.3V when pressed.
 *          External 1kΩ pull-down keeps GPIO1 LOW when not pressed.
 *          Active-high: pressed = HIGH (1), released = LOW (0).
 *
 * @param   None
 * @retval  None
 */
.global button_init
.type button_init, %function
button_init:
  li    a0, 1                                    # pin number 1
  jal   gpio_input_enable                        # call gpio_input_enable
  ret                                            # return
.size button_init, .-button_init

/**
 * @brief   Read the button state without debouncing.
 *
 * @details Reads GPIO1 input level directly (active-high).
 *          Returns 1 when button pressed (HIGH), 0 when not pressed (LOW).
 *
 * @param   None
 * @retval  a0: button state (1 = pressed, 0 = not pressed)
 */
.global button_read
.type button_read, %function
button_read:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  li    a0, 1                                    # pin number 1
  jal   gpio_read                                # call gpio_read (returns pin state)
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size button_read, .-button_read

/**
 * @brief   Read the button state with debouncing.
 *
 * @details Implements software debouncing by waiting for stable state.
 *          Delays for DEBOUNCE_DELAY_MS after detecting state change.
 *
 * @param   None
 * @retval  a0: debounced button state (1 = pressed, 0 = not pressed)
 */
.global button_read_debounced
.type button_read_debounced, %function
button_read_debounced:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  sw    s0, 8(sp)                                # save s0 (callee-saved)
  jal   button_read                              # read initial button state
  mv    s0, a0                                   # store initial state in s0
  li    a0, DEBOUNCE_DELAY_MS                    # load debounce delay
  jal   delay_ms                                 # call delay_ms
  jal   button_read                              # read button state again
  mv    t0, a0                                   # move new state to t0
  bne   s0, t0, .button_read_debounced_unstable  # if states differ, unstable
  mv    a0, s0                                   # return stable state
  j     .button_read_debounced_ret               # jump to return
.button_read_debounced_unstable:
  li    a0, 0                                    # return 0 for unstable state
.button_read_debounced_ret:
  lw    s0, 8(sp)                                # restore s0
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size button_read_debounced, .-button_read_debounced

/**
 * @brief   Wait for button press event.
 *
 * @details Polls button until pressed state is detected.
 *          Uses debouncing to ensure stable press detection.
 *
 * @param   None
 * @retval  None
 */
.global button_wait_press
.type button_wait_press, %function
button_wait_press:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
.button_wait_press_loop:
  jal   button_read_debounced                    # read debounced button state
  beq   a0, zero, .button_wait_press_loop        # loop if not pressed
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size button_wait_press, .-button_wait_press

/**
 * @brief   Wait for button release event.
 *
 * @details Polls button until released state is detected.
 *          Uses debouncing to ensure stable release detection.
 *
 * @param   None
 * @retval  None
 */
.global button_wait_release
.type button_wait_release, %function
button_wait_release:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
.button_wait_release_loop:
  jal   button_read_debounced                    # read debounced button state
  bne   a0, zero, .button_wait_release_loop      # loop if still pressed
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size button_wait_release, .-button_wait_release

/**
 * @brief   Wait for button press and release cycle.
 *
 * @details Waits for complete button press-release cycle.
 *          Ensures button is pressed then released before returning.
 *
 * @param   None
 * @retval  None
 */
.global button_wait_click
.type button_wait_click, %function
button_wait_click:
  addi  sp, sp, -16                              # allocate stack space
  sw    ra, 0(sp)                                # save return address
  jal   button_wait_press                        # wait for button press
  jal   button_wait_release                      # wait for button release
  lw    ra, 0(sp)                                # restore return address
  addi  sp, sp, 16                               # deallocate stack space
  ret                                            # return
.size button_wait_click, .-button_wait_click
```

# main Code 
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
