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
