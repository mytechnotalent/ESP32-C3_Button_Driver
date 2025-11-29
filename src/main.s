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
