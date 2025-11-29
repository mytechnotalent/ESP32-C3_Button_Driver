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
