// Generated register defines for soc_ctrl

// Copyright information found in source file:
// Copyright lowRISC contributors.

// Licensing information found in source file:
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef _SOC_CTRL_REG_DEFS_
#define _SOC_CTRL_REG_DEFS_

#ifdef __cplusplus
extern "C" {
#endif
// Register width
#define SOC_CTRL_PARAM_REG_WIDTH 32

// Exit Valid - Used to write exit valid bit
#define SOC_CTRL_EXIT_VALID_REG_OFFSET 0x0
#define SOC_CTRL_EXIT_VALID_EXIT_VALID_BIT 0

// Exit Value - Used to write exit value register
#define SOC_CTRL_EXIT_VALUE_REG_OFFSET 0x4

// Boot Select Value - Used to decide whether to boot from JTAG or FLASH
#define SOC_CTRL_BOOT_SELECT_REG_OFFSET 0x8
#define SOC_CTRL_BOOT_SELECT_BOOT_SELECT_BIT 0

// Boot Exit Loop Value - Set externally (e.g. JTAG, TESTBENCH, or another
// MASTER) to make the CPU jump to the main function entry
#define SOC_CTRL_BOOT_EXIT_LOOP_REG_OFFSET 0xc
#define SOC_CTRL_BOOT_EXIT_LOOP_BOOT_EXIT_LOOP_BIT 0

// Boot Address Value - Used in the boot rom or power-on-reset functions
#define SOC_CTRL_BOOT_ADDRESS_REG_OFFSET 0x10

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // _SOC_CTRL_REG_DEFS_
// End generated register defines for soc_ctrl