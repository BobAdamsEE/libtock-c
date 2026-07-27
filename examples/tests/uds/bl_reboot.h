/* The kernel's bootloader-reboot driver.
 *
 * A vendor capsule (`boards/sam_v71_xult/src/bl_reboot.rs`) that only answers
 * a process holding a specific ShortId. The kernel grants that identity to a
 * process whose signature credential was accepted, so an unsigned build of
 * this app still loads and runs, still speaks UDS, and is simply refused the
 * two services that restart the board.
 */
#pragma once

#include <stdbool.h>

#include <libtock/tock.h>

// Is the driver present at all?
bool bl_reboot_exists(void);

// Does this process hold the privilege? Answered without attempting to use it.
bool bl_reboot_permitted(void);

// Restart into the bootloader, which then owns the CAN bus for reprogramming.
// Does not return unless the request was refused.
returncode_t bl_reboot_to_bootloader(void);

// Restart into the kernel. Does not return unless the request was refused.
returncode_t bl_reboot_restart(void);
