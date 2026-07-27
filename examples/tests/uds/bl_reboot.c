#include "bl_reboot.h"

#define BL_REBOOT_DRIVER_NUM 0x90001

// Command 0 is the driver-existence check, which `driver_exists` issues.
#define CMD_PERMITTED 1
#define CMD_REBOOT    2
#define CMD_RESTART   3

bool bl_reboot_exists(void) {
  return driver_exists(BL_REBOOT_DRIVER_NUM);
}

bool bl_reboot_permitted(void) {
  syscall_return_t r = command(BL_REBOOT_DRIVER_NUM, CMD_PERMITTED, 0, 0);
  return r.type == TOCK_SYSCALL_SUCCESS_U32 && r.data[0] != 0;
}

returncode_t bl_reboot_to_bootloader(void) {
  return tock_command_return_novalue_to_returncode(
    command(BL_REBOOT_DRIVER_NUM, CMD_REBOOT, 0, 0));
}

returncode_t bl_reboot_restart(void) {
  return tock_command_return_novalue_to_returncode(
    command(BL_REBOOT_DRIVER_NUM, CMD_RESTART, 0, 0));
}
