# Interactive SD command shell over the console UART. Needs a kernel built
# with the sd_write feature for anything beyond reads.

TOCK_USERLAND_BASE_DIR = ../../..

PACKAGE_NAME = sd_shell

C_SRCS := $(wildcard *.c)

# The 32 KiB data buffer is static; the stack only holds small locals.
STACK_SIZE = 4096
APP_HEAP_SIZE = 1024

include $(TOCK_USERLAND_BASE_DIR)/AppMakefile.mk
