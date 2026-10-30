# SD sector dumper: sends requested sectors over the console for merging into a ddrescue image.

TOCK_USERLAND_BASE_DIR = ../../..

PACKAGE_NAME = sd_dump

C_SRCS := $(wildcard *.c)

# The 32 KiB read buffer is static; the stack only holds small locals.
STACK_SIZE = 4096
APP_HEAP_SIZE = 1024

include $(TOCK_USERLAND_BASE_DIR)/AppMakefile.mk
