# SD card inspector: prints card registers and reports live and deleted files.

TOCK_USERLAND_BASE_DIR = ../../..

PACKAGE_NAME = sd_inspect

C_SRCS := $(wildcard *.c)

# Directory walking recurses (bounded at MAX_DEPTH) and keeps several
# 512-byte sector buffers live at once.
STACK_SIZE = 8192
APP_HEAP_SIZE = 1024

include $(TOCK_USERLAND_BASE_DIR)/AppMakefile.mk
