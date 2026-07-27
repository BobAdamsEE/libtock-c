# Stand-in UDS app: tests the bootloader-reboot privilege gate.

TOCK_USERLAND_BASE_DIR = ../../..

# The package name matters: the board derives the privileged ShortId from it.
PACKAGE_NAME = uds

C_SRCS := $(wildcard *.c)

include $(TOCK_USERLAND_BASE_DIR)/AppMakefile.mk
