# Makefile for CAN loopback test application

# Specify this directory relative to the current application.
TOCK_USERLAND_BASE_DIR = ../../..

# Which files to compile.
C_SRCS := $(wildcard *.c)

CFLAGS += -Wno-unused-result

# NOTE: this app requires -mno-unaligned-access (Tock maps process memory as
# Strongly-ordered, where unaligned accesses always fault). That now lives in
# CPPFLAGS_cortex-m in Configuration.mk so it covers libtock/libtocksync too.

# Include userland master makefile. Contains rules and flags for actually
# building the application.
include $(TOCK_USERLAND_BASE_DIR)/AppMakefile.mk
