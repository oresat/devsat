
# Directories for NUCLEO64 configuration

NUCLEO64_TOP           = ../..
NUCLEO64_TOOLCHAIN     = $(NUCLEO64_TOP)/toolchain
OPENOCD_DIR            = $(NUCLEO64_TOOLCHAIN)/openocd_f4

# make rules
NUCLEO64_RULES         = $(OPENOCD_DIR)/openocd_stlinkv2-1.mk

ifeq ($(shell git diff-index --quiet HEAD $(NUCLEO64_TOP)/src ; echo $$?), 1)
INDEX_DIRTY = _INDEX_DIRTY
else
INDEX_DIRTY =
endif

VERSION_PREFIX = git-

NUCLEO64_VERSION = "\"$(VERSION_PREFIX)`git rev-parse --short HEAD`$(INDEX_DIRTY)\""

