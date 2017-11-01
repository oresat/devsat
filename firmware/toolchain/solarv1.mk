
# Directories for SOLARV1 configuration

SOLARV1_TOP           = ../..
SOLARV1_TOOLCHAIN     = $(SOLARV1_TOP)/toolchain/
OPENOCD_DIR           = $(SOLARV1_TOOLCHAIN)/openocd_f0

# make rules
SOLARV1_RULES         = $(OPENOCD_DIR)/openocd_stlinkv2.mk

ifeq ($(shell git diff-index --quiet HEAD $(SOLARV1_TOP)/src ; echo $$?), 1)
INDEX_DIRTY = _INDEX_DIRTY
else
INDEX_DIRTY =
endif

VERSION_PREFIX = git-

SOLARV1_VERSION = "\"$(VERSION_PREFIX)`git rev-parse --short HEAD`$(INDEX_DIRTY)\""

