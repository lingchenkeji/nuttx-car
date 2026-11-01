############################################################################
# apps/myprojects/modules/vtx/Makefile
#
# VTX316 Voice Synthesis Module Makefile
#
############################################################################

include $(APPDIR)/Make.defs

# VTX316 Module Source Files

CSRCS = vtx_api.c

# Add include paths

CFLAGS += -I$(APPDIR)/myprojects/include
CFLAGS += -I$(TOPDIR)/arch/arm/src/stm32

# NSH Command (if enabled)

ifeq ($(CONFIG_MYPROJECTS_MODULES_VTX_NSH),y)
MAINSRC = vtx_main.c
PROGNAME = vtx
PRIORITY = 100
STACKSIZE = 1536
MODULE = $(CONFIG_MYPROJECTS_MODULES_VTX)
endif

include $(APPDIR)/Application.mk
