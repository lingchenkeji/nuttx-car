############################################################################
# apps/myprojects/modules/rfid/Makefile
#
# RFID Module Makefile
#
############################################################################

include $(APPDIR)/Make.defs

# RFID Module Source Files - select based on chip choice

ifeq ($(CONFIG_MYPROJECTS_MODULES_RFID_CV520),y)
CSRCS = cv520_api.c
else
CSRCS = rfid_api.c
endif

# Add include paths

CFLAGS += -I$(APPDIR)/myprojects/include
CFLAGS += -I$(TOPDIR)/arch/arm/src/stm32

# NSH Command (if enabled)

ifeq ($(CONFIG_MYPROJECTS_MODULES_RFID_NSH),y)
MAINSRC = rfid_main.c
PROGNAME = rfid
PRIORITY = 100
STACKSIZE = 1536
MODULE = $(CONFIG_MYPROJECTS_MODULES_RFID)
endif

include $(APPDIR)/Application.mk
