############################################################################
# apps/myprojects/modules/func_led/Makefile
#
# Functional LED Module Makefile
#
############################################################################

include $(APPDIR)/Make.defs

# Functional LED Module Source Files

CSRCS = func_led_api.c

# Add include paths

CFLAGS += -I$(APPDIR)/myprojects/include
CFLAGS += -I$(TOPDIR)/arch/arm/src/stm32

# NSH Command (if enabled)

ifeq ($(CONFIG_MYPROJECTS_MODULES_FUNC_LED_NSH),y)
MAINSRC = func_led_main.c
PROGNAME = func_led
PRIORITY = 100
STACKSIZE = 1536
MODULE = $(CONFIG_MYPROJECTS_MODULES_FUNC_LED)
endif

include $(APPDIR)/Application.mk
