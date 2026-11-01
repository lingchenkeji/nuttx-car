############################################################################
# apps/myprojects/modules/ws2812/Makefile
#
# WS2812B RGB LED Module Makefile
#
############################################################################

include $(APPDIR)/Make.defs

MODULE = $(CONFIG_MYPROJECTS_MODULES_WS2812)

# WS2812B Module Source Files

CSRCS = ws2812_api.c

# Add include paths

CFLAGS += -I$(APPDIR)/myprojects/include
CFLAGS += -I$(TOPDIR)/arch/arm/src/stm32

include $(APPDIR)/Application.mk
