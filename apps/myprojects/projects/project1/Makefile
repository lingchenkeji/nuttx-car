############################################################################
# apps/myprojects/projects/project1/Makefile
#
# Project 1 Makefile
#
############################################################################

include $(APPDIR)/Make.defs

# Include paths for module APIs and STM32 HAL

CFLAGS += -I$(APPDIR)/myprojects/include
CFLAGS += -I$(TOPDIR)/arch/arm/src/stm32

# WS2812 module sources (linked directly since module has no NSH entry)

CSRCS += $(APPDIR)/myprojects/modules/ws2812/ws2812_api.c
VPATH += :$(APPDIR)/myprojects/modules/ws2812

# Main application

ifeq ($(CONFIG_MYPROJECTS_PROJECT1_APP),y)
MAINSRC   = project1_main.c
PROGNAME  = project1
PRIORITY  = 100
STACKSIZE = 2560
MODULE    = $(CONFIG_MYPROJECTS_PROJECT1)
endif

include $(APPDIR)/Application.mk
