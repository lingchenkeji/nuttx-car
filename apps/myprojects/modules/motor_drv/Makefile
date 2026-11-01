############################################################################
# apps/myprojects/modules/motor_drv/Makefile
#
# DRV8212 Motor Driver Module Makefile
#
############################################################################

include $(APPDIR)/Make.defs

# DRV8212 module source files

CSRCS = drv8212_api.c

# Add include paths

CFLAGS += -I$(APPDIR)/myprojects/include
CFLAGS += -I$(TOPDIR)/arch/arm/src/stm32

# NSH command (if enabled)

ifeq ($(CONFIG_MYPROJECTS_MODULES_MOTOR_DRV_NSH),y)
MAINSRC = drv8212_main.c
PROGNAME = drv8212
PRIORITY = 100
STACKSIZE = 1536
MODULE = $(CONFIG_MYPROJECTS_MODULES_MOTOR_DRV)
endif

include $(APPDIR)/Application.mk
