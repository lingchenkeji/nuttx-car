############################################################################
# apps/myprojects/modules/cdc_svc/Makefile
############################################################################

include $(APPDIR)/Make.defs

CSRCS += cdc_service.c

CFLAGS += -I$(APPDIR)/myprojects/include
CFLAGS += -I$(TOPDIR)/arch/arm/src/stm32

ifeq ($(CONFIG_MYPROJECTS_MODULES_CDC_SVC_NSH),y)
MAINSRC = cdc_service_main.c
PROGNAME = p1ctl
PRIORITY = 100
STACKSIZE = 1536
MODULE = $(CONFIG_MYPROJECTS_MODULES_CDC_SVC)
endif

include $(APPDIR)/Application.mk

