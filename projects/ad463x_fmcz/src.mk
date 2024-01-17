################################################################################
#									       #
#     Shared variables:							       #
#	- PROJECT							       #
#	- DRIVERS							       #
#	- INCLUDE							       #
#	- PLATFORM_DRIVERS						       #
#	- NO-OS								       #
#									       #
################################################################################

SRCS := $(PROJECT)/src/ad463x_fmc.c
SRCS += $(DRIVERS)/api/no_os_spi.c \
	$(DRIVERS)/api/no_os_pwm.c \
	$(DRIVERS)/api/no_os_gpio.c \
	$(DRIVERS)/api/no_os_irq.c \
	$(DRIVERS)/api/no_os_uart.c \
	$(DRIVERS)/adc/ad463x/ad463x.c \
	$(DRIVERS)/axi_core/axi_dmac/axi_dmac.c \
	$(DRIVERS)/axi_core/clk_axi_clkgen/clk_axi_clkgen.c \
	$(DRIVERS)/axi_core/axi_pwmgen/axi_pwm.c \
	$(DRIVERS)/axi_core/spi_engine/spi_engine.c \
	$(NO-OS)/util/no_os_util.c \
	$(NO-OS)/util/no_os_alloc.c \
	$(NO-OS)/util/no_os_mutex.c
SRCS +=	$(PLATFORM_DRIVERS)/xilinx_axi_io.c \
	$(PLATFORM_DRIVERS)/xilinx_gpio.c \
	$(PLATFORM_DRIVERS)/xilinx_spi.c \
	$(PLATFORM_DRIVERS)/xilinx_delay.c
#SRCS +=	drivers/platform/xilinx/xilinx_axi_io.c \
#	drivers/platform/xilinx/xilinx_gpio.c \
#	drivers/platform/xilinx/xilinx_spi.c \
#	drivers/platform/xilinx/xilinx_delay.c

#ifeq (y,$(strip $(IIOD)))
#LIBRARIES += iio
	#drivers/platform/xilinx/xilinx_uart.c
#SRC_DIRS += $(NO-OS)/iio/iio_app
SRCS += $(PLATFORM_DRIVERS)/$(PLATFORM)_uart.c \
	$(NO-OS)/util/no_os_lf256fifo.c \
	$(PLATFORM_DRIVERS)/$(PLATFORM)_irq.c \
	$(DRIVERS)/api/no_os_irq.c \
	$(DRIVERS)/adc/ad463x/iio_ad463x.c \
	$(NO-OS)/util/no_os_fifo.c \
	$(NO-OS)/util/no_os_list.c
#endif
INCS += $(PROJECT)/src/parameters.h
INCS += $(DRIVERS)/adc/ad463x/ad463x.h \
	$(DRIVERS)/axi_core/axi_dmac/axi_dmac.h \
	$(DRIVERS)/axi_core/clk_axi_clkgen/clk_axi_clkgen.h \
	$(DRIVERS)/axi_core/axi_pwmgen/axi_pwm_extra.h \
	$(DRIVERS)/axi_core/spi_engine/spi_engine.h \
	$(DRIVERS)/axi_core/spi_engine/spi_engine_private.h

INCS +=	$(PLATFORM_DRIVERS)/$(PLATFORM)_spi.h \
	$(PLATFORM_DRIVERS)/$(PLATFORM)_irq.h \
	$(PLATFORM_DRIVERS)/$(PLATFORM)_uart.h \
	$(PLATFORM_DRIVERS)/$(PLATFORM)_gpio.h
#INCS +=	drivers/platform/xilinx/xilinx_spi.h \
#	drivers/platform/xilinx/xilinx_irq.h \
#	drivers/platform/xilinx/xilinx_uart.h
INCS +=	$(INCLUDE)/no_os_axi_io.h \
	$(INCLUDE)/no_os_spi.h \
	$(INCLUDE)/no_os_pwm.h \
	$(INCLUDE)/no_os_gpio.h \
	$(INCLUDE)/no_os_error.h \
	$(INCLUDE)/no_os_delay.h \
	$(INCLUDE)/no_os_irq.h \
	$(INCLUDE)/no_os_uart.h \
	$(INCLUDE)/no_os_lf256fifo.h \
	$(INCLUDE)/no_os_util.h \
	$(INCLUDE)/no_os_units.h \
	$(INCLUDE)/no_os_print_log.h \
	$(INCLUDE)/no_os_alloc.h \
	$(INCLUDE)/no_os_mutex.h
#ifeq (y,$(strip $(IIOD)))
INCS += $(DRIVERS)/adc/ad463x/iio_ad463x.h \
	$(INCLUDE)/no_os_fifo.h \
	$(INCLUDE)/no_os_list.h
#endif
ifeq (y,$(strip $(ADAQ4224)))
CFLAGS += -DADAQ4224_DEV=1
else ifeq (y,$(strip $(AD4030)))
CFLAGS += -DAD4030_DEV=1
else
CFLAGS += -DAD463X_DEV=1
endif
