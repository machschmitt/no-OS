
SRCS += $(PROJECT)/src/example_$(EXAMPLE_NUMBER).c

SRCS += $(PROJECT)/src/iio_adxl355.c
INCS += $(PROJECT)/src/iio_adxl355.h

TINYIIOD=y
SRC_DIRS += $(NO-OS)/iio/iio_app
INCS += $(PROJECT)/src/parameters.h

#	$(INCLUDE)/no_os_alloc.h  was added
#	$(INCLUDE)/no_os_mutex.h \

INCS += $(INCLUDE)/no_os_delay.h      \
	$(INCLUDE)/no_os_alloc.h \
	$(INCLUDE)/no_os_mutex.h \
	$(INCLUDE)/no_os_error.h      \
	$(INCLUDE)/no_os_gpio.h       \
	$(INCLUDE)/no_os_i2c.h        \
	$(INCLUDE)/no_os_print_log.h  \
	$(INCLUDE)/no_os_spi.h        \
	$(INCLUDE)/no_os_irq.h        \
	$(INCLUDE)/no_os_rtc.h        \
	$(INCLUDE)/no_os_list.h       \
	$(INCLUDE)/no_os_timer.h      \
	$(INCLUDE)/no_os_uart.h       \
	$(INCLUDE)/no_os_lf256fifo.h  \
	$(INCLUDE)/no_os_util.h       \
	$(INCLUDE)/no_os_units.h

SRCS += $(DRIVERS)/api/no_os_gpio.c      \
	$(DRIVERS)/api/no_os_i2c.c       \
	$(NO-OS)/util/no_os_lf256fifo.c  \
	$(DRIVERS)/api/no_os_irq.c       \
	$(DRIVERS)/api/no_os_spi.c       \
	$(DRIVERS)/api/no_os_timer.c     \
	$(NO-OS)/util/no_os_list.c       \
		$(NO-OS)/util/no_os_alloc.c \
		$(NO-OS)/util/no_os_mutex.c \
		$(DRIVERS)/api/no_os_uart.c \
	$(NO-OS)/util/no_os_util.c

#projects/fae2022/src.mk
# /include/no_os_alloc.h
#		$(NO-OS)/util/no_os_alloc.c \ was added
#		$(NO-OS)/util/no_os_mutex.c \
#		$(DRIVERS)/api/no_os_uart.c \

INCS += $(PLATFORM_DRIVERS)/maxim_delay.h     \
        $(PLATFORM_DRIVERS)/maxim_gpio.h      \
        $(PLATFORM_DRIVERS)/maxim_spi.h       \
        $(PLATFORM_DRIVERS)/maxim_gpio_irq.h  \
        $(PLATFORM_DRIVERS)/maxim_irq.h       \
        $(PLATFORM_DRIVERS)/maxim_rtc.h       \
        $(PLATFORM_DRIVERS)/maxim_uart.h      \
        $(PLATFORM_DRIVERS)/maxim_uart_stdio.h

#        $(PLATFORM_DRIVERS)/maxim_stdio.h

SRCS += $(PLATFORM_DRIVERS)/maxim_delay.c     \
        $(PLATFORM_DRIVERS)/maxim_gpio.c      \
        $(PLATFORM_DRIVERS)/maxim_spi.c       \
        $(PLATFORM_DRIVERS)/maxim_rtc.c       \
        $(PLATFORM_DRIVERS)/maxim_gpio_irq.c  \
        $(PLATFORM_DRIVERS)/maxim_irq.c       \
        $(PLATFORM_DRIVERS)/maxim_uart.c      \
        $(PLATFORM_DRIVERS)/maxim_uart_stdio.c

#        $(PLATFORM_DRIVERS)/maxim_stdio.c

INCS += $(DRIVERS)/accel/adxl355/adxl355.h
SRCS += $(DRIVERS)/accel/adxl355/adxl355.c
