
#include "no_os_delay.h"
#include "no_os_uart.h"
#include "maxim_uart.h"
#include "maxim_stdio.h"

#include "no_os_spi.h"
#include "maxim_spi.h"
#include "adxl355.h"

#include <stdio.h>

int main()
{
	struct max_uart_init_param max_uart_extra_ip = {
		.flow = UART_FLOW_DIS
	};
	struct no_os_uart_init_param uart_ip = {
		.device_id = 0,
		.baud_rate = 57600,
		.size = NO_OS_UART_CS_8,
		.parity = NO_OS_UART_PAR_NO,
		.stop = NO_OS_UART_STOP_1_BIT,
		.extra = &max_uart_extra_ip,
	};
	struct max_spi_init_param adxl355_spi_extra_ip  = {
		.numSlaves = 1,
		.polarity = SPI_SS_POL_LOW
	};
	struct no_os_spi_init_param adxl355_spi_ip = {
		.device_id = 1,
		.max_speed_hz = 1000000,
		.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
		.mode = NO_OS_SPI_MODE_0,
		.platform_ops = &max_spi_ops,
		.chip_select = 1,
		.extra = &adxl355_spi_extra_ip,
	};
	struct adxl355_init_param adxl355_ip = {
		.comm_type = ADXL355_SPI_COMM,
		.comm_init = {.spi_init = adxl355_spi_ip},
		.dev_type = ID_ADXL355,
	};
	struct no_os_uart_desc *uart_desc;
	struct adxl355_dev *adxl355_desc;
	struct adxl355_frac_repr temp;
	uint16_t raw_temp;
	int ret;

	ret = no_os_uart_init(&uart_desc, &uart_ip);
	if (ret < 0)
		return ret;
	
	maxim_uart_stdio(uart_desc);

	ret = adxl355_init(&adxl355_desc, adxl355_ip);
	if (ret < 0)
		goto error_1;

	ret = adxl355_set_op_mode(adxl355_desc, ADXL355_MEAS_TEMP_ON_DRDY_OFF);
	if (ret < 0)
		goto error_2;

	while(1) {
		ret = adxl355_get_temp(adxl355_desc, &temp);
		if (ret < 0)
			goto error_2;

		printf("Current temperature is %d.%09u millidegress Celsius\n\r",
			(int)temp.integer, (abs)(temp.fractional));

		no_os_mdelay(1000);
	}

	return 0;

error_2:
	adxl355_remove(adxl355_desc);
error_1:
	no_os_uart_remove(uart_desc);
}
