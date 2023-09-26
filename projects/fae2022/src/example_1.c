
#include "no_os_delay.h"
#include "no_os_uart.h"
#include "maxim_uart.h"

int main()
{
	struct max_uart_init_param max_uart_extra_ip = {
		.flow = UART_FLOW_DIS
	};
	struct no_os_uart_init_param uart_ip = {
		.device_id = 0,
		//.baud_rate = 57600,
		.baud_rate = 115200,
		//.baud_rate = 9600,
		.size = NO_OS_UART_CS_8,
		.parity = NO_OS_UART_PAR_NO,
		//.stop = NO_OS_UART_STOP_1_BIT,
		.stop = NO_OS_UART_STOP_2_BIT,
		.extra = &max_uart_extra_ip,
	};
	struct no_os_uart_desc *uart_desc;
	int ret;

	ret = no_os_uart_init(&uart_desc, &uart_ip);
	if (ret < 0)
		return ret;

	while(1) {
		ret = no_os_uart_write(uart_desc, "Hello World\n\r", 14);
		if (ret < 0)
			goto error;
		no_os_mdelay(1000);
	}

	return 0;

error:
	no_os_uart_remove(uart_desc);
}
