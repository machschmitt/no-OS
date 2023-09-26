
#include "no_os_util.h"
#include "iio_adxl355.h"
#include "iio_app.h"

#define DATA_BUFFER_SIZE 400

uint8_t iio_data_buffer[DATA_BUFFER_SIZE*3*sizeof(int)];

int main()
{
	struct adxl355_iio_dev *adxl355_iio_desc;
	struct adxl355_iio_dev_init_param adxl355_iio_ip;
	struct iio_data_buffer accel_buff = {
		.buff = (void *)iio_data_buffer,
		.size = DATA_BUFFER_SIZE*3*sizeof(int)
	};
	int ret;

	ret = adxl355_iio_init(&adxl355_iio_desc, &adxl355_iio_ip);
	if (ret < 0)
		return ret;

	struct iio_app_device iio_devices[] = {
		{
			.name = "adxl355",
			.dev = adxl355_iio_desc,
			.dev_descriptor = adxl355_iio_desc->iio_dev,
			.read_buff = &accel_buff,
		}
	};

	return iio_app_run(iio_devices, NO_OS_ARRAY_SIZE(iio_devices));
}
