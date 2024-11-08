/***************************************************************************//**
 *   @file   iio_ad4170.c
 *   @brief  Implementation of IIO AD4170 Driver.
 *   @author Marcelo Schmitt (marcelo.schmitt@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <errno.h>
#include "iio_ad4170.h"
#include "ad4170.h"
#include "no_os_alloc.h"
#include "no_os_error.h"

//#define AD4170_IIO_CH_ATTR_RW(_name, _priv)	\
//	{					\
//		.name = _name,			\
//		.priv = _priv,			\
//		.show = ad4170_ch_attr_show,	\
//		.store = ad4170_ch_attr_store,	\
//	}
//
//#define AD4170_IIO_CH_ATTR_RO(_name, _priv)	\
//	{					\
//		.name = _name,			\
//		.priv = _priv,			\
//		.show = ad4170_ch_attr_show,	\
//		.store = NULL,			\
//	}
//
//#define AD4170_IIO_CH_ATTR_WO(_name, _priv)	\
//	{					\
//		.name = _name,			\
//		.priv = _priv,			\
//		.show = NULL,			\
//		.store = ad4170_ch_attr_store,	\
//	}
//
//#define AD4170_IIO_CH_ATTR_READ_RAW(_name, _priv)	\
//	{						\
//		.name = _name,				\
//		.priv = _priv,				\
//		.show = ad4170_read_raw,		\
//		.store = NULL,				\
//	}
//
//#define AD4170_IIO_CH_ATTR_READ_SCALE(_name, _priv)	\
//	{						\
//		.name = _name,				\
//		.priv = _priv,				\
//		.show = ad4170_read_scale,		\
//		.store = NULL,				\
//	}

enum ad4170_attr_priv {
	AD4170_RAW,
	AD4170_SCALE,
};

/******************************************************************************/
/************************ Variable Declarations *******************************/
/******************************************************************************/
static struct iio_attribute ad4170_iio_attrs[] = {
	{
		.name = "raw",
		.show = ad4170_iio_read_raw,
	},
	{
		.name   = "scale",
		.show   = ad4170_iio_read_scale,
	},
	END_ATTRIBUTES_ARRAY
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

static int32_t ad4170_iio_reg_read(struct ad4170_iio_device *dev,
				   uint32_t reg, uint32_t *readval)
{
	uint16_t temp;
	int ret;

	ret = ad4170_reg_read(dev->dev, (uint8_t)reg, &temp, false);
	if (!ret)
		*readval = temp;

	return ret;
}

static int32_t ad4170_iio_reg_write(struct ad4170_iio_device *dev,
				    uint32_t reg, uint32_t writeval)
{
	return ad4170_reg_write(dev->dev, (uint8_t)reg, (uint16_t)writeval,
				false);
}

static struct iio_attribute ad4170_debug_attrs[] = {
	AD4170_IIO_CH_ATTR_RO("pout", AD4170_POUT),
	AD4170_IIO_CH_ATTR_RO("pin", AD4170_PIN),
	AD4170_IIO_CH_ATTR_RO("efficiency", AD4170_EFF),
	AD4170_IIO_CH_ATTR_RO("charging_stage", AD4170_CHARGING_STAGE),
	AD4170_IIO_CH_ATTR_RO("charging_status", AD4170_CHARGING_STATUS),
	AD4170_IIO_CH_ATTR_RW("enable", AD4170_ENABLE),
	AD4170_IIO_CH_ATTR_WO("reset", AD4170_RESTART),
	AD4170_IIO_CH_ATTR_RW("scratch", AD4170_SCRATCH),
	AD4170_IIO_CH_ATTR_RO("serial_id", AD4170_SERIAL_ID),
	END_ATTRIBUTES_ARRAY
};

static int ad4170_read_raw(void *ddev, char *buf, uint32_t len,
			   const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad4170_iio_device *dev = ddev;
	int ret;
	int32_t val;
	uint32_t uval;



	switch (priv) {
	case AD4170_RAW:
		ret = ad4170_set_channel_en(dev->dev, uint16_t channel_en)
		if (ret)
			return ret;

		ret = ad4170_get_ch_data(dev->dev, channel->cha, &val);
		if (ret)
			return ret;

		val /= 100;

		return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
	case AD4170_SCALE:
		ret = ad4170_read_iout(dev->dev, &uval);
		if (ret)
			return ret;

		////bipolar?
		//*data = no_os_sign_extend32(reg, );

		val = uval;

		return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
	default:
		return -EOPNOTSUPP;
	}
}

static int ad4170_iio_read_scale(void *ddev, char *buf, uint32_t len,
				 const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t val;

	switch (priv) {
	case AD4170_RAW:
		val = 100;
		return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
	case AD4170_SCALE:
		val = 1;

		return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
	default:
		return -EOPNOTSUPP;
	}
}

static struct scan_type ad4170_signed_scan_type = {
	.sign = 's',
	.realbits = 16,
	.storagebits = 16,
	.shift = 0,
	.is_big_endian = false,
};

static struct scan_type ad4170_unsigned_scan_type = {
	.sign = 'u',
	.realbits = 16,
	.storagebits = 16,
	.shift = 0,
	.is_big_endian = false,
};

static struct iio_attribute ad4170_temp_ch_attrs[] = {
	AD4170_IIO_CH_ATTR_READ_RAW("raw", AD4170_TBAT),
	AD4170_IIO_CH_ATTR_READ_SCALE("scale", AD4170_TBAT),
	END_ATTRIBUTES_ARRAY
};

static struct iio_attribute ad4170_out_current_ch_attrs[] = {
	AD4170_IIO_CH_ATTR_READ_RAW("raw", AD4170_IOUT),
	AD4170_IIO_CH_ATTR_READ_SCALE("scale", AD4170_IOUT),
	END_ATTRIBUTES_ARRAY
};

static struct iio_attribute ad4170_in_current_ch_attrs[] = {
	AD4170_IIO_CH_ATTR_READ_RAW("supply_raw", AD4170_IIN),
	AD4170_IIO_CH_ATTR_READ_SCALE("supply_scale", AD4170_IIN),
	END_ATTRIBUTES_ARRAY
};

static struct iio_attribute ad4170_out_voltage_ch_attrs[] = {
	AD4170_IIO_CH_ATTR_READ_RAW("raw", AD4170_VBAT),
	AD4170_IIO_CH_ATTR_READ_SCALE("scale", AD4170_VBAT),
	END_ATTRIBUTES_ARRAY
};

static struct iio_attribute ad4170_in_voltage_ch_attrs[] = {
	AD4170_IIO_CH_ATTR_READ_RAW("supply_raw", AD4170_VIN),
	AD4170_IIO_CH_ATTR_READ_SCALE("supply_scale", AD4170_VIN),
	END_ATTRIBUTES_ARRAY
};

static struct iio_channel ad4170_channels[] = {
	{
		.name = "temp",
		.ch_type = IIO_TEMP,
		.channel = 0,
		.address = 0,
		.scan_index = 0,
		.scan_type = &ad4170_signed_scan_type,
		.attributes = ad4170_temp_ch_attrs,
		.ch_out = false,
	}, {
		.name = "current",
		.ch_type = IIO_CURRENT,
		.channel = 0,
		.address = 0,
		.scan_index = 0,
		.scan_type = &ad4170_unsigned_scan_type,
		.attributes = ad4170_out_current_ch_attrs,
		.ch_out = true,
	}, {
		.name = "current",
		.ch_type = IIO_CURRENT,
		.channel = 0,
		.address = 0,
		.scan_index = 0,
		.scan_type = &ad4170_unsigned_scan_type,
		.attributes = ad4170_in_current_ch_attrs,
		.ch_out = false,
	}, {
		.name = "voltage",
		.ch_type = IIO_VOLTAGE,
		.channel = 0,
		.address = 0,
		.scan_index = 0,
		.scan_type = &ad4170_unsigned_scan_type,
		.attributes = ad4170_out_voltage_ch_attrs,
		.ch_out = true,
	}, {
		.name = "voltage",
		.ch_type = IIO_VOLTAGE,
		.channel = 0,
		.address = 0,
		.scan_index = 0,
		.scan_type = &ad4170_unsigned_scan_type,
		.attributes = ad4170_in_voltage_ch_attrs,
		.ch_out = false,
	}
};

static struct iio_device ad4170_iio_dev = {
	.num_ch = NO_OS_ARRAY_SIZE(ad4170_channels),
	.channels = ad4170_channels,
	.debug_reg_read = (int32_t (*)()) ad4170_iio_reg_read,
	.debug_reg_write = (int32_t (*)()) ad4170_iio_reg_write,
	.debug_attributes = ad4170_debug_attrs,
};

/**
 * @brief Initializes the AD4170 IIO driver
 * @param iio_device - The iio device structure.
 * @param iio_init_param - Parameters for the initialization of iio_dev
 * @return 0 in case of success, errno errors otherwise
 */
int ad4170_iio_init(struct ad4170_iio_device **iio_device,
		    struct ad4170_iio_init_param *iio_init_param)
{
	struct ad4170_iio_device *iio_device_temp;
	int ret;

	if (!iio_init_param || !iio_init_param->init_param)
		return -EINVAL;

	iio_device_temp = no_os_calloc(1, sizeof(*iio_device_temp));
	if (!iio_device_temp)
		return -ENOMEM;

	ret = ad4170_init(&iio_device_temp->dev, iio_init_param->init_param);
	if (ret)
		goto free_dev;

	iio_device_temp->iio_dev = &ad4170_iio_dev;

	*iio_device = iio_device_temp;

	return 0;

free_dev:
	no_os_free(iio_device_temp);

	return ret;
}

/**
 * @brief Free resources allocated by the init function
 * @param iio_device - The iio device structure.
 * @return 0 in case of success, errno errors otherwise
 */
int ad4170_iio_remove(struct ad4170_iio_device *iio_device)
{
	ad4170_remove(iio_device->dev);
	no_os_free(iio_device);
	return 0;
}
