/***************************************************************************//**
 *   @file   ad7091r8.c
 *   @brief  Implementation of AD7091R-8 Driver
 *   @author Marcelo Schmitt (marcelo.schmitt@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <stdlib.h>
#include <errno.h>

#include "ad7091r8.h"
#include "no_os_alloc.h"
#include "no_os_delay.h"
#include "no_os_util.h"

/**
 * Pull the CONVST line up then down to signal to the start of a read/write
 * operation.
 * @param dev - The device structure.
 */
uint16_t ad7091r8_pulse_convst(struct ad7091r8_dev *dev)
{
	/* convst pulse width must be 10 ns minimum, 500 ns maximum */
	ret |= no_os_gpio_set_value(dev->gpio_convst, NO_OS_GPIO_LOW);

	ret |= no_os_gpio_set_value(dev->gpio_convst, NO_OS_GPIO_HIGH);

	return ret;
}

/**
 * Write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_spi_reg_write(struct ad7091r8_dev *dev,
			       uint8_t reg_addr,
			       uint16_t reg_data)
{
	uint8_t buf[2];
	int32_t ret;

	if (!dev || !reg_data)
		return -EINVAL;

	ret = ad7091r8_pulse_convst(dev);
	if (ret < 0)
		return ret;

	/* Assumes controller host machine is little-endian */
	buf[0] = AD7091R8_REG_WRITE(reg_addr);
	buf[0] |= reg_data & 0x03;
	buf[1] = reg_data & 0xFF;

	return no_os_spi_write(dev->spi_desc, buf, 2);
}

/**
 * Read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_spi_reg_read(struct ad7091r8_dev *dev,
			      uint8_t reg_addr,
			      uint16_t *reg_data)
{
	uint8_t buf[2];
	int32_t ret;

	if (!dev || !reg_data)
		return -EINVAL;

	ret = ad7091r8_pulse_convst(dev);
	if (ret < 0)
		return ret;

	/* Assumes controller host machine is little-endian */
	buf[0] = AD7091R8_REG_READ(reg_addr);
	buf[1] = 0x00;

	ret = no_os_spi_write_and_read(dev->spi_desc, buf, buf, 2);
	if (ret < 0)
		return ret;

	/* Big-endian to little-endian */
	*reg_data = (buf[0] << 8) | buf[1];

	return ret;
}

/**
 * SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_spi_write_mask(struct ad7091r8_dev *dev,
				uint8_t reg_addr,
				uint16_t mask,
				uint16_t data)
{
	uint16_t reg_data;
	int32_t ret;

	if (!dev)
		return -EINVAL;

	ret = ad7091r8_spi_reg_read(dev, reg_addr, &reg_data);
	if (ret < 0)
		return ret;

	reg_data &= ~mask;
	reg_data |= data;

	return ad7091r8_spi_reg_write(dev, reg_addr, reg_data);
}

/**
 * @brief Set device sleep mode.
 *
 * @param dev - The device structure.
 * @param mode - The device sleep mode to set.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_set_sleep_mode(struct ad7091r8_dev *dev,
				enum ad7091r8_sleep_mode mode);
{

	if (!dev)
		return -EINVAL;

	return ad7091r8_spi_write_mask(dev, AD7091R8_REG_CONF,
				       REG_CONF_SLEEP_MODE_MASK,
				       REG_CONF_SLEEP_MODE(mode));
}

/**
 * Set output port value.
 * @param dev - The device structure.
 * @param port - Port number.
 * @param value - Value.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_set_port(struct ad7091r8_dev *dev, enum ad7091r8_port port,
			  bool value)
{
	uint16_t mask;
	uint16_t val;

	if (!dev)
		return -EINVAL;

	switch (port) {
	case AD7091R8_GPO0:
		mask = REG_CONF_GPO0_MASK;
		val = REG_CONF_GPO0(value);
		break;
	case AD7091R8_GPO1:
		mask = REG_CONF_GPO1_MASK;
		val = REG_CONF_GPO1(value);
		break;
	default:
		return -EINVAL;
	}

	return ad7091r8_spi_write_mask(dev, AD7091R8_REG_CONF, mask, val);
}

/**
 * Set GPO0 mode.
 * @param dev - The device structure.
 * @param mode - GPO0 new mode.
 * @param is_cmos - 0: GPO0 is open drain
 * 		  - 1: GPO0 is CMOS.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_set_gpo0_mode(struct ad7091r8_dev *dev,
			       enum ad7091r8_gpo0_mode mode, bool is_cmos)
{
	uint16_t value;

	if (!dev)
		return -EINVAL;

	switch (mode) {
	case AD7091R8_GPO0_ENABLED:
		value = 0;
		break;
	case AD7091R8_GPO0_ALERT:
		value = REG_CONF_GPO0_ALERT(1);
		break;
	case AD7091R8_GPO0_BUSY:
		value = REG_CONF_GPO0_BUSY(1) | REG_CONF_GPO0_ALERT(1);
		break;
	default:
		return -EINVAL;
		break;
	}
	value |= REG_CONF_GPO0_DRIVE_TYPE(is_cmos);

	return ad7091r8_spi_write_mask(dev, AD7091R8_REG_CONF,
				       REG_CONF_GPO0_MODE_MASK, value);
}

/**
 * Set high limit, low limit, hysteresis.
 * Device accepts 9 bits provided by the user and sets them as the MSBs.
 * The 3 LSBs of the internal 12-bit registers are set either to 000 or 111.
 * Round user input according to each case.
 * @param dev - The device structure.
 * @param limit - Limit.
 * @param channel - Channel.
 * @param value - Value.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_set_limit(struct ad7091r8_dev *dev,
			   enum ad7091r8_limit limit,
			   uint8_t channel,
			   uint16_t value)
{
	uint16_t reg;

	if (!dev)
		return -EINVAL;

	switch (limit) {
	case AD7091R8_LOW_LIMIT:
		reg = AD7091R8_REG_CH_LOW_LIMIT(channel);
		value += (value & 0x04) ? 8 : 0;
		break;
	case AD7091R8_HIGH_LIMIT:
		reg = AD7091R8_REG_CH_HIGH_LIMIT(channel);
		value -= (value & 0x04) ? 8 : 0;
		break;
	case AD7091R8_HYSTERESIS:
		reg = AD7091R8_REG_CH_HYSTERESIS(channel);
		value += (value & 0x04) ? 8 : 0;
		break;
	default:
		return -EINVAL;
	}
	value = value >> 3;

	return ad7091r8_spi_reg_write(dev, reg, REG_RESULT_CONV_DATA(value));
}

/**
 * Get alert.
 * @param dev - The device structure.
 * @param channel - Channel.
 * @param alert - Alert type.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_get_alert(struct ad7091r8_dev *dev, uint8_t channel,
			   enum ad7091r8_alert_type *alert)
{
	int32_t ret;
	uint16_t data;

	if (!dev || !alert)
		return -EINVAL;

	if (channel >= AD7091R_NUM_CHANNELS(dev->device_id))
		return -EINVAL;

	ret = ad7091r8_spi_reg_read(dev, AD7091R8_REG_ALERT, &data);
	if (ret)
		return ret;

	*alert = REG_ALERT_MASK(data, channel);

	return 0;
}

/**
 * Get high limit, low limit, hysteresis.
 * The 3 LSBs of the internal 12-bit registers are set either to 000 or 111.
 * Adjust limit data to reflect actual limit in use by the device.
 * @param dev - The device structure.
 * @param limit - Limit.
 * @param channel - Channel.
 * @param value - Value.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_get_limit(struct ad7091r8_dev *dev,
			   enum ad7091r8_limit limit,
			   uint8_t channel,
			   uint16_t *value)
{
	int32_t ret;
	uint16_t reg, data;

	if (!dev || !value)
		return -EINVAL;

	switch (limit) {
	case AD7091R8_LOW_LIMIT:
		reg = AD7091R8_REG_CH_LOW_LIMIT(channel);
		break;
	case AD7091R8_HIGH_LIMIT:
		reg = AD7091R8_REG_CH_HIGH_LIMIT(channel);
		*value = 0x07;
		break;
	case AD7091R8_HYSTERESIS:
		reg = AD7091R8_REG_CH_HYSTERESIS(channel);
		break;
	default:
		return -EINVAL;
	}

	ret = ad7091r8_spi_reg_read(dev, reg, &data);
	if (ret)
		return ret;

	*value |= REG_CHAN_LIMIT_DATA(data) << 3;

	return 0;
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and the initial Values for
 *        AD7092R-8 Board.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return ret - The result of the initialization procedure.
 *               Example: -1 - SPI peripheral was not initialized or the
 *                             device is not present.
 *                         0 - SPI peripheral was initialized and the
 *                             device is present.
*******************************************************************************/
int8_t ad7091r8_init(struct ad7091r8_dev **device,
		     struct ad7091r8_init_param init_param)
{
	struct ad7091r8_dev *dev;
	uint8_t status = 0;
	//uint8_t tmp_val = 0xFF;

	if (!device || !init_param)
		return -EINVAL;

	dev = (struct ad7091r8_dev *)no_os_malloc(sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	ret = no_os_spi_init(&dev->spi_desc, &init_param.spi_init);
	if (ret < 0)
		return ret;

	dev->gpio_reset = NULL;
	dev->gpio_alert = NULL;
	dev->device_id = init_param.device_id;

	ret = no_os_gpio_get(&dev->gpio_convst, init_param->gpio_convst);
	if (ret < 0)
		return ret;

	ret = no_os_gpio_direction_output(&dev->gpio_convst, NO_OS_GPIO_HIGH);
	if (ret < 0)
		return ret;

	no_os_gpio_get_optional(&dev->gpio_reset, init_param->gpio_reset);
	if (dev->gpio_reset != NULL) {
		ret = no_os_gpio_direction_output(&dev->gpio_reset,
						  NO_OS_GPIO_HIGH);
		if (ret < 0)
			return ret;
		ad7091r8_reset(dev, false);
	}

	no_os_gpio_get_optional(&dev->gpio_alert, init_param->gpio_alert);

	/* Device powers-up in normal mode */

	*device = dev;

	return ret;
}

/**
 * @brief Initiate a software reset or hardware reset through the RESET pin.
 * @param dev - ad7091r8_dev device handler.
 * @param is_software - true: Software reset
 * 		      - false: hardware reset
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_reset(struct ad7091r8_dev *dev, bool is_software)
{
	int32_t ret;

	if (!dev)
		return -EINVAL;

	if (is_software) {
		/* Bit is cleared automatically after reset */
		return ad7091r8_spi_write_mask(dev, AD7091R8_REG_CONF,
					       REG_CONF_RESET_MASK,
					       REG_CONF_RESET(1));
	} else {
		/* reset pulse delay upon power up, at least 50 ns */
		no_os_udelay(5);
		ret = no_os_gpio_set_value(dev->gpio_resetn, NO_OS_GPIO_LOW);
		if (ret < 0)
			return ret;

		/* reset pulse width, at least 10 ns */
		no_os_udelay(1);
		return no_os_gpio_set_value(dev->gpio_resetn, NO_OS_GPIO_HIGH);
	}
}

/***************************************************************************//**
 * @brief Free the resources allocated by ad7091r8_init().
 *
 * @param dev - The device structure.
 *
 * @return ret - The result of the remove procedure.
*******************************************************************************/
int32_t ad7091r8_remove(struct ad7091r8_dev *dev)
{
	int32_t ret;

	ret = no_os_spi_remove(dev->spi_desc);

	no_os_free(dev);

	return ret;
}

/**
 * Set device channel.
 * @param dev - The device structure.
 * @param channel - Channel.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r8_set_channel(struct ad7091r8_dev *dev, uint8_t channel)
{
	uint16_t foo;
	int32_t ret;

	if (!dev)
		return -EINVAL;

	if (channel >= AD7091R_NUM_CHANNELS(dev->device_id))
		return -EINVAL;

	ret = ad7091r8_spi_reg_write(dev, AD7091R8_REG_CHANNEL,
				     NO_OS_BIT(channel));
	if (ret)
		return ret;

	/* There is a latency of one conversion before the channel conversion
	 * sequence is updated */
	return ad7091r8_spi_reg_read(dev, AD7091R8_REG_RESULT, &foo);
}

/***************************************************************************//**
 * @brief Initiates one conversion and reads back the result. During this
 *        process the device runs in normal mode and operates without the busy
 *        indicator.
 *
 * @param dev - The device structure.
 *
 * @return conversionResult - 12bit conversion result.
*******************************************************************************/
/**
 * Read one sample.
 * @param dev - The device structure.
 * @param channel - Channel.
 * @param read_val - Value.
 * @return 0 in case of success, negative error code otherwise.
 */
uint16_t ad7091r8_read_one(struct ad7091r8_dev *dev, uint8_t chan,
			   uint16_t *read_val)
{
	if (!dev || !read_val)
		return -EINVAL;

	if (channel >= AD7091R_NUM_CHANNELS(dev->device_id))
		return -EINVAL;

	ret = ad7091r8_set_channel(dev, channel);
	if (ret)
		return ret;

	ret = ad7091r8_spi_reg_read(dev, AD7091R8_REG_RESULT, &val);
	if (ret)
		return ret;

	if (REG_RESULT_CH_ID(val) != channel)
		return -1;

	*read_val = REG_RESULT_CONV_DATA(val);

	return 0;
}

/**
 * Read the next channel set in the channel sequencer (channel register).
 * @param dev - The device structure.
 * @param read_val - Value.
 * @return 0 in case of success, negative error code otherwise.
 */
uint16_t ad7091r8_sequenced_read(struct ad7091r8_dev *dev, uint16_t *read_val)
{

	uint8_t buf[2];
	int32_t ret;

	if (!dev)
		return -EINVAL;

	ret = ad7091r8_pulse_convst(dev);
	if (ret < 0)
		return ret;

	/* Assumes controller host machine is little-endian */
	buf[0] = 0xf8; /* NOP command */
	buf[1] = 0x00;

	ret = no_os_spi_write_and_read(dev->spi_desc, buf, buf, 2);
	if (ret < 0)
		return ret;

	/* Big-endian to little-endian */
	*read_val = (buf[0] << 8) | buf[1];

	return 0;
}
