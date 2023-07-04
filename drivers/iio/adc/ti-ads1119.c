// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADS1119 - Texas Instruments Analog-to-Digital Converter
 *
 * Copyright (C) 2023 PHYTEC Messtechnik GmbH
 *
 * This driver is partly based on ti-ads1015.c and ti-ads124s08.c from Kernel 5.4
 *
 *
 * IIO driver for ADS1119 ADC 7-bit I2C slave address:
 * ---------------------------------------
 *   A1	    |     A0	|   I2C-ADDRESS
 * ---------------------------------------
 *  DGND	|    DGND	|    0x40
 *  DGND	|    DVDD	|    0x41
 *  DGND	|    SDA	|    0x42
 *  DGND	|    SCL	|    0x43
 *  DVDD	|    DGND	|    0x44
 *  DVDD	|    DVDD	|    0x45
 *  DVDD	|    SDA	|    0x46
 *  DVDD	|    SCL	|    0x47
 *  SDA		|    DGND	|    0x48
 *  SDA		|    DVDD	|    0x49
 *  SDA		|    SDA	|    0x4A
 *  SDA		|    SCL	|    0x4B
 *  SCL		|    DGND	|    0x4C
 *  SCL		|    DVDD	|    0x4D
 *  SCL		|    SDA	|    0x4E
 *  SCL		|    SCL	|    0x4F
 * --------------------------------------
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>

#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>

/* driver name */
#define ADS1119_DRV_NAME "ads1119"

/* number of device channels */
#define ADS1119_CHANNELS	8

/* register address */
#define ADS1119_CFG_REG		0x00
#define ADS1119_STAT_REG	0x01

/* cfg register shifts */
#define ADS1119_CFG_MUX_SHIFT	5
#define ADS1119_CFG_GAIN_SHIFT	4
#define ADS1119_CFG_RATE_SHIFT	2
#define ADS1119_CFG_CONV_MODE_SHIFT	1
#define ADS1119_CFG_VREF_SHIFT	0

/* stat register shifts */
#define ADS1119_STAT_DRDY_SHIFT	7
#define ADS1119_STAT_ID_SHIFT	0

/* cfg register masks */
#define ADS1119_CFG_MUX_MASK	GENMASK(7, 5)
#define ADS1119_CFG_GAIN_MASK	BIT(4)
#define ADS1119_CFG_RATE_MASK	GENMASK(3, 2)
#define ADS1119_CFG_CONV_MODE_MASK	BIT(1)
#define ADS1119_CFG_VREF_MASK	BIT(0)

/* stat register masks */
#define ADS1119_STAT_DRDY_MASK	BIT(7)
#define ADS1119_STAT_ID_MASK	GENMASK(6, 0)

/* device commands */
#define ADS1119_CMD_PWRDWN	0x02
#define ADS1119_CMD_RESET	0x06
#define ADS1119_CMD_START	0x08
#define ADS1119_CMD_RDATA	0x10
#define ADS1119_CMD_RREG_CFG	0x20
#define ADS1119_CMD_RREG_STAT	0x24
#define ADS1119_CMD_WREG	0x40

/* channel definition */
#define ADS1119_AIN0_1		0
#define ADS1119_AIN2_3		1
#define ADS1119_AIN1_2		2
#define ADS1119_AIN0_G		3
#define ADS1119_AIN1_G		4
#define ADS1119_AIN2_G		5
#define ADS1119_AIN3_G		6
#define ADS1119_AIN_OFFSET_CAL	7

/* config definition */
#define ADS1119_GAIN_1		0
#define ADS1119_GAIN_4		1
#define ADS1119_DR_20		0
#define ADS1119_DR_90		1
#define ADS1119_DR_330		2
#define ADS1119_DR_1000		3
#define ADS1119_CM_SINGLE	0
#define ADS1119_CM_CONT		1
#define ADS1119_VREF_INT	0 /* 2.048V */
#define ADS1119_VREF_EXT	1 /* REFP / REFN */
#define ADS1119_DRDY_NEW_DATA	1
#define ADS1119_DRDY_OLD_DATA	0

/* device operating modes */
#define ADS1119_CONTINUOUS	ADS1119_CM_CONT
#define ADS1119_SINGLESHOT	ADS1119_CM_SINGLE

/* driver defaults */
#define ADS1119_SLEEP_DELAY_MS		2000
#define ADS1119_DEFAULT_GAIN		ADS1119_GAIN_1
#define ADS1119_DEFAULT_SAMPLE_RATE	ADS1119_DR_20
#define ADS1119_DEFAULT_CHAN		ADS1119_AIN0_1
#define ADS1119_DEFAULT_REF		ADS1119_VREF_INT
#define ADS1119_DEFAULT_REF_VOLTAGE	ADS1119_VOLTAGE_INTREF

/* refvoltage defines */
#define ADS1119_VOLTAGE_INTREF		2048000
#define ADS1119_VOLTAGE_EXTREF		5500000

/* useful CALCULATION MAKROS */
#define BITCOUNT		(1<<15)
#define BITCOUNT_ROUND		((1<<15)-1)
#define DATAMASK		GENMASK(15, 0)

enum chip_ids {
	ADS1119 = 0,
};

enum ads1119_channels {
	ADS1119_AIN0_AIN1 = 0,
	ADS1119_AIN2_AIN3,
	ADS1119_AIN1_AIN2,
	ADS1119_AIN0,
	ADS1119_AIN1,
	ADS1119_AIN2,
	ADS1119_AIN3,
	ADS1119_OFFSET_CAL,
};

static const unsigned int ads1119_gain[] = {
	1, 4
};

static const unsigned int ads1119_sample_rate[] = {
	20, 90, 330, 1000
};

static const unsigned int ads1119_ref_source[] = {
	0, 1
};

static int ads1119_ref_voltage[] = {
	ADS1119_VOLTAGE_EXTREF, (ADS1119_VOLTAGE_EXTREF>>2),
	ADS1119_VOLTAGE_INTREF, (ADS1119_VOLTAGE_INTREF>>2)
};

#define ADS1119_V_CHAN(_chan, _addr) {	\
	.type = IIO_VOLTAGE,		\
	.indexed = 1,			\
	.address = _addr,		\
	.channel = _chan,		\
	.info_mask_separate =	BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_HARDWAREGAIN) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ) |		\
				BIT(IIO_CHAN_INFO_REFSOURCE) |		\
				BIT(IIO_CHAN_INFO_REFVOLTAGE),		\
	.scan_index = _addr,		\
	.scan_type = {			\
		.sign = 's',		\
		.realbits = 16,		\
		.storagebits = 16,	\
		.endianness = IIO_BE,	\
	},				\
	.datasheet_name = "AIN"#_chan,	\
}

#define ADS1119_V_DIFF_CHAN(_chan, _chan2, _addr) {	\
	.type = IIO_VOLTAGE,				\
	.differential = 1,				\
	.indexed = 1,					\
	.address = _addr,				\
	.channel = _chan,				\
	.channel2 = _chan2,				\
	.info_mask_separate =	BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_HARDWAREGAIN) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ) |		\
				BIT(IIO_CHAN_INFO_REFSOURCE) |		\
				BIT(IIO_CHAN_INFO_REFVOLTAGE),		\
	.scan_index = _addr,				\
	.scan_type = {					\
		.sign = 's',				\
		.realbits = 16,				\
		.storagebits = 16,			\
		.endianness = IIO_BE,			\
	},						\
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,	\
}

struct ads1119_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	const int   *data_rate;
	const int   data_rate_len;
	const int   *gain;
	const int   gain_len;
	const int   *ref_source;
	const int   ref_source_len;
	unsigned int *ref_voltage;
	const int   ref_voltage_len;
};

struct ads1119_channel_conf {
	unsigned int data_rate;
	unsigned int gain;
	bool ref_source;
	unsigned int ref_voltage;
};

struct ads1119_private {
	struct i2c_client *client;
	struct mutex lock;
	struct ads1119_channel_conf channel_conf[ADS1119_CHANNELS];
	bool conv_mode;
	const struct ads1119_chip_info *chip;
};

static const struct iio_chan_spec ads1119_channels[] = {
	ADS1119_V_DIFF_CHAN(0, 1, ADS1119_AIN0_AIN1),
	ADS1119_V_DIFF_CHAN(2, 3, ADS1119_AIN2_AIN3),
	ADS1119_V_DIFF_CHAN(1, 2, ADS1119_AIN1_AIN2),
	ADS1119_V_CHAN(0, ADS1119_AIN0),
	ADS1119_V_CHAN(1, ADS1119_AIN1),
	ADS1119_V_CHAN(2, ADS1119_AIN2),
	ADS1119_V_CHAN(3, ADS1119_AIN3),
	ADS1119_V_DIFF_CHAN(4, 4, ADS1119_OFFSET_CAL),
};

static const struct ads1119_chip_info ads1119_chip_info_tbl[] = {
	[ADS1119] = {
		.channels = ads1119_channels,
		.num_channels = ARRAY_SIZE(ads1119_channels),
		.data_rate = ads1119_sample_rate,
		.data_rate_len = ARRAY_SIZE(ads1119_sample_rate),
		.gain       = ads1119_gain,
		.gain_len   = ARRAY_SIZE(ads1119_gain),
		.ref_source = ads1119_ref_source,
		.ref_source_len = ARRAY_SIZE(ads1119_ref_source),
		.ref_voltage = ads1119_ref_voltage,
		.ref_voltage_len = ARRAY_SIZE(ads1119_ref_voltage),
	},
};

static int ads1119_write_cmd(struct iio_dev *indio_dev, u8 command)
{
	struct ads1119_private *priv = iio_priv(indio_dev);
	struct i2c_client *client = priv->client;

	return i2c_smbus_write_byte(client, command);
}

static int ads1119_write_reg(struct iio_dev *indio_dev, u8 data)
{
	struct ads1119_private *priv = iio_priv(indio_dev);
	struct i2c_client *client = priv->client;

	return i2c_smbus_write_byte_data(client, ADS1119_CMD_WREG, data);
}

static int ads1119_read_reg(struct iio_dev *indio_dev, u8 reg)
{
	struct ads1119_private *priv = iio_priv(indio_dev);
	struct i2c_client *client = priv->client;
	int ret;

	ret = ads1119_write_cmd(indio_dev, reg);
	if (ret)
		return ret;

	return i2c_smbus_read_byte(client);
}

static int ads1119_read(struct iio_dev *indio_dev)
{
	struct ads1119_private *priv = iio_priv(indio_dev);
	struct i2c_client *client = priv->client;
	s32 val1 = 0;
	s32 val2 = 0;
	int ret;

	ret = i2c_smbus_write_byte(client, ADS1119_CMD_RDATA);
	if (ret)
		return ret;

	val1 = i2c_smbus_read_byte(client);
	if (val1 < 0)
		return val1;

	val2 = i2c_smbus_read_byte(client);
	if (val2 < 0)
		return val2;

	val1 = (val1 << 8) | val2;

	return val1;
}

static int ads1119_wait_complete(struct iio_dev *indio_dev)
{
	int ret;

	do {
		ret = ads1119_read_reg(indio_dev, ADS1119_CMD_RREG_STAT);
		if (ret < 0)
			return ret;
	} while (!(ret & ADS1119_STAT_DRDY_MASK));

	return 0;
}

static IIO_CONST_ATTR_NAMED(ads1119_ref_source_available, reference_source_available,
	"0=INT ( 2.048 V ) 1=EXT ( reference_voltage_available )");

static IIO_CONST_ATTR_NAMED(ads1119_ref_voltage_available, reference_voltage_available,
	"0 up to 5.500 V in microvolts ");

static IIO_CONST_ATTR_NAMED(ads1119_hardwaregain_available, hardwaregain_available,
	"1 4");

static IIO_CONST_ATTR_NAMED(ads1119_sampling_frequency_available,
	sampling_frequency_available, "20 90 330 1000");

static struct attribute *ads1119_attributes[] = {
	&iio_const_attr_ads1119_hardwaregain_available.dev_attr.attr,
	&iio_const_attr_ads1119_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_ads1119_ref_source_available.dev_attr.attr,
	&iio_const_attr_ads1119_ref_voltage_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads1119_attribute_group = {
	.attrs = ads1119_attributes,
};

static int ads1119_client_get_channels_config(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads1119_private *data = iio_priv(indio_dev);
	struct device *dev = &client->dev;
	struct fwnode_handle *node;
	int i = -1;

	device_for_each_child_node(dev, node) {
		u32 pval;
		unsigned int channel;
		unsigned int gain = ADS1119_DEFAULT_GAIN;
		unsigned int sample_rate = ADS1119_DEFAULT_SAMPLE_RATE;
		unsigned int extref = ADS1119_DEFAULT_REF;
		unsigned int ref_voltage = ADS1119_DEFAULT_REF_VOLTAGE;

		if (fwnode_property_read_u32(node, "reg", &pval)) {
			dev_err(dev, "invalid reg on %pfw\n", node);
			continue;
		}

		channel = pval;
		if (channel >= ADS1119_CHANNELS) {
			dev_err(dev, "invalid channel index %d on %pfw\n", channel, node);
			continue;
		}

		if (!fwnode_property_read_u32(node, "ti,gain", &pval)) {
			gain = pval;
			if (gain != 1 && gain != 4) {
				dev_err(dev, "invalid gain on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
		}

		if (!fwnode_property_read_u32(node, "ti,extref", &pval)) {
			extref = pval;
			if (extref != 1 && extref != 0) {
				dev_err(dev, "invalid extref on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
		}

		if (!fwnode_property_read_u32(node, "ti,samplerate", &pval)) {
			sample_rate = pval;
			if (sample_rate != 20 && sample_rate != 90 && sample_rate !=  330
					&& sample_rate != 1000) {
				dev_err(dev, "invalid sample_rate on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
		}

		if (!fwnode_property_read_u32(node, "ti,refvoltage", &pval)) {
			ref_voltage = pval;
			if (ref_voltage < 0 || ref_voltage > 5500000) {
				dev_err(dev, "invalid ref_voltage on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
		}

		switch (gain) {
		case 1:
			data->channel_conf[channel].gain = 0;
			break;
		case 4:
			data->channel_conf[channel].gain = 1;
			break;
		}

		switch (sample_rate) {
		case 20:
			data->channel_conf[channel].data_rate = 0;
			break;
		case 90:
			data->channel_conf[channel].data_rate = 1;
			break;
		case 330:
			data->channel_conf[channel].data_rate = 2;
			break;
		case 1000:
			data->channel_conf[channel].data_rate = 3;
			break;
		}

		data->channel_conf[channel].ref_source = extref;
		data->channel_conf[channel].ref_voltage = ref_voltage;

		i++;
	}

	return i < 0 ? -EINVAL : 0;
}

static void ads1119_get_channels_config(struct i2c_client *client)
{
	unsigned int k;

	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads1119_private *data = iio_priv(indio_dev);

	data->conv_mode = ADS1119_SINGLESHOT;

	if (!ads1119_client_get_channels_config(client))
		return;

	for (k = 0; k < ADS1119_CHANNELS; ++k) {
		data->channel_conf[k].gain = ADS1119_DEFAULT_GAIN;
		data->channel_conf[k].data_rate = ADS1119_DEFAULT_SAMPLE_RATE;
		data->channel_conf[k].ref_source = ADS1119_DEFAULT_REF;
		data->channel_conf[k].ref_voltage = ADS1119_DEFAULT_REF_VOLTAGE;
	}
}

static int ads1119_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan, int val,
			int val2, long m)
{
	struct ads1119_private *priv = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&priv->lock);

	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		switch (val) {
		case 1:
			priv->channel_conf[chan->address].gain = 0;
			break;
		case 4:
			priv->channel_conf[chan->address].gain = 1;
			break;
		default:
			ret = -EINVAL;
			goto out;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (val) {
		case 20:
			priv->channel_conf[chan->address].data_rate = 0;
			break;
		case 90:
			priv->channel_conf[chan->address].data_rate = 1;
			break;
		case 330:
			priv->channel_conf[chan->address].data_rate = 2;
			break;
		case 1000:
			priv->channel_conf[chan->address].data_rate = 3;
			break;
		default:
			ret = -EINVAL;
			goto out;
		}
		break;
	case IIO_CHAN_INFO_REFSOURCE:
		switch (val) {
		case 0:
			priv->channel_conf[chan->address].ref_source = 0;
			priv->channel_conf[chan->address].ref_voltage = ADS1119_VOLTAGE_INTREF;
			break;
		case 1:
			priv->channel_conf[chan->address].ref_source = 1;
			break;
		default:
			ret = -EINVAL;
			goto out;
		}
		break;
	case IIO_CHAN_INFO_REFVOLTAGE:
		if (val < 0 || val > 5500000) {
			ret = -EINVAL;
			goto out;
		} else {
			priv->channel_conf[chan->address].ref_voltage = val;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ads1119_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long m)
{
	struct ads1119_private *priv = iio_priv(indio_dev);
	int ret;
	s16 data = 0;
	u8 reg_tmp;
	u8 gain_var = 0;

	mutex_lock(&priv->lock);

	switch (m) {
	case IIO_CHAN_INFO_RAW:

		reg_tmp = (((chan->address) << ADS1119_CFG_MUX_SHIFT) |
			((priv->channel_conf[chan->address].gain) << ADS1119_CFG_GAIN_SHIFT) |
			((priv->channel_conf[chan->address].data_rate) << ADS1119_CFG_RATE_SHIFT) |
			((priv->conv_mode) << ADS1119_CFG_CONV_MODE_SHIFT) |
			(priv->channel_conf[chan->address].ref_source) << ADS1119_CFG_VREF_SHIFT);

		ret = ads1119_write_cmd(indio_dev, ADS1119_CMD_START); /*wakeup ADC from sleep*/
		if (ret) {
			dev_err(&indio_dev->dev, "Power ON ADC failed\n");
			goto out;
		}

		ret = ads1119_write_reg(indio_dev, reg_tmp);
		if (ret) {
			dev_err(&indio_dev->dev, "Set ADC CH/CONFIG failed\n");
			goto out;
		}

		fsleep(30); /*regarding datasheet you should wait after switching channels*/

		ret = ads1119_write_cmd(indio_dev, ADS1119_CMD_START); /*start conversion*/
		if (ret) {
			dev_err(&indio_dev->dev, "Start conversion failed\n");
			goto out;
		}

		ret = ads1119_wait_complete(indio_dev);
		if (ret) {
			dev_err(&indio_dev->dev, "Wait conversion complete failed\n");
			goto out;
		}

		ret = ads1119_read(indio_dev);
		if (ret < 0) {
			dev_err(&indio_dev->dev, "Read ADC failed\n");
			goto out;
		}

		data = (ret & DATAMASK);

		if (priv->channel_conf[chan->address].gain)
			gain_var = 4;
		else
			gain_var = 1;

		if ((priv->channel_conf[chan->address].ref_source)) {
			ret = (int) (data *
				((((priv->channel_conf[chan->address].ref_voltage +
				(gain_var - 1)) / gain_var) + BITCOUNT_ROUND)
				/ BITCOUNT));
			*val = ret;
		} else {
			ret = (int) (data *
				((((ADS1119_VOLTAGE_INTREF + (gain_var - 1)) /
				gain_var) + BITCOUNT_ROUND) / BITCOUNT));
			*val = ret;
		}

		ret = ads1119_write_cmd(indio_dev, ADS1119_CMD_PWRDWN);
		if (ret) {
			dev_err(&indio_dev->dev, "Power Down ADC failed\n");
			goto out;
		}

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = priv->chip->gain[(priv->channel_conf[chan->address].gain)];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = priv->chip->data_rate[(priv->channel_conf[chan->address].data_rate)];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_REFSOURCE:
		*val = (priv->channel_conf[chan->address].ref_source);
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_REFVOLTAGE:
		*val = (priv->channel_conf[chan->address].ref_voltage);
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static const struct iio_info ads1119_info = {
	.read_raw = &ads1119_read_raw,
	.write_raw = &ads1119_write_raw,
	.attrs    = &ads1119_attribute_group,
};


static int ads1119_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ads1119_private *ads1119_priv;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*ads1119_priv));
	if (indio_dev == NULL)
		return -ENOMEM;

	ads1119_priv = iio_priv(indio_dev);

	i2c_set_clientdata(client, indio_dev);
	ads1119_priv->client = client;
	ads1119_priv->chip = &ads1119_chip_info_tbl[id->driver_data];

	mutex_init(&ads1119_priv->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->dev.of_node = client->dev.of_node;
	indio_dev->name = ADS1119_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads1119_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads1119_channels);
	indio_dev->info = &ads1119_info;

	ads1119_get_channels_config(client);

	ret = ads1119_write_cmd(indio_dev, ADS1119_CMD_RESET);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to RESET IIO device\n");
		return ret;
	}

	ret = ads1119_write_cmd(indio_dev, ADS1119_CMD_PWRDWN);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to Power Down IIO device\n");
		return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register IIO device\n");
		return ret;
	}

	return 0;
}

static int ads1119_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	int ret;

	ret = ads1119_write_cmd(indio_dev, ADS1119_CMD_PWRDWN);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to Power Down IIO device\n");
		return ret;
	}

	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);

	return 0;
}


static const struct i2c_device_id ads1119_id[] = {
	{ "ads1119", ADS1119 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads1119_id);

static const struct of_device_id ads1119_of_table[] = {
	{ .compatible = "ti,ads1119" },
	{ },
};
MODULE_DEVICE_TABLE(of, ads1119_of_table);

static struct i2c_driver ads1119_driver = {
	.driver = {
		.name	= "ads1119",
		.of_match_table = ads1119_of_table,
	},
	.probe		= ads1119_probe,
	.remove     = ads1119_remove,
	.id_table	= ads1119_id,
};
module_i2c_driver(ads1119_driver);

MODULE_AUTHOR("Felix Siebel <f.siebel@phytec.de>");
MODULE_DESCRIPTION("Texas Instruments ADS1119 ADC driver");
