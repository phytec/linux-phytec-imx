/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021 PHYTEC Messtechnik GmbH
 * Author: Stefan Riedmueller <s.riedmueller@phytec.de>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <linux/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define	UB953_GENERAL_CFG				0x02
#define		BIT_CONTS_CLK				BIT(6)
#define		BIT_CSI_LANE_SEL(n)			((n) << 4)
#define		BIT_CSI_LANE_SEL_MASK			GENMASK(5, 4)
#define UB953_CLKOUT_CTRL0				0x06
#define		BIT_HS_CLK_DIV(n)			((n) << 5)
#define		BIT_HS_CLK_DIV_MASK			GENMASK(7, 5)
#define		BIT_DIV_M_VAL(n)			(n)
#define		BIT_DIV_M_VAL_MASK			GENMASK(4, 0)
#define UB953_CLKOUT_CTRL1				0x07
#define UB953_SCL_HIGH_TIME				0x0b
#define UB953_SCL_LOW_TIME				0x0c
#define UB953_LOCAL_GPIO_DATA				0x0d
#define		BIT_GPIO_RMTEN_SHIFT			4
#define		BIT_GPIO_OUT_SRC_SHIFT			0
#define UB953_GPIO_INPUT_CTRL				0x0e
#define		BIT_OUT_EN_SHIFT			4
#define	UB953_BC_CTRL					0x49
#define		BIT_CRC_ERR_CLR				BIT(3)
#define		BIT_BIST_CTC_ERR_CLR			BIT(5)
#define	UB953_GPIO_PIN_STS				0x53


/* Currently only Port0 will be supported */
#define UB953_IN_PAD 			0
#define UB953_OUT_PAD	 		1
#define UB953_NUM_PADS			2

#define UB953_MAX_GPIOS			4

#define UB953_DEFAULT_I2C_BITRATE	400000
#define UB953_DEFAULT_CLKOUT_FREQ	27000000UL

struct ub953_source {
	struct v4l2_subdev *sd;
	struct fwnode_handle *fwnode;
	struct fwnode_handle *endpoint;
};

struct ub953_asd {
	struct v4l2_async_subdev base;
	struct ub953_source *source;
};

struct ub953 {
	struct i2c_client *i2c;
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler ctrls;
	struct media_pad pad[UB953_NUM_PADS];

	struct gpio_chip gpio;
	struct mutex gpio_lock;

	struct clk *refclk;
	struct clk_hw clkout;
	unsigned int clkout_target;
	unsigned long clkout_freq;
	unsigned int i2c_bitrate;

	bool streaming;
	unsigned int num_lanes;

	struct ub953_source source;
	struct v4l2_async_notifier notifier;
};

static inline struct ub953 *to_ub953(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ub953, subdev);
}

static inline struct ub953_asd *to_ub953_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct ub953_asd, base);
}

static int ub953_read(struct i2c_client *client, u8 reg)
{
	int ret;

	if (!client)
		return -ENODEV;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev,
			"%s: register 0x%02x read failed (%d)\n",
			__func__, reg, ret);
	else
		dev_dbg(&client->dev, "%s: client: 0x%02x 0x%02x 0x%02x\n",
			__func__, client->addr, reg, ret);

	return ret;
}

static int ub953_write(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;

	if (!client)
		return -ENODEV;

	dev_dbg(&client->dev, "%s: client: 0x%02x 0x%02x 0x%02x\n",
		__func__, client->addr, reg, value);

	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret)
		dev_err(&client->dev,
			"%s: register 0x%02x write failed (%d)\n",
			__func__, reg, ret);

	return ret;
}

static int ub953_update_bits(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	u8 orig, tmp;
	int ret;

	ret = ub953_read(client, reg);
	if (ret < 0)
		return ret;

	tmp = (u8)ret & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		ret = ub953_write(client, reg, tmp);

	return ret;
}

static void ub953_gpio_set(struct gpio_chip *chip,
			   unsigned int offset, int value)
{
	struct ub953 *state = gpiochip_get_data(chip);

	dev_dbg(chip->parent, "%s: offset: %u\n", __func__, offset);

	mutex_lock(&state->gpio_lock);
	ub953_update_bits(state->i2c, UB953_LOCAL_GPIO_DATA,
			  BIT(offset), (value << offset));
	mutex_unlock(&state->gpio_lock);
}

static int ub953_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ub953 *state = gpiochip_get_data(chip);
	int val;

	dev_dbg(chip->parent, "%s: offset: %u\n", __func__, offset);

	mutex_lock(&state->gpio_lock);
	val = ub953_read(state->i2c, UB953_GPIO_PIN_STS);
	mutex_unlock(&state->gpio_lock);
	if (val < 0)
		return val;

	return !!(val & BIT(offset));
}

static int ub953_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct ub953 *state = gpiochip_get_data(chip);
	int val;

	dev_dbg(chip->parent, "%s: offset: %u\n", __func__, offset);

	mutex_lock(&state->gpio_lock);
	val = ub953_read(state->i2c, UB953_GPIO_INPUT_CTRL);
	mutex_unlock(&state->gpio_lock);
	if (val < 0)
		return val;

	return !!(val & BIT(offset));
}

static int ub953_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct ub953 *state = gpiochip_get_data(chip);
	int ret;

	dev_dbg(chip->parent, "%s: offset: %u\n", __func__, offset);

	mutex_lock(&state->gpio_lock);
	ret = ub953_update_bits(state->i2c, UB953_GPIO_INPUT_CTRL,
				BIT(offset) | BIT(offset + BIT_OUT_EN_SHIFT),
				BIT(offset));
	mutex_unlock(&state->gpio_lock);

	return ret;
}

static int ub953_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	struct ub953 *state = gpiochip_get_data(chip);
	int ret;

	dev_dbg(chip->parent, "%s: offset: %u\n", __func__, offset);

	mutex_lock(&state->gpio_lock);
	ret = ub953_update_bits(state->i2c, UB953_LOCAL_GPIO_DATA,
				BIT(offset + BIT_GPIO_RMTEN_SHIFT) |
				BIT(offset),
				value << offset);
	if (ret)
		goto out;

	ret = ub953_update_bits(state->i2c, UB953_GPIO_INPUT_CTRL,
				BIT(offset) | BIT(offset + BIT_OUT_EN_SHIFT),
				BIT(offset + BIT_OUT_EN_SHIFT));
out:
	mutex_unlock(&state->gpio_lock);
	return ret;
}

static int ub953_init_gpiochip(struct ub953 *state)
{
	struct device *dev = &state->i2c->dev;
	struct gpio_chip *gc = &state->gpio;
	int ret;

	gc->label = "ds90ub953_gpio";
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->of_node = dev->of_node;
	gc->ngpio = UB953_MAX_GPIOS;
	gc->base = -1;
	gc->can_sleep = true;
	gc->set = ub953_gpio_set;
	gc->get = ub953_gpio_get;
	gc->get_direction = ub953_gpio_get_direction;
	gc->direction_input = ub953_gpio_direction_input;
	gc->direction_output = ub953_gpio_direction_output;

	ret = devm_gpiochip_add_data(dev, gc, state);
	if (ret)
		dev_err(dev, "Unable to create gpio_chip (%d)\n", ret);

	return ret;
}

static int ub953_setup_pll(struct ub953 *state)
{
	struct device *dev = &state->i2c->dev;
	unsigned long refclk = clk_get_rate(state->refclk);
	unsigned long target_rate = state->clkout_target;
	unsigned long fc_data_rate = refclk * 160;
	unsigned long rate, pre_rate;
	unsigned long diff = target_rate;
	unsigned int m, n, div;
	unsigned int m_min = 1;
	unsigned int m_max = 31;
	unsigned int n_min = 1;
	unsigned int n_max = 255;
	unsigned int div_range[] = {1, 2, 4, 8, 16};
	int div_index = 0;
	int ret;
	u8 m_set, n_set, div_set;
	u8 val;
	/*
	 * 27 MHz from 24 MHz REFCLK on deserializer side
	 * Forward channel line rate = 24 MHz * 160 = 3840 MHz
	 * 3840 MHz * 9 / (160*8) = 27 MHz
	 *
	 * u8 m = 9;
	 * u8 n = 160;
	 * u8 div = 8;
	 */

	while (div_index < ARRAY_SIZE(div_range)) {
		div = div_range[div_index];
		pre_rate = div_u64(fc_data_rate, div);
		if (pre_rate >= 1050000000) {
			div_index++;
			continue;
		}

		for (m = m_min; m <= m_max; m++)
			for (n = n_min; n <= n_max; n++) {
				rate = pre_rate * m;
				rate = div_u64(rate, n);

				if (diff <= abs(target_rate - rate))
					continue;

				diff = abs(target_rate - rate);
				m_set = m;
				n_set = n;
				div_set = div_index;
				state->clkout_freq = rate;
			}

		div_index++;
	}

	dev_dbg(dev, "%s: div: %u, m: %u, n: %u, rate: %lu\n",
		__func__, div_range[div_set], m_set, n_set,
		state->clkout_freq);

	val = BIT_HS_CLK_DIV(div_set) & BIT_HS_CLK_DIV_MASK;
	val |= BIT_DIV_M_VAL(m_set) & BIT_DIV_M_VAL_MASK;
	ret = ub953_write(state->i2c, UB953_CLKOUT_CTRL0, val);
	if (ret)
		return ret;

	ret = ub953_write(state->i2c, UB953_CLKOUT_CTRL1, n_set);
	if (ret)
		return ret;

	return 0;
}

static unsigned long ub953_clk_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct ub953 *state = container_of(hw, struct ub953, clkout);
	return state->clkout_freq;
}

static const struct clk_ops ub953_clk_ops = {
	.recalc_rate = ub953_clk_recalc_rate,
};

static int ub953_init_clkout(struct ub953 *state)
{
	struct device *dev = &state->i2c->dev;
	struct clk *clk;
	struct clk_init_data init;
	int ret;

	ret = ub953_setup_pll(state);
	if (ret)
		return ret;

	memset(&init, 0, sizeof(init));

	init.name = "clkout";
	of_property_read_string(dev->of_node, "clock-output-names", &init.name);
	init.ops = &ub953_clk_ops;

	state->clkout.init = &init;

	clk = devm_clk_register(dev, &state->clkout);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	of_clk_add_provider(dev->of_node, of_clk_src_simple_get, clk);

	return 0;
}

static u64 ub953_get_pixelrate(struct ub953 *state)
{
	struct v4l2_subdev *sd = state->source.sd;
	struct v4l2_ctrl *ctrl;

	if (!sd)
		return 0;

	ctrl = v4l2_ctrl_find(sd->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (!ctrl) {
		dev_warn(&state->i2c->dev,
			"No pixelrate control available on active source\n");
		return 0;
	}

	return v4l2_ctrl_g_ctrl_int64(ctrl);
}

static int ub953_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ub953 *state = container_of(ctrl->handler,
						struct ub953,
						ctrls);

	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		*ctrl->p_new.p_s64 = ub953_get_pixelrate(state);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ub953_ctrl_ops = {
	.g_volatile_ctrl = ub953_g_volatile_ctrl,
};

static int ub953_power_on(struct ub953 *state)
{
	/* TODO: Enable power, clocks, etc... */
	return 0;
}

static void ub953_power_off(struct ub953 *state)
{
	/* TODO: Disable power, clocks, etc... */
}

/* V4L2 subdev core ops */
/* TODO: Add pm_runtime */
static int ub953_s_power(struct v4l2_subdev *sd, int on)
{
	struct ub953 *state = to_ub953(sd);
	int ret = 0;

	dev_dbg(sd->dev, "%s on: %d\n", __func__, on);

	if (on) {
		ret = ub953_power_on(state);
		if (ret) {
			ub953_power_off(state);
			goto out;
		}
	} else {
		ub953_power_off(state);
	}

out:
	return ret;
}

static int ub953_start_stream(struct ub953 *state,
			      struct v4l2_subdev *upstream_sd)
{
	int ret;

	ret = ub953_update_bits(state->i2c, UB953_GENERAL_CFG,
				BIT_CSI_LANE_SEL_MASK,
				BIT_CSI_LANE_SEL(state->num_lanes - 1));
	if (ret)
		return ret;

	ret = ub953_update_bits(state->i2c, UB953_BC_CTRL,
				BIT_BIST_CTC_ERR_CLR | BIT_CRC_ERR_CLR,
				BIT_BIST_CTC_ERR_CLR | BIT_CRC_ERR_CLR);
	if (ret)
		return ret;

	ret = v4l2_subdev_call(upstream_sd, video, s_stream, 1);
	if (ret)
		return ret;

	return 0;
}

static void ub953_stop_stream(struct ub953 *state,
			      struct v4l2_subdev *upstream_sd)
{
	v4l2_subdev_call(upstream_sd, video, s_stream, 0);
}

/* V4L2 subdev video ops */
static int ub953_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ub953 *state = to_ub953(sd);
	struct v4l2_subdev *upstream_sd;
	struct media_pad *pad;
	int ret = 0;

	dev_dbg(sd->dev, "%s enable: %d\n", __func__, enable);

	pad = media_entity_remote_pad(&sd->entity.pads[UB953_IN_PAD]);
	if (!pad) {
		dev_err(sd->dev, "Failed to find remote source pad\n");
		return -ENOLINK;
	}

	if (!is_media_entity_v4l2_subdev(pad->entity)) {
		dev_err(sd->dev, "Upstream entity is not a v4l2 subdev\n");
		return -ENODEV;
	};

	upstream_sd = state->source.sd;

	if (enable) {
		ret = ub953_start_stream(state, upstream_sd);
		if (!ret)
			state->streaming = true;
	} else {
		ub953_stop_stream(state, upstream_sd);
		state->streaming = false;
	}

	return ret;
}

static int ub953_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *sd_state,
			 struct v4l2_subdev_format *format)
{
	struct ub953 *state = to_ub953(sd);
	struct ub953_source *source;
	struct media_pad *pad;
	struct v4l2_subdev_format fmt = {
		.which = format->which,
	};
	int ret;

	dev_dbg(sd->dev, "%s\n", __func__);

	source = &state->source;
	if (!source->sd)
		return -EINVAL;

	if (format->pad != UB953_IN_PAD)
		return -EINVAL;

	pad = media_entity_remote_pad(&state->pad[format->pad]);
	if (!pad) {
		dev_err(sd->dev, "Failed to find remote source pad\n");
		return -ENOLINK;
	}

	fmt.pad = pad->index;
	ret = v4l2_subdev_call(source->sd, pad, get_fmt, sd_state, &fmt);
	if (ret)
		return ret;

	format->format = fmt.format;

	return 0;
}

static int ub953_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *sd_state,
			 struct v4l2_subdev_format *format)
{
	dev_dbg(sd->dev, "%s\n", __func__);
	return ub953_get_fmt(sd, sd_state, format);
}

static const struct v4l2_subdev_core_ops ub953_subdev_core_ops = {
	.s_power		= ub953_s_power,
};

static const struct v4l2_subdev_video_ops ub953_subdev_video_ops = {
	.s_stream		= ub953_s_stream,
};

static const struct v4l2_subdev_pad_ops ub953_subdev_pad_ops = {
	.set_fmt		= ub953_set_fmt,
	.get_fmt		= ub953_get_fmt,
};

static const struct v4l2_subdev_ops ub953_subdev_ops = {
	.core			= &ub953_subdev_core_ops,
	.video			= &ub953_subdev_video_ops,
	.pad			= &ub953_subdev_pad_ops,
};

static const struct media_entity_operations ub953_entity_ops = {
	.get_fwnode_pad		= v4l2_subdev_get_fwnode_pad_1_to_1,
};

static int ub953_subdev_registered(struct v4l2_subdev *sd)
{
	dev_info(sd->dev, "Registered DS90UB953 serializer\n");

	return 0;
}

static const struct v4l2_subdev_internal_ops ub953_subdev_internal_ops = {
	.registered		= ub953_subdev_registered,
};

static int ub953_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_subdev *asd)
{
	struct ub953 *state = to_ub953(notifier->sd);
	struct ub953_source *source = to_ub953_asd(asd)->source;
	int src_pad;
	int ret;

	src_pad = media_entity_get_fwnode_pad(&subdev->entity, source->endpoint,
					      MEDIA_PAD_FL_SOURCE);

	if (src_pad < 0) {
		dev_err(notifier->sd->dev,
			"Failed to find source pad on %s (%d)\n",
			subdev->name, src_pad);
		return src_pad;
	}

	ret = media_create_pad_link(&subdev->entity, src_pad,
				    &state->subdev.entity, UB953_IN_PAD,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret) {
		dev_err(notifier->sd->dev,
			"Failed to link %s:%d -> %s:%d (%d)\n",
			subdev->name, src_pad, state->subdev.name,
			UB953_IN_PAD, ret);
		return ret;
	};

	source->sd = subdev;

	dev_dbg(notifier->sd->dev, "Linked %s:%d -> %s:%d\n",
		subdev->name, src_pad, state->subdev.name, UB953_IN_PAD);

	return 0;
}

static void ub953_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct ub953_source *source = to_ub953_asd(asd)->source;

	source->sd = NULL;
	dev_dbg(subdev->dev, "%s\n", __func__);
}

static const struct v4l2_async_notifier_operations sd90_notify_ops = {
	.bound = ub953_notify_bound,
	.unbind = ub953_notify_unbind,
};

static int ub953_v4l2_notifier_register(struct ub953 *state)
{
	struct device *dev = &state->i2c->dev;
	struct v4l2_async_subdev *asd;
	struct ub953_source *source;
	struct ub953_asd *uas;
	int ret;

	source = &state->source;
	if (!source->fwnode)
		return 0;

	dev_dbg(dev, "%s\n", __func__);

	v4l2_async_notifier_init(&state->notifier);

	uas = v4l2_async_notifier_add_fwnode_subdev(&state->notifier,
						    source->fwnode,
						    struct ub953_asd);
	if (IS_ERR(asd)) {
		dev_err(dev, "Adding subdev failed(%ld)\n", PTR_ERR(asd));
		v4l2_async_notifier_cleanup(&state->notifier);
		return PTR_ERR(asd);
	}

	uas->source = source;

	state->notifier.ops = &sd90_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&state->subdev,
						  &state->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev notifier");
		v4l2_async_notifier_cleanup(&state->notifier);
		return ret;
	};

	return 0;
}

static int ub953_init_serializer(struct ub953 *state)
{
	struct i2c_client *i2c = state->i2c;
	struct device *dev = &state->i2c->dev;
	union i2c_smbus_data data;
	unsigned long refclk_rate = clk_get_rate(state->refclk);
	int high, low;
	int ret;

	/*
	 * Setup i2c connection on serializer side
	 * Set to 400 kHz @ 24 MHz REFCLK
	 * nom_delay = 1/400 kHz / 2 = 1.25 us
	 * step = 1 / 24 MHz = 41.66 ns
	 * SCL_HIGH/LOW_TIME = (nom_delay / step) - 5 = 25
	 * TODO: Setup only 1 of the three available modes
	 * Standard 100 kHz HIGH: 5.03us (0x7F) LOW: 5.03us (0x7F)
	 * Fast 400 kHz HIGH: 0.914us (0x13) LOW: 1.64us (0x26)
	 * Fast Plus 1 MHz HIGH: 0.419 (0x06) LOW: 0.648us (0x0B)
	 * Need to be adjusted to ref clock.
	 */
	high = (refclk_rate / (2 * state->i2c_bitrate)) - 5;
	high = clamp_t(int, high, 0, 0xff);
	low = high;

	dev_dbg(dev, "%s: SCL_HIGH_TIME: %d\n", __func__, high);
	dev_dbg(dev, "%s: SCL_LOW_TIME: %d\n", __func__, low);

	i2c_lock_bus(i2c->adapter, I2C_LOCK_SEGMENT);

	data.byte = high;
	ret = __i2c_smbus_xfer(i2c->adapter, i2c->addr, i2c->flags,
			       I2C_SMBUS_WRITE, UB953_SCL_HIGH_TIME,
			       I2C_SMBUS_BYTE_DATA, &data);
	if (ret)
		goto out;

	data.byte = low;
	ret = __i2c_smbus_xfer(i2c->adapter, i2c->addr, i2c->flags,
			       I2C_SMBUS_WRITE, UB953_SCL_LOW_TIME,
			       I2C_SMBUS_BYTE_DATA, &data);
	if (ret)
		goto out;

	msleep(200);

out:
	i2c_unlock_bus(i2c->adapter, I2C_LOCK_SEGMENT);
	return ret;
}

static int ub953_init(struct ub953 *state)
{
	int ret;

	ret = ub953_init_serializer(state);
	if (ret)
		return ret;

	ret = ub953_init_gpiochip(state);
	if (ret)
		return ret;

	ret = ub953_init_clkout(state);
	if (ret)
		return ret;

	ret = ub953_v4l2_notifier_register(state);
	if (ret)
		return ret;

	return ret;
}

static void ub953_cleanup_source(struct ub953 *state)
{
	struct ub953_source *source;

	source = &state->source;

	fwnode_handle_put(source->fwnode);
	fwnode_handle_put(source->endpoint);
	source->fwnode = NULL;
	source->endpoint = NULL;
}

static int ub953_of_probe(struct device *dev, struct ub953 *state)
{
	struct device_node *node = NULL;
	struct device_node *i2c_bus_node = NULL;
	struct fwnode_handle *fwnode;
	int ret;

	state->refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR_OR_NULL(state->refclk)) {
		ret = PTR_ERR_OR_ZERO(state->refclk);
		if (ret == -EPROBE_DEFER)
			return ret;

		dev_err(dev, "Failed to get ref clock (%d)\n", ret);
		if (!ret)
			return -ENODEV;

		return ret;
	}

	state->i2c_bitrate = UB953_DEFAULT_I2C_BITRATE;
	i2c_bus_node = of_get_parent(dev->of_node);
	of_property_read_u32(i2c_bus_node, "clock-frequency",
			     &state->i2c_bitrate);
	of_node_put(i2c_bus_node);

	state->clkout_target = UB953_DEFAULT_CLKOUT_FREQ;
	of_property_read_u32(dev->of_node, "clock-frequency",
			     &state->clkout_target);

	for_each_endpoint_of_node(dev->of_node, node) {
		struct of_endpoint ep;
		struct fwnode_handle *fwremote, *endpoint;
		struct v4l2_fwnode_endpoint vep = {
			.bus_type = V4L2_MBUS_CSI2_DPHY
		};

		of_graph_parse_endpoint(node, &ep);

		if (ep.port > UB953_NUM_PADS) {
			dev_warn(dev, "Invalid port %d", ep.port);
			continue;
		}

		if (ep.port != UB953_IN_PAD)
			continue;

		if (state->source.fwnode) {
			dev_warn(dev, "Only one endpoint supported\n");
			continue;
		}

		fwnode = of_fwnode_handle(node);
		ret = v4l2_fwnode_endpoint_parse(fwnode, &vep);
		if (ret)
			goto out;

		state->num_lanes = vep.bus.mipi_csi2.num_data_lanes;
		if (state->num_lanes != 1 &&
		    state->num_lanes != 2 &&
		    state->num_lanes != 4) {
			dev_err(dev, "Incompatible number of lanes\n");
			ret = -EINVAL;
			goto out;
		}

		fwremote = fwnode_graph_get_remote_port_parent(fwnode);
		if (!fwremote) {
			dev_warn(dev, "Endpoint %pOF has no remote endpoint\n",
				 ep.local_node);
			goto out;
		}
		endpoint = fwnode_graph_get_remote_endpoint(fwnode);

		state->source.fwnode = fwremote;
		state->source.endpoint = endpoint;
	}
	of_node_put(node);

	return 0;
out:
	of_node_put(node);
	return ret;
}

static int ub953_probe(struct i2c_client *i2c, const struct i2c_device_id *did)
{
	struct ub953 *state;
	struct device *dev = &i2c->dev;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl *ctrl;
	int ret;

	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	sd = &state->subdev;
	state->i2c = i2c;

	ret = ub953_of_probe(dev, state);
	if (ret)
		return ret;

	mutex_init(&state->gpio_lock);

	v4l2_ctrl_handler_init(&state->ctrls, 1);
	ctrl = v4l2_ctrl_new_std(&state->ctrls, &ub953_ctrl_ops,
				 V4L2_CID_PIXEL_RATE, 0, INT_MAX, 1, 0);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	sd->ctrl_handler = &state->ctrls;

	v4l2_i2c_subdev_init(sd, i2c, &ub953_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &ub953_subdev_internal_ops;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &ub953_entity_ops;

	state->pad[UB953_IN_PAD].flags = MEDIA_PAD_FL_SINK;
	state->pad[UB953_OUT_PAD].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, UB953_NUM_PADS, state->pad);
	if (ret)
		goto out;

	ret = ub953_init(state);
	if (ret)
		goto out_init;

	ret = v4l2_async_register_subdev(&state->subdev);
	if (ret)
		goto out_init;

	dev_info(dev, "Probed DS90UB953 Driver succesfully\n");

	return 0;

out_init:
	v4l2_async_notifier_unregister(&state->notifier);
	v4l2_async_notifier_cleanup(&state->notifier);
out:
	ub953_cleanup_source(state);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&state->gpio_lock);

	dev_err(dev, "Failed to probe DS90UB953 Driver (%d)\n", ret);
	return ret;
}

static int ub953_remove(struct i2c_client *i2c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(i2c);
	struct ub953 *state = to_ub953(sd);

	v4l2_async_notifier_unregister(&state->notifier);
	v4l2_async_notifier_cleanup(&state->notifier);
	v4l2_async_unregister_subdev(sd);

	ub953_cleanup_source(state);

	media_entity_cleanup(&sd->entity);
	mutex_destroy(&state->gpio_lock);

	return 0;
}

static const struct of_device_id ub953_of_match[] = {
        { .compatible = "ti,ds90ub953" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ub953_of_match);

static struct i2c_driver ub953_i2c_driver = {
        .driver         = {
                .name   = "ds90ub953",
                .of_match_table = of_match_ptr(ub953_of_match),
        },
        .probe          = ub953_probe,
        .remove         = ub953_remove,
};
module_i2c_driver(ub953_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stefan Riedmueller <s.riedmueller@phytec.de>");
