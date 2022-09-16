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

#define UB954_SR_RESET					0x01
#define		BIT_DIGITAL_RESET1			BIT(1)
#define		BIT_DIGITAL_RESET0			BIT(0)
#define UB954_SR_CSI_PLL_CTL				0x1f
#define UB954_SR_FWD_CTL1				0x20
#define		BIT_FWD_PORT1_DIS			BIT(5)
#define		BIT_FWD_PORT0_DIS			BIT(4)
#define	UB954_SR_CSI_CTL				0x33
#define		BIT_CSI_CAL_EN				BIT(6)
#define		BIT_LANE_COUNT(n)			((n) << 4)
#define		BIT_LANE_COUNT_MASK			GENMASK(5, 4)
#define		BIT_CSI_ULP(n)				((n) << 2)
#define		BIT_CSI_ULP_MASK			GENMASK(3, 2)
#define		BIT_CSI_CONTS_CLOCK			BIT(1)
#define		BIT_CSI_EN				BIT(0)
#define	UB954_PR_SFILTER_CFG				0x41
#define		BIT_SFILTER_MAX(n)			((n) << 4)
#define		BIT_SFILTER_MIN(n)			(n)
#define	UB954_PR_RX_PORT_STS1				0x4d
#define		BIT_LOCK_STS_CHG			BIT(4)
#define		BIT_LOCK_STS				BIT(0)
#define	UB954_PR_BCC_CONFIG				0x58
#define		BIT_I2C_PASS_THRU_ALL			BIT(7)
#define		BIT_I2C_PASS_THRU			BIT(6)
#define		BIT_AUTO_ACK_ALL			BIT(5)
#define UB954_PR_SER_ID					0x5b
#define		BIT_SER_ID_MASK				GENMASK(7, 1)
#define		BIT_SER_ID_SHIFT			1
#define	UB954_PR_SER_ALIAS_ID				0x5c
#define		BIT_SER_ALIAS_ID(n)			((n) << 1)
#define		BIT_SER_AUTO_ACK			BIT(0)
#define UB954_PR_SLAVE_ID_BASE				0x5d
#define		BIT_SLAVE_ID_MASK			GENMASK(7, 1)
#define		BIT_SLAVE_ID_SHIFT			1
#define UB954_PR_SLAVE_ALIAS_BASE			0x65
#define		BIT_SLAVE_ALIAS_SHIFT			1
#define		BIT_SLAVE_AUTO_ACK			BIT(0)
#define	UB954_PR_AEQ_CTL2				0xd2
#define		BIT_AEQ_RESTART				BIT(3)
#define		BIT_SET_AEQ_FLOOR			BIT(2)
#define	UB954_PR_AEQ_MIN_MAX				0xd5
#define		BIT_AEQ_MAX(n)				((n) << 4)
#define		BIT_AEQ_FLOOR_VALUE(n)			(n)
#define	UB954_SR_I2C_RX0_ID				0xf8
#define	UB954_SR_I2C_RX1_ID				0xf9
#define		BIT_RX_PORT_ID(n)			((n) << 1)


#define UB954_PORT0_PAD 		0
#define UB954_PORT1_PAD 		1
#define UB954_SOURCE_PAD 		2
#define UB954_NUM_PADS			3

#define UB954_NUM_PORTS			2
#define UB954_MAX_I2C_DEVS		8

#define UB954_1600MBPS_LINK_FREQ	800000000UL
#define UB954_800MBPS_LINK_FREQ		400000000UL
#define UB954_400MBPS_LINK_FREQ		200000000UL

#define UB954_DEFAULT_STROBE_MIN	7
#define UB954_DEFAULT_STROBE_MAX	10
#define UB954_DEFAULT_EQ_MIN		2
#define UB954_DEFAULT_EQ_MAX		14
#define UB954_DEFAULT_RESET_DELAY_MS	200

#define UB954_LOCK_STEP			100
#define UB954_LOCK_TIMEOUT		5000

struct ub954_i2c_dev {
	u8 i2c_addr;
	u8 i2c_alias;
};

struct ub954_rxport {
	struct i2c_client *i2c;
	struct v4l2_subdev *sd;
	struct fwnode_handle *fwnode;
	struct fwnode_handle *endpoint;

	struct ub954_i2c_dev i2c_dev[UB954_MAX_I2C_DEVS];
	unsigned int num_devs;

	unsigned int eq_min;
	unsigned int eq_max;
	unsigned int strobe_min;
	unsigned int strobe_max;

	int port;
	u16 ser_id;
	u16 ser_alias_id;
	bool active;
};

struct ub954_asd {
	struct v4l2_async_subdev base;
	struct ub954_rxport *rxport;
};

struct ub954 {
	struct i2c_client *i2c;
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler ctrls;
	struct media_pad pad[UB954_NUM_PADS];

	struct i2c_mux_core *mux;
	unsigned int last_chan;

	struct mutex lock;

	int last_port;
	int active_port;
	unsigned int num_lanes;
	unsigned int reset_delay_ms;

	bool streaming;

	u64 *link_freq;
	u64 pixelrate;

	struct ub954_rxport rxport[UB954_NUM_PORTS];

	struct v4l2_async_notifier notifier;
};

#define for_each_port(port) \
	for ((port) = 0; (port) < UB954_NUM_PORTS; (port)++)

static inline struct ub954 *to_ub954(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ub954, subdev);
}

static inline struct ub954_asd *to_ub954_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct ub954_asd, base);
}

static int ub954_read(struct i2c_client *client, u8 reg)
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

static int ub954_write(struct i2c_client *client, u8 reg, u8 value)
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

static int ub954_update_bits(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	u8 orig, tmp;
	int ret;

	ret = ub954_read(client, reg);
	if (ret < 0)
		return ret;

	tmp = (u8)ret & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		ret = ub954_write(client, reg, tmp);

	return ret;
}

static int ub954_set_bits(struct i2c_client *client, u8 reg, u8 bits)
{
	return ub954_update_bits(client, reg, bits, bits);
}

static int ub954_clear_bits(struct i2c_client *client, u8 reg, u8 bits)
{
	return ub954_update_bits(client, reg, bits, 0);
}

static int ub954_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct ub954 *state = i2c_mux_priv(muxc);
	struct ub954_rxport *rxport;
	unsigned char buf[8];
	int id;
	int ret;

	if (chan == state->last_chan)
		return 0;

	if (chan >= UB954_NUM_PORTS)
		return -EINVAL;

	memset(&buf, 0, sizeof(buf));

	rxport = &state->rxport[!chan];
	ret = i2c_smbus_write_i2c_block_data(rxport->i2c,
					     UB954_PR_SLAVE_ALIAS_BASE,
					     sizeof(buf), buf);
	if (ret)
		return ret;

	ret = ub954_write(state->i2c, UB954_PR_SER_ALIAS_ID, 0);
	if (ret)
		return ret;

	rxport = &state->rxport[chan];

	for (id = 0; id < rxport->num_devs; id++) {
		buf[id] = rxport->i2c_dev[id].i2c_alias;
		buf[id] <<= BIT_SLAVE_ALIAS_SHIFT;
	}

	ret = i2c_smbus_write_i2c_block_data(rxport->i2c,
					     UB954_PR_SLAVE_ALIAS_BASE,
					     sizeof(buf), buf);
	if (ret)
		return ret;

	ret = ub954_write(state->i2c, UB954_PR_SER_ALIAS_ID,
			  rxport->ser_alias_id);
	if (ret)
		return ret;

	state->last_chan = chan;
	return 0;
}

static int ub954_i2c_mux_init(struct ub954 *state)
{
	struct device *dev = &state->i2c->dev;
	int port;
	int ret;

	if (!i2c_check_functionality(state->i2c->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;


	state->mux = i2c_mux_alloc(state->i2c->adapter, dev,
				    UB954_NUM_PORTS, 0, I2C_MUX_LOCKED,
				    ub954_i2c_mux_select, NULL);

	if (!state->mux)
		return -ENOMEM;

	state->mux->priv = state;

	for_each_port(port) {
		if (state->rxport[port].active) {
			ret = i2c_mux_add_adapter(state->mux, 0, port, 0);
			if (ret) {
				 i2c_mux_del_adapters(state->mux);
				 return ret;
			}
		}
	}

	return 0;
}

static u64 ub954_get_pixelrate(struct ub954 *state)
{
	struct ub954_rxport *rxport;
	struct v4l2_ctrl *ctrl;
	int port = state->active_port;

	if (port < 0)
		return -EINVAL;

	rxport = &state->rxport[port];

	ctrl = v4l2_ctrl_find(rxport->sd->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (!ctrl) {
		dev_warn(&state->i2c->dev,
			"No pixelrate control available on active source\n");
		return 0;
	}

	return v4l2_ctrl_g_ctrl_int64(ctrl);
}

static int ub954_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ub954 *state = container_of(ctrl->handler, struct ub954, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		*ctrl->p_new.p_s64 = ub954_get_pixelrate(state);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ub954_ctrl_ops = {
	.g_volatile_ctrl = ub954_g_volatile_ctrl,
};

static int ub954_enable_csi_tx(struct ub954 *state)
{
	unsigned int csi_ctl = BIT_CSI_EN;
	int ret;

	csi_ctl |= BIT_LANE_COUNT(4 - state->num_lanes);

	if (*(state->link_freq) == UB954_1600MBPS_LINK_FREQ)
		csi_ctl |= BIT_CSI_CAL_EN;

	ret = ub954_write(state->i2c, UB954_SR_CSI_CTL, csi_ctl);
	if (ret)
		return ret;

	return 0;
}

static void ub954_disable_csi_tx(struct ub954 *state)
{
	ub954_clear_bits(state->i2c, UB954_SR_CSI_CTL,
			 BIT_CSI_EN | BIT_CSI_CAL_EN | BIT_CSI_CONTS_CLOCK);
}

/* V4L2 subdev core ops */
/* TODO: Add pm_runtime to enable/disable power, clocks, etc. */
/* TODO: Set LP-11 mode here */
static int ub954_s_power(struct v4l2_subdev *sd, int on)
{
	dev_dbg(sd->dev, "%s on: %d\n", __func__, on);
	return 0;
}

static int ub954_start_stream(struct ub954 *state, int port)
{
	struct ub954_rxport *rxport = &state->rxport[port];
	int ret;

	ret = ub954_enable_csi_tx(state);
	if (ret)
		goto out;

	ret = ub954_write(state->i2c, UB954_SR_FWD_CTL1,
			  port ? BIT_FWD_PORT0_DIS : BIT_FWD_PORT1_DIS);
	if (ret)
		goto out_abort;

	ret = v4l2_subdev_call(rxport->sd, video, s_stream, 1);
	if (ret)
		goto out_abort;

	return 0;

out_abort:
	ub954_write(state->i2c, UB954_SR_FWD_CTL1, 0);
out:
	ub954_disable_csi_tx(state);
	dev_err(&state->i2c->dev, "%s Failed to start streaming (%d)\n",
		__func__, ret);
	return ret;
}

static void ub954_stop_stream(struct ub954 *state, int port)
{
	struct ub954_rxport *rxport = &state->rxport[port];

	v4l2_subdev_call(rxport->sd, video, s_stream, 0);
	ub954_write(state->i2c, UB954_SR_FWD_CTL1, 0);
	ub954_disable_csi_tx(state);
}

/* V4L2 subdev video ops */
static int ub954_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ub954 *state = to_ub954(sd);
	struct media_pad *pad;
	int port = state->active_port;
	int ret = 0;

	dev_dbg(sd->dev, "%s enable: %d\n", __func__, enable);

	if (port < 0)
		return -EPIPE;

	pad = media_entity_remote_pad(&sd->entity.pads[port]);
	if (!pad) {
		dev_err(sd->dev, "Failed to find remote source pad\n");
		return -ENOLINK;
	}

	if (!is_media_entity_v4l2_subdev(pad->entity)) {
		dev_err(sd->dev, "Upstream entity is not a v4l2 subdev\n");
		return -ENODEV;
	};

	mutex_lock(&state->lock);

	if (enable) {
		ret = ub954_start_stream(state, port);
		if (!ret)
			state->streaming = true;
	} else {
		ub954_stop_stream(state, port);
		state->streaming = false;
	}

	mutex_unlock(&state->lock);

	dev_dbg(sd->dev, "%s ret: %d\n", __func__, ret);
	return ret;
}

static int ub954_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct ub954 *state = to_ub954(sd);
	struct ub954_rxport *rxport;
	struct v4l2_subdev_format fmt = {
		.which = format->which,
	};
	unsigned int pad;
	int port;
	int ret;

	dev_dbg(sd->dev, "%s\n", __func__);

	if (format->pad < UB954_NUM_PORTS)
		return -EINVAL;

	if (state->active_port < 0)
		return -EINVAL;

	port = state->active_port;

	rxport = &state->rxport[port];
	if (!rxport->sd)
		return -EINVAL;

	for (pad = 0; pad < rxport->sd->entity.num_pads; pad++)
		if (rxport->sd->entity.pads[pad].flags & MEDIA_PAD_FL_SINK)
			fmt.pad = pad;

	ret = v4l2_subdev_call(rxport->sd, pad, get_fmt, cfg, &fmt);
	if (ret)
		return ret;

	format->format = fmt.format;

	return 0;
}

static int ub954_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	dev_dbg(sd->dev, "%s\n", __func__);
	return ub954_get_fmt(sd, cfg, format);
}

static const struct v4l2_subdev_core_ops ub954_subdev_core_ops = {
	.s_power		= ub954_s_power,
};

static const struct v4l2_subdev_video_ops ub954_subdev_video_ops = {
	.s_stream		= ub954_s_stream,
};

static const struct v4l2_subdev_pad_ops ub954_subdev_pad_ops = {
	.set_fmt		= ub954_set_fmt,
	.get_fmt		= ub954_get_fmt,
};

static const struct v4l2_subdev_ops ub954_subdev_ops = {
	.core			= &ub954_subdev_core_ops,
	.video			= &ub954_subdev_video_ops,
	.pad			= &ub954_subdev_pad_ops,
};

static int ub954_link_setup(struct media_entity *entity,
			    struct media_pad const *local,
			    struct media_pad const *remote,
			    u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct ub954 *state = to_ub954(sd);
	bool enable = flags & MEDIA_LNK_FL_ENABLED;

	if (state->streaming)
		return -EBUSY;

	if (local->index == UB954_SOURCE_PAD)
		return 0;

	if (state->active_port >= 0 && enable)
		return -EINVAL;

	mutex_lock(&state->lock);

	if (!enable && state->active_port == local->index)
		state->active_port = -1;

	if (enable)
		state->active_port = local->index;

	mutex_unlock(&state->lock);

	return 0;
}

static const struct media_entity_operations ub954_entity_ops = {
	.link_setup		= ub954_link_setup,
};

static int ub954_subdev_registered(struct v4l2_subdev *sd)
{
	struct ub954 *state = to_ub954(sd);
	int ret;

	ret = v4l2_ctrl_handler_setup(&state->ctrls);
	if (ret)
		return ret;

	dev_info(sd->dev, "Registered DS90UB954 deserializer\n");

	return 0;
}

static const struct v4l2_subdev_internal_ops ub954_subdev_internal_ops = {
	.registered		= ub954_subdev_registered,
};

static int ub954_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_subdev *asd)
{
	struct ub954 *state = to_ub954(notifier->sd);
	struct ub954_rxport *rxport = to_ub954_asd(asd)->rxport;
	int port = rxport->port;
	int src_pad;
	int ret;

	src_pad = media_entity_get_fwnode_pad(&subdev->entity, rxport->endpoint,
					      MEDIA_PAD_FL_SOURCE);

	if (src_pad < 0) {
		dev_err(notifier->sd->dev,
			"Failed to find source pad on %s (%d)\n",
			subdev->name, src_pad);
		return src_pad;
	}

	ret = media_create_pad_link(&subdev->entity, src_pad,
				    &state->subdev.entity, port, 0);
	if (ret) {
		dev_err(notifier->sd->dev,
			"Failed to link %s:%d -> %s:%d (%d)\n",
			subdev->name, src_pad, state->subdev.name, port, ret);
		return ret;
	};

	rxport->sd = subdev;

	dev_dbg(notifier->sd->dev, "Linked %s:%d -> %s:%d\n",
		subdev->name, src_pad, state->subdev.name, port);

	dev_dbg(subdev->dev, "%s: Bound %s on port %d\n", __func__,
		subdev->name, port);

	return 0;
}

static void ub954_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct ub954_rxport *rxport = to_ub954_asd(asd)->rxport;

	rxport->sd = NULL;
	dev_dbg(subdev->dev, "%s\n", __func__);
}

static const struct v4l2_async_notifier_operations sd90_notify_ops = {
	.bound = ub954_notify_bound,
	.unbind = ub954_notify_unbind,
};

static int ub954_v4l2_notifier_register(struct ub954 *state)
{
	struct device *dev = &state->i2c->dev;
	struct v4l2_async_subdev *asd;
	struct ub954_rxport *rxport;
	unsigned int ub954_asd_size = sizeof(struct ub954_asd);
	int port;
	int ret;

	if (!state->rxport[0].fwnode && !state->rxport[1].fwnode)
		return 0;

	dev_dbg(dev, "%s\n", __func__);

	v4l2_async_notifier_init(&state->notifier);

	for_each_port(port) {
		rxport = &state->rxport[port];
		if (!rxport->fwnode)
			continue;

		dev_dbg(dev, "%s port: %d\n", __func__, port);

		asd = v4l2_async_notifier_add_fwnode_subdev(&state->notifier,
							    rxport->fwnode,
							    ub954_asd_size);
		if (IS_ERR(asd)) {
			dev_err(dev, "Adding subdev on port: %d failed(%ld)\n",
				port, PTR_ERR(asd));
			v4l2_async_notifier_cleanup(&state->notifier);
			return PTR_ERR(asd);
		}

		to_ub954_asd(asd)->rxport = rxport;
	};

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

static int ub954_init_aeq(struct ub954 *state, struct ub954_rxport *rxport)
{
	int time_ms = 0;
	int status;
	int ret;

	if (!rxport->fwnode && !rxport->active)
		return 0;

	ret = ub954_write(rxport->i2c, UB954_PR_SFILTER_CFG,
			  BIT_SFILTER_MAX(rxport->strobe_max) |
			  BIT_SFILTER_MIN(rxport->strobe_min));
	if (ret)
		return ret;

	ret = ub954_write(rxport->i2c, UB954_PR_AEQ_MIN_MAX,
			  BIT_AEQ_MAX(rxport->eq_max) |
			  BIT_AEQ_FLOOR_VALUE(rxport->eq_min));
	if (ret)
		return ret;

	/* Reset AEQ */
	ret = ub954_set_bits(rxport->i2c, UB954_PR_AEQ_CTL2,
			     BIT_AEQ_RESTART | BIT_SET_AEQ_FLOOR);
	if (ret)
		return ret;

	while (time_ms < UB954_LOCK_TIMEOUT) {
		status = ub954_read(rxport->i2c, UB954_PR_RX_PORT_STS1);
		if ((status & BIT_LOCK_STS) &&
		    !(status & BIT_LOCK_STS_CHG))
			break;

		msleep(UB954_LOCK_STEP);
		time_ms += UB954_LOCK_STEP;
	}

	dev_dbg(&state->i2c->dev, "%s LOCK after %d ms\n", __func__, time_ms);

	return 0;
}

static int ub954_init_link(struct ub954 *state)
{
	u8 data_rate_index;

	switch (*(state->link_freq)) {
	case UB954_1600MBPS_LINK_FREQ:
		data_rate_index = 0;
		break;
	case UB954_800MBPS_LINK_FREQ:
		data_rate_index = 2;
		break;
	case UB954_400MBPS_LINK_FREQ:
		data_rate_index = 3;
		break;
	default:
		return -EINVAL;
	}

	return ub954_write(state->i2c, UB954_SR_CSI_PLL_CTL,
			  data_rate_index);
}

static int ub954_init_serializer(struct ub954 *state)
{
	struct i2c_client *new;
	struct device *dev = &state->i2c->dev;
	struct ub954_rxport *rxport;
	int port;
	int ret;

	for_each_port(port) {
		char *name;
		int addr, index;
		rxport = &state->rxport[port];

		switch (port) {
		case 0:
			name = "rx0_id";
			addr = 0x70;
			break;
		case 1:
			name = "rx1_id";
			addr = 0x71;
			break;
		default:
			return -EINVAL;
		}

		new = i2c_new_ancillary_device(state->i2c, name, addr);
		if (IS_ERR(new)) {
			return PTR_ERR(new);
		}

		ret = ub954_write(state->i2c, UB954_SR_I2C_RX0_ID + port,
				  BIT_RX_PORT_ID(new->addr));
		if (ret)
			return ret;

		rxport->i2c = new;

		ret = ub954_init_aeq(state, rxport);
		if (ret)
			return ret;

		if (!rxport->active)
			continue;

		addr = ub954_read(rxport->i2c, UB954_PR_SER_ID);
		if (addr < 0)
			return addr;

		addr &= BIT_SER_ID_MASK;
		rxport->ser_alias_id = addr;
		if (addr == 0) {
			dev_err(dev, "Serializer port %d not connected\n",
				port);
			return -EINVAL;
		}

		for (index = 0; index < rxport->num_devs; index++) {
			addr = rxport->i2c_dev[index].i2c_addr;
			addr <<= BIT_SLAVE_ID_SHIFT;
			addr &= BIT_SLAVE_ID_MASK;
			ret = ub954_write(rxport->i2c,
					  UB954_PR_SLAVE_ID_BASE + index,
					  addr);
			if (ret)
				return ret;
		}

		if (rxport->num_devs > 0)
			ret = ub954_set_bits(rxport->i2c, UB954_PR_BCC_CONFIG,
					     BIT_I2C_PASS_THRU);
		else
			ret = ub954_set_bits(rxport->i2c, UB954_PR_BCC_CONFIG,
					     BIT_I2C_PASS_THRU_ALL);

		if (ret)
			return ret;
	}

	return 0;
}

static int ub954_reset(struct ub954 *state)
{
	int ret;

	ret = ub954_write(state->i2c, UB954_SR_RESET,
			 BIT_DIGITAL_RESET0 | BIT_DIGITAL_RESET1);

	msleep(state->reset_delay_ms);

	return ret;
}

static void ub954_cleanup_ports(struct ub954 *state)
{
	struct ub954_rxport *rxport;
	int port;

	for_each_port(port) {
		rxport = &state->rxport[port];
		if (rxport->i2c)
			i2c_unregister_device(rxport->i2c);

		fwnode_handle_put(rxport->fwnode);
		fwnode_handle_put(rxport->endpoint);
		rxport->fwnode = NULL;
		rxport->endpoint = NULL;
		rxport->active = false;
		rxport->num_devs = 0;
	}
}

static int ub954_init(struct ub954 *state)
{
	int ret;

	ret = ub954_reset(state);
	if (ret)
		return ret;

	ret = ub954_init_serializer(state);
	if (ret)
		goto out;

	ret = ub954_init_link(state);
	if (ret)
		goto out;

	ret = ub954_v4l2_notifier_register(state);
	if (ret)
		goto out;

	ret = ub954_i2c_mux_init(state);
	if (ret)
		goto out_notifier;

	return 0;

out_notifier:
	v4l2_async_notifier_unregister(&state->notifier);
	v4l2_async_notifier_cleanup(&state->notifier);
out:
	ub954_cleanup_ports(state);
	return ret;
}

static void ub954_parse_port_properties(struct ub954_rxport *rxport,
					struct device_node *node)
{
	rxport->strobe_min = UB954_DEFAULT_STROBE_MIN;
	of_property_read_u32(node, "ti,aeq-strobe-min", &rxport->strobe_min);
	rxport->strobe_min = clamp_t(unsigned int, rxport->strobe_min, 0, 14);

	rxport->strobe_max = UB954_DEFAULT_STROBE_MAX;
	of_property_read_u32(node, "ti,aeq-strobe-max", &rxport->strobe_max);
	rxport->strobe_max = clamp_t(unsigned int, rxport->strobe_max, 0, 14);

	rxport->eq_min = UB954_DEFAULT_EQ_MIN;
	of_property_read_u32(node, "ti,aeq-eq-min", &rxport->eq_min);
	rxport->eq_min = clamp_t(unsigned int, rxport->eq_min, 0, 14);

	rxport->eq_max = UB954_DEFAULT_EQ_MAX;
	of_property_read_u32(node, "ti,aeq-eq-max", &rxport->eq_max);
	rxport->eq_max = clamp_t(unsigned int, rxport->eq_max, 0, 14);
}

static void ub954_parse_i2c_devices(struct ub954 *state, u32 port_id,
				    struct device_node *i2c_node)
{
	struct device_node *node;
	struct device *dev = &state->i2c->dev;
	struct ub954_rxport *rxport = &state->rxport[port_id];
	int index;

	for_each_child_of_node(i2c_node, node) {
		u32 i2c_addr;
		u32 i2c_alias;

		if (!of_device_is_available(node))
			continue;

		if (rxport->num_devs >= UB954_MAX_I2C_DEVS) {
			dev_warn(dev, "No more than %d i2c devices per port\n",
				 UB954_MAX_I2C_DEVS);
			continue;
		}

		of_property_read_u32(node, "reg", &i2c_addr);

		index = of_property_match_string(node, "reg-names", "alias");
		if (index >= 0)
			of_property_read_u32_index(node, "reg", index,
						   &i2c_alias);
		else
			i2c_alias = i2c_addr;

		rxport->i2c_dev[rxport->num_devs].i2c_addr = i2c_addr;
		rxport->i2c_dev[rxport->num_devs].i2c_alias = i2c_alias;
		rxport->num_devs++;

		dev_dbg(dev, "%s: port: %d i2c addr: 0x%02x (0x%02x)\n",
			__func__, port_id, i2c_addr, i2c_alias);
	}
	of_node_put(node);
}

static int ub954_parse_src_pad(struct device *dev, struct ub954 *state,
			       struct fwnode_handle *fwnode)
{
	struct v4l2_fwnode_endpoint vep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	u64 *link_freq;
	int ret;

	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &vep);
	if (ret)
		return ret;

	if (vep.nr_of_link_frequencies != 1) {
		dev_err(dev, "MIPI link frequency required\n");
		ret = -EINVAL;
		goto out;
	}

	if (vep.link_frequencies[0] != UB954_400MBPS_LINK_FREQ &&
	    vep.link_frequencies[0] != UB954_800MBPS_LINK_FREQ &&
	    vep.link_frequencies[0] != UB954_1600MBPS_LINK_FREQ) {
		dev_err(dev, "Unsupported link frequency\n");
		ret = -EINVAL;
		goto out;
	};

	link_freq = devm_kcalloc(dev, 1, sizeof(*link_freq),
				  GFP_KERNEL);
	if (!link_freq) {
		ret = -ENOMEM;
		goto out;
	}

	*link_freq = vep.link_frequencies[0];
	state->link_freq = link_freq;

	state->num_lanes = vep.bus.mipi_csi2.num_data_lanes;
	if (state->num_lanes != 1 &&
	    state->num_lanes != 2 &&
	    state->num_lanes != 4) {
		dev_err(dev, "Incompatible number of lanes\n");
		ret = -EINVAL;
		goto out;
	}

out:
	v4l2_fwnode_endpoint_free(&vep);
	return ret;
}

static int ub954_of_probe(struct device *dev, struct ub954 *state)
{
	struct device_node *i2c_mux;
	struct device_node * node = NULL;
	int ret;

	/* Balance the of_node_put() performed by of_find_node_by_name(). */
	of_node_get(dev->of_node);
	i2c_mux = of_find_node_by_name(dev->of_node, "i2c-mux");
	if (!i2c_mux) {
		dev_err(dev, "Failed to find i2c-mux node\n");
		return -EINVAL;
	}

	state->reset_delay_ms = UB954_DEFAULT_RESET_DELAY_MS;
	of_property_read_u32(dev->of_node, "ti,reset-delay-ms",
			     &state->reset_delay_ms);

	for_each_child_of_node(i2c_mux, node) {
		u32 id = 0;

		of_property_read_u32(node, "reg", &id);
		if (id >= UB954_NUM_PORTS) {
			dev_warn(dev, "No more than %d ports supported\n",
				 UB954_NUM_PORTS);
			continue;
		}

		if (!of_device_is_available(node)) {
			dev_dbg(dev, "Skipping disabled I2C bus port %u\n", id);
			continue;
		}

		state->rxport[id].active = true;
		state->rxport[id].port = id;

		ub954_parse_i2c_devices(state, id, node);
	}
	of_node_put(node);
	of_node_put(i2c_mux);

	for_each_endpoint_of_node(dev->of_node, node) {
		struct of_endpoint ep;
		struct device_node *port_node;
		struct fwnode_handle *fwnode, *fwremote, *endpoint;

		of_graph_parse_endpoint(node, &ep);

		if (ep.port > UB954_NUM_PADS) {
			dev_warn(dev, "Invalid port %d", ep.port);
			continue;
		}

		fwnode = of_fwnode_handle(node);
		if (ep.port == UB954_SOURCE_PAD) {
			ret = ub954_parse_src_pad(dev, state, fwnode);
			if (ret) {
				of_node_put(node);
				goto out;
			}
			continue;
		}

		if (state->rxport[ep.port].fwnode) {
			dev_warn(dev, "Only one endpoint per port supported\n");
			continue;
		}

		port_node = of_get_parent(node);
		ub954_parse_port_properties(&state->rxport[ep.port], port_node);
		of_node_put(port_node);

		fwremote = fwnode_graph_get_remote_port_parent(fwnode);
		if (!fwremote) {
			dev_warn(dev, "Endpoint %pOF has no remote endpoint\n",
				 ep.local_node);
			continue;
		}
		endpoint = fwnode_graph_get_remote_endpoint(fwnode);
		state->rxport[ep.port].fwnode = fwremote;
		state->rxport[ep.port].endpoint = endpoint;
	}
	of_node_put(node);

	return 0;
out:
	ub954_cleanup_ports(state);
	return ret;
}

static int ub954_probe(struct i2c_client *i2c, const struct i2c_device_id *did)
{
	struct ub954 *state;
	struct device *dev = &i2c->dev;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl *ctrl;
	int ret;

	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	sd = &state->subdev;
	state->i2c = i2c;

	ret = ub954_of_probe(dev, state);
	if (ret)
		return ret;

	mutex_init(&state->lock);

	v4l2_ctrl_handler_init(&state->ctrls, 2);

	ctrl = v4l2_ctrl_new_int_menu(&state->ctrls, &ub954_ctrl_ops,
				      V4L2_CID_LINK_FREQ, 0, 0,
				      state->link_freq);
	ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrl = v4l2_ctrl_new_std(&state->ctrls, &ub954_ctrl_ops,
				 V4L2_CID_PIXEL_RATE, 0, INT_MAX, 1, 0);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	sd->ctrl_handler = &state->ctrls;

	v4l2_i2c_subdev_init(sd, i2c, &ub954_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &ub954_subdev_internal_ops;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &ub954_entity_ops;

	state->pad[UB954_PORT0_PAD].flags = MEDIA_PAD_FL_SINK;
	state->pad[UB954_PORT1_PAD].flags = MEDIA_PAD_FL_SINK;
	state->pad[UB954_SOURCE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, UB954_NUM_PADS, state->pad);
	if (ret)
		goto out;

	state->last_port = -1;
	state->last_chan = UB954_NUM_PORTS;
	state->active_port = -1;

	ret = ub954_init(state);
	if (ret)
		goto out;

	ret = v4l2_async_register_subdev(&state->subdev);
	if (ret)
		goto out_init;

	dev_info(dev, "Probed DS90UB954 Driver succesfully\n");

	return 0;

out_init:
	v4l2_async_notifier_unregister(&state->notifier);
	v4l2_async_notifier_cleanup(&state->notifier);
	i2c_mux_del_adapters(state->mux);
	ub954_cleanup_ports(state);
out:
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&state->lock);

	dev_err(dev, "Failed to probe DS90UB954 Driver (%d)\n", ret);
	return ret;
}

static int ub954_remove(struct i2c_client *i2c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(i2c);
	struct ub954 *state = to_ub954(sd);

	v4l2_async_notifier_unregister(&state->notifier);
	v4l2_async_notifier_cleanup(&state->notifier);
	v4l2_async_unregister_subdev(sd);
	i2c_mux_del_adapters(state->mux);

	ub954_cleanup_ports(state);

	media_entity_cleanup(&sd->entity);
	mutex_destroy(&state->lock);

	return 0;
}

static const struct of_device_id ub954_of_match[] = {
        { .compatible = "ti,ds90ub954" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ub954_of_match);

static struct i2c_driver ub954_i2c_driver = {
        .driver         = {
                .name   = "ds90ub954",
                .of_match_table = of_match_ptr(ub954_of_match),
        },
        .probe          = ub954_probe,
        .remove         = ub954_remove,
};
module_i2c_driver(ub954_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stefan Riedmueller <s.riedmueller@phytec.de>");
