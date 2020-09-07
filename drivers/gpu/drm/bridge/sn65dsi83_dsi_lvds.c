// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 PHYTEC Messtechnik GmbH
 * Author: Janine Hagemann <j.hagemann@phytec.de>
 */

#include <linux/i2c.h>
#include <linux/regmap.h>

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/gpio/consumer.h>

#include <video/mipi_display.h>
#include <video/videomode.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

/* ID Register */
#define ID0_REGISTER				0x00 /* ID 0 Register */

/* Reset and Clock Registers */
#define REG_SOFT_RESET				0x09 /* REG_SOFT_RESET */
#define REG_HS_CLK_SRC				0x0A /* LVDS output clock */
#define REG_REFCLK_MULTIPLIER			0x0B /* divider for mipi-clk */
#define REG_PLL_EN				0x0D /* PLL enable */

/* DSI Registers */
#define REG_SOT_ERR_TOL_DIS			0x10
#define REG_CHA_DSI_CLK_RANGE			0x12 /* DSI clock settings */

/* LVDS Registers */
#define REG_CHA_24BPP_FORMAT1			0x18
#define REG_CHA_LVDS_VOD_SWING			0x19
#define REG_CHA_LVDS_TERM			0x1A

/* Video Registers */
#define REG_CHA_ACTIVE_LINE_LENGTH_LOW		0x20
#define REG_CHA_ACTIVE_LINE_LENGTH_HIGH		0x21
#define REG_CHA_SYNC_DELAY_LOW			0x28
#define REG_CHA_SYNC_DELAY_HIGH			0x29
#define REG_CHA_HSYNC_PULSE_WIDTH_LOW		0x2C
#define REG_CHA_HSYNC_PULSE_WIDTH_HIGH		0x2D
#define REG_CHA_VSYNC_PULSE_WIDTH_LOW		0x30
#define REG_CHA_VSYNC_PULSE_WIDTH_HIGH		0x31
#define REG_CHA_HORIZONTAL_BACK_PORCH		0x34
#define REG_CHA_VERTICAL_BACK_PORCH		0x36
#define REG_CHA_TEST_PATTERN			0x3C

/* Register Mask */
#define MASK_CHA_DSI_LANES			0x18
#define MASK_CHA_24BPP				0x08
#define MASK_LVDS_CLK_RANGE			0x0E
#define MASK_LVDS_VOD_SWING			0x0C
#define MASK_CHA_REVERS_LVDS			0x20
#define MASK_CHA_LVDS_TERM			0x02
#define MASK_VS_NEG_POLARITY			0x20
#define MASK_HS_NEG_POLARITY			0x40
#define MASK_LOW				0xFF

/* Register Shift */
#define SHIFT_LVDS_VOD_SWING			0x02
#define SHIFT_REFCLK_MULTIPLIER			0x03
#define SHIFT_HIGH				0x08

/* Register Value */
#define SOFT_RESET_EN				0x01
#define PLL_EN					0x01
#define CHA_24BPP_MODE24			0x08
#define CHA_24BPP_MODE18			0x00
#define VS_POS_POL				0x00
#define HS_POS_POL				0x00
#define CHA_SYNC_DELAY_LOW			0x28
#define CHA_SYNC_DELAY_HIGH			0x00
#define CHA_REVERSE_LVDS			0x00
#define CHA_LVDS_TERM_100			0x00

#define CLK_RANGE_STEP				0x1388

#define NUM_DSI_LANES4				0x00
#define NUM_DSI_LANES3				0x08
#define NUM_DSI_LANES2				0x10
#define NUM_DSI_LANES1				0x18

/* ID Register Values */
static u8 id_reg_val[] = {0x35, 0x38, 0x49, 0x53, 0x44, 0x20,
					0x20, 0x20, 0x01};
#define NUM_ID_REGS			ARRAY_SIZE(id_reg_val)

struct sn65dsi83 {
	struct device *dev;
	struct regmap *regmap;

	struct drm_connector connector;
	struct drm_bridge bridge;
	struct drm_panel *panel;
	bool enabled;
	struct gpio_desc *gpio_enable;
	struct device_node *host_node;
	struct mipi_dsi_device *dsi;
	struct drm_display_mode mode;
	u32 num_dsi_lanes;
	u32 lvds_vod_swing;
};

static inline struct sn65dsi83 *
		bridge_to_sn65dsi83(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sn65dsi83, bridge);
}

static inline struct sn65dsi83 *
		connector_to_sn65dsi83(struct drm_connector *connector)
{
	return container_of(connector, struct sn65dsi83, connector);
}

static void sn65dsi83_enable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *sn_bridge = bridge_to_sn65dsi83(bridge);

	if (drm_panel_enable(sn_bridge->panel)) {
		DRM_ERROR("failed to enable panel\n");
		return;
	}
}

static void sn65dsi83_disable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *sn_bridge = bridge_to_sn65dsi83(bridge);

	if (!sn_bridge->enabled)
		return;

	sn_bridge->enabled = false;

	if (drm_panel_disable(sn_bridge->panel)) {
		DRM_ERROR("failed to disable panel\n");
		return;
	}
}

static int sn65dsi83_get_modes(struct drm_connector *connector)
{
	struct sn65dsi83 *sn_bridge = container_of(connector,
					struct sn65dsi83, connector);

	if (sn_bridge->panel)
		return drm_panel_get_modes(sn_bridge->panel);

	DRM_ERROR("no panel found\n");
	return -ENODEV;
}

static enum drm_connector_status drm_sn65dsi83_detect(struct drm_connector
						*connector, bool force)
{
	return connector_status_connected;
}

static const struct drm_connector_funcs sn65dsi83_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = drm_sn65dsi83_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs sn65dsi83_helper_funcs = {
	.get_modes = sn65dsi83_get_modes,
};

static void sn65dsi83_mode_set(struct drm_bridge *bridge,
				const struct drm_display_mode *mode,
				const struct drm_display_mode *adj_mode)
{
	struct sn65dsi83 *sn_bridge = bridge_to_sn65dsi83(bridge);

	drm_mode_copy(&sn_bridge->mode, mode);
}


static void sn65dsi83_pre_enable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *sn_bridge = bridge_to_sn65dsi83(bridge);
	struct mipi_dsi_device *dsi = sn_bridge->dsi;
	struct drm_display_mode *mode = &sn_bridge->mode;
	int clk_range, mipi_clk, lvds_clk, clk_div;
	int bpp, lanes, i;
	char lvds_clk_range;
	int clk[6] = {37500, 62500, 87500, 112500, 137500, 154000};
	int clk_value[6] = {0x0, 0x2, 0x4, 0x6, 0x8, 0xA};

	gpiod_set_value_cansleep(sn_bridge->gpio_enable, 0);
	usleep_range(10 * 1000, 11 * 1000);
	gpiod_set_value_cansleep(sn_bridge->gpio_enable, 1);
	usleep_range(10 * 1000, 11 * 1000);

	/* Configure LVDS output voltage */
	regmap_update_bits(sn_bridge->regmap, REG_CHA_LVDS_VOD_SWING,
			MASK_LVDS_VOD_SWING,
			sn_bridge->lvds_vod_swing << SHIFT_LVDS_VOD_SWING);

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	if (bpp == 24)
		regmap_update_bits(sn_bridge->regmap, REG_CHA_24BPP_FORMAT1,
					MASK_CHA_24BPP, CHA_24BPP_MODE24);
	else
		regmap_update_bits(sn_bridge->regmap, REG_CHA_24BPP_FORMAT1,
					MASK_CHA_24BPP, CHA_24BPP_MODE18);

	if (!mode) {
		DRM_ERROR("failed to get displaymode\n");
		return;
	}

	lvds_clk = mode->clock;
	/* calculate current mipi_clk with the pixelclock entry from
	 * panel-simple.c, bpp and the number of dsi lanes
	 */
	mipi_clk = ((mode->clock * bpp) / (sn_bridge->num_dsi_lanes * 2));

	clk_range = mipi_clk / CLK_RANGE_STEP;

	/* calculate the needed clock divider to set device
	 * configuration
	 */
	clk_div =  (mipi_clk / lvds_clk) - 1;

	lvds_clk_range = clk_value[5];
	for (i = 5; i >= 0; i--) {
		if (lvds_clk < clk[i])
			lvds_clk_range = clk_value[i];
		else
			break;
	}

	regmap_update_bits(sn_bridge->regmap, REG_HS_CLK_SRC,
				MASK_LVDS_CLK_RANGE, lvds_clk_range);
	regmap_update_bits(sn_bridge->regmap, REG_HS_CLK_SRC,
				0x1, 0x1);
	regmap_write(sn_bridge->regmap, REG_REFCLK_MULTIPLIER,
			clk_div << SHIFT_REFCLK_MULTIPLIER);

	switch (dsi->lanes) {
	case 4:
		lanes = NUM_DSI_LANES4;
		break;
	case 3:
		lanes = NUM_DSI_LANES3;
		break;
	case 2:
		lanes = NUM_DSI_LANES2;
		break;
	default:
		lanes = NUM_DSI_LANES1;
		break;
	}

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		regmap_update_bits(sn_bridge->regmap, REG_CHA_24BPP_FORMAT1,
					MASK_VS_NEG_POLARITY, VS_POS_POL);
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		regmap_update_bits(sn_bridge->regmap, REG_CHA_24BPP_FORMAT1,
					MASK_HS_NEG_POLARITY, HS_POS_POL);

	regmap_update_bits(sn_bridge->regmap, REG_SOT_ERR_TOL_DIS,
				MASK_CHA_DSI_LANES, lanes);
	regmap_write(sn_bridge->regmap, REG_CHA_DSI_CLK_RANGE, clk_range);
	regmap_update_bits(sn_bridge->regmap, REG_CHA_LVDS_TERM,
				MASK_CHA_REVERS_LVDS, CHA_REVERSE_LVDS);
	regmap_update_bits(sn_bridge->regmap, REG_CHA_LVDS_TERM,
				MASK_CHA_LVDS_TERM, CHA_LVDS_TERM_100);
	regmap_write(sn_bridge->regmap, REG_CHA_ACTIVE_LINE_LENGTH_LOW,
			mode->hdisplay & MASK_LOW);
	regmap_write(sn_bridge->regmap, REG_CHA_ACTIVE_LINE_LENGTH_HIGH,
			mode->hdisplay >> SHIFT_HIGH);
	regmap_write(sn_bridge->regmap, REG_CHA_SYNC_DELAY_LOW,
			CHA_SYNC_DELAY_LOW);
	regmap_write(sn_bridge->regmap, REG_CHA_SYNC_DELAY_HIGH,
			CHA_SYNC_DELAY_HIGH);
	regmap_write(sn_bridge->regmap, REG_CHA_HSYNC_PULSE_WIDTH_LOW,
			(mode->hsync_end - mode->hsync_start) & MASK_LOW);
	regmap_write(sn_bridge->regmap, REG_CHA_HSYNC_PULSE_WIDTH_HIGH,
			(mode->hsync_end - mode->hsync_start) >> SHIFT_HIGH);
	regmap_write(sn_bridge->regmap, REG_CHA_VSYNC_PULSE_WIDTH_LOW,
			(mode->vsync_end - mode->vsync_start) & MASK_LOW);
	regmap_write(sn_bridge->regmap, REG_CHA_VSYNC_PULSE_WIDTH_HIGH,
			(mode->vsync_end - mode->vsync_start) >> SHIFT_HIGH);
	regmap_write(sn_bridge->regmap, REG_CHA_HORIZONTAL_BACK_PORCH,
			mode->htotal - mode->hsync_end);
	regmap_write(sn_bridge->regmap, REG_CHA_VERTICAL_BACK_PORCH,
			mode->vtotal - mode->vsync_end);
	regmap_write(sn_bridge->regmap, REG_PLL_EN, PLL_EN);
	mdelay(10);
	regmap_write(sn_bridge->regmap, REG_SOFT_RESET, SOFT_RESET_EN);

	drm_panel_prepare(sn_bridge->panel);

}

static int sn65dsi83_bridge_attach(struct drm_bridge *bridge)
{
	struct sn65dsi83 *sn_bridge = bridge_to_sn65dsi83(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = {
						   .type = "sn65dsi83",
						   .node = NULL,
						 };
	int ret = 0;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found\n");
		return -ENODEV;
	}

	sn_bridge->connector.polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(bridge->dev, &sn_bridge->connector,
			&sn65dsi83_connector_funcs, DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR("failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&sn_bridge->connector,
				&sn65dsi83_helper_funcs);
	drm_connector_attach_encoder(&sn_bridge->connector,
						bridge->encoder);

	host = of_find_mipi_dsi_host_by_node(sn_bridge->host_node);
	if (!host) {
		DRM_ERROR("failed to find dsi host\n");
		return ret;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		return ret;
	}

	dsi->lanes = sn_bridge->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_ERROR("failed to attach dsi to host\n");
		mipi_dsi_device_unregister(dsi);
	}

	sn_bridge->dsi = dsi;

	ret = drm_panel_attach(sn_bridge->panel, &sn_bridge->connector);
	if (ret) {
		DRM_ERROR("failed to attach panel\n");
		drm_connector_cleanup(&sn_bridge->connector);
		return ret;
	}

	return 0;
}

static const struct drm_bridge_funcs sn65dsi83_bridge_funcs = {
	.enable = sn65dsi83_enable,
	.disable = sn65dsi83_disable,
	.mode_set = sn65dsi83_mode_set,
	.attach = sn65dsi83_bridge_attach,
	.pre_enable = sn65dsi83_pre_enable,
};

static const struct regmap_range sn65dsi83_lvds_volatile_ranges[] = {
	{ .range_min = 0x00, .range_max = 0x0B},
	{ .range_min = 0x0D, .range_max = 0x0D},
	{ .range_min = 0x10, .range_max = 0x12},
	{ .range_min = 0x18, .range_max = 0x1B},
	{ .range_min = 0x20, .range_max = 0x3C},
	{ .range_min = 0xE0, .range_max = 0xE5},
};

static const struct regmap_access_table sn65dsi83_lvds_volatile_table = {
	.yes_ranges = sn65dsi83_lvds_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(sn65dsi83_lvds_volatile_ranges),
};

static const struct regmap_config sn65dsi83_lvds_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &sn65dsi83_lvds_volatile_table,
	.cache_type = REGCACHE_NONE,
	.max_register = 0xE5,
};

void sn65dsi83_detach_dsi(struct sn65dsi83 *sn_bridge)
{
	mipi_dsi_detach(sn_bridge->dsi);
	mipi_dsi_device_unregister(sn_bridge->dsi);
}

int sn65dsi83_parse_dt(struct sn65dsi83 *sn_bridge)
{
	struct device *dev = sn_bridge->dev;
	struct device_node *np = dev->of_node;
	int ret;

	sn_bridge->host_node = of_graph_get_remote_node(np, 1, 0);
	if (!sn_bridge->host_node) {
		return -ENODEV;
	}

	sn_bridge->gpio_enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(sn_bridge->gpio_enable)) {
		DRM_ERROR("failed to parse enable gpio");
		return PTR_ERR(sn_bridge->gpio_enable);
	}

	ret = of_property_read_u32(np, "lanes_in",
				&sn_bridge->num_dsi_lanes);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "lvds_vod_swing",
				&sn_bridge->lvds_vod_swing);
	/* If not set, use default */
	if (ret)
		sn_bridge->lvds_vod_swing = 0x01;

	return 0;
}

static int sn65dsi83_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sn65dsi83 *sn_bridge;
	u8 id_reg[NUM_ID_REGS];
	int ret, i;

	sn_bridge = devm_kzalloc(dev, sizeof(*sn_bridge), GFP_KERNEL);
	if (!sn_bridge)
		return -ENOMEM;

	sn_bridge->regmap = devm_regmap_init_i2c(client,
					&sn65dsi83_lvds_regmap_config);
	if (IS_ERR(sn_bridge->regmap))
		return  PTR_ERR(sn_bridge->regmap);

	ret = drm_of_find_panel_or_bridge(dev->of_node, 0, 0,
					  &sn_bridge->panel, NULL);
	if (ret) {
		DRM_ERROR("could not find any panel node\n");
		return -EPROBE_DEFER;
	}

	sn_bridge->dev = &client->dev;

	dev_set_drvdata(&client->dev, sn_bridge);

	ret = sn65dsi83_parse_dt(sn_bridge);
	if (ret)
		return -EINVAL;

	ret = regmap_raw_read(sn_bridge->regmap, ID0_REGISTER, &id_reg,
			NUM_ID_REGS);
	if (ret) {
		dev_err(dev, "can not read device ID: %d\n", ret);
		return ret;
	}

	for (i = 0; i < NUM_ID_REGS; i++) {
		if (id_reg[i] != id_reg_val[i]) {
			dev_err(dev, "Invalid device ID.");
			return -ENODEV;
		}
	}

	i2c_set_clientdata(client, sn_bridge);
	sn_bridge->bridge.funcs = &sn65dsi83_bridge_funcs;
	sn_bridge->bridge.of_node = dev->of_node;
	drm_bridge_add(&sn_bridge->bridge);

	dev_info(dev, "Successfully probed sn65dsi83.");

	return 0;
}

static int sn65dsi83_remove(struct i2c_client *client)
{
	struct sn65dsi83 *sn_bridge = i2c_get_clientdata(client);

	i2c_set_clientdata(client, sn_bridge);
	sn65dsi83_detach_dsi(sn_bridge);
	drm_bridge_remove(&sn_bridge->bridge);

	return 0;
}

static const struct of_device_id sn65dsi83_dt_ids[] = {
	{ .compatible = "ti,sn65dsi83", },
	{ }
};
MODULE_DEVICE_TABLE(of, sn65dsi83_dt_ids);

static const struct i2c_device_id sn65dsi83_i2c_ids[] = {
	{ "sn65dsi83", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sn65dsi83_i2c_ids);

static struct i2c_driver sn65dsi83_driver = {
	.driver = {
		.name = "sn65dsi83",
		.of_match_table = sn65dsi83_dt_ids,
	},
	.id_table = sn65dsi83_i2c_ids,
	.probe = sn65dsi83_probe,
	.remove = sn65dsi83_remove,
};
module_i2c_driver(sn65dsi83_driver);

MODULE_AUTHOR("Janine Hagemann <j.hagemann@phytec.de>");
MODULE_DESCRIPTION("SN65DSI83 MIPI/LVDS transmitter driver");
MODULE_LICENSE("GPL v2");
