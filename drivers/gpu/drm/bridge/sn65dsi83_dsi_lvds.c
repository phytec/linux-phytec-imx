/*
 * Copyright (C) 2018 PHYTEC Messtechnik GmbH
 * Author: Janine Hagemann <j.hagemann@phytec.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/regmap.h>

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>

#include <video/mipi_display.h>
#include <video/videomode.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>

/* Reset and Clock Registers */
#define LVDS_REG_SW_RST			0x09	/* SOFT_RESET */
#define LVDS_REG_CLK_RANGE		0x0A	/* LVDS output clock range */
#define LVDS_REG_DSI_CLK_DIVIDER	0x0B	/* divider for mipi-clk */
#define LVDS_REG_PLL_EN			0x0D	/* PLL enable */

/* DSI Registers */
#define LVDS_REG_DSI_LANES		0x10	/* number of DSI lanes */
#define LVDS_REG_DSI_CLK_RANGE		0x12	/* DSI clock frequency range */

/* LVDS Registers */
#define LVDS_REG_24BPP				0x18
#define LVDS_REG_REVERSE_LVDS			0x1a

/* Video Registers */
#define LVDS_REG_ACTIVE_LINE_LENGTH_LOW		0x20
#define LVDS_REG_ACTIVE_LINE_LENGTH_HIGH	0x21
#define LVDS_REG_SYNC_DELAY_LOW			0x28
#define LVDS_REG_SYNC_DELAY_HIGH		0x29
#define LVDS_REG_HSYNC_PULSE_WIDTH_LOW		0x2c
#define LVDS_REG_HSYNC_PULSE_WIDTH_HIGH		0x2d
#define LVDS_REG_VSYNC_PULSE_WIDTH_LOW		0x30
#define LVDS_REG_VSYNC_PULSE_WIDTH_HIGH		0x31
#define LVDS_REG_HORIZONTAL_BACK_PORCH		0x34
#define LVDS_REG_VERTICAL_BACK_PORCH		0x36
#define LVDS_REG_TEST_PATTERN			0x3c

/* Register Mask */
#define CHA_DSI_LANES			0x19
#define CHA_24BPP			0xEA
#define LVDS_CLK_RANGE			0x8f
#define CHA_REVERS_LVDS			0x3e

/* Register Value */
#define SOFT_RESET_EN			0x01
#define SOFT_RESET_DE			0x00
#define PLL_EN				0x01
#define CHA_24BPP_MODE24		0x08
#define CHA_24BPP_MODE18		0x00
#define LOW_MASK			0xff
#define HIGH_MASK			0x08
#define CHA_SYNC_DELAY_LOW		0x28
#define CHA_SYNC_DELAY_HIGH		0x00
#define CHA_LVDS_TERM			0x00

#define CLK_RANGE_STEP			0x1388

#define NUM_DSI_LANES4			0x00
#define NUM_DSI_LANES3			0x08
#define NUM_DSI_LANES2			0x10
#define NUM_DSI_LANES1			0x18

struct sn65dsi83 {
	struct i2c_client *i2c;
	struct regmap *i2c_regmap;

	struct drm_connector connector;
	struct drm_bridge bridge;
	struct drm_panel *panel;
	bool enabled;

	struct device_node *host_node;
	struct mipi_dsi_device *dsi;
	u32 num_dsi_lanes;
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
				struct drm_display_mode *mode,
				struct drm_display_mode *adj_mode)
{
	struct sn65dsi83 *sn_bridge = bridge_to_sn65dsi83(bridge);
	struct mipi_dsi_device *dsi = sn_bridge->dsi;
	int clk_range, mipi_clk, lvds_clk, clk_div;
	int bpp, lanes, i;
	char lvds_clk_range;
	int clk[5] = {37500, 62500, 87500, 112500, 137500};
	int clk_value[5] = {0x81, 0x83, 0x85, 0x89, 0x8b};

	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_SW_RST, SOFT_RESET_DE);
	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	if (bpp == 24)
		regmap_update_bits(sn_bridge->i2c_regmap, LVDS_REG_24BPP,
					CHA_24BPP, CHA_24BPP_MODE24);
	else
		regmap_update_bits(sn_bridge->i2c_regmap, LVDS_REG_24BPP,
					CHA_24BPP, CHA_24BPP_MODE18);

	if (!mode) {
		DRM_ERROR("failed to get displaymode\n");
		return;
	}

	/* calculate current mipi_clk with the pixelclock entry from
	 * panel-simple.c, bpp and the number of dsi lanes
	 */
	mipi_clk = (((mode->clock * bpp) / (8 * (sn_bridge->num_dsi_lanes + 1)))
			* bpp) / (2 * sn_bridge->num_dsi_lanes);

	/* Calculate the lvds clock to configure the clk_range
	 * at the sn65dsi83 device
	 */
	lvds_clk = (mipi_clk * 2 * sn_bridge->num_dsi_lanes) / bpp;
	clk_range = mipi_clk / CLK_RANGE_STEP;

	/* calculate the needed clock divider to set device
	 * configuration
	 */
	clk_div =  (mipi_clk / lvds_clk) - 1;

	for (i = 5; i >= 0; i--) {
		if (lvds_clk < clk[i])
			lvds_clk_range = clk_value[i];
		else
			lvds_clk_range = clk_value[4];
	}
	regmap_update_bits(sn_bridge->i2c_regmap, LVDS_REG_CLK_RANGE,
				LVDS_CLK_RANGE,	lvds_clk_range);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_DSI_CLK_DIVIDER,
			clk_div << 3);

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

	regmap_update_bits(sn_bridge->i2c_regmap, LVDS_REG_DSI_LANES,
				CHA_DSI_LANES, lanes);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_DSI_CLK_RANGE, clk_range);
	regmap_update_bits(sn_bridge->i2c_regmap, LVDS_REG_REVERSE_LVDS,
				CHA_REVERS_LVDS, CHA_LVDS_TERM);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_ACTIVE_LINE_LENGTH_LOW,
			mode->htotal & LOW_MASK);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_ACTIVE_LINE_LENGTH_HIGH,
			mode->htotal >> HIGH_MASK);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_SYNC_DELAY_LOW,
			CHA_SYNC_DELAY_LOW);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_SYNC_DELAY_HIGH,
			CHA_SYNC_DELAY_HIGH);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_HSYNC_PULSE_WIDTH_LOW,
			(mode->hsync_end-mode->hsync_start) & LOW_MASK);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_HSYNC_PULSE_WIDTH_HIGH,
			(mode->hsync_end-mode->hsync_start) >> HIGH_MASK);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_VSYNC_PULSE_WIDTH_LOW,
			(mode->vsync_end-mode->vsync_start) & LOW_MASK);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_VSYNC_PULSE_WIDTH_HIGH,
			(mode->vsync_end-mode->vsync_start) >> HIGH_MASK);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_HORIZONTAL_BACK_PORCH,
			mode->htotal-mode->hsync_end);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_VERTICAL_BACK_PORCH,
			mode->vtotal-mode->vsync_end);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_SW_RST, SOFT_RESET_EN);
	regmap_write(sn_bridge->i2c_regmap, LVDS_REG_PLL_EN, PLL_EN);
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
	drm_mode_connector_attach_encoder(&sn_bridge->connector,
						bridge->encoder);

	ret = drm_panel_attach(sn_bridge->panel, &sn_bridge->connector);
	if (ret) {
		DRM_ERROR("failed to attach panel\n");
		drm_connector_cleanup(&sn_bridge->connector);
		return ret;
	}

	host = of_find_mipi_dsi_host_by_node(sn_bridge->host_node);
	if (!host) {
		DRM_ERROR("failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		return ret;
	}

	sn_bridge->dsi = dsi;

	dsi->lanes = sn_bridge->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_ERROR("failed to attach dsi to host\n");
		mipi_dsi_device_unregister(dsi);
	}

	return 0;
}

static const struct drm_bridge_funcs sn65dsi83_bridge_funcs = {
	.enable = sn65dsi83_enable,
	.disable = sn65dsi83_disable,
	.mode_set = sn65dsi83_mode_set,
	.attach = sn65dsi83_bridge_attach,
};

static const struct regmap_range sn65dsi83_lvds_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0xff},
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
};

void sn65dsi83_detach_dsi(struct sn65dsi83 *sn_bridge)
{
	mipi_dsi_detach(sn_bridge->dsi);
	mipi_dsi_device_unregister(sn_bridge->dsi);
}

int sn65dsi83_parse_dt(struct device_node *np, struct sn65dsi83 *sn_bridge)
{
	struct device_node *endpoint0, *endpoint1;

	endpoint0 = of_graph_get_next_endpoint(np, NULL);
	if (!endpoint0)
		return -ENODEV;

	endpoint1 = of_graph_get_next_endpoint(np, endpoint0);
	if (!endpoint1)
		return -ENODEV;

	sn_bridge->host_node = of_graph_get_remote_port_parent(endpoint1);
	if (!sn_bridge->host_node) {
		of_node_put(endpoint1);
		return -ENODEV;
	}

	of_node_put(endpoint1);
	of_node_put(sn_bridge->host_node);

	return 0;
}

static int sn65dsi83_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sn65dsi83 *sn_bridge;
	struct device_node *endpoint, *panel_node;
	int ret = 0;


	sn_bridge = devm_kzalloc(dev, sizeof(*sn_bridge), GFP_KERNEL);
	if (!sn_bridge)
		return -ENOMEM;

	sn_bridge->i2c_regmap = devm_regmap_init_i2c(client,
					&sn65dsi83_lvds_regmap_config);
	if (IS_ERR(sn_bridge->i2c)) {
		ret = PTR_ERR(sn_bridge->i2c_regmap);
		return ret;
	}

	of_property_read_u32(dev->of_node, "lanes_in",
				&sn_bridge->num_dsi_lanes);

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (endpoint) {
		panel_node = of_graph_get_remote_port_parent(endpoint);
		if (panel_node) {
			sn_bridge->panel = of_drm_find_panel(panel_node);
			of_node_put(panel_node);
			if (!sn_bridge->panel)
				return -EPROBE_DEFER;
		}
	}

	sn65dsi83_parse_dt(dev->of_node, sn_bridge);
	sn_bridge->i2c = client;
	i2c_set_clientdata(client, sn_bridge);
	sn_bridge->bridge.funcs = &sn65dsi83_bridge_funcs;
	sn_bridge->bridge.of_node = dev->of_node;
	ret = drm_bridge_add(&sn_bridge->bridge);
	if (ret) {
		DRM_ERROR("failed to add bridge\n");
		return ret;
	}

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

static struct mipi_dsi_driver sn65dsi83_dsi_driver = {
	.driver.name = "sn65dsi83_dsi",
};

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

static int __init sn65dsi83_init(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_register(&sn65dsi83_dsi_driver);

	return i2c_add_driver(&sn65dsi83_driver);
}
module_init(sn65dsi83_init);

static void __exit sn65dsi83_exit(void)
{
	i2c_del_driver(&sn65dsi83_driver);

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&sn65dsi83_dsi_driver);
}
module_exit(sn65dsi83_exit);

MODULE_AUTHOR("Janine Hagemann <j.hagemann@phytec.de>");
MODULE_DESCRIPTION("SN65DSI83 MIPI/LVDS transmitter driver");
MODULE_LICENSE("GPL v2");
