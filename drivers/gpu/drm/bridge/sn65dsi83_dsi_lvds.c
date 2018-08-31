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

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <drm/drm_mipi_dsi.h>

#define LVDS_REG_SW_RST			0x09		/* SOFT_RESET */
#define LVDS_REG_CLK_RANGE		0x0A		/* LVDS output clock range */
#define LVDS_REG_DSI_CLK_DIVIDER	0x0B		/* divider or multiplier for mipi-clk */
#define LVDS_REG_PLL_EN			0x0D		/* PLL enable */
#define LVDS_REG_CHA_DSI_LANES		0x10		/* number of DSI lanes */
#define LVDS_CHA_DSI_EQ			0x11
#define LVDS_REG_CHA_DSI_CLK_RANGE	0x12		/* DSI clock frequency range */

/* CST register */
#define CHA_24BPP_MODE			0x18
#define CHA_LVDS_SWING			0x19
#define CHA_REVERSE_LVDS		0x1a
#define CHA_LVDS_CM_ADJUST		0x1b
#define CHA_ACTIVE_LINE_LENGTH_LOW	0x20
#define CHA_ACTIVE_LINE_LENGTH_HIGH	0x21
#define CHA_VERTICAL_DISPLAY_SIZE_LOW	0x24
#define CHA_VERTICAL_DISPLAY_SIZE_HIGH	0x25
#define CHA_SYNC_DELAY_LOW		0x28
#define CHA_SYNC_DELAY_HIGH		0x29
#define CHA_HSYNC_PULSE_WIDTH_LOW	0x2c
#define CHA_HSYNC_PULSE_WIDTH_HIGH	0x2d
#define CHA_VSYNC_PULSE_WIDTH_LOW	0x30
#define CHA_VSYNC_PULSE_WIDTH_HIGH	0x31
#define CHA_HORIZONTAL_BACK_PORCH	0x34
#define CHA_VERTICAL_BACK_PORCH		0x36
#define CHA_HORIZONTAL_FRONT_PORCH	0x38
#define CHA_VERTICAL_FRONT_PORCH	0x3a

/* Test pattern register */
#define CHA_TEST_PATTERN		0x3c


struct sn65dsi83 {
	struct i2c_client *i2c;
	struct regmap *i2c_regmap;
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

static void sn65dsi83_lvds_set_interface(struct sn65dsi83 *sn65dsi83)
{
	struct regmap *regmap = sn65dsi83->i2c_regmap;

	/* CLK configuration */
	regmap_write(regmap, LVDS_REG_SW_RST, 0x00);
	regmap_write(regmap, LVDS_REG_CLK_RANGE, 0x05);
	regmap_write(regmap, LVDS_REG_DSI_CLK_DIVIDER, 0x10);
	regmap_write(regmap, LVDS_REG_CHA_DSI_LANES, 0x26);
	regmap_write(regmap, LVDS_CHA_DSI_EQ, 0x00);
	regmap_write(regmap, LVDS_REG_CHA_DSI_CLK_RANGE, 0x2a);

	/*CSR configuration */
	regmap_write(regmap, CHA_24BPP_MODE, 0x78);
	regmap_write(regmap, CHA_LVDS_SWING, 0x00);
	regmap_write(regmap, CHA_REVERSE_LVDS, 0x03);
	regmap_write(regmap, CHA_LVDS_CM_ADJUST, 0x00);
	regmap_write(regmap, CHA_ACTIVE_LINE_LENGTH_LOW, 0x00);
	regmap_write(regmap, CHA_ACTIVE_LINE_LENGTH_HIGH, 0x05);
	regmap_write(regmap, CHA_VERTICAL_DISPLAY_SIZE_LOW, 0x00);
	regmap_write(regmap, CHA_VERTICAL_DISPLAY_SIZE_HIGH, 0x00);
	regmap_write(regmap, CHA_SYNC_DELAY_LOW, 0x20);
	regmap_write(regmap, CHA_SYNC_DELAY_HIGH, 0x00);
	regmap_write(regmap, CHA_HSYNC_PULSE_WIDTH_LOW, 0x32);
	regmap_write(regmap, CHA_HSYNC_PULSE_WIDTH_HIGH, 0x00);
	regmap_write(regmap, CHA_VSYNC_PULSE_WIDTH_LOW, 0x05);
	regmap_write(regmap, CHA_VSYNC_PULSE_WIDTH_HIGH, 0x00);
	regmap_write(regmap, CHA_HORIZONTAL_BACK_PORCH, 0x32);
	regmap_write(regmap, CHA_VERTICAL_BACK_PORCH, 0x00);
	regmap_write(regmap, CHA_HORIZONTAL_FRONT_PORCH, 0x00);
	regmap_write(regmap, CHA_VERTICAL_FRONT_PORCH, 0x00);
	regmap_write(regmap, LVDS_REG_SW_RST, 0x01);

	/* Enable PLL*/
	regmap_write(regmap, LVDS_REG_PLL_EN, 0x01);

}
static int sn65dsi83_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sn65dsi83 *sn65dsi83;
	int ret = 0;

	sn65dsi83 = devm_kzalloc(dev, sizeof(*sn65dsi83), GFP_KERNEL);
	if (!sn65dsi83)
		return -ENOMEM;

	sn65dsi83->i2c = client;
	sn65dsi83->i2c_regmap = devm_regmap_init_i2c(client,
					&sn65dsi83_lvds_regmap_config);
	if (IS_ERR(sn65dsi83->i2c)) {
		ret = PTR_ERR(sn65dsi83->i2c_regmap);
		return ret;
	}

	//evtl. sw reset

	sn65dsi83_lvds_set_interface(sn65dsi83);

	return 0;
};

static int sn65dsi83_remove(struct i2c_client *client)
{
	struct sn65dsi83 *sn65dsi83 = i2c_get_clientdata(client);

	i2c_unregister_device(sn65dsi83->i2c);

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
