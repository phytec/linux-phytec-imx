// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device_cooling.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#include "thermal_core.h"
#include "thermal_hwmon.h"

#define TER		0x0	/* TMU enable */
#define TSR		0x4	/* TMU status */
#define TIER		0x8	/* TMU interrupt enable */
#define TIDR		0xc	/* TMU interrupt detect */
#define TMHTITR		0x10	/* TMU high immediate threshold */
#define TMHTATR		0x14	/* TMU high average threshold */
#define TMHTCATR	0x18	/* TMU high average critical threshold */
#define TSCR		0x1c	/* TMU sensor val(raw, no calibration) */
#define TRITSR		0x20	/* TMU immediate temp */
#define TRATSR		0x24	/* TMU average temp */

#define TER_EN		BIT(31)
#define TRITSR_VALID	BIT(31)
#define TEMP_VAL_MASK	0xff

#define TEMP_LOW_LIMIT	10

#define IMX_TEMP_PASSIVE_COOL_DELTA 10000

struct imx8mm_tmu {
	struct thermal_zone_device *tzd;
	struct thermal_cooling_device *cdev;
	struct clk *clk;
	void __iomem *tmu_base;
	bool enabled;
	int temp_passive;
	int temp_critical;
};

/* The driver support 1 passive trip point and 1 critical trip point */
enum imx_thermal_trip {
	IMX_TRIP_PASSIVE,
	IMX_TRIP_CRITICAL,
	IMX_TRIP_NUM,
};

static int tmu_get_temp(void *data, int *temp)
{
	struct imx8mm_tmu *tmu = data;
	u32 val;

	/* the temp sensor need about 1ms to finish the measurement */
	msleep(1);

	/* read the calibrated temp value */
	val = readl_relaxed(tmu->tmu_base + TRITSR) & TEMP_VAL_MASK;

	/* check if the temp in the sensor's range */
	if (val < TEMP_LOW_LIMIT)
		return -EAGAIN;

	*temp = val * 1000;

	return 0;
}

static int tmu_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	int trip_temp;
	struct imx8mm_tmu *tmu = p;

	if (!tmu->tzd)
		return 0;

	trip_temp = (trip == IMX_TRIP_PASSIVE) ? tmu->temp_passive : tmu->temp_critical;

	if (tmu->tzd->temperature >= (trip_temp - IMX_TEMP_PASSIVE_COOL_DELTA))
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

static int tmu_set_trip_temp(void *p, int trip, int temp)
{
	struct imx8mm_tmu *tmu = p;

	if (trip == IMX_TRIP_CRITICAL)
		tmu->temp_critical = temp;

	if (trip == IMX_TRIP_PASSIVE)
		tmu->temp_passive = temp;

	return 0;
}

static struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
	.get_trend = tmu_get_trend,
	.set_trip_temp = tmu_set_trip_temp,
};

static const struct of_device_id imx8mm_tmu_table[] = {
	{ .compatible = "fsl,imx8mm-tmu", },
	{ },
};

#define IMX8MM_OCOTP_TESTER3     0x0440

static int imx_init_temp_grade(struct platform_device *pdev)
{
	struct imx8mm_tmu *tmu = platform_get_drvdata(pdev);
	struct regmap *map;
	int ret, temp_max;
	u32 val;

	map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
					      "fsl,tempmon-data");
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		dev_err(&pdev->dev, "failed to get sensor regmap: %d\n", ret);
		return ret;
	}

	ret = regmap_read(map, IMX8MM_OCOTP_TESTER3, &val);

	if (ret) {
		dev_err(&pdev->dev, "failed to read sensor data: %d\n", ret);
		return ret;
	}

	switch ((val >> 6) & 0x3) {
	case 0: /* Commercial (0 to 95 °C) */
		temp_max = 95000;
		break;
	case 2: /* Industrial (-40 °C to 105 °C) */
		temp_max = 105000;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Set the critical trip point at 5 °C under max
	 * Set the passive trip point at 10 °C under max (changeable via sysfs)
	 */
	tmu->temp_critical = temp_max - (1000 * 5);
	tmu->temp_passive = temp_max - (1000 * 10);

	return ret;
}

static int imx8mm_tmu_probe(struct platform_device *pdev)
{
	int ret;
	u32 val;
	struct imx8mm_tmu *tmu;
	const struct thermal_trip *trips;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "device node NOT found\n");
		return -ENODEV;
	}

	tmu = devm_kzalloc(&pdev->dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	platform_set_drvdata(pdev, tmu);

	tmu->tmu_base = of_iomap(np, 0);
	if (!tmu->tmu_base) {
		dev_err(&pdev->dev, "Failed to map the memory\n");
		return -ENODEV;
	}

	tmu->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(tmu->clk)) {
		ret = PTR_ERR(tmu->clk);
		dev_err(&pdev->dev, "Failed to get the tmu clk\n");
		goto err;
	}

	/* register the thermal zone sensor */
	tmu->tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, tmu, &tmu_tz_ops);

	if (IS_ERR(tmu->tzd)) {
		ret = PTR_ERR(tmu->tzd);
		dev_err(&pdev->dev, "Failed to register thermal zone sensor %d\n", ret);
		goto err;
	}

	tmu->cdev = devfreq_cooling_register();
	if (IS_ERR(tmu->cdev)) {
		ret = PTR_ERR(tmu->cdev);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to register devfreq cooling device %d\n", ret);
		goto err;
	}

	ret = thermal_zone_bind_cooling_device(tmu->tzd,
		IMX_TRIP_PASSIVE,
		tmu->cdev,
		THERMAL_NO_LIMIT,
		THERMAL_NO_LIMIT,
		THERMAL_WEIGHT_DEFAULT);
	if (ret) {
		dev_err(&pdev->dev,
			"binding zone %s with cdev %s failed:%d\n",
			tmu->tzd->type, tmu->cdev->type, ret);
		devfreq_cooling_unregister(tmu->cdev);
		goto err;
	}

	ret = imx_init_temp_grade(pdev);
	if (ret) {
		dev_info(&pdev->dev, "failed to init from fsl,tempmon-data use temp from devicetree\n");
		trips = of_thermal_get_trip_points(tmu->tzd);

		/* get the thermal trip temp */
		tmu->temp_passive = trips[0].temperature;
		tmu->temp_critical = trips[1].temperature;
	} else {
		/* set the thermal trip temp */
		tmu->tzd->ops->set_trip_temp(tmu->tzd, IMX_TRIP_PASSIVE,
					tmu->temp_passive);
		tmu->tzd->ops->set_trip_temp(tmu->tzd, IMX_TRIP_CRITICAL,
					tmu->temp_critical);

	}


	/* enable the tmu clock */
	ret = clk_prepare_enable(tmu->clk);
	if (ret) {
		dev_warn(&pdev->dev, "tmu clock enable failed:%d\n", ret);
		thermal_zone_unbind_cooling_device(tmu->tzd, IMX_TRIP_PASSIVE, tmu->cdev);
		devfreq_cooling_unregister(tmu->cdev);
		goto err;
	}

	/* enable the monitor */
	val = readl_relaxed(tmu->tmu_base + TER);
	val |= TER_EN;
	writel_relaxed(val, tmu->tmu_base + TER);

	tmu->tzd->tzp->no_hwmon = false;
	ret = thermal_add_hwmon_sysfs(tmu->tzd);
	if (ret)
		goto clk_err;

	return 0;

clk_err:
	clk_disable_unprepare(tmu->clk);
err:
	iounmap(tmu->tmu_base);
	return ret;
}

static int imx8mm_tmu_remove(struct platform_device *pdev)
{
	u32 val;
	struct imx8mm_tmu *tmu = platform_get_drvdata(pdev);

	thermal_zone_unbind_cooling_device(tmu->tzd, IMX_TRIP_PASSIVE, tmu->cdev);
	devfreq_cooling_unregister(tmu->cdev);

	/* disable TMU */
	val = readl_relaxed(tmu->tmu_base + TER);
	val &= ~TER_EN;
	writel_relaxed(val, tmu->tmu_base + TER);

	/* disable TMU clk */
	clk_disable_unprepare(tmu->clk);

	iounmap(tmu->tmu_base);

	return 0;
}

static struct platform_driver imx8mm_tmu = {
	.driver = {
		.name	= "i.mx8mm_thermal",
		.of_match_table = imx8mm_tmu_table,
	},
	.probe = imx8mm_tmu_probe,
	.remove = imx8mm_tmu_remove,
};
module_platform_driver(imx8mm_tmu);

MODULE_AUTHOR("Jacky Bai <ping.bai@nxp.com>");
MODULE_DESCRIPTION("i.MX8MM Thermal Monitor Unit driver");
MODULE_LICENSE("GPL v2");
