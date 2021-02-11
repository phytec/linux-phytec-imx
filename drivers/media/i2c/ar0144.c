// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2018 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 * Copyright (C) 2020 PHYTEC Messtechnik GmbH
 * Author: Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 * Author: Stefan Riedmueller <s.riedmueller@phytec.de>
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#include <linux/of_graph.h>

#include <linux/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ctrls.h>

#define AR0144_MODEL_ID					0x3000
#define AR0144_Y_ADDR_START				0x3002
#define AR0144_X_ADDR_START				0x3004
#define AR0144_Y_ADRR_END				0x3006
#define AR0144_X_ADRR_END				0x3008
#define AR0144_FRAME_LENGTH_LINES			0x300a
#define AR0144_LINE_LENGTH_PCK				0x300c
#define AR0144_REVISION					0x300e
#define	AR0144_COARSE_INT_TIME				0x3012
#define	AR0144_FINE_INT_TIME				0x3014
#define AR0144_RESET_REGISTER				0x301a
#define		AR0144_FLD_GROUPED_PARAM_HOLD		BIT(15)
#define		AR0144_FLD_SMIA_SER_DIS			BIT(12)
#define		AR0144_FLD_FORCED_PLL_ON		BIT(11)
#define		AR0144_FLD_RESTART_BAD			BIT(10)
#define		AR0144_FLD_MASK_BAD			BIT(9)
#define		AR0144_FLD_GPI_EN			BIT(8)
#define		AR0144_FLD_PARALLEL_EN			BIT(7)
#define		AR0144_FLD_DRIVE_PINS			BIT(6)
#define		AR0144_FLD_LOCK_REG			BIT(3)
#define		AR0144_FLD_STREAM			BIT(2)
#define		AR0144_FLD_RESTART			BIT(1)
#define		AR0144_FLD_RESET			BIT(0)
#define	AR0144_DATA_PEDESTAL				0x301e
#define	AR0144_VT_PIX_CLK_DIV				0x302a
#define	AR0144_VT_SYS_CLK_DIV				0x302c
#define AR0144_PRE_PLL_CLK_DIV				0x302e
#define AR0144_PLL_MUL					0x3030
#define	AR0144_OP_PIX_CLK_DIV				0x3036
#define	AR0144_OP_SYS_CLK_DIV				0x3038
#define AR0144_FRAME_COUNT				0x303a
#define AR0144_FRAME_STATUS				0x303c
#define		AR0144_FLD_PLL_LOCKED			BIT(3)
#define		AR0144_FLD_FRAME_START_DURING_GPH	BIT(2)
#define		AR0144_FLD_STANDBY_STATUS		BIT(1)
#define		AR0144_FLD_FRAMESYNC			BIT(0)
#define	AR0144_READ_MODE				0x3040
#define		AR0144_FLD_VERT_FLIP			BIT(15)
#define		AR0144_FLD_HORIZ_MIRROR			BIT(14)
#define		AR0144_FLD_RM_COL_BIN			BIT(13)
#define		AR0144_FLD_RM_ROW_BIN			BIT(12)
#define		AR0144_FLD_RM_COL_SF_BIN		BIT(9)
#define		AR0144_FLD_RM_COL_SF_BIN_MONO		BIT(7)
#define		AR0144_FLD_RM_COL_SUM			BIT(5)
#define	AR0144_GREENR_GAIN				0x3056
#define	AR0144_BLUE_GAIN				0x3058
#define	AR0144_RED_GAIN					0x305a
#define	AR0144_GREENB_GAIN				0x305c
#define	AR0144_GLOBAL_GAIN				0x305e
#define	AR0144_ANALOGUE_GAIN				0x3060
#define		AR0144_FLD_COARSE_GAIN_MASK		0x70
#define		AR0144_FLD_COARSE_GAIN_SHIFT		4
#define		AR0144_FLD_FINE_GAIN_MASK		0xf
#define		AR0144_FLD_FINE_GAIN_SHIFT		0
#define AR0144_SMIA_TEST				0x3064
#define		AR0144_FLD_EMBEDDED_DATA		BIT(8)
#define		AR0144_FLD_EMBEDDED_STATS_EN		BIT(7)
#define	AR0144_DATAPATH_SEL				0x306e
#define		AR0144_FLD_SLEW_RATE_DAT_MASK		0xe000
#define		AR0144_FLD_SLEW_RATE_DAT_SHIFT		13
#define		AR0144_FLD_SLEW_RATE_CLK_MASK		0x1c00
#define		AR0144_FLD_SLEW_RATE_CLK_SHIFT		10
#define	AR0144_TEST_PATTERN				0x3070
#define		AR0144_FLD_NO_TESTPATTERN		0
#define		AR0144_FLD_SOLIDCOLOR			1
#define		AR0144_FLD_FULL_COLOR_BAR		2
#define		AR0144_FLD_FADE_TO_GRAY			3
#define		AR0144_FLD_WALKING_ONES			256
#define	AR0144_TEST_DATA_RED				0x3072
#define	AR0144_TEST_DATA_GREENR				0x3074
#define	AR0144_TEST_DATA_BLUE				0x3076
#define	AR0144_TEST_DATA_GREENB				0x3078
#define	AR0144_X_ODD_INC				0x30a2
#define	AR0144_Y_ODD_INC				0x30a6
#define	AR0144_DIGITAL_TEST				0x30b0
#define		AR0144_FLD_PLL_BYPASS			BIT(14)
#define		AR0144_FLD_PIXCLK_ON			BIT(8)
#define		AR0144_FLD_MONOCHROME_OP		BIT(7)
#define AR0144_TEMPSENS_DATA				0x30b2
#define AR0144_TEMPSENS_CTRL				0x30b4
#define		AR0144_FLD_RETRIG_THRESHOLD_MASK	0xfc
#define		AR0144_FLD_RETRIG_THRESHOLD_SHIFT	6
#define		AR0144_FLD_TEMP_CLEAR			BIT(5)
#define		AR0144_FLD_TEMP_START_CONV		BIT(4)
#define		AR0144_FLD_TEMPSENS_PWRON		BIT(0)
#define AR0144_TEMPSENS_CALIB1				0x30c6
#define AR0144_TEMPSENS_CALIB2				0x30c8

#define AR0144_AECTRL					0x3100
#define		AR0144_FLD_MIN_ANA_GAIN_MASK		0x60
#define		AR0144_FLD_MIN_ANA_GAIN_SHIFT		5
#define		AR0144_FLD_AUTO_DG_EN			BIT(4)
#define		AR0144_FLD_AUTO_AG_EN			BIT(1)
#define		AR0144_FLD_AE_EN			BIT(0)
#define	AR0144_AE_LUMA_TGT				0x3102
#define AR0144_AE_MAX_EXPOSURE				0x311c
#define	AR0144_AE_MIN_EXPOSURE				0x311e
#define	AR0144_AE_COARSE_INT_TIME			0x3164
#define	AR0144_DELTA_DK_CTRL				0x3180
#define		AR0144_FLD_DK_SUB_EN			BIT(15)
#define		AR0144_FLD_DK_EVERY_FRAME		BIT(14)
#define		AR0144_FLD_DK_RECALC			BIT(13)
#define	AR0144_DATA_FORMAT_BITS				0x31ac
#define		AR0144_FLD_DATA_FMT_IN_MASK		0x1f00
#define		AR0144_FLD_DATA_FMT_IN_SHIFT		8
#define		AR0144_FLD_DATA_FMT_OUT_MASK		0x1f
#define		AR0144_FLD_DATA_FMT_OUT_SHIFT		0
#define	AR0144_SERIAL_FORMAT				0x31ae
#define		AR0144_FLD_DUAL_LANE			BIT(1)
#define		AR0144_FLD_SINGLE_LANE			BIT(0)
#define AR0144_MIPI_TIMING_0				0x31b4
#define		AR0144_FLD_HS_PREP_MASK			0xf000
#define		AR0144_FLD_HS_PREP_SHIFT		12
#define		AR0144_FLD_HS_ZERO_MASK			0xf00
#define		AR0144_FLD_HS_ZERO_SHIFT		8
#define		AR0144_FLD_HS_TRAIL_MASK		0xf0
#define		AR0144_FLD_HS_TRAIL_SHIFT		4
#define		AR0144_FLD_CLK_TRAIL_MASK		0xf
#define		AR0144_FLD_CLK_TRAIL_SHIFT		0
#define AR0144_MIPI_TIMING_1				0x31b6
#define		AR0144_FLD_CLK_PREP_MASK		0xf000
#define		AR0144_FLD_CLK_PREP_SHIFT		12
#define		AR0144_FLD_HS_EXIT_MASK			0xfc0
#define		AR0144_FLD_HS_EXIT_SHIFT		6
#define		AR0144_FLD_CLK_ZERO_MASK		0x1f
#define		AR0144_FLD_CLK_ZERO_SHIFT		0
#define AR0144_MIPI_TIMING_2				0x31b8
#define		AR0144_FLD_BGAP_MASK			0xf000
#define		AR0144_FLD_BGAP_SHIFT			12
#define		AR0144_FLD_CLK_PRE_MASK			0xfc0
#define		AR0144_FLD_CLK_PRE_SHIFT		6
#define		AR0144_FLD_CLK_POST_MASK		0x1f
#define		AR0144_FLD_CLK_POST_SHIFT		0
#define AR0144_MIPI_TIMING_3				0x31ba
#define		AR0144_FLD_LPX_MASK			0x1f80
#define		AR0144_FLD_LPX_SHIFT			7
#define		AR0144_FLD_WAKE_UP_MASK			0x7f
#define		AR0144_FLD_WAKE_UP_SHIFT		0
#define AR0144_MIPI_TIMING_4				0x31bc
#define		AR0144_FLD_CONT_TX_CLK			BIT(15)
#define		AR0144_FLD_HEAVY_LP_LOAD		BIT(14)
#define		AR0144_FLD_INIT_MASK			0x7f
#define		AR0144_FLD_INIT_SHIFT			0
#define AR0144_SER_CTRL_STAT				0x31c6
#define		AR0144_FLD_FRAMER_TEST_MODE		BIT(7)
#define	AR0144_COMPANDING				0x31d0
#define		AR0144_FLD_COMPAND_EN			BIT(0)
#define AR0144_SERIAL_TEST				0x31d8
#define		AR0144_FLD_TEST_LANE_EN_MASK		0xf00
#define		AR0144_FLD_TEST_LANE_EN_SHIFT		8
#define		AR0144_FLD_TEST_LANE_0			0x1
#define		AR0144_FLD_TEST_LANE_1			0x2
#define		AR0144_FLD_TEST_LANE_2			0x4
#define		AR0144_FLD_TEST_LANE_3			0x8
#define		AR0144_FLD_TEST_MODE_MASK		0xf0
#define		AR0144_FLD_TEST_MODE_SHIFT		4
#define		AR0144_FLD_TEST_MODE_LP11		0x1
#define	AR0144_PIX_DEF_ID				0x31e0
#define		AR0144_FLD_PIX_DEF_1D_DDC_EN		BIT(3)
#define AR0144_CUSTOMER_REV				0x31fe

#define	AR0144_FLASH_CTRL				0x3270
#define		AR0144_FLD_LED_FLASH_EN			BIT(8)
#define		AR0144_FLD_LED_DELAY_MASK		0xff
#define		AR0144_FLD_LED_DELAY_SHIFT		0

#define AR0144_MIPI_CNTRL				0x3354
#define		AR0144_FLD_CHAN_NUM_MASK		0xc0
#define		AR0144_FLD_CHAN_NUM_SHIFT		6
#define		AR0144_FLD_DATA_TYPE_MASK		0x3f
#define		AR0144_FLD_DATA_TYPE_SHIFT		0
#define		AR0144_FLD_CSI2_DT_RAW8			0x2a
#define		AR0144_FLD_CSI2_DT_RAW10		0x2b
#define		AR0144_FLD_CSI2_DT_RAW12		0x2c

#define AR0144_NO_SLEW_RATE		(~0u)

#define AR0144_CHIP_VERSION		0x0356
#define AR0144_DEF_WIDTH		1280
#define AR0144_DEF_HEIGHT		800

#define AR0144_MIPI_SINK		0
#define AR0144_PAR_SINK			1
#define AR0144_NUM_PADS			2

enum {
	V4L2_CID_USER_BASE_AR0144		= V4L2_CID_USER_BASE + 0x2500,
	V4L2_CID_X_EXPOSURE_FINE,
	V4L2_CID_X_AUTO_EXPOSURE_MIN,
	V4L2_CID_X_AUTO_EXPOSURE_MAX,
	V4L2_CID_X_AUTO_EXPOSURE_TGT,
	V4L2_CID_X_AUTO_EXPOSURE_CUR,

	V4L2_CID_X_AUTOGAIN_ANALOGUE,
	V4L2_CID_X_AUTOGAIN_DIGITAL,

	V4L2_CID_X_AUTOGAIN_ANALOGUE_MIN,

	V4L2_CID_X_BINNING_ROW,
	V4L2_CID_X_BINNING_COL,
	V4L2_CID_X_COMPANDING,

	V4L2_CID_X_DIGITAL_GAIN_RED,
	V4L2_CID_X_DIGITAL_GAIN_GREENR,
	V4L2_CID_X_DIGITAL_GAIN_BLUE,
	V4L2_CID_X_DIGITAL_GAIN_GREENB,

	V4L2_CID_X_EMBEDDED_DATA,
	V4L2_CID_X_BLACK_LEVEL_AUTO,
	V4L2_CID_X_FLASH_DELAY,
	V4L2_CID_X_DYNAMIC_PIXEL_CORRECTION,
};

enum {
	V4L2_X_EMBEDDED_OFF,
	V4L2_X_EMBEDDED_STAT,
	V4L2_X_EMBEDDED_DATA,
	V4L2_X_EMBEDDED_BOTH,
};

enum ar0144_model {
	AR0144_MODEL_UNKNOWN,
	AR0144_MODEL_COLOR,
	AR0144_MODEL_MONOCHROME,
};

enum ar0144_bus {
	AR0144_BUS_UNKNOWN,
	AR0144_BUS_PARALLEL,
	AR0144_BUS_MIPI,
};

struct ar0144_format {
	unsigned int code;
	unsigned int bpp;
	unsigned int x_offset;
	unsigned int y_offset;
};

static const struct ar0144_format ar0144_mono_formats[] = {
	{
		.code	= MEDIA_BUS_FMT_Y8_1X8,
		.bpp	= 8,
	}, {
		.code	= MEDIA_BUS_FMT_Y10_1X10,
		.bpp	= 10,
	}, {
		.code	= MEDIA_BUS_FMT_Y12_1X12,
		.bpp	= 12,
	},
};

static const struct ar0144_format ar0144_col_formats[] = {
	{
		.code		= MEDIA_BUS_FMT_SGBRG8_1X8,
		.bpp		= 8,
		.x_offset	= 1,
		.y_offset	= 1,
	}, {
		.code		= MEDIA_BUS_FMT_SBGGR8_1X8,
		.bpp		= 8,
		.x_offset	= 0,
		.y_offset	= 1,
	}, {
		.code		= MEDIA_BUS_FMT_SRGGB8_1X8,
		.bpp		= 8,
		.x_offset	= 1,
		.y_offset	= 0,
	}, {
		.code		= MEDIA_BUS_FMT_SGRBG8_1X8,
		.bpp		= 8,
		.x_offset	= 0,
		.y_offset	= 0,
	}, {
		.code		= MEDIA_BUS_FMT_SGBRG10_1X10,
		.bpp		= 10,
		.x_offset	= 1,
		.y_offset	= 1,
	}, {
		.code		= MEDIA_BUS_FMT_SBGGR10_1X10,
		.bpp		= 10,
		.x_offset	= 0,
		.y_offset	= 1,
	}, {
		.code		= MEDIA_BUS_FMT_SRGGB10_1X10,
		.bpp		= 10,
		.x_offset	= 1,
		.y_offset	= 0,
	}, {
		.code		= MEDIA_BUS_FMT_SGRBG10_1X10,
		.bpp		= 10,
		.x_offset	= 0,
		.y_offset	= 0,
	}, {
		.code		= MEDIA_BUS_FMT_SGBRG12_1X12,
		.bpp		= 12,
		.x_offset	= 1,
		.y_offset	= 1,
	}, {
		.code		= MEDIA_BUS_FMT_SBGGR12_1X12,
		.bpp		= 12,
		.x_offset	= 0,
		.y_offset	= 1,
	}, {
		.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
		.bpp		= 12,
		.x_offset	= 1,
		.y_offset	= 0,
	}, {
		.code		= MEDIA_BUS_FMT_SGRBG12_1X12,
		.bpp		= 12,
		.x_offset	= 0,
		.y_offset	= 0,
	},
};

struct limit_range {
	unsigned long min;
	unsigned long max;
};

struct ar0144_sensor_limits {
	struct limit_range x;
	struct limit_range y;

	struct limit_range hlen;
	struct limit_range vlen;

	struct limit_range hblank;
	struct limit_range vblank;

	struct limit_range pre_pll_div;
	struct limit_range pre_pll_mul;

	struct limit_range pll_vt_sys_clk_div;
	struct limit_range pll_vt_pix_clk_div;
	struct limit_range pll_op_sys_clk_div;
	struct limit_range pll_op_pix_clk_div;

	struct limit_range ext_clk;
	struct limit_range pix_clk;
	struct limit_range vco;
};

struct ar0144_parallel_businfo {
	unsigned long link_freq;
	unsigned int slew_rate_dat;
	unsigned int slew_rate_clk;
};

struct ar0144_mipi_businfo {
	unsigned int num_lanes;

	u16 t_hs_prep;
	u16 t_hs_zero;
	u16 t_hs_trail;
	u16 t_clk_trail;
	u16 t_clk_prep;
	u16 t_hs_exit;
	u16 t_clk_zero;
	u16 t_bgap;
	u16 t_clk_pre;
	u16 t_clk_post;
	u16 t_lpx;
	u16 t_wakeup;
	u16 t_init;
	bool cont_tx_clk;
	bool heavy_lp_load;
};

struct ar0144_pll_config {
	unsigned int pre_pll_div;
	unsigned int pre_pll_mul;
	unsigned int vt_sys_div;
	unsigned int vt_pix_div;
	unsigned int op_sys_div;
	unsigned int op_pix_div;
	unsigned long vco_freq;
	unsigned long pix_freq;
	unsigned long ser_freq;
	bool update;
};

struct ar0144_gains {
	struct v4l2_ctrl *red_ctrl;
	struct v4l2_ctrl *greenb_ctrl;
	struct v4l2_ctrl *greenr_ctrl;
	struct v4l2_ctrl *blue_ctrl;
	unsigned int red_clip;
	unsigned int greenb_clip;
	unsigned int greenr_clip;
	unsigned int blue_clip;
};

struct ar0144 {
	struct v4l2_subdev subdev;
	struct device *dev;
	struct v4l2_ctrl_handler ctrls;
	struct media_pad pad[2];

	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct v4l2_fract interval;
	unsigned int bpp;
	unsigned int w_scale;
	unsigned int h_scale;
	unsigned int vblank;
	unsigned int hblank;
	bool embedded_data;
	bool embedded_stat;

	struct ar0144_parallel_businfo pinfo;
	struct ar0144_mipi_businfo minfo;
	struct ar0144_pll_config pll;
	struct ar0144_sensor_limits limits;
	enum ar0144_bus active_bus;
	enum ar0144_model model;

	const struct ar0144_format *formats;
	unsigned int num_fmts;

	struct v4l2_ctrl *link_freq_ctrl;
	struct ar0144_gains gains;

	struct clk *extclk;
	struct gpio_desc *reset_gpio;

	struct mutex lock;

	int power_user;
	bool is_streaming;
};

static inline struct ar0144 *to_ar0144(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0144, subdev);
}

static int ar0144_read(struct ar0144 *sensor, u16 reg, u16 *val)
{
	struct i2c_client *i2c = v4l2_get_subdevdata(&sensor->subdev);
	unsigned char reg_buf[2], read_buf[2];
	int ret;
	u16 result;
	struct i2c_msg xfer[] = {
		[0] = {
			.addr = i2c->addr,
			.flags = 0,
			.len = 2,
			.buf = reg_buf,
		},
		[1] = {
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = read_buf,
		},
	};

	reg_buf[0] = (reg >> 8) & 0xff;
	reg_buf[1] = reg & 0xff;

	ret = i2c_transfer(i2c->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret == ARRAY_SIZE(xfer))
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	if (ret < 0)
		return ret;

	result = read_buf[0] << 8;
	result |= read_buf[1];

	*val = result;

	if (ret)
		dev_err(&i2c->dev, "Failed to read i2c message (%d)\n", ret);

	return ret;
}

static int ar0144_write(struct ar0144 *sensor, u16 reg, u16 val)
{
	struct i2c_client *i2c = v4l2_get_subdevdata(&sensor->subdev);
	unsigned char write_buf[4];
	int ret;
	struct i2c_msg xfer[] = {
		[0] = {
			.addr = i2c->addr,
			.flags = 0,
			.len = 4,
			.buf = write_buf,
		},
	};

	write_buf[0] = (reg >> 8) & 0xff;
	write_buf[1] = reg & 0xff;

	write_buf[2] = (val >> 8) & 0xff;
	write_buf[3] = val & 0xff;

	ret = i2c_transfer(i2c->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret == ARRAY_SIZE(xfer))
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	dev_dbg(&i2c->dev, "Wrote i2c message 0x%02x at 0x%02x (%d)\n",
		val, reg, ret);

	if (ret)
		dev_err(&i2c->dev, "Failed to write i2c message (%d)\n", ret);

	return ret;
}

static int ar0144_update_bits(struct ar0144 *sensor, u16 reg,
			      u16 mask, u16 val)
{
	u16 orig, tmp;
	int ret;

	ret = ar0144_read(sensor, reg, &orig);
	if (ret)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		ret = ar0144_write(sensor, reg, tmp);

	return ret;
}

static const struct ar0144_format *ar0144_find_format(struct ar0144 *sensor,
						      u32 code)
{
	int i;

	for (i = 0; i < sensor->num_fmts; i++)
		if (sensor->formats[i].code == code)
			return &sensor->formats[i];

	return &sensor->formats[sensor->num_fmts - 1];
}

static const unsigned int ar0144_match_col_format(struct ar0144 *sensor,
						  struct v4l2_rect *crop)
{
	unsigned int x_offset = crop->left % 2;
	unsigned int y_offset = crop->top % 2;
	int i;

	for (i = 0; i < sensor->num_fmts; i++) {
		if (sensor->bpp != sensor->formats[i].bpp)
			continue;

		if (sensor->formats[i].x_offset == x_offset &&
		    sensor->formats[i].y_offset == y_offset)
			return sensor->formats[i].code;
	}

	return sensor->formats[sensor->num_fmts - 1].code;
}

static int ar0144_find_active_pad(struct ar0144 *sensor)
{
	struct media_pad *remote;
	int i;

	for (i = 0; i < AR0144_NUM_PADS; i++) {
		remote = media_entity_remote_pad(&sensor->pad[i]);
		if (remote)
			return i;
	}

	v4l2_warn(&sensor->subdev, "No active pad found during stream on\n");
	return -1;
}

static int ar0144_enter_standby(struct ar0144 *sensor)
{
	unsigned int timeout = 1000;
	int ret;
	u16 val;

	ret = ar0144_update_bits(sensor, AR0144_RESET_REGISTER,
				 AR0144_FLD_STREAM | AR0144_FLD_GPI_EN, 0);
	if (ret)
		return ret;

	while (timeout) {
		ar0144_read(sensor, AR0144_FRAME_STATUS, &val);

		if (val & AR0144_FLD_STANDBY_STATUS) {
			dev_dbg(sensor->dev, "reached standby state\n");
			break;
		}

		timeout--;

		if (timeout == 0) {
			dev_warn(sensor->dev,
				  "timeout while trying to enter standby\n");
			break;
		}

		usleep_range(2000, 3000);
	}

	ar0144_read(sensor, AR0144_RESET_REGISTER, &val);

	if ((val & AR0144_FLD_SMIA_SER_DIS) == 0) {
		/* TODO: Calculate frametime and use it for this wait period */
		msleep(100);
		ret = ar0144_update_bits(sensor, AR0144_RESET_REGISTER,
					 AR0144_FLD_SMIA_SER_DIS,
					 AR0144_FLD_SMIA_SER_DIS);
		if (ret)
			return ret;
	}

	/* In MIPI mode the sensor might be in LP-11 test mode so make sure
	 * to disable it.
	 */
	if (sensor->active_bus == AR0144_BUS_MIPI)
		ret = ar0144_update_bits(sensor, AR0144_SER_CTRL_STAT,
					 AR0144_FLD_FRAMER_TEST_MODE, 0);

	return ret;
}

static int ar0144_mipi_enter_lp11(struct ar0144 *sensor)
{
	int ret = 0;
	u16 val;

	ret = ar0144_read(sensor, AR0144_SER_CTRL_STAT, &val);
	if (ret)
		return ret;

	val = AR0144_FLD_TEST_MODE_LP11 << AR0144_FLD_TEST_MODE_SHIFT |
	      AR0144_FLD_TEST_LANE_0 << AR0144_FLD_TEST_LANE_EN_SHIFT;

	if (sensor->minfo.num_lanes == 2)
		val |= AR0144_FLD_TEST_LANE_1 << AR0144_FLD_TEST_LANE_EN_SHIFT;

	ret = ar0144_write(sensor, AR0144_SERIAL_TEST, val);
	if (ret)
		return ret;

	ret = ar0144_update_bits(sensor,
				 AR0144_SER_CTRL_STAT,
				 AR0144_FLD_FRAMER_TEST_MODE,
				 AR0144_FLD_FRAMER_TEST_MODE);
	if (ret)
		return ret;

	ret = ar0144_update_bits(sensor,
				 AR0144_RESET_REGISTER,
				 AR0144_FLD_STREAM |
				 AR0144_FLD_SMIA_SER_DIS,
				 AR0144_FLD_STREAM);
	return ret;
}

static void ar0144_reset(struct ar0144 *sensor)
{
	unsigned long ext_freq = clk_get_rate(sensor->extclk);
	unsigned long ext_freq_mhz = ext_freq / 1000000;
	unsigned long wait_usecs;

	if (sensor->reset_gpio) {
		gpiod_set_value_cansleep(sensor->reset_gpio, 1);
		usleep_range(1000, 1100);
		gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	} else {
		ar0144_update_bits(sensor, AR0144_RESET_REGISTER,
				   AR0144_FLD_RESET, AR0144_FLD_RESET);
	}

	wait_usecs = 160000 / ext_freq_mhz;
	usleep_range(wait_usecs, wait_usecs + 1000);
}

static int ar0144_power_on(struct ar0144 *sensor)
{
	/* TODO: Enable power, clocks, etc... */
	/* TODO: Implement use counter */
	return 0;
}

static void ar0144_power_off(struct ar0144 *sensor)
{
	/* TODO: Disable power, clocks, etc... */
}

/* V4L2 subdev core ops */
static int ar0144_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0144 *sensor = to_ar0144(sd);
	unsigned int active_pad;
	int ret = 0;
	int link_freq;

	dev_dbg(sd->dev, "%s on: %d\n", __func__, on);

	mutex_lock(&sensor->lock);

	if (sensor->active_bus == AR0144_BUS_UNKNOWN) {
		active_pad = ar0144_find_active_pad(sensor);

		switch (active_pad) {
		case AR0144_PAR_SINK:
			sensor->active_bus = AR0144_BUS_PARALLEL;
			break;

		case AR0144_MIPI_SINK:
			sensor->active_bus = AR0144_BUS_MIPI;
			break;

		default:
			break;
		}
	}

	if (on) {
		if (sensor->power_user > 0) {
			sensor->power_user++;
			goto out;
		}

		ret = ar0144_power_on(sensor);
		if (ret)
			goto out;

		/* Enable MIPI LP-11 test mode as required by e.g. i.MX 6 */
		if (sensor->active_bus == AR0144_BUS_MIPI &&
		    !sensor->is_streaming) {
			ret = ar0144_mipi_enter_lp11(sensor);
			if (ret) {
				ar0144_power_off(sensor);
				goto out;
			}

			/* Select the correct link frequency depending on
			 * bits per pixel
			 */
			/* TODO: Can we remove these magic values here? */
			link_freq = (sensor->bpp - 8) / 2;
			if (sensor->minfo.num_lanes == 1)
				link_freq += 3;

			ret = v4l2_ctrl_s_ctrl(sensor->link_freq_ctrl,
					       link_freq);
			if (ret) {
				v4l2_err(&sensor->subdev,
					 "Failed to set link freq ctrl\n");
				ar0144_power_off(sensor);
				goto out;
			}
		}

		sensor->power_user++;

	} else {
		sensor->power_user--;
		if (sensor->power_user < 0) {
			dev_err(sd->dev, "More s_power OFF than ON\n");
			ret = -EINVAL;
			goto out;
		}

		if (sensor->power_user == 0) {
			ar0144_enter_standby(sensor);
			ar0144_power_off(sensor);
		}
	}

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ar0144_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct ar0144 *sensor = to_ar0144(sd);
	int ret;

	ret = ar0144_write(sensor, reg->reg, reg->val);
	return ret;
}

static int ar0144_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct ar0144 *sensor = to_ar0144(sd);
	int ret;

	ret = ar0144_read(sensor, reg->reg, (u16 *)&reg->val);
	return ret;
}
#endif

static unsigned long ar0144_clk_div_mul(unsigned long freq,
					unsigned int div,
					unsigned int mul)
{
	uint64_t result;

	if (WARN_ON(div == 0))
		return 0;

	result = freq;
	result *= mul;
	result = div_u64(result, div);

	return result;
}

static int ar0144_calculate_pll(struct ar0144 *sensor)
{
	struct ar0144_sensor_limits *limits = &sensor->limits;
	struct v4l2_subdev *sd = &sensor->subdev;
	unsigned long ext_freq = clk_get_rate(sensor->extclk);
	unsigned long op_clk;
	unsigned long vco;
	unsigned long pix_clk;
	unsigned long pix_clk_target;
	long diff, diff_old;
	unsigned int pll_div, pll_div_min, pll_div_max;
	unsigned int pll_mul, pll_mul_min, pll_mul_max;
	unsigned int op_div, op_div_min, op_div_max;
	unsigned int tmp_div;
	unsigned int bpp = sensor->bpp;
	unsigned int lanes = sensor->minfo.num_lanes;

	if (!sensor->pll.update)
		return 0;

	if (bpp == 12 && lanes < 2) {
		v4l2_err(sd, "PLL config: 12 bpp require at least 2 lanes\n");
		return -EINVAL;
	}

	dev_dbg(sd->dev, "%s: lanes: %d bpp: %d\n", __func__, lanes, bpp);

	if (sensor->active_bus == AR0144_BUS_PARALLEL) {
		pix_clk_target = sensor->pinfo.link_freq;
		lanes = 1;
	} else {
		pix_clk_target = 74250000;
	}

	/* Init diff value */
	diff_old = pix_clk_target;

	pll_div_min = limits->pre_pll_div.min;
	pll_div_max = limits->pre_pll_div.max;
	pll_mul_min = limits->pre_pll_mul.min;
	pll_mul_max = limits->pre_pll_mul.max;
	op_div_min = limits->pll_op_sys_clk_div.min;
	op_div_max = limits->pll_op_sys_clk_div.max;
	sensor->pll.pre_pll_div = 1;
	sensor->pll.pre_pll_mul = 0;
	sensor->pll.op_sys_div = 1;

	for (pll_mul = pll_mul_min; pll_mul <= pll_mul_max; pll_mul++) {
		for (pll_div = pll_div_min; pll_div <= pll_div_max; pll_div++) {
			for (op_div = op_div_min; op_div <= op_div_max;
			     op_div = op_div == 1 ? 2 : op_div+2) {
				vco = ar0144_clk_div_mul(ext_freq, pll_div,
							 pll_mul);

				if (vco < limits->vco.min ||
				    vco > limits->vco.max)
					continue;

				tmp_div = bpp * op_div;
				op_clk = ar0144_clk_div_mul(vco, tmp_div, 1);
				pix_clk = op_clk * lanes;

				if (pix_clk > 74250000)
					continue;

				diff = abs(pix_clk_target - pix_clk);
				if (diff >= diff_old)
					continue;

				dev_dbg(sd->dev, "%s: vco: %lu op_clk: %lu\n",
					__func__, vco, op_clk);
				dev_dbg(sd->dev,
					"%s op_div: %d pll_div: %d pll_mul: %d\n",
					__func__, op_div, pll_div, pll_mul);

				diff_old = diff;

				sensor->pll.pre_pll_div = pll_div;
				sensor->pll.pre_pll_mul = pll_mul;
				sensor->pll.op_sys_div = op_div;
			}
		}
	}

	if (sensor->pll.pre_pll_mul == 0) {
		v4l2_err(sd, "Unable to find matching pll config\n");
		return -EINVAL;
	}

	sensor->pll.op_pix_div = bpp;
	sensor->pll.vt_sys_div = sensor->pll.op_sys_div;
	sensor->pll.vt_pix_div = bpp / lanes;

	sensor->pll.vco_freq = ar0144_clk_div_mul(ext_freq,
						  sensor->pll.pre_pll_div,
						  sensor->pll.pre_pll_mul);

	sensor->pll.pix_freq = ar0144_clk_div_mul(sensor->pll.vco_freq,
						  sensor->pll.vt_sys_div *
						  sensor->pll.vt_pix_div, 1);

	sensor->pll.ser_freq = ar0144_clk_div_mul(sensor->pll.vco_freq,
						  sensor->pll.op_sys_div, 1);

	sensor->pll.update = false;

	dev_dbg(sd->dev, "VCO: %ld PIX: %ld SER: %ld\n",
		sensor->pll.vco_freq, sensor->pll.pix_freq,
		sensor->pll.ser_freq);

	dev_dbg(sd->dev, "PLLDIV: %d PLLMUL: %d\n",
		sensor->pll.pre_pll_div, sensor->pll.pre_pll_mul);
	dev_dbg(sd->dev, "VTDIV: %d VTDIVPIX: %d\n",
		sensor->pll.vt_sys_div, sensor->pll.vt_pix_div);
	dev_dbg(sd->dev, "OPDIV: %d OPDIVPIX: %d\n",
		sensor->pll.op_sys_div, sensor->pll.op_pix_div);

	return 0;
}

static int ar0144_config_pll(struct ar0144 *sensor)
{
	int ret;

	ret = ar0144_write(sensor, AR0144_VT_PIX_CLK_DIV,
			   sensor->pll.vt_pix_div);
	if (ret)
		return ret;

	ret = ar0144_write(sensor, AR0144_VT_SYS_CLK_DIV,
			   sensor->pll.vt_sys_div);
	if (ret)
		return ret;

	ret = ar0144_write(sensor, AR0144_PRE_PLL_CLK_DIV,
			   sensor->pll.pre_pll_div);
	if (ret)
		return ret;

	ret = ar0144_write(sensor, AR0144_PLL_MUL,
			   sensor->pll.pre_pll_mul);
	if (ret)
		return ret;

	ret = ar0144_write(sensor, AR0144_OP_PIX_CLK_DIV,
			   sensor->pll.op_pix_div);
	if (ret)
		return ret;

	ret = ar0144_write(sensor, AR0144_OP_SYS_CLK_DIV,
			   sensor->pll.op_sys_div);
	if (ret)
		return ret;

	/* Wait for PLL to lock */
	usleep_range(1000, 1500);

	return 0;
}

static int ar0144_config_frame(struct ar0144 *sensor)
{
	unsigned int hlength;
	unsigned int vlength;
	unsigned int height = sensor->fmt.height * sensor->h_scale;
	unsigned int width = sensor->fmt.width * sensor->w_scale;
	int ret;
	u16 x_end, y_end;

	ret = ar0144_write(sensor, AR0144_Y_ADDR_START, sensor->crop.top);
	if (ret)
		goto out;

	ret = ar0144_write(sensor, AR0144_X_ADDR_START, sensor->crop.left);
	if (ret)
		goto out;

	y_end = sensor->crop.top + height - 1;

	if (sensor->embedded_stat)
		y_end -= 2;
	if (sensor->embedded_data)
		y_end -= 2;

	ret = ar0144_write(sensor, AR0144_Y_ADRR_END, y_end);
	if (ret)
		goto out;

	x_end = sensor->crop.left + width - 1;
	ret = ar0144_write(sensor, AR0144_X_ADRR_END, x_end);
	if (ret)
		goto out;

	hlength = sensor->fmt.width;
	hlength += sensor->hblank;
	hlength = clamp_t(unsigned int, hlength,
			  sensor->limits.hlen.min, sensor->limits.hlen.max);

	vlength = sensor->fmt.height;
	vlength += sensor->vblank;
	vlength = clamp_t(unsigned int, vlength,
			  sensor->limits.vlen.min, sensor->limits.vlen.max);

	ret = ar0144_write(sensor, AR0144_FRAME_LENGTH_LINES, vlength);
	if (ret)
		goto out;

	ret = ar0144_write(sensor, AR0144_LINE_LENGTH_PCK, hlength);
	if (ret)
		goto out;

	ret = ar0144_write(sensor, AR0144_X_ODD_INC,
			   (sensor->w_scale << 1) - 1);
	if (ret)
		goto out;

	ret = ar0144_write(sensor, AR0144_Y_ODD_INC,
			   (sensor->h_scale << 1) - 1);
	if (ret)
		goto out;


	/* Enable embedded statistics for Auto Exposure to work
	 * Since it is embedded after the active frame there is no issue
	 * enabling it all the time.
	 */
	ret = ar0144_update_bits(sensor, AR0144_SMIA_TEST,
				 AR0144_FLD_EMBEDDED_STATS_EN,
				 AR0144_FLD_EMBEDDED_STATS_EN);

out:
	return ret;
}

static int ar0144_config_parallel(struct ar0144 *sensor)
{
	unsigned int slew_rate_dat = sensor->pinfo.slew_rate_dat;
	unsigned int slew_rate_clk = sensor->pinfo.slew_rate_clk;
	u16 val = 0;
	u16 mask = 0;
	int ret = 0;

	if (slew_rate_dat != AR0144_NO_SLEW_RATE) {
		val |= slew_rate_dat << AR0144_FLD_SLEW_RATE_DAT_SHIFT;
		mask |= AR0144_FLD_SLEW_RATE_DAT_MASK;
	}

	if (slew_rate_clk != AR0144_NO_SLEW_RATE) {
		val |= slew_rate_clk << AR0144_FLD_SLEW_RATE_CLK_SHIFT;
		mask |= AR0144_FLD_SLEW_RATE_CLK_MASK;
	}

	if (mask) {
		ret = ar0144_update_bits(sensor, AR0144_DATAPATH_SEL,
					 mask, val);
		if (ret)
			return ret;
	}

	val = sensor->bpp << AR0144_FLD_DATA_FMT_IN_SHIFT;
	val |= sensor->bpp << AR0144_FLD_DATA_FMT_OUT_SHIFT;
	ret = ar0144_write(sensor, AR0144_DATA_FORMAT_BITS, val);
	if (ret)
		return ret;

	ret = ar0144_update_bits(sensor, AR0144_SERIAL_FORMAT,
				 AR0144_FLD_DUAL_LANE |
				 AR0144_FLD_SINGLE_LANE,
				 0);
	if (ret)
		return ret;

	ret = ar0144_update_bits(sensor, AR0144_RESET_REGISTER,
				 AR0144_FLD_SMIA_SER_DIS |
				 AR0144_FLD_PARALLEL_EN |
				 AR0144_FLD_DRIVE_PINS,
				 AR0144_FLD_SMIA_SER_DIS |
				 AR0144_FLD_PARALLEL_EN |
				 AR0144_FLD_DRIVE_PINS);
	if (ret)
		return ret;

	ret = ar0144_update_bits(sensor, AR0144_RESET_REGISTER,
				 AR0144_FLD_STREAM | AR0144_FLD_MASK_BAD,
				 AR0144_FLD_STREAM | AR0144_FLD_MASK_BAD);

	return ret;
}

static int ar0144_config_mipi(struct ar0144 *sensor)
{
	int ret = 0;
	u16 val;

	switch (sensor->bpp) {
	case 8:
		val = 0x2a;
		break;
	case 10:
		val = 0x2b;
		break;
	case 12:
		val = 0x2c;
		break;
	}

	ret = ar0144_write(sensor, AR0144_MIPI_CNTRL, val);
	if (ret)
		return ret;

	val = (sensor->minfo.t_hs_prep << AR0144_FLD_HS_PREP_SHIFT) |
	      (sensor->minfo.t_hs_zero << AR0144_FLD_HS_ZERO_SHIFT) |
	      (sensor->minfo.t_hs_trail << AR0144_FLD_HS_TRAIL_SHIFT) |
	      (sensor->minfo.t_clk_trail << AR0144_FLD_CLK_TRAIL_SHIFT);

	ret = ar0144_write(sensor, AR0144_MIPI_TIMING_0, val);
	if (ret)
		return ret;

	val = (sensor->minfo.t_clk_prep << AR0144_FLD_CLK_PREP_SHIFT) |
	      (sensor->minfo.t_hs_exit << AR0144_FLD_HS_EXIT_SHIFT) |
	      (sensor->minfo.t_clk_zero << AR0144_FLD_CLK_ZERO_SHIFT);

	ret = ar0144_write(sensor, AR0144_MIPI_TIMING_1, val);
	if (ret)
		return ret;

	val = (sensor->minfo.t_bgap << AR0144_FLD_BGAP_SHIFT) |
	      (sensor->minfo.t_clk_pre << AR0144_FLD_CLK_PRE_SHIFT) |
	      (sensor->minfo.t_clk_post << AR0144_FLD_CLK_POST_SHIFT);

	ret = ar0144_write(sensor, AR0144_MIPI_TIMING_2, val);
	if (ret)
		return ret;

	val = (sensor->minfo.t_lpx << AR0144_FLD_LPX_SHIFT) |
	      (sensor->minfo.t_wakeup << AR0144_FLD_WAKE_UP_SHIFT);

	ret = ar0144_write(sensor, AR0144_MIPI_TIMING_3, val);
	if (ret)
		return ret;

	val = (sensor->minfo.t_init << AR0144_FLD_INIT_SHIFT) |
	      (sensor->minfo.cont_tx_clk ? (u16) AR0144_FLD_CONT_TX_CLK : 0) |
	      (sensor->minfo.heavy_lp_load ?
	       (u16) AR0144_FLD_HEAVY_LP_LOAD : 0);

	ret = ar0144_write(sensor, AR0144_MIPI_TIMING_4, val);
	if (ret)
		return ret;

	ret = ar0144_write(sensor, AR0144_DATA_FORMAT_BITS,
			   sensor->bpp << AR0144_FLD_DATA_FMT_IN_SHIFT |
			   sensor->bpp << AR0144_FLD_DATA_FMT_OUT_SHIFT);
	if (ret)
		return ret;

	if (sensor->minfo.num_lanes == 1)
		val = AR0144_FLD_SINGLE_LANE;
	else
		val = AR0144_FLD_DUAL_LANE;

	ret = ar0144_update_bits(sensor, AR0144_SERIAL_FORMAT,
				 AR0144_FLD_DUAL_LANE |
				 AR0144_FLD_SINGLE_LANE,
				 val);
	if (ret)
		return ret;

	ret = ar0144_update_bits(sensor, AR0144_RESET_REGISTER,
				 AR0144_FLD_PARALLEL_EN |
				 AR0144_FLD_DRIVE_PINS,
				 0);
	if (ret)
		return ret;

	ret = ar0144_update_bits(sensor, AR0144_RESET_REGISTER,
				 AR0144_FLD_STREAM | AR0144_FLD_MASK_BAD,
				 AR0144_FLD_STREAM | AR0144_FLD_MASK_BAD);
	if (ret)
		return ret;

	ret = ar0144_update_bits(sensor, AR0144_RESET_REGISTER,
				 AR0144_FLD_SMIA_SER_DIS, 0);

	return ret;
}

static int ar0144_stream_on(struct ar0144 *sensor)
{
	struct v4l2_subdev *sd = &sensor->subdev;
	u16 mono_op;
	int ret;

	if (sensor->active_bus == AR0144_BUS_UNKNOWN) {
		v4l2_warn(sd, "No bus selected for streaming\n");
		return -EPIPE;
	}

	/* If the MIPI bus is in use the data and clk lanes are in LP-11 state.
	 * So we have to unset streaming and disable test mode before
	 * configuring the sensor.
	 */
	if (sensor->active_bus == AR0144_BUS_MIPI) {
		ret = ar0144_enter_standby(sensor);
		if (ret)
			return ret;
	}

	ret = ar0144_calculate_pll(sensor);
	if (ret)
		return ret;

	ret = ar0144_config_pll(sensor);
	if (ret)
		return ret;

	ret = ar0144_config_frame(sensor);
	if (ret)
		return ret;

	mono_op = sensor->model == AR0144_MODEL_MONOCHROME;

	ret = ar0144_update_bits(sensor, AR0144_DIGITAL_TEST,
				 AR0144_FLD_MONOCHROME_OP,
				 mono_op ? AR0144_FLD_MONOCHROME_OP : 0);
	if (ret)
		return ret;

	if (sensor->active_bus == AR0144_BUS_PARALLEL) {
		ret = ar0144_config_parallel(sensor);
		if (ret)
			return ret;
	} else {
		ret = ar0144_config_mipi(sensor);
		if (ret)
			return ret;
	}

	sensor->is_streaming = true;
	return 0;
}

static int ar0144_stream_off(struct ar0144 *sensor)
{
	int ret = 0;

	ret = ar0144_enter_standby(sensor);
	sensor->is_streaming = false;

	return ret;
}

/* V4L2 subdev video ops */
static int ar0144_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0144 *sensor = to_ar0144(sd);
	int ret = 0;

	dev_dbg(sd->dev, "%s enable: %d\n", __func__, enable);

	mutex_lock(&sensor->lock);

	if (enable && sensor->is_streaming) {
		ret = -EBUSY;
		goto out;
	}

	if (!enable && !sensor->is_streaming)
		goto out;

	if (enable)
		ret = ar0144_stream_on(sensor);
	else
		ret = ar0144_stream_off(sensor);

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ar0144_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct ar0144 *sensor = to_ar0144(sd);

	mutex_lock(&sensor->lock);

	/* TODO: Calculate current frame interval here or in set_selection to
	 * always have the correct frame interval.
	 */
	interval->interval = sensor->interval;

	mutex_unlock(&sensor->lock);
	return 0;
}

static struct v4l2_rect *ar0144_get_pad_crop(struct ar0144 *sensor,
					     struct v4l2_subdev_pad_config *cfg,
					     unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&sensor->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->crop;
	default:
		return NULL;
	}
}

static struct v4l2_mbus_framefmt *ar0144_get_pad_fmt(struct ar0144 *sensor,
					    struct v4l2_subdev_pad_config *cfg,
					    unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&sensor->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->fmt;
	default:
		return NULL;
	}
}

static unsigned int ar0144_find_skipfactor(unsigned int reduced,
					   unsigned int orig)
{
	unsigned int factor;
	unsigned int lower, upper, half;
	int i;

	/* We need to determine the closest supported skipfactor.
	 * Supported factors are:
	 * No Skip
	 * Skip 2
	 * Skip 4
	 * Skip 8
	 * Skip 16
	 */
	factor = DIV_ROUND_CLOSEST(orig, reduced);

	for (i = 0; i <= 4; i++) {
		if ((1 << i) > factor)
			break;
	}

	lower = (1 << (i - 1));
	upper = (1 << i);
	half = lower + ((upper - lower) / 2);

	if (factor <= half)
		return lower;
	else
		return upper;
}

/* V4L2 subdev pad ops */
static int ar0144_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0144 *sensor = to_ar0144(sd);

	if (code->index >= 0 && code->index < sensor->num_fmts) {
		code->code = sensor->formats[code->index].code;
		return 0;
	} else {
		return -EINVAL;
	}
}

static int ar0144_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0144 *sensor = to_ar0144(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	int ret = 0;

	mutex_lock(&sensor->lock);

	fmt = ar0144_get_pad_fmt(sensor, cfg, fse->pad, fse->which);
	crop = ar0144_get_pad_crop(sensor, cfg, fse->pad, fse->which);

	if (fse->index >= 4 || fse->code != fmt->code) {
		ret = -EINVAL;
		goto out;
	}

	fse->min_width = crop->width / (1u << fse->index);
	fse->max_width = fse->min_width;
	fse->min_height = crop->height / (1u << fse->index);
	fse->max_height = fse->min_height;

	if (fse->min_width > 1 && fse->min_height > 1)
		ret = 0;
	else
		ret = -EINVAL;

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ar0144_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ar0144 *sensor = to_ar0144(sd);
	const struct ar0144_format *sensor_format;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	unsigned int width, height;
	unsigned int w_scale, h_scale;
	int ret = 0;


	if (sensor->is_streaming && format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EBUSY;

	mutex_lock(&sensor->lock);

	fmt = ar0144_get_pad_fmt(sensor, cfg, format->pad, format->which);
	crop = ar0144_get_pad_crop(sensor, cfg, format->pad, format->which);

	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;

	sensor_format = ar0144_find_format(sensor, format->format.code);
	fmt->code = sensor_format->code;

	if (sensor_format->bpp != 8 &&
	    sensor_format->bpp != 10 &&
	    sensor_format->bpp != 12) {
		v4l2_err(sd,
			 "Something went wrong bpp is neither 8, 10 nor 12\n");
		ret = -EINVAL;
		goto out;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (sensor->bpp != sensor_format->bpp) {
			sensor->bpp = sensor_format->bpp;
			sensor->pll.update = true;
		}
	}

	if (sensor->model == AR0144_MODEL_COLOR)
		fmt->code = ar0144_match_col_format(sensor, crop);

	width = clamp_t(unsigned int, format->format.width,
			1, crop->width);
	height = clamp_t(unsigned int, format->format.height,
			 1, crop->height);

	w_scale = ar0144_find_skipfactor(width, crop->width);
	h_scale = ar0144_find_skipfactor(height, crop->height);

	fmt->width = crop->width / w_scale;
	fmt->height = crop->height / h_scale;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sensor->w_scale = w_scale;
		sensor->h_scale = h_scale;
	}

	format->format = *fmt;

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ar0144_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ar0144 *sensor = to_ar0144(sd);
	struct v4l2_mbus_framefmt *fmt;

	mutex_lock(&sensor->lock);

	fmt = ar0144_get_pad_fmt(sensor, cfg, format->pad, format->which);
	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int ar0144_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct ar0144 *sensor = to_ar0144(sd);
	struct v4l2_rect *_crop;
	unsigned int max_w, max_h;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (sensor->is_streaming &&
	    (sel->r.width != sensor->crop.width ||
	     sel->r.height != sensor->crop.height))
		return -EBUSY;

	mutex_lock(&sensor->lock);

	_crop = ar0144_get_pad_crop(sensor, cfg, sel->pad, sel->which);

	/* Check againts max, min values */
	max_w = sensor->limits.x.max - sensor->limits.x.min + 1;
	max_h = sensor->limits.y.max - sensor->limits.y.min + 1;

	_crop->top = min_t(unsigned int, sel->r.top, max_h);
	_crop->left = min_t(unsigned int, sel->r.left, max_w);
	_crop->width = min_t(unsigned int, sel->r.width, max_w);
	_crop->height = min_t(unsigned int, sel->r.height, max_h);

	if (sensor->is_streaming) {
		/* TODO: Add on the fly cropping change */
		mutex_unlock(&sensor->lock);
		return -EBUSY;
	}

	sel->r = *_crop;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int ar0144_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct ar0144 *sensor = to_ar0144(sd);
	struct v4l2_rect *_crop;
	unsigned int x_min = sensor->limits.x.min;
	unsigned int y_min = sensor->limits.y.min;
	unsigned int x_max = sensor->limits.x.max;
	unsigned int y_max = sensor->limits.y.max;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		mutex_lock(&sensor->lock);

		_crop = ar0144_get_pad_crop(sensor, cfg, sel->pad, sel->which);
		sel->r = *_crop;

		mutex_unlock(&sensor->lock);
		break;

	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = x_min;
		sel->r.top = y_min;
		sel->r.width = (x_max - x_min + 1);
		sel->r.height = (y_max - y_min + 1);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops ar0144_subdev_core_ops = {
	.s_power		= ar0144_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.s_register		= ar0144_s_register,
	.g_register		= ar0144_g_register,
#endif
};

static const struct v4l2_subdev_video_ops ar0144_subdev_video_ops = {
	.s_stream		= ar0144_s_stream,
	.g_frame_interval	= ar0144_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0144_subdev_pad_ops = {
	.enum_mbus_code		= ar0144_enum_mbus_code,
	.enum_frame_size	= ar0144_enum_frame_size,
	.set_fmt		= ar0144_set_fmt,
	.get_fmt		= ar0144_get_fmt,
	.set_selection		= ar0144_set_selection,
	.get_selection		= ar0144_get_selection,
};

static const struct v4l2_subdev_ops ar0144_subdev_ops = {
	.core			= &ar0144_subdev_core_ops,
	.video			= &ar0144_subdev_video_ops,
	.pad			= &ar0144_subdev_pad_ops,
};

static int ar0144_link_setup(struct media_entity *entity,
			     struct media_pad const *local,
			     struct media_pad const *remote,
			     u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct ar0144 *sensor = to_ar0144(sd);
	int ret = 0;

	if (sensor->is_streaming)
		return -EBUSY;

	if (!(flags & MEDIA_LNK_FL_ENABLED))
		return 0;

	mutex_lock(&sensor->lock);

	switch (local->index) {
	case AR0144_PAR_SINK:
		sensor->active_bus = AR0144_BUS_PARALLEL;
		break;

	case AR0144_MIPI_SINK:
		sensor->active_bus = AR0144_BUS_MIPI;
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct media_entity_operations ar0144_entity_ops = {
	.link_setup		= ar0144_link_setup,
};

static int ar0144_subdev_registered(struct v4l2_subdev *sd)
{
	struct ar0144 *sensor = to_ar0144(sd);

	ar0144_reset(sensor);
	v4l2_ctrl_handler_setup(&sensor->ctrls);
	return 0;
}

static const struct v4l2_subdev_internal_ops ar0144_subdev_internal_ops = {
	.registered		= ar0144_subdev_registered,
};

static int ar0144_set_analogue_gain(struct ar0144 *sensor, unsigned int val)
{
	unsigned int coarse, fine;

	for (coarse = 0; coarse < 5; coarse++)
		if (val < (1u << (coarse + 1)) * 1000)
			break;

	val = val / (1u << coarse);
	fine = 32 * (val - 1000) / val;

	if (fine > 15)
		fine = 15;

	ar0144_update_bits(sensor, AR0144_ANALOGUE_GAIN,
			   AR0144_FLD_COARSE_GAIN_MASK |
			   AR0144_FLD_FINE_GAIN_MASK,
			   (coarse << AR0144_FLD_COARSE_GAIN_SHIFT) |
			   (fine << AR0144_FLD_FINE_GAIN_SHIFT));

	return 1000 * (1u << coarse) * 32 / (32 - fine);
}

unsigned int ar0144_get_min_color_gain(struct ar0144 *sensor)
{
	unsigned int gains[4];
	int min_idx = 0;
	int i;

	gains[0] = sensor->gains.red_ctrl->cur.val;
	gains[1] = sensor->gains.greenr_ctrl->cur.val;
	gains[2] = sensor->gains.greenb_ctrl->cur.val;
	gains[3] = sensor->gains.blue_ctrl->cur.val;

	for (i = 0; i < 4; i++) {
		if (gains[i] < gains[min_idx])
			min_idx = i;
	}

	return gains[min_idx];
}

static int ar0144_set_digital_gain(struct ar0144 *sensor,
				   struct v4l2_ctrl *ctrl)
{
	unsigned int coarse, fine;
	unsigned int gain, gain_min, gain_factor;
	int ret = 0;

	coarse = ctrl->val / 1000;
	fine = (ctrl->val % 1000) * 128 / 1000;

	switch (ctrl->id) {
	case V4L2_CID_DIGITAL_GAIN:
		if (sensor->model == AR0144_MODEL_MONOCHROME) {
			ret = ar0144_write(sensor, AR0144_GLOBAL_GAIN,
					   (coarse << 7) | fine);
			return ret;
		}

		gain_min = ar0144_get_min_color_gain(sensor);
		gain_factor = (ctrl->val * 1000) / gain_min;

		gain = sensor->gains.red_ctrl->cur.val;
		gain += sensor->gains.red_clip;
		gain = (gain * gain_factor) / 1000;

		if (gain > 15999)
			sensor->gains.red_clip = gain - 15999;
		else
			sensor->gains.red_clip = 0;

		gain = clamp_t(unsigned int, gain, 1000, 15999);
		coarse = gain / 1000;
		fine = (gain % 1000) * 128 / 1000;
		ret = ar0144_write(sensor, AR0144_RED_GAIN,
				   (coarse << 7) | fine);
		if (ret)
			return ret;

		sensor->gains.red_ctrl->val = gain;
		sensor->gains.red_ctrl->cur.val = gain;

		gain = sensor->gains.greenr_ctrl->cur.val;
		gain += sensor->gains.greenr_clip;
		gain = (gain * gain_factor) / 1000;

		if (gain > 15999)
			sensor->gains.greenr_clip = gain - 15999;
		else
			sensor->gains.greenr_clip = 0;

		gain = clamp_t(unsigned int, gain, 1000, 15999);
		coarse = gain / 1000;
		fine = (gain % 1000) * 128 / 1000;
		ret = ar0144_write(sensor, AR0144_GREENR_GAIN,
				   (coarse << 7) | fine);
		if (ret)
			return ret;

		sensor->gains.greenr_ctrl->val = gain;
		sensor->gains.greenr_ctrl->cur.val = gain;

		gain = sensor->gains.greenb_ctrl->cur.val;
		gain += sensor->gains.greenb_clip;
		gain = (gain * gain_factor) / 1000;

		if (gain > 15999)
			sensor->gains.greenb_clip = gain - 15999;
		else
			sensor->gains.greenb_clip = 0;


		gain = clamp_t(unsigned int, gain, 1000, 15999);
		coarse = gain / 1000;
		fine = (gain % 1000) * 128 / 1000;
		ret = ar0144_write(sensor, AR0144_GREENB_GAIN,
				   (coarse << 7) | fine);
		if (ret)
			return ret;

		sensor->gains.greenb_ctrl->val = gain;
		sensor->gains.greenb_ctrl->cur.val = gain;

		gain = sensor->gains.blue_ctrl->cur.val;
		gain += sensor->gains.blue_clip;
		gain = (gain * gain_factor) / 1000;

		if (gain > 15999)
			sensor->gains.blue_clip = gain - 15999;
		else
			sensor->gains.blue_clip = 0;

		gain = clamp_t(unsigned int, gain, 1000, 15999);
		coarse = gain / 1000;
		fine = (gain % 1000) * 128 / 1000;
		ret = ar0144_write(sensor, AR0144_BLUE_GAIN,
				   (coarse << 7) | fine);
		if (ret)
			return ret;

		sensor->gains.blue_ctrl->val = gain;
		sensor->gains.blue_ctrl->cur.val = gain;

		break;

	case V4L2_CID_X_DIGITAL_GAIN_RED:
		ret = ar0144_write(sensor, AR0144_RED_GAIN,
				   (coarse << 7) | fine);
		if (!ret)
			sensor->gains.red_clip = 0;
		break;

	case V4L2_CID_X_DIGITAL_GAIN_GREENR:
		ret = ar0144_write(sensor, AR0144_GREENR_GAIN,
				   (coarse << 7) | fine);
		if (!ret)
			sensor->gains.greenr_clip = 0;
		break;

	case V4L2_CID_X_DIGITAL_GAIN_GREENB:
		ret = ar0144_write(sensor, AR0144_GREENB_GAIN,
				   (coarse << 7) | fine);
		if (!ret)
			sensor->gains.greenb_clip = 0;
		break;

	case V4L2_CID_X_DIGITAL_GAIN_BLUE:
		ret = ar0144_write(sensor, AR0144_BLUE_GAIN,
				   (coarse << 7) | fine);
		if (!ret)
			sensor->gains.blue_clip = 0;
		break;
	}

	return ret;
}

static int ar0144_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0144 *sensor = ctrl->priv;
	int ret = 0;
	u16 val;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		if (sensor->is_streaming) {
			ret = -EBUSY;
			goto out;
		}
		sensor->vblank = ctrl->val;
		break;

	case V4L2_CID_HBLANK:
		if (sensor->is_streaming) {
			ret = -EBUSY;
			goto out;
		}
		sensor->hblank = ctrl->val;
		break;

	case V4L2_CID_HFLIP:
		ret = ar0144_update_bits(sensor, AR0144_READ_MODE,
					 AR0144_FLD_HORIZ_MIRROR,
					 ctrl->val ?
					 AR0144_FLD_HORIZ_MIRROR : 0);
		break;

	case V4L2_CID_VFLIP:
		ret = ar0144_update_bits(sensor, AR0144_READ_MODE,
					 AR0144_FLD_VERT_FLIP,
					 ctrl->val ? AR0144_FLD_VERT_FLIP : 0);
		break;

	case V4L2_CID_EXPOSURE:
		/* TODO: implement EXPOSURE -> 100us * conversion and combine
		 * with V4L2_CID_X_EXPOSURE_FINE
		 */
		ret = ar0144_write(sensor, AR0144_COARSE_INT_TIME, ctrl->val);
		break;

	case V4L2_CID_X_EXPOSURE_FINE:
		/* TODO: remove me; see V4L2_CID_EXPOSURE */
		ret = ar0144_write(sensor, AR0144_FINE_INT_TIME, ctrl->val);
		break;

	case V4L2_CID_TEST_PATTERN_RED:
		ret = ar0144_write(sensor, AR0144_TEST_DATA_RED, ctrl->val);
		break;

	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = ar0144_write(sensor, AR0144_TEST_DATA_GREENR, ctrl->val);
		break;

	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = ar0144_write(sensor, AR0144_TEST_DATA_BLUE, ctrl->val);
		break;

	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = ar0144_write(sensor, AR0144_TEST_DATA_GREENB, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		val = ctrl->val == V4L2_EXPOSURE_AUTO ? AR0144_FLD_AE_EN : 0;
		ret = ar0144_update_bits(sensor, AR0144_AECTRL,
					 AR0144_FLD_AE_EN, val);
		break;

	case V4L2_CID_X_AUTO_EXPOSURE_TGT:
		ret = ar0144_write(sensor, AR0144_AE_LUMA_TGT, ctrl->val);
		break;

	case V4L2_CID_X_AUTO_EXPOSURE_MIN:
		ret = ar0144_write(sensor, AR0144_AE_MIN_EXPOSURE, ctrl->val);
		break;

	case V4L2_CID_X_AUTO_EXPOSURE_MAX:
		ret = ar0144_write(sensor, AR0144_AE_MAX_EXPOSURE, ctrl->val);
		break;

	case V4L2_CID_X_AUTOGAIN_ANALOGUE:
		ret = ar0144_update_bits(sensor, AR0144_AECTRL,
					 AR0144_FLD_AUTO_AG_EN,
					 ctrl->val ? AR0144_FLD_AUTO_AG_EN : 0);
		break;

	case V4L2_CID_X_AUTOGAIN_DIGITAL:
		ret = ar0144_update_bits(sensor, AR0144_AECTRL,
					 AR0144_FLD_AUTO_DG_EN,
					 ctrl->val ? AR0144_FLD_AUTO_DG_EN : 0);
		break;

	case V4L2_CID_X_AUTOGAIN_ANALOGUE_MIN:
		val = ctrl->val << AR0144_FLD_MIN_ANA_GAIN_SHIFT;
		ret = ar0144_update_bits(sensor, AR0144_AECTRL,
					 AR0144_FLD_MIN_ANA_GAIN_MASK, val);
		break;

	case V4L2_CID_X_EMBEDDED_DATA:
		if (sensor->is_streaming) {
			ret = -EBUSY;
			goto out;
		}

		/*
		 * Embedded statistics are always enabled but only shown when
		 * when the corresponding ctrl is set.
		 */
		val = ctrl->val & V4L2_X_EMBEDDED_DATA ?
		       AR0144_FLD_EMBEDDED_DATA : 0;
		ret = ar0144_update_bits(sensor, AR0144_SMIA_TEST,
					 AR0144_FLD_EMBEDDED_DATA, val);

		if (ret)
			goto out;

		sensor->embedded_stat = ctrl->val & V4L2_X_EMBEDDED_STAT ?
					true : false;
		sensor->embedded_data = ctrl->val & V4L2_X_EMBEDDED_DATA ?
					true : false;

		break;

	case V4L2_CID_TEST_PATTERN:
		ret = ar0144_write(sensor, AR0144_TEST_PATTERN,
				   ctrl->val < 4 ? ctrl->val : 256);
		if (ret)
			goto out;
		/* see note in register reference: "avoid pixel clipping to
		 * 3952"; 0x3044 itself is undocumented
		 */
		ret = ar0144_update_bits(sensor, 0x3044, 3u << 4, 0);
		break;

	case V4L2_CID_X_BINNING_COL:
		switch (ctrl->val) {
		case 0:
			val = 0;
			break;
		case 1:
			val = AR0144_FLD_RM_COL_BIN;
			break;
		case 2:
			val = AR0144_FLD_RM_COL_SUM;
			break;
		}
		ret = ar0144_update_bits(sensor, AR0144_READ_MODE,
					 AR0144_FLD_RM_COL_BIN |
					 AR0144_FLD_RM_COL_SUM,
					 val);
		break;

	case V4L2_CID_X_BINNING_ROW:
		ret = ar0144_update_bits(sensor, AR0144_READ_MODE,
					 AR0144_FLD_RM_ROW_BIN,
					 ctrl->val > 0 ?
					 AR0144_FLD_RM_ROW_BIN : 0);
		break;

	case V4L2_CID_X_COMPANDING:
		ret = ar0144_update_bits(sensor, AR0144_COMPANDING,
					 AR0144_FLD_COMPAND_EN,
					 ctrl->val ? AR0144_FLD_COMPAND_EN : 0);
		break;

	case V4L2_CID_DIGITAL_GAIN:
	case V4L2_CID_X_DIGITAL_GAIN_RED:
	case V4L2_CID_X_DIGITAL_GAIN_GREENR:
	case V4L2_CID_X_DIGITAL_GAIN_BLUE:
	case V4L2_CID_X_DIGITAL_GAIN_GREENB:
		ret = ar0144_set_digital_gain(sensor, ctrl);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		ctrl->val = ar0144_set_analogue_gain(sensor, ctrl->val);
		break;

	case V4L2_CID_X_BLACK_LEVEL_AUTO:
		ret = ar0144_update_bits(sensor, AR0144_DELTA_DK_CTRL,
					 AR0144_FLD_DK_SUB_EN,
					 ctrl->val ? AR0144_FLD_DK_SUB_EN : 0);
		break;

	case V4L2_CID_FLASH_LED_MODE:
		val = ctrl->val ? AR0144_FLD_LED_FLASH_EN : 0;
		ret = ar0144_update_bits(sensor, AR0144_FLASH_CTRL,
					 AR0144_FLD_LED_FLASH_EN, val);
		break;

	case V4L2_CID_X_FLASH_DELAY:
		val = ctrl->val << AR0144_FLD_LED_DELAY_SHIFT;
		val &= AR0144_FLD_LED_DELAY_MASK;
		ret = ar0144_update_bits(sensor, AR0144_FLASH_CTRL,
					 AR0144_FLD_LED_DELAY_MASK, val);
		break;

	case V4L2_CID_X_DYNAMIC_PIXEL_CORRECTION:
		val = ctrl->val ? AR0144_FLD_PIX_DEF_1D_DDC_EN : 0;

		ret = ar0144_update_bits(sensor, AR0144_PIX_DEF_ID,
					 AR0144_FLD_PIX_DEF_1D_DDC_EN, val);
		break;

	case V4L2_CID_LINK_FREQ:
		break;
	default:
		ret = -ENOTTY;
		break;
	}

out:
	return ret;
}

static int ar0144_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0144 *sensor = ctrl->priv;
	int ret = 0;
	u16 val;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ctrl->val = sensor->vblank;
		break;

	case V4L2_CID_HBLANK:
		ctrl->val = sensor->hblank;
		break;

	case V4L2_CID_X_AUTO_EXPOSURE_CUR:
		ret = ar0144_read(sensor, AR0144_AE_COARSE_INT_TIME, &val);
		if (ret)
			return ret;

		ctrl->val = val;
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ar0144_ctrl_ops = {
	.s_ctrl			= ar0144_s_ctrl,
	.g_volatile_ctrl	= ar0144_g_ctrl,
};

static const char * const ar0144_test_pattern_menu[] = {
	"disabled",
	"solid color",
	"color bar",
	"fade to gray",
	"walking 1 (12 bit)"
};

static char const * const ar0144_embdata_menu[] = {
	"disabled",
	"stats",
	"data",
	"both",
};

static const char * const ar0144_binning_menu[] = {
	"none",
	"avg",
	"sum",
};

static const char * const ar0144_ana_gain_min_menu[] = {
	"1x",
	"2x",
	"4x",
	"8x",
};

static const s64 ar0144_link_freq[] = {
	148500000,
	185625000,
	222750000,
	297000000,
	371250000,
	445500000,
};

static const struct v4l2_ctrl_config ar0144_ctrls[] = {
	{
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_VBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.flags		= V4L2_CTRL_FLAG_VOLATILE,
		.min		= 22,
		.max		= 65535,
		.step		= 1,
		.def		= 22,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_HBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.flags		= V4L2_CTRL_FLAG_VOLATILE,
		.min		= 208,
		.max		= 65535,
		.step		= 1,
		.def		= 208,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 800,
	}, {
		/* TODO: remove me and replace by EXPOSURE -> 100us
		 * conversion
		 */
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_EXPOSURE_FINE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure Fine",
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 10,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_RED,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 4095,
		.step		= 1,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_GREENR,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 4095,
		.step		= 1,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_GREENB,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 4095,
		.step		= 1,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_BLUE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 4095,
		.step		= 1,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_AUTO_EXPOSURE_TGT,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Auto Exposure Target",
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 0x5000,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_AUTO_EXPOSURE_MIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Auto Exposure Min",
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_AUTO_EXPOSURE_MAX,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Auto Exposure Max",
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 800,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_AUTO_EXPOSURE_CUR,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Auto Exposure Cur",
		.flags		= (V4L2_CTRL_FLAG_READ_ONLY |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 65535,
		.step		= 1,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_MENU,
		.min		= 0,
		.max		= V4L2_EXPOSURE_MANUAL,
		.menu_skip_mask	= ~(BIT(V4L2_EXPOSURE_AUTO) |
				    BIT(V4L2_EXPOSURE_MANUAL)),
		.def		= V4L2_EXPOSURE_AUTO,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_AUTOGAIN_ANALOGUE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Autogain Analogue",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_AUTOGAIN_DIGITAL,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Autogain Digital",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_test_pattern_menu) - 1,
		.qmenu		= ar0144_test_pattern_menu,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_EMBEDDED_DATA,
		.type		= V4L2_CTRL_TYPE_MENU,
		.flags		= V4L2_CTRL_FLAG_MODIFY_LAYOUT,
		.name		= "Embedded Data",
		.min		= V4L2_X_EMBEDDED_OFF,
		.max		= ARRAY_SIZE(ar0144_embdata_menu) - 1,
		.def		= V4L2_X_EMBEDDED_OFF,
		.qmenu		= ar0144_embdata_menu,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_BINNING_COL,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Col Binning",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_binning_menu) - 1,
		.qmenu		= ar0144_binning_menu,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_BINNING_ROW,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Row Binning",
		.min		= 0,
		/* filter out 'sum' from the menu by omitting last entry */
		.max		= ARRAY_SIZE(ar0144_binning_menu) - 2,
		.qmenu		= ar0144_binning_menu,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_COMPANDING,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Compading",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_ANALOGUE_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 1000,
		.step		= 1,
		.max		= 16000,
		.def		= 2000,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_DIGITAL_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.flags		= (V4L2_CTRL_FLAG_EXECUTE_ON_WRITE |
				   V4L2_CTRL_FLAG_UPDATE),
		.min		= 1000,
		.step		= 1,
		.max		= 15999,
		.def		= 1000,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_RED,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Digital Gain Red",
		.min		= 1000,
		.step		= 1,
		.max		= 15999,
		.def		= 1300,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_GREENR,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Digital Gain Green (Red)",
		.min		= 1000,
		.step		= 1,
		.max		= 15999,
		.def		= 1000,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_GREENB,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Digital Gain Green (Blue)",
		.min		= 1000,
		.step		= 1,
		.max		= 15999,
		.def		= 1000,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_BLUE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Digital Gain Blue",
		.min		= 1000,
		.step		= 1,
		.max		= 15999,
		.def		= 1500,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_AUTOGAIN_ANALOGUE_MIN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Analogue Gain Auto Min",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_ana_gain_min_menu) - 1,
		.qmenu		= ar0144_ana_gain_min_menu,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_LINK_FREQ,
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_link_freq) - 1,
		.def		= 0,
		.qmenu_int	= ar0144_link_freq,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_BLACK_LEVEL_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Black Level Correction",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_FLASH_LED_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.min		= 0,
		.max		= V4L2_FLASH_LED_MODE_FLASH,
		.menu_skip_mask = BIT(V4L2_FLASH_LED_MODE_TORCH),
		.def		= V4L2_FLASH_LED_MODE_NONE,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_FLASH_DELAY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Flash Delay",
		.min		= -128,
		.step		= 1,
		.max		= 127,
		.def		= 0,
	}, {
		.ops		= &ar0144_ctrl_ops,
		.id		= V4L2_CID_X_DYNAMIC_PIXEL_CORRECTION,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Dynamic Defect Pixel Correction",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
	},
};

static int ar0144_create_ctrls(struct ar0144 *sensor)
{
	struct v4l2_ctrl_config ctrl_cfg;
	struct v4l2_ctrl *ctrl;
	int i;
	int ret;

	ret = v4l2_ctrl_handler_init(&sensor->ctrls, 10);
	if (ret < 0)
		return ret;

	sensor->subdev.ctrl_handler = &sensor->ctrls;

	for (i = 0; i < ARRAY_SIZE(ar0144_ctrls); i++) {
		ctrl_cfg = ar0144_ctrls[i];

		switch (ctrl_cfg.id) {
		case V4L2_CID_X_DIGITAL_GAIN_RED:
		case V4L2_CID_X_DIGITAL_GAIN_GREENR:
		case V4L2_CID_X_DIGITAL_GAIN_BLUE:
		case V4L2_CID_X_DIGITAL_GAIN_GREENB:
			if (sensor->model == AR0144_MODEL_MONOCHROME)
				continue;

			break;

		case V4L2_CID_X_EMBEDDED_DATA:
			if (sensor->active_bus == AR0144_BUS_MIPI)
				continue;

			break;

		case V4L2_CID_HBLANK:
			ctrl_cfg.min = sensor->limits.hblank.min;
			ctrl_cfg.def = ctrl_cfg.min;
			break;

		case V4L2_CID_VBLANK:
			ctrl_cfg.min = sensor->limits.vblank.min;
			ctrl_cfg.def = ctrl_cfg.min;
			break;

		default:
			break;
		}

		ctrl = v4l2_ctrl_new_custom(&sensor->ctrls,
					    &ctrl_cfg, sensor);

		ret = sensor->ctrls.error;
		if (ret) {
			dev_warn(sensor->dev,
				  "failed to register control "
				  "'%s'(0x%x): %d\n",
				  ctrl_cfg.name ? ctrl_cfg.name :
				  v4l2_ctrl_get_name(ctrl_cfg.id),
				  ctrl_cfg.id, ret);
			return ret;
		}

		switch (ctrl->id) {
		case V4L2_CID_LINK_FREQ:
			sensor->link_freq_ctrl = ctrl;
			break;

		case V4L2_CID_X_DIGITAL_GAIN_RED:
			if (sensor->model == AR0144_MODEL_COLOR)
				sensor->gains.red_ctrl = ctrl;
			break;

		case V4L2_CID_X_DIGITAL_GAIN_GREENB:
			if (sensor->model == AR0144_MODEL_COLOR)
				sensor->gains.greenb_ctrl = ctrl;
			break;

		case V4L2_CID_X_DIGITAL_GAIN_GREENR:
			if (sensor->model == AR0144_MODEL_COLOR)
				sensor->gains.greenr_ctrl = ctrl;
			break;

		case V4L2_CID_X_DIGITAL_GAIN_BLUE:
			if (sensor->model == AR0144_MODEL_COLOR)
				sensor->gains.blue_ctrl = ctrl;
			break;

		default:
			break;
		}
	}

	return 0;
}

static void ar0144_set_defaults(struct ar0144 *sensor)
{
	sensor->limits = (struct ar0144_sensor_limits) {
					/* min		max	 */
		.x			= {0,		1295      },
		.y			= {0,		807       },
		.hlen			= {1488,	65534     },
		.vlen			= {29,		65535     },
		.hblank			= {208,		65535     },
		.vblank			= {22,		65535     },
		.pre_pll_div		= {1,		63        },
		.pre_pll_mul		= {32,		255       },
		.pll_vt_sys_clk_div	= {1,		16        },
		.pll_vt_pix_clk_div	= {4,		16        },
		.pll_op_sys_clk_div	= {1,		16        },
		.pll_op_pix_clk_div	= {8,		12        },
		.ext_clk		= {6000000,	48000000  },
		.pix_clk		= {0,		74250000  },
		.vco			= {384000000,	768000000 },
	};

	sensor->crop.left = 4;
	sensor->crop.top = 4;
	sensor->crop.width = AR0144_DEF_WIDTH;
	sensor->crop.height = AR0144_DEF_HEIGHT;

	sensor->fmt.width = AR0144_DEF_WIDTH;
	sensor->fmt.height = AR0144_DEF_HEIGHT;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->fmt.colorspace = V4L2_COLORSPACE_SRGB;
	sensor->interval.numerator = 1;
	sensor->interval.denominator = 60;

	if (sensor->model == AR0144_MODEL_MONOCHROME) {
		sensor->formats = ar0144_mono_formats;
		sensor->num_fmts = ARRAY_SIZE(ar0144_mono_formats);
	} else {
		sensor->formats = ar0144_col_formats;
		sensor->num_fmts = ARRAY_SIZE(ar0144_col_formats);
	}

	sensor->fmt.code = sensor->formats[sensor->num_fmts - 1].code;
	sensor->bpp = sensor->formats[sensor->num_fmts - 1].bpp;
	sensor->pll.update = true;

	sensor->w_scale = 1;
	sensor->h_scale = 1;
	sensor->hblank = sensor->limits.hblank.min;
	sensor->vblank = sensor->limits.vblank.min;

	if (sensor->pinfo.link_freq == 0)
		sensor->pinfo.link_freq = sensor->limits.pix_clk.max;
}

static int ar0144_check_chip_id(struct ar0144 *sensor)
{
	struct device *dev = sensor->dev;
	u16 model_id, customer_rev;
	int ret = 0;

	ret = ar0144_power_on(sensor);
	if (ret) {
		dev_err(dev, "Failed to power on sensor (%d)\n", ret);
		return ret;
	}

	ret = ar0144_read(sensor, AR0144_MODEL_ID, &model_id);
	if (ret) {
		dev_err(dev, "Failed to read model ID (%d)\n", ret);
		goto out;
	}

	if (model_id != AR0144_CHIP_VERSION) {
		dev_err(dev, "Wrong chip version: 0x%04x <-> 0x%04x\n",
			model_id, AR0144_CHIP_VERSION);
		ret = -ENOENT;
		goto out;
	}

	ret = ar0144_read(sensor, AR0144_CUSTOMER_REV, &customer_rev);
	if (ret)
		goto out;

	dev_info(dev, "Device ID: 0x%04x customer rev: 0x%04x\n",
		 model_id, customer_rev);

	if (sensor->model == AR0144_MODEL_UNKNOWN) {
		if (customer_rev & BIT(4))
			sensor->model = AR0144_MODEL_COLOR;
		else
			sensor->model = AR0144_MODEL_MONOCHROME;
	}

out:
	ar0144_power_off(sensor);
	return ret;
}

static int ar0144_parse_par_ep(struct ar0144 *sensor, struct device_node *ep)
{
	struct device *dev = sensor->dev;
	struct fwnode_handle *fwnode;
	struct v4l2_fwnode_endpoint endpoint = {
		.bus_type = V4L2_MBUS_PARALLEL
	};
	int ret;
	int32_t tmp;

	/* Skip parsing if no endpoint was found */
	if (!ep)
		return 0;

	fwnode = of_fwnode_handle(ep);

	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &endpoint);
	if (ret) {
		dev_err(dev, "Failed to parse parallel endpoint (%d)\n", ret);
		return ret;
	}

	if (endpoint.nr_of_link_frequencies > 0)
		sensor->pinfo.link_freq = endpoint.link_frequencies[0];
	else
		sensor->pinfo.link_freq = 0;

	tmp = AR0144_NO_SLEW_RATE;
	of_property_read_s32(ep, "onsemi,slew-rate-dat", &tmp);
	sensor->pinfo.slew_rate_dat = clamp_t(unsigned int, tmp, 0, 0x7);

	tmp = AR0144_NO_SLEW_RATE;
	of_property_read_s32(ep, "onsemi,slew-rate-clk", &tmp);
	sensor->pinfo.slew_rate_clk = clamp_t(unsigned int, tmp, 0, 0x7);

	return 0;
}

static int ar0144_parse_mipi_ep(struct ar0144 *sensor, struct device_node *ep)
{
	struct device *dev = sensor->dev;
	struct fwnode_handle *fwnode;
	struct v4l2_fwnode_endpoint endpoint = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret;
	unsigned int tmp;

	/* Skip parsing if no endpoint was found */
	if (!ep)
		return 0;

	fwnode = of_fwnode_handle(ep);

	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &endpoint);
	if (ret) {
		dev_err(dev, "Failed to parse MIPI endpoint (%d)\n", ret);
		return ret;
	}

	sensor->minfo.num_lanes = endpoint.bus.mipi_csi2.num_data_lanes;
	if (sensor->minfo.num_lanes < 1 || sensor->minfo.num_lanes > 2) {
		dev_err(dev, "Wrong number of lanes configured");
		return -EINVAL;
	}

	tmp = 2;
	of_property_read_u32(ep, "onsemi,t-hs-prep", &tmp);
	sensor->minfo.t_hs_prep = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 6;
	of_property_read_u32(ep, "onsemi,t-hs-zero", &tmp);
	sensor->minfo.t_hs_zero = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 6;
	of_property_read_u32(ep, "onsemi,t-hs-trail", &tmp);
	sensor->minfo.t_hs_trail = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 5;
	of_property_read_u32(ep, "onsemi,t-clk-trail", &tmp);
	sensor->minfo.t_clk_trail = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 1;
	of_property_read_u32(ep, "onsemi,t-clk-prep", &tmp);
	sensor->minfo.t_clk_prep = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 4;
	of_property_read_u32(ep, "onsemi,t-hs-exit", &tmp);
	sensor->minfo.t_hs_exit = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 14;
	of_property_read_u32(ep, "onsemi,t-clk-zero", &tmp);
	sensor->minfo.t_clk_zero = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 2;
	of_property_read_u32(ep, "onsemi,t-bgap", &tmp);
	sensor->minfo.t_bgap = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 1;
	of_property_read_u32(ep, "onsemi,t-clk-pre", &tmp);
	sensor->minfo.t_clk_pre = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 7;
	of_property_read_u32(ep, "onsemi,t-clk-post", &tmp);
	sensor->minfo.t_clk_post = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 2;
	of_property_read_u32(ep, "onsemi,t-lpx", &tmp);
	sensor->minfo.t_lpx = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 5;
	of_property_read_u32(ep, "onsemi,t-wakeup", &tmp);
	sensor->minfo.t_wakeup = clamp_t(unsigned int, tmp, 0, 0x7f);

	tmp = 0;
	of_property_read_u32(ep, "onsemi,cont-tx-clk", &tmp);
	sensor->minfo.cont_tx_clk = tmp ? true : false;

	tmp = 0;
	of_property_read_u32(ep, "onsemi,heavy-lp-load", &tmp);
	sensor->minfo.heavy_lp_load = tmp ? true : false;

	tmp = 4;
	of_property_read_u32(ep, "onsemi,t-init", &tmp);
	sensor->minfo.t_init = clamp_t(unsigned int, tmp, 0, 0x7f);

	return 0;
}

static int ar0144_of_probe(struct ar0144 *sensor)
{
	struct device *dev = sensor->dev;
	struct device_node *mipi_ep, *par_ep;
	struct clk *clk;
	struct gpio_desc *gpio;
	int ret;

	clk = devm_clk_get(dev, "ext");
	ret = PTR_ERR_OR_ZERO(clk);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret < 0) {
		dev_err(dev, "Failed to get external clock (%d)\n", ret);
		return ret;
	}

	sensor->extclk = clk;

	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	ret = PTR_ERR_OR_ZERO(clk);
	if (ret < 0) {
		dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
		return ret;
	}

	sensor->reset_gpio = gpio;

	par_ep = of_graph_get_endpoint_by_regs(dev->of_node,
					       AR0144_PAR_SINK, 0);
	mipi_ep = of_graph_get_endpoint_by_regs(dev->of_node,
						AR0144_MIPI_SINK, 0);

	if (!par_ep && !mipi_ep) {
		dev_err(dev, "Neither MIPI nor parallel endpoint found\n");
		return -ENODEV;
	}

	if (par_ep && !mipi_ep)
		sensor->active_bus = AR0144_BUS_PARALLEL;
	else if (!par_ep && mipi_ep)
		sensor->active_bus = AR0144_BUS_MIPI;
	else
		sensor->active_bus = AR0144_BUS_UNKNOWN;

	ret = ar0144_parse_par_ep(sensor, par_ep);
	if (ret)
		goto out;

	ret = ar0144_parse_mipi_ep(sensor, mipi_ep);

out:
	of_node_put(par_ep);
	of_node_put(mipi_ep);

	return ret;
}

static int ar0144_probe(struct i2c_client *i2c,
			const struct i2c_device_id *did)
{
	struct ar0144 *sensor;
	struct v4l2_subdev *sd;
	int ret;

	sensor = devm_kzalloc(&i2c->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sd = &sensor->subdev;
	sensor->model = did->driver_data;
	sensor->dev = &i2c->dev;

	dev_info(sensor->dev, "Probing AR0144 Driver\n");

	ret = ar0144_of_probe(sensor);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	v4l2_i2c_subdev_init(sd, i2c, &ar0144_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &ar0144_subdev_internal_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &ar0144_entity_ops;

	sensor->pad[AR0144_PAR_SINK].flags = MEDIA_PAD_FL_SOURCE;
	sensor->pad[AR0144_MIPI_SINK].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, AR0144_NUM_PADS, sensor->pad);
	if (ret)
		goto out_media;

	ret = ar0144_check_chip_id(sensor);
	if (ret)
		goto out_media;

	ar0144_set_defaults(sensor);

	ret = ar0144_create_ctrls(sensor);
	if (ret)
		goto out;

	ret = v4l2_async_register_subdev(&sensor->subdev);
	if (ret)
		goto out;

	return 0;

out:
	v4l2_ctrl_handler_free(&sensor->ctrls);
out_media:
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int ar0144_remove(struct i2c_client *i2c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(i2c);
	struct ar0144 *sensor = to_ar0144(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&sensor->lock);

	return 0;
}

static const struct i2c_device_id ar0144_id_table[] = {
	{ "ar0144", AR0144_MODEL_UNKNOWN },
	{ "ar0144c", AR0144_MODEL_COLOR },
	{ "ar0144m", AR0144_MODEL_MONOCHROME },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ar0144_id_table);

static const struct of_device_id ar0144_of_match[] = {
	{ .compatible = "onsemi,ar0144" },
	{ .compatible = "onsemi,ar0144c" },
	{ .compatible = "onsemi,ar0144m" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0144_of_match);

static struct i2c_driver ar0144_i2c_driver = {
	.driver		= {
		.name	= "ar0144",
	.of_match_table	= of_match_ptr(ar0144_of_match),
	},
	.probe		= ar0144_probe,
	.remove		= ar0144_remove,
	.id_table	= ar0144_id_table,
};
module_i2c_driver(ar0144_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stefan Riedmueller <s.riedmueller@phytec.de>");
