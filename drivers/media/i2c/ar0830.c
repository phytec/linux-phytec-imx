// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2026 PHYTEC Messtechnik GmbH
 * Author: Stefan Riedmüller <s.riedmueller@phytec.de>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/v4l2-mediabus.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "ccs-pll.h"

#define AR0830_CHIP_VERSION_REG					CCI_REG16(0x0016)
#define AR0830_CUSTOMER_REV					CCI_REG16(0x0022)
#define		AR0830_CUSTOMER_REV_CFA(n)				(((n) >> 4) & 0x7)
#define		AR0830_CUSTOMER_REV_CFA_RGB				0
#define		AR0830_CUSTOMER_REV_CFA_MONO				1
#define		AR0830_CUSTOMER_REV_CFA_RGBIR				2
#define AR0830_MODE_SELECT					CCI_REG8(0x0100)
#define AR0830_IMAGE_ORIENTATION				CCI_REG8(0x0101)
#define		AR0830_VFLIP						BIT(1)
#define		AR0830_HMIRROR						BIT(0)
#define AR0830_SOFTWARE_RESET					CCI_REG8(0x0103)
#define AR0830_CSI_DATA_FORMAT					CCI_REG16(0x0112)
#define		AR0830_DATA_FORMAT_IN(n)				(((n) & 0xf) << 8)
#define		AR0830_DATA_FORMAT_OUT(n)				(((n) & 0xf) << 0)
#define AR0830_CSI_LANE_MODE					CCI_REG8(0x0114)
#define AR0830_COARSE_INTEGRATION_TIME				CCI_REG16(0x0202)
#define AR0830_VT_PIX_CLK_DIV					CCI_REG16(0x0300)
#define AR0830_VT_SYS_CLK_DIV					CCI_REG16(0x0302)
#define AR0830_VT_PRE_PLL_CLK_DIV				CCI_REG16(0x0304)
#define AR0830_VT_PLL_MULTIPLIER				CCI_REG16(0x0306)
#define AR0830_OP_PIX_CLK_DIV					CCI_REG16(0x0308)
#define AR0830_OP_SYS_CLK_DIV					CCI_REG16(0x030a)
#define AR0830_OP_PRE_PLL_CLK_DIV				CCI_REG16(0x030c)
#define AR0830_OP_PLL_MULTIPLIER				CCI_REG16(0x030e)
#define AR0830_FRAME_LENGTH_LINES				CCI_REG16(0x0340)
#define AR0830_LINE_LENGTH_PCK					CCI_REG16(0x0342)
#define AR0830_X_ADDR_START					CCI_REG16(0x0344)
#define AR0830_Y_ADDR_START					CCI_REG16(0x0346)
#define AR0830_X_ADDR_END					CCI_REG16(0x0348)
#define AR0830_Y_ADDR_END					CCI_REG16(0x034a)
#define AR0830_X_OUTPUT_SIZE					CCI_REG16(0x034c)
#define AR0830_Y_OUTPUT_SIZE					CCI_REG16(0x034e)
#define AR0830_X_EVEN_INC					CCI_REG16(0x0380)
#define AR0830_X_ODD_INC					CCI_REG16(0x0382)
#define AR0830_Y_EVEN_INC					CCI_REG16(0x0384)
#define AR0830_Y_ODD_INC					CCI_REG16(0x0386)
#define AR0830_MONOCHROME_EN					CCI_REG16(0x0390)
#define AR0830_TEST_PATTERN_MODE				CCI_REG16(0x0600)
#define		AR0830_TEST_PATTERN_SELECT_T2(n)			(((n) & 0x7) << 4)
#define		AR0830_TEST_PATTERN_SELECT_T2_MASK			GENMASK(7, 4)
#define		AR0830_TEST_PATTERN_SELECT(n)				(((n) & 0x7) << 0)
#define		AR0830_TEST_PATTERN_SELECT_MASK				GENMASK(3, 0)
#define		AR0830_TEST_PATTERN_DISABLE				0
#define		AR0830_TEST_PATTERN_SOLID_COLOR				1
#define		AR0830_TEST_PATTERN_COLOR_BARS				2
#define		AR0830_TEST_PATTERN_GREY_COLOR_BARS			3
#define		AR0830_TEST_PATTERN_PN9					4
#define		AR0830_TEST_PATTERN_COLOR_TILE				5
#define AR0830_TEST_DATA_RED					CCI_REG16(0x0602)
#define AR0830_TEST_DATA_GREENR					CCI_REG16(0x0604)
#define AR0830_TEST_DATA_BLUE					CCI_REG16(0x0606)
#define AR0830_TEST_DATA_GREENB					CCI_REG16(0x0608)
#define AR0830_BINNING_MODE					CCI_REG8(0x0900)
#define AR0830_BINNING_TYPE					CCI_REG8(0x0901)
#define		AR0830_COLUMN_BINNING_FACTOR(n)				(((n) & 0xf) << 4)
#define		AR0830_COLUMN_BINNING_FACTOR_MASK			GENMASK(7, 4)
#define		AR0830_ROW_BINNING_FACTOR(n)				(((n) & 0xf) << 0)
#define		AR0830_ROW_BINNING_FACTOR_MASK				GENMASK(3, 0)
#define AR0830_RESET_REGISTER					CCI_REG16(0x301a)
#define		AR0830_PLL_ALWAYS_ON					BIT(11)
#define		AR0830_RESTART_BAD_FRAMES				BIT(10)
#define		AR0830_GPI_EN						BIT(8)
#define		AR0830_LOCK_REG						BIT(3)
#define		AR0830_RESTART_CTRL					BIT(1)
#define	AR0830_FRAME_STATUS					CCI_REG16(0x3056)
#define		AR0830_FRAME_STATUS_BAD_FRAME				BIT(2)
#define		AR0830_FRAME_STATUS_STANDBY				BIT(1)
#define AR0830_GAIN_CODE					CCI_REG16(0x3062)
#define		AR0830_SHORT_GLOBAL_GAIN_CODE(n)			(((n) & 0xff) << 8)
#define		AR0830_GLOBAL_GAIN_CODE(n)				(((n) & 0xff) << 0)
#define AR0830_DIGITAL_GAIN_GREENR				CCI_REG16(0x36c2)
#define AR0830_DIGITAL_GAIN_BLUE				CCI_REG16(0x36c4)
#define AR0830_DIGITAL_GAIN_RED					CCI_REG16(0x36c6)
#define AR0830_DIGITAL_GAIN_GREENB				CCI_REG16(0x36c8)
#define AR0830_PIX_DEF_CORR					CCI_REG16(0x3980)
#define		AR0830_1D_DDC_EN						BIT(1)
#define AR0830_AE_CTRL						CCI_REG16(0x3d00)
#define		AR0830_AE_MAX_ANA_GAIN(n)				(((n) & 0x7) << 8)
#define		AR0830_AE_MAX_ANA_GAIN_MASK				GENMASK(10, 8)
#define		AR0830_AE_MIN_ANA_GAIN(n)				(((n) & 0x7) << 5)
#define		AR0830_AE_MIN_ANA_GAIN_MASK				GENMASK(7, 5)
#define		AR0830_AE_AUTO_DG_EN					BIT(4)
#define		AR0830_AE_AUTO_AG_EN					BIT(1)
#define		AR0830_AE_EN						BIT(0)
#define AR0830_AE_LUMA_TARGET					CCI_REG16(0x3d02)
#define AR0830_AE_MAX_EXPOSURE					CCI_REG16(0x3d1a)
#define AR0830_AE_MIN_EXPOSURE					CCI_REG16(0x3d1c)
#define AR0830_AE_CURRENT_GAINS					CCI_REG16(0x3d36)
#define		AR0830_AE_ANALOG_GAIN(n)				(((n) >> 11) & 0x7)
#define		AR0830_AE_DIGITAL_GAIN(n)				(((n) >> 0) & 0x7ff)
#define AR0830_AE_COARSE_INTEGRATION_TIME			CCI_REG16(0x3d38)
#define AR0830_MIPI_TIMING_0					CCI_REG16(0x3f02)
#define		AR0830_T_HS_PREPARE(n)					((n) << 12)
#define		AR0830_T_HS_ZERO(n)					((n) << 6)
#define		AR0830_T_HS_TRAIL(n)					((n) << 1)
#define AR0830_MIPI_TIMING_1					CCI_REG16(0x3f04)
#define		AR0830_T_CLK_PREPARE(n)					((n) << 12)
#define		AR0830_T_CLK_ZERO(n)					((n) << 5)
#define		AR0830_T_CLK_TRAIL(n)					((n) << 0)
#define AR0830_MIPI_TIMING_2					CCI_REG16(0x3f06)
#define		AR0830_T_CLK_PRE(n)					((n) << 6)
#define		AR0830_T_CLK_POST(n)					((n) << 0)
#define AR0830_MIPI_TIMING_3					CCI_REG16(0x3f08)
#define		AR0830_T_LPX(n)						((n) << 7)
#define		AR0830_T_WAKE_UP(n)					((n) << 0)
#define AR0830_MIPI_TIMING_4					CCI_REG16(0x3f0a)
#define		AR0830_CONT_TX_CLK					BIT(15)
#define		AR0830_HEAVY_LP_LOAD					BIT(14)
#define		AR0830_T_HS_EXIT(n)					((n) << 7)
#define		AR0830_T_INIT(n)					((n) << 0)
#define AR0830_MIPI_TIMING_5					CCI_REG16(0x3f0c)
#define		AR0830_T_BGAP(n)					((n) << 0)

#define AR0830_CHIP_VERSION			0x0553

#define AR0830_MIN_LINE_LENGTH_PCK		4496U
#define AR0830_MAX_LINE_LENGTH_PCK		65520U
#define AR0830_MIN_HBLANK			824U

#define AR0830_MAX_FRAME_LENGTH_LINES		65535U
#define AR0830_MIN_VBLANK			14U

#define AR0830_X_ADDR_MIN			8U
#define AR0830_X_ADDR_MAX			3847U
#define AR0830_Y_ADDR_MIN			8U
#define AR0830_Y_ADDR_MAX			2167U

#define AR0830_DEF_WIDTH			3840U
#define AR0830_MIN_WIDTH			32U
#define AR0830_MAX_WIDTH			3840U
#define AR0830_STEP_WIDTH			4U
#define AR0830_DEF_HEIGHT			2160U
#define AR0830_MIN_HEIGHT			32U
#define AR0830_MAX_HEIGHT			2160U
#define AR0830_STEP_HEIGHT			2U

#define V4L2_CID_USER_AR0830_BASE				(V4L2_CID_USER_BASE + 0x2500)
#define V4L2_CID_AR0830_ROW_BINNING				(V4L2_CID_USER_AR0830_BASE + 1)
#define V4L2_CID_AR0830_COLUMN_BINNING				(V4L2_CID_USER_AR0830_BASE + 2)
#define V4L2_CID_AR0830_DYNAMIC_PIXEL_CORRECTION		(V4L2_CID_USER_AR0830_BASE + 3)
#define V4L2_CID_AR0830_AUTO_EXP_MAX				(V4L2_CID_USER_AR0830_BASE + 4)
#define V4L2_CID_AR0830_AUTO_EXP_MIN				(V4L2_CID_USER_AR0830_BASE + 5)
#define V4L2_CID_AR0830_AUTO_EXP_TARGET				(V4L2_CID_USER_AR0830_BASE + 6)
#define V4L2_CID_AR0830_AUTOGAIN_DIGITAL			(V4L2_CID_USER_AR0830_BASE + 7)
#define V4L2_CID_AR0830_AE_ANALOG_GAIN				(V4L2_CID_USER_AR0830_BASE + 8)
#define V4L2_CID_AR0830_AE_DIGITAL_GAIN				(V4L2_CID_USER_AR0830_BASE + 9)

struct ar0830_model {
	bool mono;
};

struct ar0830_format_info {
	u32 color;
	u32 mono;
	u16 bpp;
	u16 dt;
};

static const struct ar0830_format_info ar0830_formats[] = {
	{
		.color = MEDIA_BUS_FMT_SGRBG10_1X10,
		.mono = MEDIA_BUS_FMT_Y10_1X10,
		.bpp = 10,
		.dt = MIPI_CSI2_DT_RAW10,
	}, {
		.color = MEDIA_BUS_FMT_SGRBG8_1X8,
		.mono = MEDIA_BUS_FMT_Y8_1X8,
		.bpp = 8,
		.dt = MIPI_CSI2_DT_RAW8,
	},
};

static const char * const ar0830_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"Color Bars",
	"Gray Color Bars",
	"PN9",
	"Color Tile",
};

static const int ar0830_test_pattern_val[] = {
	AR0830_TEST_PATTERN_DISABLE,
	AR0830_TEST_PATTERN_SOLID_COLOR,
	AR0830_TEST_PATTERN_COLOR_BARS,
	AR0830_TEST_PATTERN_GREY_COLOR_BARS,
	AR0830_TEST_PATTERN_PN9,
	AR0830_TEST_PATTERN_COLOR_TILE,
};

struct ar0830 {
	struct device *dev;

	struct regmap *regmap;
	struct clk *extclk;
	struct gpio_desc *reset;

	struct v4l2_fwnode_endpoint buscfg;
	u64 valid_link_freqs[ARRAY_SIZE(ar0830_formats)];
	u32 valid_formats;

	struct ccs_pll pll;

	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;
	struct media_pad pad;

	struct {
		/* exposure cluster */
		struct v4l2_ctrl *auto_exposure;
		struct v4l2_ctrl *exposure;
	};
	struct {
		/* gain cluster */
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *ae_exp_max;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *red_balance;
	struct v4l2_ctrl *blue_balance;
	struct v4l2_ctrl *row_binning;
	struct v4l2_ctrl *col_binning;

	struct ar0830_model model;

	struct mutex lock;
};

static inline struct ar0830 *to_ar0830(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0830, sd);
}

static u32 ar0830_format_code(struct ar0830 *sensor, const struct ar0830_format_info *info)
{
	return sensor->model.mono ? info->mono : info->color;
}

struct sensor_reg {
	u32 reg;
	char *name;
};

static const struct ar0830_format_info *
ar0830_get_format_info(struct ar0830 *sensor, u32 code, bool use_def)
{
	const struct ar0830_format_info *def = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ar0830_formats); i++) {
		const struct ar0830_format_info *info = &ar0830_formats[i];
		u32 info_code = ar0830_format_code(sensor, info);

		if (!(sensor->valid_formats & BIT(i)))
			continue;

		if (info_code == code)
			return info;

		if (!def && use_def)
			def = info;
	}

	return def;
}

static const struct cci_reg_sequence ar0830_recommended_regs[] = {
	{ CCI_REG16(0x44C6), 0x54E2 },
	{ CCI_REG16(0x44C4), 0x0FD3 },
	{ CCI_REG16(0x44BA), 0x3349 },
	{ CCI_REG16(0x3284), 0x8100 },
	{ CCI_REG16(0x44F4), 0x2A2A },
	{ CCI_REG16(0x44F6), 0x8B2B },
	{ CCI_REG16(0x44F8), 0x6462 },
	{ CCI_REG16(0x4062), 0xF016 },
	{ CCI_REG16(0x40CE), 0x1D9F },
	{ CCI_REG16(0x549E), 0x14E3 },
};

static const struct cci_reg_sequence ar0830_otp_regs[] = {
	{ CCI_REG16(0x4000), 0x011B },
	{ CCI_REG16(0x4002), 0x212A },
	{ CCI_REG16(0x4004), 0x45FF },
	{ CCI_REG16(0x4006), 0xFFFF },
	{ CCI_REG16(0x4008), 0x0810 },
	{ CCI_REG16(0x400A), 0x0225 },
	{ CCI_REG16(0x400C), 0x108F },
	{ CCI_REG16(0x400E), 0x3003 },
	{ CCI_REG16(0x4010), 0x92F0 },
	{ CCI_REG16(0x4012), 0x0097 },
	{ CCI_REG16(0x4014), 0x30D8 },
	{ CCI_REG16(0x4016), 0xF007 },
	{ CCI_REG16(0x4018), 0x919A },
	{ CCI_REG16(0x401A), 0xF001 },
	{ CCI_REG16(0x401C), 0x99F0 },
	{ CCI_REG16(0x401E), 0x0285 },
	{ CCI_REG16(0x4020), 0xF000 },
	{ CCI_REG16(0x4022), 0x30C0 },
	{ CCI_REG16(0x4024), 0xF004 },
	{ CCI_REG16(0x4026), 0x8B89 },
	{ CCI_REG16(0x4028), 0xF007 },
	{ CCI_REG16(0x402A), 0x9CF0 },
	{ CCI_REG16(0x402C), 0x0082 },
	{ CCI_REG16(0x402E), 0x3018 },
	{ CCI_REG16(0x4030), 0x8BB1 },
	{ CCI_REG16(0x4032), 0xF001 },
	{ CCI_REG16(0x4034), 0xB6F0 },
	{ CCI_REG16(0x4036), 0x0021 },
	{ CCI_REG16(0x4038), 0x58F0 },
	{ CCI_REG16(0x403A), 0x0F99 },
	{ CCI_REG16(0x403C), 0xF000 },
	{ CCI_REG16(0x403E), 0x98F0 },
	{ CCI_REG16(0x4040), 0x03A2 },
	{ CCI_REG16(0x4042), 0xF003 },
	{ CCI_REG16(0x4044), 0xA296 },
	{ CCI_REG16(0x4046), 0xB4F0 },
	{ CCI_REG16(0x4048), 0x029D },
	{ CCI_REG16(0x404A), 0xF004 },
	{ CCI_REG16(0x404C), 0xA1F0 },
	{ CCI_REG16(0x404E), 0x20A1 },
	{ CCI_REG16(0x4050), 0xF004 },
	{ CCI_REG16(0x4052), 0x9DF0 },
	{ CCI_REG16(0x4054), 0x008B },
	{ CCI_REG16(0x4056), 0xF000 },
	{ CCI_REG16(0x4058), 0x1009 },
	{ CCI_REG16(0x405A), 0x83F0 },
	{ CCI_REG16(0x405C), 0x0088 },
	{ CCI_REG16(0x405E), 0xF002 },
	{ CCI_REG16(0x4060), 0x3600 },
	{ CCI_REG16(0x4062), 0xF003 },
	{ CCI_REG16(0x4064), 0x9088 },
	{ CCI_REG16(0x4066), 0xF004 },
	{ CCI_REG16(0x4068), 0x3600 },
	{ CCI_REG16(0x406A), 0x83F0 },
	{ CCI_REG16(0x406C), 0x138B },
	{ CCI_REG16(0x406E), 0xF014 },
	{ CCI_REG16(0x4070), 0xA3F0 },
	{ CCI_REG16(0x4072), 0x03A3 },
	{ CCI_REG16(0x4074), 0xF004 },
	{ CCI_REG16(0x4076), 0x9DF0 },
	{ CCI_REG16(0x4078), 0x04A1 },
	{ CCI_REG16(0x407A), 0xF020 },
	{ CCI_REG16(0x407C), 0xA1F0 },
	{ CCI_REG16(0x407E), 0x5821 },
	{ CCI_REG16(0x4080), 0xEDB4 },
	{ CCI_REG16(0x4082), 0x40C2 },
	{ CCI_REG16(0x4084), 0xF000 },
	{ CCI_REG16(0x4086), 0x1FF6 },
	{ CCI_REG16(0x4088), 0xF001 },
	{ CCI_REG16(0x408A), 0x3003 },
	{ CCI_REG16(0x408C), 0x84F0 },
	{ CCI_REG16(0x408E), 0x008B },
	{ CCI_REG16(0x4090), 0xF007 },
	{ CCI_REG16(0x4092), 0x86F0 },
	{ CCI_REG16(0x4094), 0x0086 },
	{ CCI_REG16(0x4096), 0xF005 },
	{ CCI_REG16(0x4098), 0x8080 },
	{ CCI_REG16(0x409A), 0x8202 },
	{ CCI_REG16(0x409C), 0x0887 },
	{ CCI_REG16(0x409E), 0x30C0 },
	{ CCI_REG16(0x40A0), 0xF000 },
	{ CCI_REG16(0x40A2), 0x3600 },
	{ CCI_REG16(0x40A4), 0xF010 },
	{ CCI_REG16(0x40A6), 0x3600 },
	{ CCI_REG16(0x40A8), 0xF000 },
	{ CCI_REG16(0x40AA), 0x30C0 },
	{ CCI_REG16(0x40AC), 0x8702 },
	{ CCI_REG16(0x40AE), 0x0882 },
	{ CCI_REG16(0x40B0), 0x8080 },
	{ CCI_REG16(0x40B2), 0xF000 },
	{ CCI_REG16(0x40B4), 0x8202 },
	{ CCI_REG16(0x40B6), 0x0887 },
	{ CCI_REG16(0x40B8), 0x30C0 },
	{ CCI_REG16(0x40BA), 0xF000 },
	{ CCI_REG16(0x40BC), 0x3600 },
	{ CCI_REG16(0x40BE), 0xF00F },
	{ CCI_REG16(0x40C0), 0x3600 },
	{ CCI_REG16(0x40C2), 0xF000 },
	{ CCI_REG16(0x40C4), 0x30C0 },
	{ CCI_REG16(0x40C6), 0x8702 },
	{ CCI_REG16(0x40C8), 0x0882 },
	{ CCI_REG16(0x40CA), 0xF000 },
	{ CCI_REG16(0x40CC), 0x80F0 },
	{ CCI_REG16(0x40CE), 0x309F },
	{ CCI_REG16(0x40D0), 0xF027 },
	{ CCI_REG16(0x40D2), 0x1300 },
	{ CCI_REG16(0x40D4), 0xF01C },
	{ CCI_REG16(0x40D6), 0xB7E0 },
	{ CCI_REG16(0x40D8), 0xF000 },
	{ CCI_REG16(0x40DA), 0x0401 },
	{ CCI_REG16(0x40DC), 0xF003 },
	{ CCI_REG16(0x40DE), 0x82F0 },
	{ CCI_REG16(0x40E0), 0x0302 },
	{ CCI_REG16(0x40E2), 0x0885 },
	{ CCI_REG16(0x40E4), 0xF015 },
	{ CCI_REG16(0x40E6), 0x8587 },
	{ CCI_REG16(0x40E8), 0xF033 },
	{ CCI_REG16(0x40EA), 0x87F1 },
	{ CCI_REG16(0x40EC), 0x8388 },
	{ CCI_REG16(0x40EE), 0xF005 },
	{ CCI_REG16(0x40F0), 0x88F0 },
	{ CCI_REG16(0x40F2), 0x0189 },
	{ CCI_REG16(0x40F4), 0xF000 },
	{ CCI_REG16(0x40F6), 0x0048 },
	{ CCI_REG16(0x40F8), 0xF002 },
	{ CCI_REG16(0x40FA), 0x86F0 },
	{ CCI_REG16(0x40FC), 0x0482 },
	{ CCI_REG16(0x40FE), 0xF017 },
	{ CCI_REG16(0x4100), 0x8AF0 },
	{ CCI_REG16(0x4102), 0x1780 },
	{ CCI_REG16(0x4104), 0xF004 },
	{ CCI_REG16(0x4106), 0xE0E0 },
	{ CCI_REG16(0x4108), 0xF000 },
	{ CCI_REG16(0x410A), 0x0401 },
	{ CCI_REG16(0x410C), 0xF015 },
	{ CCI_REG16(0x410E), 0x020C },
	{ CCI_REG16(0x4110), 0xF016 },
	{ CCI_REG16(0x4112), 0x87F0 },
	{ CCI_REG16(0x4114), 0x0287 },
	{ CCI_REG16(0x4116), 0xF061 },
	{ CCI_REG16(0x4118), 0xE839 },
	{ CCI_REG16(0x411A), 0x20F0 },
	{ CCI_REG16(0x411C), 0x0534 },
	{ CCI_REG16(0x411E), 0x90F0 },
	{ CCI_REG16(0x4120), 0x0032 },
	{ CCI_REG16(0x4122), 0x48F0 },
	{ CCI_REG16(0x4124), 0x0039 },
	{ CCI_REG16(0x4126), 0x20F0 },
	{ CCI_REG16(0x4128), 0x0E39 },
	{ CCI_REG16(0x412A), 0x20F0 },
	{ CCI_REG16(0x412C), 0x0032 },
	{ CCI_REG16(0x412E), 0x48F0 },
	{ CCI_REG16(0x4130), 0x0034 },
	{ CCI_REG16(0x4132), 0x90F0 },
	{ CCI_REG16(0x4134), 0x05C1 },
	{ CCI_REG16(0x4136), 0x1BF0 },
	{ CCI_REG16(0x4138), 0x0439 },
	{ CCI_REG16(0x413A), 0x20F0 },
	{ CCI_REG16(0x413C), 0x03B0 },
	{ CCI_REG16(0x413E), 0xF000 },
	{ CCI_REG16(0x4140), 0x0208 },
	{ CCI_REG16(0x4142), 0xF0AA },
	{ CCI_REG16(0x4144), 0xB0F0 },
	{ CCI_REG16(0x4146), 0x1CE9 },
	{ CCI_REG16(0x4148), 0x8A00 },
	{ CCI_REG16(0x414A), 0x05F0 },
	{ CCI_REG16(0x414C), 0x93E0 },
	{ CCI_REG16(0x414E), 0xE0E0 },
	{ CCI_REG16(0x4150), 0xF063 },
	{ CCI_REG16(0x4152), 0x0830 },
	{ CCI_REG16(0x4154), 0x0205 },
	{ CCI_REG16(0x4156), 0x108F },
	{ CCI_REG16(0x4158), 0x3003 },
	{ CCI_REG16(0x415A), 0x92F0 },
	{ CCI_REG16(0x415C), 0x0097 },
	{ CCI_REG16(0x415E), 0x30D8 },
	{ CCI_REG16(0x4160), 0xF007 },
	{ CCI_REG16(0x4162), 0x919A },
	{ CCI_REG16(0x4164), 0xF001 },
	{ CCI_REG16(0x4166), 0x99F0 },
	{ CCI_REG16(0x4168), 0x0285 },
	{ CCI_REG16(0x416A), 0xF000 },
	{ CCI_REG16(0x416C), 0x30C0 },
	{ CCI_REG16(0x416E), 0xF004 },
	{ CCI_REG16(0x4170), 0x8B89 },
	{ CCI_REG16(0x4172), 0xF007 },
	{ CCI_REG16(0x4174), 0x9CF0 },
	{ CCI_REG16(0x4176), 0x0082 },
	{ CCI_REG16(0x4178), 0x3018 },
	{ CCI_REG16(0x417A), 0x8BB1 },
	{ CCI_REG16(0x417C), 0xF001 },
	{ CCI_REG16(0x417E), 0xB6F0 },
	{ CCI_REG16(0x4180), 0x009C },
	{ CCI_REG16(0x4182), 0xF00F },
	{ CCI_REG16(0x4184), 0x99F0 },
	{ CCI_REG16(0x4186), 0x0098 },
	{ CCI_REG16(0x4188), 0xF000 },
	{ CCI_REG16(0x418A), 0x2148 },
	{ CCI_REG16(0x418C), 0xF001 },
	{ CCI_REG16(0x418E), 0xA2F0 },
	{ CCI_REG16(0x4190), 0x03A2 },
	{ CCI_REG16(0x4192), 0x96B4 },
	{ CCI_REG16(0x4194), 0xF002 },
	{ CCI_REG16(0x4196), 0x9DF0 },
	{ CCI_REG16(0x4198), 0x04A1 },
	{ CCI_REG16(0x419A), 0xF020 },
	{ CCI_REG16(0x419C), 0xA1F0 },
	{ CCI_REG16(0x419E), 0x049D },
	{ CCI_REG16(0x41A0), 0xF000 },
	{ CCI_REG16(0x41A2), 0x8BF0 },
	{ CCI_REG16(0x41A4), 0x0010 },
	{ CCI_REG16(0x41A6), 0x0983 },
	{ CCI_REG16(0x41A8), 0xF000 },
	{ CCI_REG16(0x41AA), 0x88F0 },
	{ CCI_REG16(0x41AC), 0x0236 },
	{ CCI_REG16(0x41AE), 0x00F0 },
	{ CCI_REG16(0x41B0), 0x0390 },
	{ CCI_REG16(0x41B2), 0x88F0 },
	{ CCI_REG16(0x41B4), 0x0436 },
	{ CCI_REG16(0x41B6), 0x0083 },
	{ CCI_REG16(0x41B8), 0xF013 },
	{ CCI_REG16(0x41BA), 0x8BF0 },
	{ CCI_REG16(0x41BC), 0x14A3 },
	{ CCI_REG16(0x41BE), 0xF003 },
	{ CCI_REG16(0x41C0), 0xA3F0 },
	{ CCI_REG16(0x41C2), 0x049D },
	{ CCI_REG16(0x41C4), 0xF004 },
	{ CCI_REG16(0x41C6), 0xA1F0 },
	{ CCI_REG16(0x41C8), 0x20A1 },
	{ CCI_REG16(0x41CA), 0xF057 },
	{ CCI_REG16(0x41CC), 0x21ED },
	{ CCI_REG16(0x41CE), 0xB440 },
	{ CCI_REG16(0x41D0), 0xC284 },
	{ CCI_REG16(0x41D2), 0x1FF6 },
	{ CCI_REG16(0x41D4), 0x0840 },
	{ CCI_REG16(0x41D6), 0xF000 },
	{ CCI_REG16(0x41D8), 0x3003 },
	{ CCI_REG16(0x41DA), 0x86F0 },
	{ CCI_REG16(0x41DC), 0x0080 },
	{ CCI_REG16(0x41DE), 0x8082 },
	{ CCI_REG16(0x41E0), 0x0208 },
	{ CCI_REG16(0x41E2), 0x8736 },
	{ CCI_REG16(0x41E4), 0xC0F0 },
	{ CCI_REG16(0x41E6), 0x0236 },
	{ CCI_REG16(0x41E8), 0xC087 },
	{ CCI_REG16(0x41EA), 0x0208 },
	{ CCI_REG16(0x41EC), 0x8280 },
	{ CCI_REG16(0x41EE), 0x8082 },
	{ CCI_REG16(0x41F0), 0x0208 },
	{ CCI_REG16(0x41F2), 0x8736 },
	{ CCI_REG16(0x41F4), 0xC0F0 },
	{ CCI_REG16(0x41F6), 0x0236 },
	{ CCI_REG16(0x41F8), 0xC087 },
	{ CCI_REG16(0x41FA), 0x0208 },
	{ CCI_REG16(0x41FC), 0x8280 },
	{ CCI_REG16(0x41FE), 0x8082 },
	{ CCI_REG16(0x4200), 0x0208 },
	{ CCI_REG16(0x4202), 0x8736 },
	{ CCI_REG16(0x4204), 0xC0F0 },
	{ CCI_REG16(0x4206), 0x0236 },
	{ CCI_REG16(0x4208), 0xC087 },
	{ CCI_REG16(0x420A), 0x0208 },
	{ CCI_REG16(0x420C), 0x8280 },
	{ CCI_REG16(0x420E), 0x8082 },
	{ CCI_REG16(0x4210), 0x0208 },
	{ CCI_REG16(0x4212), 0x8736 },
	{ CCI_REG16(0x4214), 0xC0F0 },
	{ CCI_REG16(0x4216), 0x0236 },
	{ CCI_REG16(0x4218), 0xC087 },
	{ CCI_REG16(0x421A), 0x9FF0 },
	{ CCI_REG16(0x421C), 0x0002 },
	{ CCI_REG16(0x421E), 0x0DF0 },
	{ CCI_REG16(0x4220), 0x1313 },
	{ CCI_REG16(0x4222), 0x00F0 },
	{ CCI_REG16(0x4224), 0x1CB7 },
	{ CCI_REG16(0x4226), 0xE0E0 },
	{ CCI_REG16(0x4228), 0xF013 },
	{ CCI_REG16(0x422A), 0x80F0 },
	{ CCI_REG16(0x422C), 0x3102 },
	{ CCI_REG16(0x422E), 0x3410 },
	{ CCI_REG16(0x4230), 0xCF30 },
	{ CCI_REG16(0x4232), 0x03F0 },
	{ CCI_REG16(0x4234), 0x00B2 },
	{ CCI_REG16(0x4236), 0x30C0 },
	{ CCI_REG16(0x4238), 0x3018 },
	{ CCI_REG16(0x423A), 0x97B5 },
	{ CCI_REG16(0x423C), 0xF004 },
	{ CCI_REG16(0x423E), 0x91F0 },
	{ CCI_REG16(0x4240), 0x009A },
	{ CCI_REG16(0x4242), 0xF001 },
	{ CCI_REG16(0x4244), 0x99F0 },
	{ CCI_REG16(0x4246), 0x0330 },
	{ CCI_REG16(0x4248), 0x18F0 },
	{ CCI_REG16(0x424A), 0x0085 },
	{ CCI_REG16(0x424C), 0xF000 },
	{ CCI_REG16(0x424E), 0x30C0 },
	{ CCI_REG16(0x4250), 0x9E40 },
	{ CCI_REG16(0x4252), 0x4220 },
	{ CCI_REG16(0x4254), 0x1889 },
	{ CCI_REG16(0x4256), 0x4104 },
	{ CCI_REG16(0x4258), 0x82A0 },
	{ CCI_REG16(0x425A), 0xF002 },
	{ CCI_REG16(0x425C), 0x8B9C },
	{ CCI_REG16(0x425E), 0xF010 },
	{ CCI_REG16(0x4260), 0x99F0 },
	{ CCI_REG16(0x4262), 0x0098 },
	{ CCI_REG16(0x4264), 0xF003 },
	{ CCI_REG16(0x4266), 0xA296 },
	{ CCI_REG16(0x4268), 0xF001 },
	{ CCI_REG16(0x426A), 0xB4A2 },
	{ CCI_REG16(0x426C), 0xF004 },
	{ CCI_REG16(0x426E), 0x9DF0 },
	{ CCI_REG16(0x4270), 0x04A1 },
	{ CCI_REG16(0x4272), 0xF028 },
	{ CCI_REG16(0x4274), 0x8BA1 },
	{ CCI_REG16(0x4276), 0x1009 },
	{ CCI_REG16(0x4278), 0x83F0 },
	{ CCI_REG16(0x427A), 0x0136 },
	{ CCI_REG16(0x427C), 0x00F0 },
	{ CCI_REG16(0x427E), 0x009D },
	{ CCI_REG16(0x4280), 0x88F0 },
	{ CCI_REG16(0x4282), 0x0888 },
	{ CCI_REG16(0x4284), 0xF000 },
	{ CCI_REG16(0x4286), 0x3600 },
	{ CCI_REG16(0x4288), 0x8390 },
	{ CCI_REG16(0x428A), 0xF096 },
	{ CCI_REG16(0x428C), 0x8BF0 },
	{ CCI_REG16(0x428E), 0x0CA3 },
	{ CCI_REG16(0x4290), 0xF003 },
	{ CCI_REG16(0x4292), 0xA3F0 },
	{ CCI_REG16(0x4294), 0x049D },
	{ CCI_REG16(0x4296), 0xF004 },
	{ CCI_REG16(0x4298), 0xA1F0 },
	{ CCI_REG16(0x429A), 0x20A1 },
	{ CCI_REG16(0x429C), 0xF057 },
	{ CCI_REG16(0x429E), 0x9DB4 },
	{ CCI_REG16(0x42A0), 0xF015 },
	{ CCI_REG16(0x42A2), 0x8B91 },
	{ CCI_REG16(0x42A4), 0x848E },
	{ CCI_REG16(0x42A6), 0xF01E },
	{ CCI_REG16(0x42A8), 0xB8F1 },
	{ CCI_REG16(0x42AA), 0x9FB2 },
	{ CCI_REG16(0x42AC), 0xF060 },
	{ CCI_REG16(0x42AE), 0xA6B9 },
	{ CCI_REG16(0x42B0), 0x848E },
	{ CCI_REG16(0x42B2), 0xF004 },
	{ CCI_REG16(0x42B4), 0x0202 },
	{ CCI_REG16(0x42B6), 0xF015 },
	{ CCI_REG16(0x42B8), 0xB2F0 },
	{ CCI_REG16(0x42BA), 0x0691 },
	{ CCI_REG16(0x42BC), 0x83B8 },
	{ CCI_REG16(0x42BE), 0xF000 },
	{ CCI_REG16(0x42C0), 0x3600 },
	{ CCI_REG16(0x42C2), 0xF00D },
	{ CCI_REG16(0x42C4), 0x3600 },
	{ CCI_REG16(0x42C6), 0x83F0 },
	{ CCI_REG16(0x42C8), 0x029C },
	{ CCI_REG16(0x42CA), 0xF008 },
	{ CCI_REG16(0x42CC), 0x9CF0 },
	{ CCI_REG16(0x42CE), 0x128B },
	{ CCI_REG16(0x42D0), 0xF006 },
	{ CCI_REG16(0x42D2), 0x3018 },
	{ CCI_REG16(0x42D4), 0xA3F0 },
	{ CCI_REG16(0x42D6), 0x04A3 },
	{ CCI_REG16(0x42D8), 0xF003 },
	{ CCI_REG16(0x42DA), 0x9DF0 },
	{ CCI_REG16(0x42DC), 0x7E30 },
	{ CCI_REG16(0x42DE), 0x189D },
	{ CCI_REG16(0x42E0), 0xF002 },
	{ CCI_REG16(0x42E2), 0x8BF0 },
	{ CCI_REG16(0x42E4), 0x0082 },
	{ CCI_REG16(0x42E6), 0xF004 },
	{ CCI_REG16(0x42E8), 0x30C0 },
	{ CCI_REG16(0x42EA), 0xF014 },
	{ CCI_REG16(0x42EC), 0x30C0 },
	{ CCI_REG16(0x42EE), 0xF004 },
	{ CCI_REG16(0x42F0), 0x82F0 },
	{ CCI_REG16(0x42F2), 0x0D90 },
	{ CCI_REG16(0x42F4), 0xF002 },
	{ CCI_REG16(0x42F6), 0x8BF0 },
	{ CCI_REG16(0x42F8), 0x018C },
	{ CCI_REG16(0x42FA), 0x8FF0 },
	{ CCI_REG16(0x42FC), 0x3E30 },
	{ CCI_REG16(0x42FE), 0x18A2 },
	{ CCI_REG16(0x4300), 0xF003 },
	{ CCI_REG16(0x4302), 0xA2F0 },
	{ CCI_REG16(0x4304), 0x049D },
	{ CCI_REG16(0x4306), 0xF036 },
	{ CCI_REG16(0x4308), 0x9DF0 },
	{ CCI_REG16(0x430A), 0x0A30 },
	{ CCI_REG16(0x430C), 0x1889 },
	{ CCI_REG16(0x430E), 0xB5F0 },
	{ CCI_REG16(0x4310), 0x018B },
	{ CCI_REG16(0x4312), 0xF002 },
	{ CCI_REG16(0x4314), 0x97F0 },
	{ CCI_REG16(0x4316), 0x0017 },
	{ CCI_REG16(0x4318), 0xA621 },
	{ CCI_REG16(0x431A), 0xCD40 },
	{ CCI_REG16(0x431C), 0xC230 },
	{ CCI_REG16(0x431E), 0x0710 },
	{ CCI_REG16(0x4320), 0x4984 },
	{ CCI_REG16(0x4322), 0xF00C },
	{ CCI_REG16(0x4324), 0x80F0 },
	{ CCI_REG16(0x4326), 0x1486 },
	{ CCI_REG16(0x4328), 0xF000 },
	{ CCI_REG16(0x432A), 0x86F0 },
	{ CCI_REG16(0x432C), 0x0C80 },
	{ CCI_REG16(0x432E), 0x8283 },
	{ CCI_REG16(0x4330), 0x8730 },
	{ CCI_REG16(0x4332), 0xC036 },
	{ CCI_REG16(0x4334), 0x00F0 },
	{ CCI_REG16(0x4336), 0x1030 },
	{ CCI_REG16(0x4338), 0xC036 },
	{ CCI_REG16(0x433A), 0x0087 },
	{ CCI_REG16(0x433C), 0x8382 },
	{ CCI_REG16(0x433E), 0x8080 },
	{ CCI_REG16(0x4340), 0xF000 },
	{ CCI_REG16(0x4342), 0x8283 },
	{ CCI_REG16(0x4344), 0x8730 },
	{ CCI_REG16(0x4346), 0xC036 },
	{ CCI_REG16(0x4348), 0x00F0 },
	{ CCI_REG16(0x434A), 0x0F30 },
	{ CCI_REG16(0x434C), 0xC0F0 },
	{ CCI_REG16(0x434E), 0x0036 },
	{ CCI_REG16(0x4350), 0x0087 },
	{ CCI_REG16(0x4352), 0x8382 },
	{ CCI_REG16(0x4354), 0xF000 },
	{ CCI_REG16(0x4356), 0x80F1 },
	{ CCI_REG16(0x4358), 0xC5B8 },
	{ CCI_REG16(0x435A), 0xF00D },
	{ CCI_REG16(0x435C), 0xB7F0 },
	{ CCI_REG16(0x435E), 0x019F },
	{ CCI_REG16(0x4360), 0xF02B },
	{ CCI_REG16(0x4362), 0x1300 },
	{ CCI_REG16(0x4364), 0xB981 },
	{ CCI_REG16(0x4366), 0xE0E0 },
	{ CCI_REG16(0x4368), 0xE0E0 },
	{ CCI_REG16(0x436A), 0xE0E0 },
	{ CCI_REG16(0x436C), 0xE0E0 },
	{ CCI_REG16(0x436E), 0xE0E0 },
	{ CCI_REG16(0x4370), 0xE0E0 },
	{ CCI_REG16(0x4372), 0xE0E0 },
	{ CCI_REG16(0x4374), 0xE0E0 },
	{ CCI_REG16(0x4376), 0xE0E0 },
	{ CCI_REG16(0x4378), 0xE0E0 },
	{ CCI_REG16(0x437A), 0xE0E0 },
	{ CCI_REG16(0x437C), 0xE0E0 },
	{ CCI_REG16(0x437E), 0xE0E0 },
	{ CCI_REG16(0x4380), 0xE0E0 },
	{ CCI_REG16(0x4382), 0xE0E0 },
	{ CCI_REG16(0x4384), 0xE0E0 },
	{ CCI_REG16(0x4386), 0xE0E0 },
	{ CCI_REG16(0x4388), 0xE0E0 },
	{ CCI_REG16(0x438A), 0xE0E0 },
	{ CCI_REG16(0x438C), 0xE0E0 },
	{ CCI_REG16(0x438E), 0xE0E0 },
	{ CCI_REG16(0x4390), 0xE0E0 },
	{ CCI_REG16(0x4392), 0xE0E0 },
	{ CCI_REG16(0x4394), 0xE0E0 },
	{ CCI_REG16(0x4396), 0xE0E0 },
	{ CCI_REG16(0x4398), 0xE0E0 },
	{ CCI_REG16(0x439A), 0xE0E0 },
	{ CCI_REG16(0x439C), 0xE0E0 },
	{ CCI_REG16(0x439E), 0xE0E0 },
	{ CCI_REG16(0x43A0), 0xE0E0 },
	{ CCI_REG16(0x43A2), 0xE0E0 },
	{ CCI_REG16(0x43A4), 0xE0E0 },
	{ CCI_REG16(0x43A6), 0xE0E0 },
	{ CCI_REG16(0x43A8), 0xE0E0 },
	{ CCI_REG16(0x43AA), 0xE0E0 },
	{ CCI_REG16(0x43AC), 0xE0E0 },
	{ CCI_REG16(0x43AE), 0xE0E0 },
	{ CCI_REG16(0x43B0), 0xE0E0 },
	{ CCI_REG16(0x43B2), 0xE0E0 },
	{ CCI_REG16(0x43B4), 0xE0E0 },
	{ CCI_REG16(0x43B6), 0xE0E0 },
	{ CCI_REG16(0x43B8), 0xE0E0 },
	{ CCI_REG16(0x43BA), 0xE0E0 },
	{ CCI_REG16(0x43BC), 0xE0E0 },
	{ CCI_REG16(0x43BE), 0xE0E0 },
	{ CCI_REG16(0x43C0), 0xE0E0 },
	{ CCI_REG16(0x43C2), 0xE0E0 },
	{ CCI_REG16(0x43C4), 0xE0E0 },
	{ CCI_REG16(0x43C6), 0xE0E0 },
	{ CCI_REG16(0x43C8), 0xE0E0 },
	{ CCI_REG16(0x43CA), 0xE0E0 },
	{ CCI_REG16(0x43CC), 0xE0E0 },
	{ CCI_REG16(0x43CE), 0xE0E0 },
	{ CCI_REG16(0x43D0), 0xE0E0 },
	{ CCI_REG16(0x43D2), 0xE0E0 },
	{ CCI_REG16(0x43D4), 0xE0E0 },
	{ CCI_REG16(0x43D6), 0xE0E0 },
	{ CCI_REG16(0x43D8), 0xE0E0 },
	{ CCI_REG16(0x43DA), 0xE0E0 },
	{ CCI_REG16(0x43DC), 0xE0E0 },
	{ CCI_REG16(0x43DE), 0xE0E0 },
	{ CCI_REG16(0x43E0), 0xE0E0 },
	{ CCI_REG16(0x43E2), 0xE0E0 },
	{ CCI_REG16(0x43E4), 0xE0E0 },
	{ CCI_REG16(0x43E6), 0xE0E0 },
	{ CCI_REG16(0x43E8), 0xE0E0 },
	{ CCI_REG16(0x43EA), 0xE0E0 },
	{ CCI_REG16(0x43EC), 0xE0E0 },
	{ CCI_REG16(0x43EE), 0xE0E0 },
	{ CCI_REG16(0x43F0), 0xE0E0 },
	{ CCI_REG16(0x43F2), 0xE0E0 },
	{ CCI_REG16(0x43F4), 0xE0E0 },
	{ CCI_REG16(0x43F6), 0xE0E0 },
	{ CCI_REG16(0x43F8), 0xE0E0 },
	{ CCI_REG16(0x43FA), 0xE0E0 },
	{ CCI_REG16(0x43FC), 0xE0E0 },
	{ CCI_REG16(0x43FE), 0xE0E0 },
	{ CCI_REG16(0x4400), 0xE0E0 },
	{ CCI_REG16(0x4402), 0xE0E0 },
	{ CCI_REG16(0x4404), 0xE0E0 },
	{ CCI_REG16(0x4406), 0xE0E0 },
	{ CCI_REG16(0x4408), 0xE0E0 },
	{ CCI_REG16(0x440A), 0xE0E0 },
	{ CCI_REG16(0x440C), 0xE0E0 },
	{ CCI_REG16(0x440E), 0xE0E0 },
	{ CCI_REG16(0x4410), 0xE0E0 },
	{ CCI_REG16(0x4412), 0xE0E0 },
	{ CCI_REG16(0x4414), 0xE0E0 },
	{ CCI_REG16(0x4416), 0xE0E0 },
	{ CCI_REG16(0x4418), 0xE0E0 },
	{ CCI_REG16(0x441A), 0xE0E0 },
	{ CCI_REG16(0x441C), 0xE0E0 },
	{ CCI_REG16(0x441E), 0xE0E0 },
	{ CCI_REG16(0x4420), 0xE0E0 },
	{ CCI_REG16(0x4422), 0xE0E0 },
	{ CCI_REG16(0x4424), 0xE0E0 },
	{ CCI_REG16(0x4426), 0xE0E0 },
	{ CCI_REG16(0x4428), 0xE0E0 },
	{ CCI_REG16(0x442A), 0xE0E0 },
	{ CCI_REG16(0x442C), 0xE0E0 },
	{ CCI_REG16(0x442E), 0xE0E0 },
	{ CCI_REG16(0x4430), 0xE0E0 },
	{ CCI_REG16(0x4432), 0xE0E0 },
	{ CCI_REG16(0x4434), 0xE0E0 },
	{ CCI_REG16(0x4436), 0xE0E0 },
	{ CCI_REG16(0x4438), 0xE0E0 },
	{ CCI_REG16(0x443A), 0xE0E0 },
	{ CCI_REG16(0x443C), 0xE0E0 },
	{ CCI_REG16(0x443E), 0xE0E0 },
	{ CCI_REG16(0x4440), 0xE0E0 },
	{ CCI_REG16(0x4442), 0xE0E0 },
	{ CCI_REG16(0x4444), 0xE0E0 },
	{ CCI_REG16(0x4446), 0xE0E0 },
	{ CCI_REG16(0x4448), 0xE0E0 },
	{ CCI_REG16(0x444A), 0xE0E0 },
	{ CCI_REG16(0x444C), 0xE0E0 },
	{ CCI_REG16(0x444E), 0xE0E0 },
	{ CCI_REG16(0x4450), 0xE0E0 },
	{ CCI_REG16(0x4452), 0xE0E0 },
	{ CCI_REG16(0x4454), 0xE0E0 },
	{ CCI_REG16(0x4456), 0xE0E0 },
	{ CCI_REG16(0x4458), 0xE0E0 },
	{ CCI_REG16(0x445A), 0xE0E0 },
	{ CCI_REG16(0x445C), 0xE0E0 },
	{ CCI_REG16(0x445E), 0xE0E0 },
	{ CCI_REG16(0x4460), 0xE0E0 },
	{ CCI_REG16(0x4462), 0xE0E0 },
	{ CCI_REG16(0x4464), 0xE0E0 },
	{ CCI_REG16(0x4466), 0xE0E0 },
	{ CCI_REG16(0x4468), 0xE0E0 },
	{ CCI_REG16(0x446A), 0xE0E0 },
	{ CCI_REG16(0x446C), 0xE0E0 },
	{ CCI_REG16(0x446E), 0xE0E0 },
	{ CCI_REG16(0x4470), 0xE0E0 },
	{ CCI_REG16(0x4472), 0xE0E0 },
	{ CCI_REG16(0x4474), 0xE0E0 },
	{ CCI_REG16(0x4476), 0xE0E0 },
	{ CCI_REG16(0x4478), 0xE0E0 },
	{ CCI_REG16(0x447A), 0xE0E0 },
	{ CCI_REG16(0x447C), 0xE0E0 },
	{ CCI_REG16(0x447E), 0xE0E0 },
	{ CCI_REG16(0x4480), 0xE0E0 },
	{ CCI_REG16(0x4482), 0xE0E0 },
	{ CCI_REG16(0x4484), 0xE0E0 },
	{ CCI_REG16(0x4486), 0xE0E0 },
	{ CCI_REG16(0x4488), 0xE0E0 },
	{ CCI_REG16(0x448A), 0xE0E0 },
	{ CCI_REG16(0x448C), 0xE0E0 },
	{ CCI_REG16(0x448E), 0xE0E0 },
	{ CCI_REG16(0x4490), 0xE0E0 },
	{ CCI_REG16(0x4492), 0xE0E0 },
	{ CCI_REG16(0x4494), 0xE0E0 },
	{ CCI_REG16(0x4496), 0xE0E0 },
	{ CCI_REG16(0x4498), 0xE0E0 },
	{ CCI_REG16(0x449A), 0xE0E0 },
	{ CCI_REG16(0x449C), 0xE0E0 },
	{ CCI_REG16(0x449E), 0xE0E0 },
	{ CCI_REG16(0x44A0), 0xE0E0 },
	{ CCI_REG16(0x44A2), 0xE0E0 },
	{ CCI_REG16(0x44A4), 0xE0E0 },
	{ CCI_REG16(0x44A6), 0xE0E0 },
	{ CCI_REG16(0x44A8), 0xE0E0 },
	{ CCI_REG16(0x44AA), 0xE0E0 },
	{ CCI_REG16(0x44AC), 0xE0E0 },
	{ CCI_REG16(0x44AE), 0xE0E0 },
	{ CCI_REG16(0x44B0), 0xE0E0 },
	{ CCI_REG16(0x44B2), 0xE0E0 },
	{ CCI_REG16(0x44B4), 0xE0E0 },

	{ CCI_REG16(0x5500), 0x0000 },
	{ CCI_REG16(0x5502), 0x0001 },
	{ CCI_REG16(0x5504), 0x0006 },
	{ CCI_REG16(0x5506), 0x0008 },
	{ CCI_REG16(0x5508), 0x000F },
	{ CCI_REG16(0x550A), 0x0010 },
	{ CCI_REG16(0x550C), 0x0011 },
	{ CCI_REG16(0x550E), 0x0012 },
	{ CCI_REG16(0x5510), 0x0016 },
	{ CCI_REG16(0x5512), 0x0018 },
	{ CCI_REG16(0x5514), 0x0021 },
	{ CCI_REG16(0x5516), 0x0023 },
	{ CCI_REG16(0x5518), 0x0026 },
	{ CCI_REG16(0x551A), 0x0028 },
	{ CCI_REG16(0x551C), 0x002F },
	{ CCI_REG16(0x551E), 0x0030 },
	{ CCI_REG16(0x5400), 0x0100 },
	{ CCI_REG16(0x5402), 0x2106 },
	{ CCI_REG16(0x5404), 0x1103 },
	{ CCI_REG16(0x5406), 0x3103 },
	{ CCI_REG16(0x5408), 0x6103 },
	{ CCI_REG16(0x540A), 0x9103 },
	{ CCI_REG16(0x540C), 0xA103 },
	{ CCI_REG16(0x540E), 0xD103 },
	{ CCI_REG16(0x5410), 0xF110 },
	{ CCI_REG16(0x5412), 0xE102 },
	{ CCI_REG16(0x5414), 0xF15E },
	{ CCI_REG16(0x5416), 0xF1EE },
	{ CCI_REG16(0x5418), 0xF2BA },
	{ CCI_REG16(0x541A), 0xF3DA },
	{ CCI_REG16(0x541C), 0xF571 },
	{ CCI_REG16(0x541E), 0xF7B0 },
	{ CCI_REG16(0x5420), 0xFADD },
	{ CCI_REG16(0x5422), 0xFF58 },
	{ CCI_REG16(0x5424), 0xFFFA },
	{ CCI_REG16(0x5426), 0x5557 },
	{ CCI_REG16(0x5428), 0x0005 },
	{ CCI_REG16(0x542A), 0xA550 },
	{ CCI_REG16(0x542C), 0xAAAA },
	{ CCI_REG16(0x542E), 0x000A },
	{ CCI_REG16(0x5460), 0x2269 },
	{ CCI_REG16(0x5462), 0x0B8F },
	{ CCI_REG16(0x5464), 0x0B8F },
	{ CCI_REG16(0x5466), 0x098B },
	{ CCI_REG16(0x5498), 0x2263 },
	{ CCI_REG16(0x549A), 0x54E2 },
	{ CCI_REG16(0x549C), 0x54E3 },
	{ CCI_REG16(0x549E), 0x54E3 },
	{ CCI_REG16(0x3060), 0xFF01 },
};

static const struct cci_reg_sequence ar0830_ae_setup[] = {
	{ CCI_REG16(0x3d10), 0x00e0 },
	{ CCI_REG16(0x3d26), 0x086f },
	{ CCI_REG16(0x3d28), 0x0834 },
};

static int ar0830_init_sensor(struct ar0830 *sensor)
{
	u64 customer_rev;
	int ret = 0;

	cci_write(sensor->regmap, AR0830_MODE_SELECT, 0, &ret);
	cci_multi_reg_write(sensor->regmap, ar0830_recommended_regs,
			    ARRAY_SIZE(ar0830_recommended_regs), &ret);

	cci_read(sensor->regmap, AR0830_CUSTOMER_REV, &customer_rev, &ret);

	if (((customer_rev >> 8) & 0xf) == 0) {
		cci_multi_reg_write(sensor->regmap, ar0830_otp_regs,
				    ARRAY_SIZE(ar0830_otp_regs), &ret);
	}

	cci_multi_reg_write(sensor->regmap, ar0830_ae_setup, ARRAY_SIZE(ar0830_ae_setup), &ret);
	cci_write(sensor->regmap, AR0830_AE_CTRL,
		  AR0830_AE_MAX_ANA_GAIN(3) |
		  AR0830_AE_MIN_ANA_GAIN(0) |
		  AR0830_AE_AUTO_AG_EN |
		  AR0830_AE_AUTO_DG_EN |
		  AR0830_AE_EN,
		  &ret);

	return ret;
}

static int ar0830_configure_pll(struct ar0830 *sensor)
{
	int ret = 0;

	cci_update_bits(sensor->regmap, AR0830_RESET_REGISTER, BIT(3), 0, &ret);

	cci_write(sensor->regmap, AR0830_VT_PRE_PLL_CLK_DIV,
		  sensor->pll.vt_fr.pre_pll_clk_div, &ret);
	cci_write(sensor->regmap, AR0830_VT_PLL_MULTIPLIER,
		  sensor->pll.vt_fr.pll_multiplier, &ret);
	cci_write(sensor->regmap, AR0830_VT_PIX_CLK_DIV,
		  sensor->pll.vt_bk.pix_clk_div, &ret);
	cci_write(sensor->regmap, AR0830_VT_SYS_CLK_DIV,
		  sensor->pll.vt_bk.sys_clk_div, &ret);
	cci_write(sensor->regmap, AR0830_OP_PRE_PLL_CLK_DIV,
		  sensor->pll.op_fr.pre_pll_clk_div, &ret);
	cci_write(sensor->regmap, AR0830_OP_PLL_MULTIPLIER,
		  sensor->pll.op_fr.pll_multiplier, &ret);
	cci_write(sensor->regmap, AR0830_OP_PIX_CLK_DIV,
		  sensor->pll.op_bk.pix_clk_div, &ret);
	cci_write(sensor->regmap, AR0830_OP_SYS_CLK_DIV,
		  sensor->pll.op_bk.sys_clk_div, &ret);

	fsleep(1000);

	return ret;
}

static const struct cci_reg_sequence mipi_regs[] = {
	{ CCI_REG16(0x3ec0), 0x006f },
	{ CCI_REG16(0x3ec2), 0x0034 },
	{ CCI_REG16(0x3ec4), 0x0204 },
	{ CCI_REG16(0x3ec6), 0x000f },
	{ CCI_REG16(0x3c80), 0x0010 },
	{ CCI_REG16(0x3600), 0x94d8 },
	{ CCI_REG16(0x3f1c), 0x0ad3 },
	{ CCI_REG16(0x3f20), 0x8008 },
};

static int ar0830_configure_mipi(struct ar0830 *sensor, const struct ar0830_format_info *info)
{
	unsigned int num_lanes = sensor->buscfg.bus.mipi_csi2.num_data_lanes;
	bool cont_clk = !(sensor->buscfg.bus.mipi_csi2.flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK);
	int ret = 0;

	cci_write(sensor->regmap, AR0830_CSI_DATA_FORMAT,
		  AR0830_DATA_FORMAT_IN(info->bpp) | AR0830_DATA_FORMAT_OUT(info->bpp), &ret);

	cci_write(sensor->regmap, AR0830_CSI_LANE_MODE, num_lanes - 1, &ret);

	cci_multi_reg_write(sensor->regmap, mipi_regs, ARRAY_SIZE(mipi_regs), &ret);

	cci_write(sensor->regmap, AR0830_MIPI_TIMING_0,
		  AR0830_T_HS_PREPARE(7) | AR0830_T_HS_ZERO(14) | AR0830_T_HS_TRAIL(10), &ret);
	cci_write(sensor->regmap, AR0830_MIPI_TIMING_1,
		  AR0830_T_CLK_PREPARE(6) | AR0830_T_CLK_ZERO(44) | AR0830_T_CLK_TRAIL(9), &ret);
	cci_write(sensor->regmap, AR0830_MIPI_TIMING_2,
		  AR0830_T_CLK_PRE(3) | AR0830_T_CLK_POST(14), &ret);
	cci_write(sensor->regmap, AR0830_MIPI_TIMING_3,
		  AR0830_T_LPX(8) | AR0830_T_WAKE_UP(24), &ret);
	cci_write(sensor->regmap, AR0830_MIPI_TIMING_4,
		  (cont_clk ? AR0830_CONT_TX_CLK : 0) | AR0830_T_HS_EXIT(15) |
		  AR0830_T_INIT(14), &ret);
	cci_write(sensor->regmap, AR0830_MIPI_TIMING_5, AR0830_T_BGAP(12), &ret);

	return ret;
}

static int ar0830_start_streaming(struct ar0830 *sensor, const struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	const struct ar0830_format_info *info;
	unsigned int skip_x, skip_y, odd_inc_x, odd_inc_y, even_inc_x, even_inc_y;
	int ret;

	format = v4l2_subdev_state_get_format(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 0);
	info = ar0830_get_format_info(sensor, format->code, false);

	if (!info)
		return -EINVAL;

	ret = ar0830_configure_pll(sensor);
	if (ret)
		return ret;

	ret = ar0830_configure_mipi(sensor, info);
	if (ret)
		return ret;

	skip_x = crop->width / format->width;
	skip_y = crop->height / format->height;

	if (sensor->model.mono) {
		odd_inc_x = skip_x;
		odd_inc_y = skip_y;
		even_inc_x = skip_x;
		even_inc_y = skip_y;
	} else {
		odd_inc_x = 2 * skip_x - 1;
		odd_inc_y = 2 * skip_y - 1;
		even_inc_x = 1;
		even_inc_y = 1;
	}

	cci_write(sensor->regmap, AR0830_MONOCHROME_EN, (sensor->model.mono ? 1 : 0), &ret);

	cci_write(sensor->regmap, AR0830_X_ADDR_START, crop->left + AR0830_X_ADDR_MIN, &ret);
	cci_write(sensor->regmap, AR0830_Y_ADDR_START, crop->top + AR0830_Y_ADDR_MIN, &ret);
	cci_write(sensor->regmap, AR0830_X_ADDR_END,
		  crop->left + crop->width + AR0830_X_ADDR_MIN - odd_inc_x, &ret);
	cci_write(sensor->regmap, AR0830_Y_ADDR_END,
		  crop->top + crop->height + AR0830_Y_ADDR_MIN - odd_inc_y, &ret);

	cci_write(sensor->regmap, AR0830_X_OUTPUT_SIZE, format->width, &ret);
	cci_write(sensor->regmap, AR0830_Y_OUTPUT_SIZE, format->height, &ret);

	cci_write(sensor->regmap, AR0830_X_ODD_INC, odd_inc_x, &ret);
	cci_write(sensor->regmap, AR0830_X_EVEN_INC, even_inc_x, &ret);
	cci_write(sensor->regmap, AR0830_Y_ODD_INC, odd_inc_y, &ret);
	cci_write(sensor->regmap, AR0830_Y_EVEN_INC, even_inc_y, &ret);

	if (ret)
		return ret;

	__v4l2_ctrl_grab(sensor->link_freq, true);

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		goto error;

	cci_write(sensor->regmap, AR0830_MODE_SELECT, 1, &ret);
	if (ret)
		goto error;

	return 0;
error:
	__v4l2_ctrl_grab(sensor->link_freq, false);
	return ret;
}

static int ar0830_stop_streaming(struct ar0830 *sensor)
{
	int ret;

	__v4l2_ctrl_grab(sensor->link_freq, false);

	ret = cci_write(sensor->regmap, AR0830_MODE_SELECT, 0, NULL);
	if (ret)
		return ret;

	return 0;
}

static int ar0830_check_chip_id(struct ar0830 *sensor)
{
	struct device *dev = sensor->dev;
	u64 chip_version, customer_rev;
	int ret;

	cci_read(sensor->regmap, AR0830_CHIP_VERSION_REG, &chip_version, &ret);
	cci_read(sensor->regmap, AR0830_CUSTOMER_REV, &customer_rev, &ret);

	if (ret) {
		dev_err(dev, "Failed to read chip ID (%d)\n", ret);
		return ret;
	}

	if (chip_version != AR0830_CHIP_VERSION) {
		dev_err(dev, "Wrong chip version: 0x%04x <-> 0x%04x\n",
			(u32)chip_version, AR0830_CHIP_VERSION);
		return -ENODEV;
	}

	sensor->model.mono = AR0830_CUSTOMER_REV_CFA(customer_rev) == AR0830_CUSTOMER_REV_CFA_MONO;

	dev_info(dev, "Device ID: 0x%04x, %s model\n",
		 (u32)chip_version, sensor->model.mono ? "monochrome" : "color");

	dev_info(dev, "Customer REV: 0x%04llx (OTPM settings: 0x%llx)\n", customer_rev,
		 (customer_rev >> 8) & 0xf);

	return 0;
}

static void ar0830_reset(struct ar0830 *sensor)
{
	u64 reset_delay;
	long rate;

	rate = clk_get_rate(sensor->extclk);
	reset_delay = DIV_ROUND_UP_ULL(160000ULL * USEC_PER_SEC, rate);

	if (sensor->reset) {
		gpiod_set_value_cansleep(sensor->reset, 1);
		fsleep(1000);
		gpiod_set_value_cansleep(sensor->reset, 0);
	} else {
		cci_write(sensor->regmap, AR0830_SOFTWARE_RESET, 1, NULL);
	}

	fsleep(reset_delay);
}

static int ar0830_calculate_pll(struct ar0830 *sensor, unsigned int link_freq,
				struct ccs_pll *pll, unsigned int bpp)
{
	struct ccs_pll_limits limits = {
		.min_ext_clk_freq_hz = 6000000,
		.max_ext_clk_freq_hz = 48000000,

		.vt_fr = {
			.min_pre_pll_clk_div = 1,
			.max_pre_pll_clk_div = 63,
			.min_pll_multiplier = 16,
			.max_pll_multiplier = 255,
			.min_pll_op_clk_freq_hz = 384000000,
			.max_pll_op_clk_freq_hz = 900000000,
		},
		.vt_bk = {
			.min_sys_clk_div = 1,
			.max_sys_clk_div = 8,
			.min_pix_clk_div = 4,
			.max_pix_clk_div = 10,
			.min_pix_clk_freq_hz = 64000000,
			.max_pix_clk_freq_hz = 150000000,
		},
		.op_fr = {
			.min_pre_pll_clk_div = 1,
			.max_pre_pll_clk_div = 63,
			.min_pll_multiplier = 16,
			.max_pll_multiplier = 255,
			.min_pll_op_clk_freq_hz = 384000000,
			.max_pll_op_clk_freq_hz = 1800000000,
		},
		.op_bk = {
			.min_sys_clk_div = 1,
			.max_sys_clk_div = 8,
			.min_pix_clk_div = bpp,
			.max_pix_clk_div = bpp,
			.min_pix_clk_freq_hz = 64000000,
			.max_pix_clk_freq_hz = 150000000,
		},

		.min_line_length_pck_bin = 4496,
		.min_line_length_pck = 4496,
	};
	unsigned int num_lanes = sensor->buscfg.bus.mipi_csi2.num_data_lanes;
	int ret;

	/*
	 * There are no documented constraints on the PLL input clock
	 * frequency, for either branch. Recover them based on the external
	 * clock frequency and pre_pll_clk_div limits on one hand, and the PLL
	 * output clock and the pll_multiplier limits on the other hand.
	 */

	limits.vt_fr.min_pll_ip_clk_freq_hz =
		max(limits.min_ext_clk_freq_hz / limits.vt_fr.max_pre_pll_clk_div,
		    limits.vt_fr.min_pll_op_clk_freq_hz / limits.vt_fr.max_pll_multiplier);
	limits.vt_fr.max_pll_ip_clk_freq_hz =
		min(limits.max_ext_clk_freq_hz / limits.vt_fr.min_pre_pll_clk_div,
		    limits.vt_fr.max_pll_op_clk_freq_hz / limits.vt_fr.min_pll_multiplier);

	limits.op_fr.min_pll_ip_clk_freq_hz =
		max(limits.min_ext_clk_freq_hz / limits.op_fr.max_pre_pll_clk_div,
		    limits.op_fr.min_pll_op_clk_freq_hz / limits.op_fr.max_pll_multiplier);
	limits.op_fr.max_pll_ip_clk_freq_hz =
		min(limits.max_ext_clk_freq_hz / limits.op_fr.min_pre_pll_clk_div,
		    limits.op_fr.max_pll_op_clk_freq_hz / limits.op_fr.min_pll_multiplier);

	/*
	 * There are no documented constraints on the sys clock frequency, for
	 * either branch. Recover them based on the PLL output clock frequency
	 * and sys_clk_div limits on one hand, and the pix clock frequency and
	 * the pix_clk_div limits on the other hand.
	 */

	limits.vt_bk.min_sys_clk_freq_hz =
		max(limits.vt_fr.min_pll_op_clk_freq_hz / limits.vt_bk.max_sys_clk_div,
		    limits.vt_bk.min_pix_clk_freq_hz * limits.vt_bk.min_pix_clk_div);
	limits.vt_bk.max_sys_clk_freq_hz =
		min(limits.vt_fr.max_pll_op_clk_freq_hz / limits.vt_bk.min_sys_clk_div,
		    limits.vt_bk.max_pix_clk_freq_hz * limits.vt_bk.max_pix_clk_div);

	limits.op_bk.min_sys_clk_freq_hz =
		max(limits.op_fr.min_pll_op_clk_freq_hz / limits.op_bk.max_sys_clk_div,
		    limits.op_bk.min_pix_clk_freq_hz * limits.op_bk.min_pix_clk_div);
	limits.op_bk.max_sys_clk_freq_hz =
		min(limits.op_fr.max_pll_op_clk_freq_hz / limits.op_bk.min_sys_clk_div,
		    limits.op_bk.max_pix_clk_freq_hz * limits.op_bk.max_pix_clk_div);

	memset(pll, 0, sizeof(*pll));

	pll->bus_type = CCS_PLL_BUS_TYPE_CSI2_DPHY;
	pll->op_lanes = num_lanes;
	pll->vt_lanes = num_lanes;
	pll->csi2.lanes = num_lanes;
	/*
	 * Since we don't use FIFO derating, binning doesn't
	 * influence the PLL configuration. Hardcode the binning factors.
	 */
	pll->binning_horizontal = 1;
	pll->binning_vertical = 1;
	pll->scale_m = 1;
	pll->scale_n = 1;
	pll->bits_per_pixel = bpp;
	pll->flags = CCS_PLL_FLAG_LANE_SPEED_MODEL |
		    CCS_PLL_FLAG_DUAL_PLL |
		    CCS_PLL_FLAG_EXT_IP_PLL_DIVIDER;
	pll->link_freq = link_freq;
	pll->ext_clk_freq_hz = clk_get_rate(sensor->extclk);

	ret = ccs_pll_calculate(sensor->dev, &limits, pll);
	if (ret)
		return ret;

	return 0;
}

static int ar0830_pll_update(struct ar0830 *sensor, const struct ar0830_format_info *info)
{
	u64 link_freq;
	int ret;

	link_freq = sensor->buscfg.link_frequencies[sensor->link_freq->val];
	ret = ar0830_calculate_pll(sensor, link_freq, &sensor->pll, info->bpp);
	if (ret) {
		dev_err(sensor->dev, "PLL calculation failed: %d\n", ret);
		return ret;
	}

	__v4l2_ctrl_s_ctrl_int64(sensor->pixel_rate, sensor->pll.pixel_rate_pixel_array);

	return 0;
}

static void ar0830_update_link_freqs(struct ar0830 *sensor, const struct ar0830_format_info *info)
{
	u64 valid_link_freqs;
	unsigned int index, min, max;

	index = info - ar0830_formats;
	valid_link_freqs = sensor->valid_link_freqs[index];

	min = __ffs(valid_link_freqs);
	max = __fls(valid_link_freqs);

	__v4l2_ctrl_modify_range(sensor->link_freq, min, max, ~valid_link_freqs, max);
}

static void ar0830_update_blankings(struct ar0830 *sensor, const struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	unsigned int min, max;

	format = v4l2_subdev_state_get_format(state, 0);

	min = max_t(int, AR0830_MIN_LINE_LENGTH_PCK - format->width, AR0830_MIN_HBLANK);
	max = AR0830_MAX_LINE_LENGTH_PCK - format->width;

	__v4l2_ctrl_modify_range(sensor->hblank, min, max, 1, min);

	min = AR0830_MIN_VBLANK;
	max = AR0830_MAX_FRAME_LENGTH_LINES - format->height;

	__v4l2_ctrl_modify_range(sensor->vblank, min, max, 1, min);
}

static int ar0830_update_binning(struct ar0830 *sensor, const struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	unsigned int skip_x, skip_y;
	u64 type = 0x11;
	u64 mode = 0;
	int ret;

	format = v4l2_subdev_state_get_format(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 0);

	skip_x = crop->width / format->width;
	skip_y = crop->height / format->height;

	if (sensor->row_binning->val && skip_x > 0) {
		type &= ~AR0830_ROW_BINNING_FACTOR_MASK;
		type |= AR0830_ROW_BINNING_FACTOR(skip_x);
		mode = 1;
	}

	if (sensor->col_binning->val && skip_y > 0) {
		type &= ~AR0830_COLUMN_BINNING_FACTOR_MASK;
		type |= AR0830_COLUMN_BINNING_FACTOR(skip_y);
		mode = 1;
	}

	cci_write(sensor->regmap, AR0830_BINNING_TYPE, type, &ret);
	cci_write(sensor->regmap, AR0830_BINNING_MODE, mode, &ret);

	return ret;
}

static void ar0830_update_ae_max_exposure(struct ar0830 *sensor,
					  const struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, 0);

	__v4l2_ctrl_s_ctrl(sensor->ae_exp_max, format->height);
}

static int ar0830_set_whitebalance(struct ar0830 *sensor)
{
	u64 red_gain = sensor->red_balance->val + 256;
	u64 blue_gain = sensor->blue_balance->val + 256;
	u64 green_gain = 256;
	int ret;

	cci_write(sensor->regmap, AR0830_DIGITAL_GAIN_RED, red_gain, &ret);
	cci_write(sensor->regmap, AR0830_DIGITAL_GAIN_BLUE, blue_gain, &ret);
	cci_write(sensor->regmap, AR0830_DIGITAL_GAIN_GREENR, green_gain, &ret);
	cci_write(sensor->regmap, AR0830_DIGITAL_GAIN_GREENB, green_gain, &ret);

	return ret;
}

static int ar0830_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0830 *sensor = container_of(ctrl->handler, struct ar0830, ctrls);
	u64 value;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		cci_read(sensor->regmap, AR0830_AE_COARSE_INTEGRATION_TIME, &value, &ret);
		sensor->exposure->val = value;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int ar0830_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0830 *sensor = container_of(ctrl->handler, struct ar0830, ctrls);
	const struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *format;
	const struct ar0830_format_info *info;
	int ret = 0;

	if (ctrl->flags & (V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_GRABBED))
		return 0;

	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	format = v4l2_subdev_state_get_format(state, 0);
	info = ar0830_get_format_info(sensor, format->code, true);

	switch (ctrl->id) {
	case V4L2_CID_AR0830_ROW_BINNING:
	case V4L2_CID_AR0830_COLUMN_BINNING:
		ar0830_update_binning(sensor, state);
		break;
	case V4L2_CID_AR0830_DYNAMIC_PIXEL_CORRECTION:
		cci_update_bits(sensor->regmap, AR0830_PIX_DEF_CORR, AR0830_1D_DDC_EN,
				ctrl->val ? AR0830_1D_DDC_EN : 0, &ret);
		break;
	case V4L2_CID_AR0830_AUTO_EXP_MAX:
		cci_write(sensor->regmap, AR0830_AE_MAX_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_AR0830_AUTO_EXP_MIN:
		cci_write(sensor->regmap, AR0830_AE_MIN_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_AR0830_AUTO_EXP_TARGET:
		cci_write(sensor->regmap, AR0830_AE_LUMA_TARGET, ctrl->val, &ret);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->val == V4L2_EXPOSURE_MANUAL)
			cci_write(sensor->regmap, AR0830_COARSE_INTEGRATION_TIME,
				  sensor->exposure->val, &ret);

		cci_update_bits(sensor->regmap, AR0830_AE_CTRL, AR0830_AE_EN,
				ctrl->val == V4L2_EXPOSURE_AUTO ? AR0830_AE_EN : 0,
				&ret);
		break;
	case V4L2_CID_AUTOGAIN:
		if (ctrl->val == 0)
			cci_write(sensor->regmap, AR0830_GAIN_CODE,
				  AR0830_GLOBAL_GAIN_CODE(sensor->gain->val), &ret);

		if (ctrl->is_new) {
			cci_update_bits(sensor->regmap, AR0830_AE_CTRL,
					AR0830_AE_AUTO_AG_EN | AR0830_AE_AUTO_DG_EN,
					ctrl->val ? AR0830_AE_AUTO_AG_EN | AR0830_AE_AUTO_DG_EN : 0,
					&ret);

			v4l2_ctrl_activate(sensor->gain, !(ctrl->val));
		}
		break;
	case V4L2_CID_RED_BALANCE:
	case V4L2_CID_BLUE_BALANCE:
		ret = ar0830_set_whitebalance(sensor);
		break;
	case V4L2_CID_HBLANK:
		cci_write(sensor->regmap, AR0830_LINE_LENGTH_PCK,
			  format->width + ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, AR0830_FRAME_LENGTH_LINES,
			  format->height + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_update_bits(sensor->regmap, AR0830_TEST_PATTERN_MODE,
				AR0830_TEST_PATTERN_SELECT_MASK,
				AR0830_TEST_PATTERN_SELECT(ar0830_test_pattern_val[ctrl->val]),
				&ret);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		cci_write(sensor->regmap, AR0830_TEST_DATA_RED, ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		cci_write(sensor->regmap, AR0830_TEST_DATA_GREENR, ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		cci_write(sensor->regmap, AR0830_TEST_DATA_BLUE, ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		cci_write(sensor->regmap, AR0830_TEST_DATA_GREENB, ctrl->val, &ret);
		break;
	case V4L2_CID_LINK_FREQ:
		ret = ar0830_pll_update(sensor, info);
		break;
	default:
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ar0830_ctrl_ops = {
	.s_ctrl = ar0830_s_ctrl,
	.g_volatile_ctrl = ar0830_g_volatile_ctrl,
};

static const struct v4l2_ctrl_config ar0830_row_binning_ctrl_config = {
	.ops = &ar0830_ctrl_ops,
	.id = V4L2_CID_AR0830_ROW_BINNING,
	.name = "Row Binning Enable",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config ar0830_column_binning_ctrl_config = {
	.ops = &ar0830_ctrl_ops,
	.id = V4L2_CID_AR0830_COLUMN_BINNING,
	.name = "Column Binning Enable",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config ar0830_dynamic_pixel_correction_ctrl_config = {
	.ops = &ar0830_ctrl_ops,
	.id = V4L2_CID_AR0830_DYNAMIC_PIXEL_CORRECTION,
	.name = "Dynamic Defect Pixel Correction",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 1,
};

static const struct v4l2_ctrl_config ar0830_auto_exp_max_ctrl_config = {
	.ops = &ar0830_ctrl_ops,
	.id = V4L2_CID_AR0830_AUTO_EXP_MAX,
	.name = "Auto Exposure Max",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = AR0830_MAX_FRAME_LENGTH_LINES - 1,
	.step = 1,
	.def = AR0830_DEF_HEIGHT,
};

static const struct v4l2_ctrl_config ar0830_auto_exp_min_ctrl_config = {
	.ops = &ar0830_ctrl_ops,
	.id = V4L2_CID_AR0830_AUTO_EXP_MIN,
	.name = "Auto Exposure Min",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = AR0830_MAX_FRAME_LENGTH_LINES - 1,
	.step = 1,
	.def = 1,
};

static const struct v4l2_ctrl_config ar0830_auto_exp_target_ctrl_config = {
	.ops = &ar0830_ctrl_ops,
	.id = V4L2_CID_AR0830_AUTO_EXP_TARGET,
	.name = "Auto Exposure Target (Luma)",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 65535,
	.step = 1,
	.def = 20480,
};

static int ar0830_init_ctrls(struct ar0830 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 10);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &ar0830_ctrl_ops, &props);

	sensor->pixel_rate = v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       1, INT_MAX, 1, 1);

	sensor->link_freq = v4l2_ctrl_new_int_menu(&sensor->ctrls, &ar0830_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   sensor->buscfg.nr_of_link_frequencies - 1, 0,
						   sensor->buscfg.link_frequencies);

	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops,
					   V4L2_CID_HBLANK, AR0830_MIN_HBLANK,
					   AR0830_MIN_HBLANK, 1, AR0830_MIN_HBLANK);

	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops,
					   V4L2_CID_VBLANK, AR0830_MIN_VBLANK,
					   AR0830_MIN_VBLANK, 1, AR0830_MIN_VBLANK);

	sensor->auto_exposure = v4l2_ctrl_new_std_menu(&sensor->ctrls, &ar0830_ctrl_ops,
						       V4L2_CID_EXPOSURE_AUTO,
						       V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     1, AR0830_MAX_FRAME_LENGTH_LINES - 1, 1,
					     AR0830_DEF_HEIGHT);

	sensor->ae_exp_max = v4l2_ctrl_new_custom(&sensor->ctrls, &ar0830_auto_exp_max_ctrl_config,
						  NULL);
	v4l2_ctrl_new_custom(&sensor->ctrls, &ar0830_auto_exp_min_ctrl_config, NULL);
	v4l2_ctrl_new_custom(&sensor->ctrls, &ar0830_auto_exp_target_ctrl_config, NULL);

	sensor->auto_gain = v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops,
					      V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	sensor->gain = v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops,
					 V4L2_CID_GAIN, 0, 143, 1, 0);

	sensor->blue_balance = v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops,
						 V4L2_CID_BLUE_BALANCE, -256, 3839, 1, 0);
	sensor->red_balance = v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops,
						V4L2_CID_RED_BALANCE, -256, 3839, 1, 0);

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &ar0830_ctrl_ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ar0830_test_pattern_menu) - 1,
				     0, 0, ar0830_test_pattern_menu);
	v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops, V4L2_CID_TEST_PATTERN_RED,
			  0, 1023, 1, 1023);
	v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops, V4L2_CID_TEST_PATTERN_GREENR,
			  0, 1023, 1, 1023);
	v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops, V4L2_CID_TEST_PATTERN_BLUE,
			  0, 1023, 1, 1023);
	v4l2_ctrl_new_std(&sensor->ctrls, &ar0830_ctrl_ops, V4L2_CID_TEST_PATTERN_GREENB,
			  0, 1023, 1, 1023);

	sensor->row_binning = v4l2_ctrl_new_custom(&sensor->ctrls,
						   &ar0830_row_binning_ctrl_config, NULL);
	sensor->col_binning = v4l2_ctrl_new_custom(&sensor->ctrls,
						   &ar0830_column_binning_ctrl_config, NULL);
	v4l2_ctrl_new_custom(&sensor->ctrls, &ar0830_dynamic_pixel_correction_ctrl_config, NULL);

	if (sensor->ctrls.error) {
		ret = sensor->ctrls.error;
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	v4l2_ctrl_auto_cluster(2, &sensor->auto_exposure, V4L2_EXPOSURE_MANUAL, true);
	v4l2_ctrl_cluster(2, &sensor->auto_gain);

	sensor->sd.ctrl_handler = &sensor->ctrls;

	return 0;
}

static int ar0830_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0830 *sensor = to_ar0830(sd);
	unsigned int index = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ar0830_formats); i++) {
		const struct ar0830_format_info *info = &ar0830_formats[i];

		if (!(sensor->valid_formats & BIT(i)))
			continue;

		if (code->index == index) {
			code->code = ar0830_format_code(sensor, info);
			return 0;
		}

		index++;
	}

	return -EINVAL;
}

static int ar0830_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0830 *sensor = to_ar0830(sd);
	const struct ar0830_format_info *info;
	const struct v4l2_rect *crop;

	info = ar0830_get_format_info(sensor, fse->code, false);
	if (!info)
		return -EINVAL;

	if (fse->index > 2)
		return -EINVAL;

	crop = v4l2_subdev_state_get_crop(state, fse->pad);

	fse->min_width = crop->width / (1 << fse->index);
	fse->max_width = fse->min_width;
	fse->min_height = crop->height / (1 << fse->index);
	fse->max_height = fse->min_height;

	if (!IS_ALIGNED(fse->min_width, 4))
		return -EINVAL;

	return 0;
}

static int ar0830_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct ar0830 *sensor = to_ar0830(sd);
	const struct ar0830_format_info *info;
	struct v4l2_mbus_framefmt *fmt;
	const struct v4l2_rect *crop;
	unsigned int skip_x, skip_y;

	if (v4l2_subdev_is_streaming(sd) && format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EBUSY;

	fmt = v4l2_subdev_state_get_format(state, format->pad);
	crop = v4l2_subdev_state_get_crop(state, format->pad);

	info = ar0830_get_format_info(sensor, format->format.code, true);
	fmt->code = ar0830_format_code(sensor, info);

	fmt->width = clamp(format->format.width, 1U, crop->width);
	fmt->height = clamp(format->format.height, 1U, crop->height);
	skip_x = clamp(roundup_pow_of_two(crop->width / fmt->width), 1, 4);
	skip_y = clamp(roundup_pow_of_two(crop->height / fmt->height), 1, 4);

	fmt->width = crop->width / skip_x;
	fmt->height = crop->height / skip_y;

	while (!IS_ALIGNED(fmt->width, 4) && skip_x > 1) {
		skip_x >>= 1;
		fmt->width = crop->width / skip_x;
	}

	format->format = *fmt;

	if (format->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	ar0830_update_ae_max_exposure(sensor, state);
	ar0830_update_blankings(sensor, state);
	ar0830_update_binning(sensor, state);
	ar0830_update_link_freqs(sensor, info);
	ar0830_pll_update(sensor, info);

	return 0;
}

static int ar0830_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = AR0830_DEF_WIDTH;
		sel->r.height = AR0830_DEF_HEIGHT;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = AR0830_DEF_WIDTH;
		sel->r.height = AR0830_DEF_HEIGHT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ar0830_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct ar0830 *sensor = to_ar0830(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (v4l2_subdev_is_streaming(sd) && sel->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EBUSY;

	crop = v4l2_subdev_state_get_crop(state, sel->pad);
	fmt = v4l2_subdev_state_get_format(state, sel->pad);

	crop->left = min_t(unsigned int, ALIGN(sel->r.left, AR0830_STEP_WIDTH),
			   AR0830_MAX_WIDTH - AR0830_MIN_WIDTH);
	crop->top = min_t(unsigned int, ALIGN(sel->r.top, AR0830_STEP_HEIGHT),
			  AR0830_MAX_HEIGHT - AR0830_MIN_HEIGHT);
	crop->width = clamp(sel->r.width, AR0830_MIN_WIDTH, AR0830_MAX_WIDTH - crop->left);
	crop->height = clamp(sel->r.height, AR0830_MIN_HEIGHT, AR0830_MAX_HEIGHT - crop->top);

	sel->r = *crop;

	fmt->width = ALIGN(crop->width, AR0830_STEP_WIDTH);
	fmt->height = crop->height;

	ar0830_update_blankings(sensor, state);

	return 0;
}

static int ar0830_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *interval)
{
	struct ar0830 *sensor = to_ar0830(sd);
	const struct v4l2_mbus_framefmt *format;
	u32 pix_freq, hlen, vlen;

	format = v4l2_subdev_state_get_format(state, interval->pad);

	pix_freq = sensor->pll.pixel_rate_csi;
	hlen = format->width + sensor->hblank->val;
	vlen = format->height + sensor->vblank->val;

	interval->interval.numerator = 10;
	interval->interval.denominator = div_u64(pix_freq * 10ULL, vlen * hlen);

	return 0;
}

static int ar0830_get_frame_desc(struct v4l2_subdev *sd,
				 unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct ar0830 *sensor = to_ar0830(sd);
	const struct ar0830_format_info *info;
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	format = v4l2_subdev_state_get_format(state, 0);

	info = ar0830_get_format_info(sensor, format->code, false);
	if (!info) {
		v4l2_subdev_unlock_state(state);
		return -EINVAL;
	}

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry[0].pixelcode = format->code;
	fd->entry[0].stream = 0;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = info->dt;

	v4l2_subdev_unlock_state(state);

	return 0;
}

static int ar0830_get_mbus_config(struct v4l2_subdev *sd,
				  unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct ar0830 *sensor = to_ar0830(sd);

	cfg->type = sensor->buscfg.bus_type;
	cfg->bus.mipi_csi2 = sensor->buscfg.bus.mipi_csi2;

	return 0;
}

static int ar0830_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 u32 pad,  u64 streams_mask)
{
	struct ar0830 *sensor = to_ar0830(sd);
	int ret;

	ret = ar0830_start_streaming(sensor, state);
	if (ret) {
		dev_err(sensor->dev, "Failed to start streaming: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ar0830_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct ar0830 *sensor = to_ar0830(sd);

	return ar0830_stop_streaming(sensor);
}

static int ar0830_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	const struct ar0830_format_info *info;
	struct ar0830 *sensor = to_ar0830(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	info = ar0830_get_format_info(sensor, 0, true);

	format = v4l2_subdev_state_get_format(state, 0);
	format->width = AR0830_DEF_WIDTH;
	format->height = AR0830_DEF_HEIGHT;
	format->code = ar0830_format_code(sensor, info);
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_601;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = AR0830_DEF_WIDTH;
	crop->height = AR0830_DEF_HEIGHT;

	return 0;
}

static const struct v4l2_subdev_pad_ops ar0830_subdev_pad_ops = {
	.enum_mbus_code = ar0830_enum_mbus_code,
	.enum_frame_size = ar0830_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ar0830_set_fmt,
	.get_selection = ar0830_get_selection,
	.set_selection = ar0830_set_selection,
	.get_frame_interval = ar0830_get_frame_interval,
	.get_frame_desc = ar0830_get_frame_desc,
	.get_mbus_config = ar0830_get_mbus_config,
	.enable_streams = ar0830_enable_streams,
	.disable_streams = ar0830_disable_streams,
};

static const struct v4l2_subdev_ops ar0830_subdev_ops = {
	.pad = &ar0830_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0830_subdev_internal_ops = {
	.init_state = ar0830_init_state,
};

static const struct media_entity_operations ar0830_entity_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

static int ar0830_init_subdev(struct ar0830 *sensor)
{
	struct v4l2_subdev *sd = &sensor->sd;
	const struct v4l2_mbus_framefmt *format;
	const struct ar0830_format_info *info;
	struct v4l2_subdev_state *state;
	int ret;

	v4l2_i2c_subdev_init(sd, to_i2c_client(sensor->dev), &ar0830_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &ar0830_subdev_internal_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &ar0830_entity_ops;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret)
		return ret;

	ret = ar0830_init_ctrls(sensor);
	if (ret)
		goto err_entity;

	sensor->sd.state_lock = sensor->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_ctrls;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	format = v4l2_subdev_state_get_format(state, 0);
	info = ar0830_get_format_info(sensor, format->code, true);

	ar0830_update_link_freqs(sensor, info);
	ar0830_pll_update(sensor, info);
	ar0830_update_blankings(sensor, state);

	v4l2_subdev_unlock_state(state);

	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
err_entity:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void ar0830_cleanup_subdev(struct ar0830 *sensor)
{
	v4l2_subdev_cleanup(&sensor->sd);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	media_entity_cleanup(&sensor->sd.entity);
}

static int ar0830_parse_dt(struct ar0830 *sensor)
{
	struct v4l2_fwnode_endpoint *ep = &sensor->buscfg;
	struct fwnode_handle *endpoint;
	u64 valid_link_freqs = 0;
	unsigned int nlanes;
	unsigned int i, j;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!endpoint) {
		dev_err(sensor->dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	ep->bus_type = V4L2_MBUS_UNKNOWN;
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(sensor->dev, "Failed to parse endpoint node\n");
		goto error;
	}

	if (ep->bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(sensor->dev, "Unsupported bus type %u\n", ep->bus_type);
		goto error;
	}

	nlanes = ep->bus.mipi_csi2.num_data_lanes;
	if (nlanes != 4 && nlanes != 2 && nlanes != 1) {
		dev_err(sensor->dev, "Invalid number of data lanes: %d\n", nlanes);
		ret = -EINVAL;
		goto error;
	}

	if (!ep->nr_of_link_frequencies) {
		dev_err(sensor->dev, "No link frequency supplied\n");
		ret = -EINVAL;
		goto error;
	}

	if (ep->nr_of_link_frequencies > 64) {
		dev_err(sensor->dev, "Too many link-frequencies\n");
		ret = -EINVAL;
		goto error;
	}

	for (i = 0; i < ARRAY_SIZE(ar0830_formats); i++) {
		const struct ar0830_format_info *info = &ar0830_formats[i];

		for (j = 0; j < ep->nr_of_link_frequencies; j++) {
			u64 link_freq = ep->link_frequencies[j];
			struct ccs_pll pll;

			ret = ar0830_calculate_pll(sensor, link_freq, &pll, info->bpp);
			if (ret)
				continue;

			sensor->valid_link_freqs[i] |= BIT(j);
			valid_link_freqs |= BIT(j);
		}

		if (!sensor->valid_link_freqs[i]) {
			dev_warn(sensor->dev, "No valid link frequency for %u bpp\n", info->bpp);
			continue;
		}

		sensor->valid_formats |= BIT(i);
	}

	if (!sensor->valid_formats) {
		dev_err(sensor->dev, "No valid link frequency found for any format\n");
		ret = -EINVAL;
		goto error;
	}

	for (i = 0; i < ep->nr_of_link_frequencies; i++) {
		if (!(valid_link_freqs & BIT(i)))
			dev_warn(sensor->dev, "Link frequency %llu not valid for any formats\n",
				 ep->link_frequencies[i]);
	}

	return 0;

error:
	v4l2_fwnode_endpoint_free(ep);
	return ret;
}

static int ar0830_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar0830 *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap), "Error initializing I2C\n");

	sensor->extclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->extclk))
		return dev_err_probe(dev, PTR_ERR(sensor->extclk), "Cannot get clock\n");

	sensor->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset), "Cannot get reset gpio\n");

	ret = ar0830_parse_dt(sensor);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ar0830_reset(sensor);

	ret = ar0830_check_chip_id(sensor);
	if (ret) {
		dev_err_probe(dev, ret, "Error checking chip ID\n");
		goto err_dt;
	}

	ret = ar0830_init_subdev(sensor);
	if (ret) {
		dev_err(dev, "V4L2 subdev initialization error %d\n", ret);
		goto err_dt;
	}

	ret = ar0830_init_sensor(sensor);
	if (ret) {
		dev_err(dev, "Sensor initialization failed %d\n", ret);
		goto err_subdev;
	}

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret) {
		dev_err(dev, "Could not register V4L2 subdevice\n");
		goto err_subdev;
	}

	return 0;

err_subdev:
	ar0830_cleanup_subdev(sensor);
err_dt:
	v4l2_fwnode_endpoint_free(&sensor->buscfg);
	mutex_destroy(&sensor->lock);
	return ret;
}

static void ar0830_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0830 *sensor = to_ar0830(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	ar0830_cleanup_subdev(sensor);
	v4l2_fwnode_endpoint_free(&sensor->buscfg);
	mutex_destroy(&sensor->lock);
}

static const struct of_device_id ar0830_dt_ids[] = {
	{
		.compatible = "onnn,ar0830",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0830_dt_ids);

static struct i2c_driver ar0830_i2c_driver = {
	.driver	= {
		.name = "ar0830",
		.of_match_table	= ar0830_dt_ids,
	},
	.probe		= ar0830_probe,
	.remove		= ar0830_remove,
};
module_i2c_driver(ar0830_i2c_driver);

MODULE_DESCRIPTION("AR0830 MIPI Camera subdev driver");
MODULE_AUTHOR("Stefan Riedmüller <s.riedmueller@phytec.de>");
MODULE_LICENSE("GPL");
