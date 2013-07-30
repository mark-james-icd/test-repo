/*
 * OmniVision OV9740 Camera Driver
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * Based on ov9640 camera driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>

#define to_ov9740(sd)		container_of(sd, struct ov9740_priv, subdev)

/* General Status Registers */
#define OV9740_MODEL_ID_HI		0x0000
#define OV9740_MODEL_ID_LO		0x0001
#define OV9740_REVISION_NUMBER		0x0002
#define OV9740_MANUFACTURER_ID		0x0003
#define OV9740_SMIA_VERSION		0x0004

/* General Setup Registers */
#define OV9740_MODE_SELECT		0x0100
#define OV9740_IMAGE_ORT		0x0101
#define OV9740_SOFTWARE_RESET		0x0103
#define OV9740_GRP_PARAM_HOLD		0x0104
#define OV9740_MSK_CORRUP_FM		0x0105

/* Timing Setting */
#define OV9740_FRM_LENGTH_LN_HI		0x0340 /* VTS */
#define OV9740_FRM_LENGTH_LN_LO		0x0341 /* VTS */
#define OV9740_LN_LENGTH_PCK_HI		0x0342 /* HTS */
#define OV9740_LN_LENGTH_PCK_LO		0x0343 /* HTS */
#define OV9740_X_ADDR_START_HI		0x0344
#define OV9740_X_ADDR_START_LO		0x0345
#define OV9740_Y_ADDR_START_HI		0x0346
#define OV9740_Y_ADDR_START_LO		0x0347
#define OV9740_X_ADDR_END_HI		0x0348
#define OV9740_X_ADDR_END_LO		0x0349
#define OV9740_Y_ADDR_END_HI		0x034a
#define OV9740_Y_ADDR_END_LO		0x034b
#define OV9740_X_OUTPUT_SIZE_HI		0x034c
#define OV9740_X_OUTPUT_SIZE_LO		0x034d
#define OV9740_Y_OUTPUT_SIZE_HI		0x034e
#define OV9740_Y_OUTPUT_SIZE_LO		0x034f

/* IO Control Registers */
#define OV9740_IO_CREL00		0x3002
#define OV9740_IO_CREL01		0x3004
#define OV9740_IO_CREL02		0x3005
#define OV9740_IO_MIPI_DVP		0x3014
#define OV9740_IO_MIPI_CHAN		0x301f
#define OV9740_IO_OUTPUT_SEL01		0x3026
#define OV9740_IO_OUTPUT_SEL02		0x3027

/* AWB Registers */
#define OV9740_AWB_MANUAL_CTRL		0x3406

/* Analog Control Registers */
#define OV9740_ANALOG_CTRL01		0x3601
#define OV9740_ANALOG_CTRL02		0x3602
#define OV9740_ANALOG_CTRL03		0x3603
#define OV9740_ANALOG_CTRL04		0x3604
#define OV9740_ANALOG_CTRL10		0x3610
#define OV9740_ANALOG_CTRL12		0x3612
#define OV9740_ANALOG_CTRL15		0x3615
#define OV9740_ANALOG_CTRL20		0x3620
#define OV9740_ANALOG_CTRL21		0x3621
#define OV9740_ANALOG_CTRL22		0x3622
#define OV9740_ANALOG_CTRL30		0x3630
#define OV9740_ANALOG_CTRL31		0x3631
#define OV9740_ANALOG_CTRL32		0x3632
#define OV9740_ANALOG_CTRL33		0x3633

/* Sensor Control */
#define OV9740_SENSOR_CTRL03		0x3703
#define OV9740_SENSOR_CTRL04		0x3704
#define OV9740_SENSOR_CTRL05		0x3705
#define OV9740_SENSOR_CTRL07		0x3707

/* Timing Control */
#define OV9740_TIMING_CTRL17		0x3817
#define OV9740_TIMING_CTRL19		0x3819
#define OV9740_TIMING_CTRL33		0x3833
#define OV9740_TIMING_CTRL35		0x3835

/* Banding Filter */
#define OV9740_AEC_MAXEXPO_60_H		0x3a02
#define OV9740_AEC_MAXEXPO_60_L		0x3a03
#define OV9740_AEC_B50_STEP_HI		0x3a08
#define OV9740_AEC_B50_STEP_LO		0x3a09
#define OV9740_AEC_B60_STEP_HI		0x3a0a
#define OV9740_AEC_B60_STEP_LO		0x3a0b
#define OV9740_AEC_CTRL0D		0x3a0d
#define OV9740_AEC_CTRL0E		0x3a0e
#define OV9740_AEC_MAXEXPO_50_H		0x3a14
#define OV9740_AEC_MAXEXPO_50_L		0x3a15

/* AEC/AGC Control */
#define OV9740_AEC_ENABLE		0x3503
#define OV9740_GAIN_CEILING_01		0x3a18
#define OV9740_GAIN_CEILING_02		0x3a19
#define OV9740_AEC_HI_THRESHOLD		0x3a11
#define OV9740_AEC_3A1A			0x3a1a
#define OV9740_AEC_CTRL1B_WPT2		0x3a1b
#define OV9740_AEC_CTRL0F_WPT		0x3a0f
#define OV9740_AEC_CTRL10_BPT		0x3a10
#define OV9740_AEC_CTRL1E_BPT2		0x3a1e
#define OV9740_AEC_LO_THRESHOLD		0x3a1f

/* BLC Control */
#define OV9740_BLC_AUTO_ENABLE		0x4002
#define OV9740_BLC_MODE			0x4005

/* VFIFO */
#define OV9740_VFIFO_READ_START_HI	0x4608
#define OV9740_VFIFO_READ_START_LO	0x4609

/* DVP Control */
#define OV9740_DVP_VSYNC_CTRL02		0x4702
#define OV9740_DVP_VSYNC_MODE		0x4704
#define OV9740_DVP_VSYNC_CTRL06		0x4706

/* PLL Setting */
#define OV9740_PLL_MODE_CTRL01		0x3104
#define OV9740_PRE_PLL_CLK_DIV		0x0305
#define OV9740_PLL_MULTIPLIER		0x0307
#define OV9740_VT_SYS_CLK_DIV		0x0303
#define OV9740_VT_PIX_CLK_DIV		0x0301
#define OV9740_PLL_CTRL3010		0x3010
#define OV9740_VFIFO_CTRL00		0x460e

/* ISP Control */
#define OV9740_ISP_CTRL00		0x5000
#define OV9740_ISP_CTRL01		0x5001
#define OV9740_ISP_CTRL03		0x5003
#define OV9740_ISP_CTRL05		0x5005
#define OV9740_ISP_CTRL12		0x5012
#define OV9740_ISP_CTRL19		0x5019
#define OV9740_ISP_CTRL1A		0x501a
#define OV9740_ISP_CTRL1E		0x501e
#define OV9740_ISP_CTRL1F		0x501f
#define OV9740_ISP_CTRL20		0x5020
#define OV9740_ISP_CTRL21		0x5021

/* AWB */
#define OV9740_AWB_CTRL00		0x5180
#define OV9740_AWB_CTRL01		0x5181
#define OV9740_AWB_CTRL02		0x5182
#define OV9740_AWB_CTRL03		0x5183
#define OV9740_AWB_ADV_CTRL01		0x5184
#define OV9740_AWB_ADV_CTRL02		0x5185
#define OV9740_AWB_ADV_CTRL03		0x5186
#define OV9740_AWB_ADV_CTRL04		0x5187
#define OV9740_AWB_ADV_CTRL05		0x5188
#define OV9740_AWB_ADV_CTRL06		0x5189
#define OV9740_AWB_ADV_CTRL07		0x518a
#define OV9740_AWB_ADV_CTRL08		0x518b
#define OV9740_AWB_ADV_CTRL09		0x518c
#define OV9740_AWB_ADV_CTRL10		0x518d
#define OV9740_AWB_ADV_CTRL11		0x518e
#define OV9740_AWB_CTRL0F		0x518f
#define OV9740_AWB_CTRL10		0x5190
#define OV9740_AWB_CTRL11		0x5191
#define OV9740_AWB_CTRL12		0x5192
#define OV9740_AWB_CTRL13		0x5193
#define OV9740_AWB_CTRL14		0x5194

/* MIPI Control */
#define OV9740_MIPI_CTRL00		0x4800
#define OV9740_MIPI_3837		0x3837
#define OV9740_MIPI_CTRL01		0x4801
#define OV9740_MIPI_CTRL03		0x4803
#define OV9740_MIPI_CTRL05		0x4805
#define OV9740_VFIFO_RD_CTRL		0x4601
#define OV9740_MIPI_CTRL_3012		0x3012
#define OV9740_SC_CMMM_MIPI_CTR		0x3014

#define OV9740_MAX_WIDTH		1280
#define OV9740_MAX_HEIGHT		720

/* Misc. structures */
struct ov9740_reg {
	u16				reg;
	u8				val;
};

struct ov9740_priv {
	struct v4l2_subdev		subdev;

	int				ident;
	u16				model;
	u8				revision;
	u8				manid;
	u8				smiaver;

	bool				flag_vflip;
	bool				flag_hflip;

	/* For suspend/resume. */
	struct v4l2_mbus_framefmt	current_mf;
	bool				current_enable;
};

static const struct ov9740_reg ov9740_mipi_2lane_640_360_60[] = {
//@@ OV9740 MIPI 640x360 Binning at 60FPS in YUV format
// Date		Version		Changes
// 12152010	v05		1) initial
// 12152010	v051		1) adjust contrast threshold 0x5401 = 2c
// 03072011 	v06		1) adjust Gamma & EV level
// 24Mhz input, system clock 76Mhz, MIPI clock 304Mhz
// MIPI output; single lane, Lane 1 output 
// Software RESET
{0x0103, 0x01},	// Software RESET
// Orientation
{0x0101, 0x01},	// Orientation
// PLL setting
{0x3104, 0x20},	// PLL mode control
{0x0305, 0x03},	// PLL control
{0x0307, 0x4c},	// PLL control
{0x0303, 0x01},	// PLL control
{0x0301, 0x08},	// PLL control
{0x3010, 0x01},	// PLL control
{0x300e, 0x12},
// Timing setting
{0x0340, 0x01},	// VTS
{0x0341, 0x84},	// VTS
{0x0342, 0x06},	// HTS
{0x0343, 0x62},	// HTS
{0x0344, 0x00},	// X start
{0x0345, 0x08},	// X start
{0x0346, 0x00},	// Y start
{0x0347, 0x04},	// Y start
{0x0348, 0x04},	// X end
{0x0349, 0xff},	// X end
{0x034a, 0x02},	// Y end
{0x034b, 0xd8}, 	// Y end
{0x034c, 0x02},	// H output size
{0x034d, 0x80},	// H output size
{0x034e, 0x01},	// V output size
{0x034f, 0x68},	// V output size
{0x0383, 0x01},
{0x0387, 0x01},
// Output select 
{0x3002, 0x00},	// IO control
{0x3004, 0x00},	// IO control
{0x3005, 0x00},	// IO control
{0x3012, 0x70},	// MIPI control
{0x3013, 0x60},	// MIPI control
{0x3014, 0x01},	// MIPI control
{0x301f, 0x43},	// MIPI control
{0x3026, 0x00},	// Output select
{0x3027, 0x00},	// Output select
// Analog control
{0x3601, 0x40},	// Analog control
{0x3602, 0x16},	// Analog control
{0x3603, 0xaa},	// Analog control
{0x3604, 0x0c},	// Analog control
{0x3610, 0xa1},	// Analog control
{0x3612, 0x24},	// Analog control
{0x3620, 0x66},	// Analog control
{0x3621, 0xc0},	// Analog control
{0x3622, 0x9f},	// Analog control
{0x3630, 0xd2},	// Analog control
{0x3631, 0x5e},	// Analog control
{0x3632, 0x27},	// Analog control
{0x3633, 0x50},	// Analog control
// Sensor control
{0x3703, 0x42},	// Sensor control 
{0x3704, 0x10},	// Sensor control 
{0x3705, 0x45},	// Sensor control 
{0x3707, 0x13},	// Sensor control 
// Timing control
{0x3833, 0x04},	// Internal timing control
{0x3835, 0x04},	// Internal timing control
{0x3819, 0x6e},	// Internal timing control
{0x3817, 0x94},	// Internal timing control
{0x3831, 0x40},	// Digital gain enable
{0x381a, 0x44},	// Internal timing control
{0x3837, 0x01},	// Internal timing control
// AEC/AGC control
{0x3503, 0x10},	// AEC/AGC control
{0x3a18, 0x01},	// Gain ceiling
{0x3a19, 0xB5},	// Gain ceiling
{0x3a1a, 0x05},	// Max diff
{0x3a11, 0x90}, 	// High threshold
{0x3a1b, 0x4a}, 	// WPT 2 
{0x3a0f, 0x48}, 	// WPT  
{0x3a10, 0x44}, 	// BPT  
{0x3a1e, 0x42}, 	// BPT 2 
{0x3a1f, 0x22},  	// Low threshold 
// Banding filter
{0x3a08, 0x00},	// 50Hz banding step
{0x3a09, 0xe8},	// 50Hz banding step	
{0x3a0e, 0x01},	// 50Hz banding Max
{0x3a14, 0x07},	// 50Hz Max exposure
{0x3a15, 0x40},	// 50Hz Max exposure
{0x3a0a, 0x00},	// 60Hz banding step
{0x3a0b, 0xc0},	// 60Hz banding step
{0x3a0d, 0x02},	// 60Hz banding Max
{0x3a02, 0x0c},	// 60Hz Max exposure
{0x3a03, 0x00},	// 60Hz Max exposure
//50/60 detection
{0x3c0a, 0x9c},	// Number of samples
{0x3c0b, 0x3f},	// Number of samples
// BLC control
{0x4002, 0x45},	// BLC auto enable
{0x4005, 0x08},	// BLC mode
// VFIFO
{0x4601, 0x16},	// VFIFO control
{0x4605, 0x00},	// VFIFO control
{0x4608, 0x01},	// VFIFO control
{0x4609, 0x99},	// VFIFO control
{0x460e, 0x82},	// VFIFO control
// DVP control
{0x4702, 0x04},	// Vsync control
{0x4704, 0x00},	// Vsync mode 
{0x4706, 0x08},	// Vsync control
// MIPI control
{0x4800, 0x44},	// MIPI control
{0x4801, 0x0f},	// MIPI control
{0x4803, 0x05},	// MIPI control
{0x4805, 0x10},	// MIPI control
{0x4837, 0x20},
	// MIPI control
// ISP control
{0x5000, 0xff},	// [7]LC [6]Gamma [3]DNS [2]BPC [1]WPC [0]CIP
{0x5001, 0xff}, // [7]SDE [6]UV adjust [4]scale [3]contrast [2]UV average [1]CMX [0]AWB
{0x5002, 0xcf}, // [7]SDE [6]UV adjust [4]scale [3]contrast [2]UV average [1]CMX [0]AWB
{0x5003, 0xff},	// [7]PAD [5]Buffer [3]Vario [1]BLC [0]AWB gain
//AWB
{0x5180, 0xf0}, 	//AWB setting
{0x5181, 0x00},  	//AWB setting
{0x5182, 0x41}, 	//AWB setting 
{0x5183, 0x42}, 	//AWB setting
{0x5184, 0x80}, 	//AWB setting
{0x5185, 0x68}, 	//AWB setting
{0x5186, 0x93}, 	//AWB setting 
{0x5187, 0xa8}, 	//AWB setting
{0x5188, 0x17}, 	//AWB setting
{0x5189, 0x45}, 	//AWB setting
{0x518a, 0x27}, 	//AWB setting
{0x518b, 0x41}, 	//AWB setting
{0x518c, 0x2d}, 	//AWB setting
{0x518d, 0xf0}, 	//AWB setting
{0x518e, 0x10},  	//AWB setting
{0x518f, 0xff}, 	//AWB setting
{0x5190, 0x00},  	//AWB setting
{0x5191, 0xff}, 	//AWB setting 
{0x5192, 0x00},  	//AWB setting
{0x5193, 0xff}, 	//AWB setting 
{0x5194, 0x00}, 	//AWB setting 
// DNS
{0x529a, 0x02},	//DNS setting
{0x529b, 0x08},	//DNS setting
{0x529c, 0x0a},	//DNS setting
{0x529d, 0x10},	//DNS setting
{0x529e, 0x10}, 	//DNS setting
{0x529f, 0x28},   	//DNS setting
{0x52a0, 0x32}, 	//DNS setting
{0x52a2, 0x00},	//DNS setting 
{0x52a3, 0x02},	//DNS setting 
{0x52a4, 0x00},	//DNS setting 
{0x52a5, 0x04},	//DNS setting  
{0x52a6, 0x00},	//DNS setting  
{0x52a7, 0x08},	//DNS setting  
{0x52a8, 0x00},	//DNS setting  
{0x52a9, 0x10},  	//DNS setting
{0x52aa, 0x00},	//DNS setting  
{0x52ab, 0x38},  	//DNS setting
{0x52ac, 0x00},	//DNS setting  
{0x52ad, 0x3c}, 	//DNS setting 
{0x52ae, 0x00},	//DNS setting   
{0x52af, 0x4c}, 	//DNS setting
//CIP
{0x530d, 0x06},	//CIP setting
//CMX
{0x5380, 0x01}, 	//CMX setting  
{0x5381, 0x00}, 	//CMX setting  
{0x5382, 0x00}, 	//CMX setting   
{0x5383, 0x0d}, 	//CMX setting  
{0x5384, 0x00}, 	//CMX setting  
{0x5385, 0x2f}, 	//CMX setting   
{0x5386, 0x00}, 	//CMX setting  
{0x5387, 0x00}, 	//CMX setting   
{0x5388, 0x00}, 	//CMX setting   
{0x5389, 0xd3}, 	//CMX setting  
{0x538a, 0x00}, 	//CMX setting   
{0x538b, 0x0f}, 	//CMX setting   
{0x538c, 0x00}, 	//CMX setting   
{0x538d, 0x00}, 	//CMX setting  
{0x538e, 0x00}, 	//CMX setting  
{0x538f, 0x32}, 	//CMX setting   
{0x5390, 0x00}, 	//CMX setting  
{0x5391, 0x94}, 	//CMX setting    
{0x5392, 0x00}, 	//CMX setting  
{0x5393, 0xa4}, 	//CMX setting  
{0x5394, 0x18}, 	//CMX setting  
// Contrast
{0x5401, 0x2c},	// Contrast setting
{0x5403, 0x28},	// Contrast setting
{0x5404, 0x06},	// Contrast setting	
{0x5405, 0xe0},	// Contrast setting
//Y Gamma
{0x5480, 0x04}, 	//Y Gamma setting  
{0x5481, 0x12}, 	//Y Gamma setting 
{0x5482, 0x27}, 	//Y Gamma setting  
{0x5483, 0x49}, 	//Y Gamma setting  
{0x5484, 0x57}, 	//Y Gamma setting  
{0x5485, 0x66}, 	//Y Gamma setting  
{0x5486, 0x75}, 	//Y Gamma setting  
{0x5487, 0x81}, 	//Y Gamma setting 
{0x5488, 0x8c}, 	//Y Gamma setting 
{0x5489, 0x95}, 	//Y Gamma setting 
{0x548a, 0xa5}, 	//Y Gamma setting 
{0x548b, 0xb2}, 	//Y Gamma setting 
{0x548c, 0xc8}, 	//Y Gamma setting 
{0x548d, 0xd9}, 	//Y Gamma setting 
{0x548e, 0xec}, 	//Y Gamma setting 
//UV Gamma
{0x5490, 0x01},  	//UV Gamma setting 
{0x5491, 0xc0},  	//UV Gamma setting 
{0x5492, 0x03},  	//UV Gamma setting 
{0x5493, 0x00},  	//UV Gamma setting 
{0x5494, 0x03},  	//UV Gamma setting 
{0x5495, 0xe0},  	//UV Gamma setting 
{0x5496, 0x03},  	//UV Gamma setting 
{0x5497, 0x10},  	//UV Gamma setting 
{0x5498, 0x02},  	//UV Gamma setting 
{0x5499, 0xac},  	//UV Gamma setting 
{0x549a, 0x02},  	//UV Gamma setting 
{0x549b, 0x75},  	//UV Gamma setting 
{0x549c, 0x02},   	//UV Gamma setting 
{0x549d, 0x44},  	//UV Gamma setting 
{0x549e, 0x02},  	//UV Gamma setting 
{0x549f, 0x20},  	//UV Gamma setting 
{0x54a0, 0x02},  	//UV Gamma setting 
{0x54a1, 0x07},  	//UV Gamma setting 
{0x54a2, 0x01},  	//UV Gamma setting 
{0x54a3, 0xec},  	//UV Gamma setting 
{0x54a4, 0x01},  	//UV Gamma setting 
{0x54a5, 0xc0},  	//UV Gamma setting 
{0x54a6, 0x01},  	//UV Gamma setting 
{0x54a7, 0x9b},  	//UV Gamma setting 
{0x54a8, 0x01},  	//UV Gamma setting 
{0x54a9, 0x63},  	//UV Gamma setting 
{0x54aa, 0x01},  	//UV Gamma setting 
{0x54ab, 0x2b},  	//UV Gamma setting 
{0x54ac, 0x01},  	//UV Gamma setting 
{0x54ad, 0x22},  	//UV Gamma setting 
// UV adjust
{0x5501, 0x1c},  	//UV adjust setting 
{0x5502, 0x00},   	//UV adjust setting 
{0x5503, 0x40},  	//UV adjust setting 
{0x5504, 0x00},  	//UV adjust setting 
{0x5505, 0x80},  	//UV adjust setting 
// Lens correction
{0x5800, 0x1c},	// Lens correction setting
{0x5801, 0x16},	// Lens correction setting
{0x5802, 0x15},	// Lens correction setting
{0x5803, 0x16},	// Lens correction setting
{0x5804, 0x18},	// Lens correction setting
{0x5805, 0x1a},	// Lens correction setting
{0x5806, 0x0c},	// Lens correction setting
{0x5807, 0x0a},	// Lens correction setting
{0x5808, 0x08},	// Lens correction setting
{0x5809, 0x08},	// Lens correction setting
{0x580a, 0x0a},	// Lens correction setting
{0x580b, 0x0b},	// Lens correction setting
{0x580c, 0x05},	// Lens correction setting
{0x580d, 0x02},	// Lens correction setting
{0x580e, 0x00},	// Lens correction setting
{0x580f, 0x00},	// Lens correction setting
{0x5810, 0x02},	// Lens correction setting
{0x5811, 0x05},	// Lens correction setting
{0x5812, 0x04},	// Lens correction setting
{0x5813, 0x01},	// Lens correction setting
{0x5814, 0x00},	// Lens correction setting
{0x5815, 0x00},	// Lens correction setting
{0x5816, 0x02},	// Lens correction setting
{0x5817, 0x03},	// Lens correction setting
{0x5818, 0x0a},	// Lens correction setting
{0x5819, 0x07},	// Lens correction setting
{0x581a, 0x05},	// Lens correction setting
{0x581b, 0x05},	// Lens correction setting
{0x581c, 0x08},	// Lens correction setting
{0x581d, 0x0b},	// Lens correction setting
{0x581e, 0x15},	// Lens correction setting
{0x581f, 0x14},	// Lens correction setting
{0x5820, 0x14},	// Lens correction setting
{0x5821, 0x13},	// Lens correction setting
{0x5822, 0x17},	// Lens correction setting
{0x5823, 0x16},	// Lens correction setting
{0x5824, 0x46},	// Lens correction setting
{0x5825, 0x4c},	// Lens correction setting
{0x5826, 0x6c},	// Lens correction setting
{0x5827, 0x4c},	// Lens correction setting
{0x5828, 0x80},	// Lens correction setting
{0x5829, 0x2e},	// Lens correction setting
{0x582a, 0x48},	// Lens correction setting
{0x582b, 0x46},	// Lens correction setting
{0x582c, 0x2a},	// Lens correction setting
{0x582d, 0x68},	// Lens correction setting
{0x582e, 0x08},	// Lens correction setting
{0x582f, 0x26},	// Lens correction setting
{0x5830, 0x44},	// Lens correction setting
{0x5831, 0x46},	// Lens correction setting
{0x5832, 0x62},	// Lens correction setting
{0x5833, 0x0c},	// Lens correction setting
{0x5834, 0x28},	// Lens correction setting
{0x5835, 0x46},	// Lens correction setting
{0x5836, 0x28},	// Lens correction setting
{0x5837, 0x88},	// Lens correction setting
{0x5838, 0x0e},	// Lens correction setting
{0x5839, 0x0e},	// Lens correction setting
{0x583a, 0x2c},	// Lens correction setting
{0x583b, 0x2e},	// Lens correction setting
{0x583c, 0x46},	// Lens correction setting
{0x583d, 0xca},	// Lens correction setting
{0x583e, 0xf0},	// Lens correction setting
{0x5842, 0x02},	// Lens correction setting
{0x5843, 0x5e},	// Lens correction setting
{0x5844, 0x04},	// Lens correction setting
{0x5845, 0x32},	// Lens correction setting
{0x5846, 0x03},	// Lens correction setting
{0x5847, 0x29},	// Lens correction setting
{0x5848, 0x02},	// Lens correction setting
{0x5849, 0xcc},	// Lens correction setting
// Start streaming
{0x0100, 0x01},	// start streaming 
};

static const struct ov9740_reg ov9740_mipi_2lane_640_480_30[] = {
// @@ OV9740 MIPI 640x480 scaling at 30FPS in YUV format
// Date		Version		Changes
// 12152010	v05		1) initial
// 12152010	v051		1) adjust contrast threshold 0x5401 = 2c
// 03072011 	v06		1) adjust Gamma & EV level
// 24Mhz input, system clock 76Mhz, MIPI clock 304Mhz
// MIPI output; single lane, Lane 1 output 
// Initial sequence begin
// Software RESET
{0x0103, 0x01},	// Software RESET
// Orientation
{0x0101, 0x01},	// Orientation
// PLL setting
{0x3104, 0x20},	// PLL mode control
{0x0305, 0x03},	// PLL control
{0x0307, 0x4c},	// PLL control
{0x0303, 0x01},	// PLL control
{0x0301, 0x08},	// PLL control
{0x3010, 0x01},	// PLL control
{0x300e, 0x12},
// Timing setting
{0x0340, 0x03},	// VTS
{0x0341, 0x07},	// VTS
{0x0342, 0x06},	// HTS
{0x0343, 0x62},	// HTS
{0x0344, 0x00},	// X start
{0x0345, 0xa8},	// X start
{0x0346, 0x00},	// Y start
{0x0347, 0x04},	// Y start
{0x0348, 0x04},	// X end
{0x0349, 0x67},	// X end
{0x034a, 0x02},	// Y end
{0x034b, 0xd8}, 	// Y end
{0x034c, 0x02},	// H output size
{0x034d, 0x80},	// H output size
{0x034e, 0x01},	// V output size
{0x034f, 0xe0},	// V output size
// Output select 
{0x3002, 0x00},	// IO control
{0x3004, 0x00},	// IO control
{0x3005, 0x00},	// IO control
{0x3012, 0x70},	// MIPI control
{0x3013, 0x60},	// MIPI control
{0x3014, 0x01},	// MIPI control
{0x301f, 0x43},	// MIPI control
{0x3026, 0x00},	// Output select
{0x3027, 0x00},	// Output select
// Analog control
{0x3601, 0x40},	// Analog control
{0x3602, 0x16},	// Analog control
{0x3603, 0xaa},	// Analog control
{0x3604, 0x0c},	// Analog control
{0x3610, 0xa1},	// Analog control
{0x3612, 0x24},	// Analog control
{0x3620, 0x66},	// Analog control
{0x3621, 0xc0},	// Analog control
{0x3622, 0x9f},	// Analog control
{0x3630, 0xca},	// Analog control
{0x3631, 0x52},	// Analog control
{0x3632, 0x2f},	// Analog control
{0x3633, 0x50},	// Analog control
// Sensor control
{0x3703, 0x42},	// Sensor control 
{0x3704, 0x10},	// Sensor control 
{0x3705, 0x45},	// Sensor control 
{0x3707, 0x11},	// Sensor control 
// Timing control
{0x3833, 0x04},	// Internal timing control
{0x3835, 0x04},	// Internal timing control
{0x3819, 0x6e},	// Internal timing control
{0x3817, 0x94},	// Internal timing control
{0x3831, 0x40},	// Digital gain enable
{0x3837, 0x01},	// Internal timing control
// AEC/AGC control
{0x3503, 0x10},	// AEC/AGC control
{0x3a18, 0x01},	// Gain ceiling
{0x3a19, 0xB5},	// Gain ceiling
{0x3a1a, 0x05},	// Max diff
{0x3a11, 0x90}, 	// High threshold
{0x3a1b, 0x4a}, 	// WPT 2 
{0x3a0f, 0x48}, 	// WPT  
{0x3a10, 0x44}, 	// BPT  
{0x3a1e, 0x42}, 	// BPT 2 
{0x3a1f, 0x22},  	// Low threshold 
// Banding filter
{0x3a08, 0x00},	// 50Hz banding step
{0x3a09, 0xe8},	// 50Hz banding step	
{0x3a0e, 0x03},	// 50Hz banding Max
{0x3a14, 0x15},	// 50Hz Max exposure
{0x3a15, 0xc6},	// 50Hz Max exposure
{0x3a0a, 0x00},	// 60Hz banding step
{0x3a0b, 0xc0},	// 60Hz banding step
{0x3a0d, 0x04},	// 60Hz banding Max
{0x3a02, 0x18},	// 60Hz Max exposure
{0x3a03, 0x20},	// 60Hz Max exposure
//50/60 detection
{0x3c0a, 0x9c},	// Number of samples
{0x3c0b, 0x3f},	// Number of samples
// BLC control
{0x4002, 0x45},	// BLC auto enable
{0x4005, 0x18},	// BLC mode
// VFIFO
{0x4601, 0x16},	// VFIFO control
{0x4608, 0x02},	// VFIFO control
{0x4609, 0x70},	// VFIFO control
{0x460e, 0x82},	// VFIFO control
// DVP control
{0x4702, 0x04},	// Vsync control
{0x4704, 0x00},	// Vsync mode 
{0x4706, 0x08},	// Vsync control
// MIPI control
{0x4800, 0x44},	// MIPI control
{0x4801, 0x0f},	// MIPI control
{0x4803, 0x05},	// MIPI control
{0x4805, 0x10},	// MIPI control
{0x4837, 0x20},
	// MIPI control
// ISP control
{0x5000, 0xff},	// [7]LC [6]Gamma [3]DNS [2]BPC [1]WPC [0]CIP
{0x5001, 0xff},	// [7]SDE [6]UV adjust [4]scale [3]contrast [2]UV average [1]CMX [0]AWB
{0x5003, 0xff},	// [7]PAD [5]Buffer [3]Vario [1]BLC [0]AWB gain
// Scaling
{0x501e, 0x03},	// Scale X input
{0x501f, 0xc0}, 	// Scale X input
{0x5020, 0x02},	// Scale Y input
{0x5021, 0xd0}, 	// Scale Y input
//AWB
{0x5180, 0xf0}, 	//AWB setting
{0x5181, 0x00},  	//AWB setting
{0x5182, 0x41}, 	//AWB setting 
{0x5183, 0x42}, 	//AWB setting
{0x5184, 0x80}, 	//AWB setting
{0x5185, 0x68}, 	//AWB setting
{0x5186, 0x93}, 	//AWB setting 
{0x5187, 0xa8}, 	//AWB setting
{0x5188, 0x17}, 	//AWB setting
{0x5189, 0x45}, 	//AWB setting
{0x518a, 0x27}, 	//AWB setting
{0x518b, 0x41}, 	//AWB setting
{0x518c, 0x2d}, 	//AWB setting
{0x518d, 0xf0}, 	//AWB setting
{0x518e, 0x10},  	//AWB setting
{0x518f, 0xff}, 	//AWB setting
{0x5190, 0x00},  	//AWB setting
{0x5191, 0xff}, 	//AWB setting 
{0x5192, 0x00},  	//AWB setting
{0x5193, 0xff}, 	//AWB setting 
{0x5194, 0x00}, 	//AWB setting 
// DNS
{0x529a, 0x02},	//DNS setting
{0x529b, 0x08},	//DNS setting
{0x529c, 0x0a},	//DNS setting
{0x529d, 0x10},	//DNS setting
{0x529e, 0x10}, 	//DNS setting
{0x529f, 0x28},   	//DNS setting
{0x52a0, 0x32}, 	//DNS setting
{0x52a2, 0x00},	//DNS setting 
{0x52a3, 0x02},	//DNS setting 
{0x52a4, 0x00},	//DNS setting 
{0x52a5, 0x04},	//DNS setting  
{0x52a6, 0x00},	//DNS setting  
{0x52a7, 0x08},	//DNS setting  
{0x52a8, 0x00},	//DNS setting  
{0x52a9, 0x10},  	//DNS setting
{0x52aa, 0x00},	//DNS setting  
{0x52ab, 0x38},  	//DNS setting
{0x52ac, 0x00},	//DNS setting  
{0x52ad, 0x3c}, 	//DNS setting 
{0x52ae, 0x00},	//DNS setting   
{0x52af, 0x4c}, 	//DNS setting
//CIP
{0x530d, 0x06},	//CIP setting
//CMX
{0x5380, 0x01}, 	//CMX setting  
{0x5381, 0x00}, 	//CMX setting  
{0x5382, 0x00}, 	//CMX setting   
{0x5383, 0x0d}, 	//CMX setting  
{0x5384, 0x00}, 	//CMX setting  
{0x5385, 0x2f}, 	//CMX setting   
{0x5386, 0x00}, 	//CMX setting  
{0x5387, 0x00}, 	//CMX setting   
{0x5388, 0x00}, 	//CMX setting   
{0x5389, 0xd3}, 	//CMX setting  
{0x538a, 0x00}, 	//CMX setting   
{0x538b, 0x0f}, 	//CMX setting   
{0x538c, 0x00}, 	//CMX setting   
{0x538d, 0x00}, 	//CMX setting  
{0x538e, 0x00}, 	//CMX setting  
{0x538f, 0x32}, 	//CMX setting   
{0x5390, 0x00}, 	//CMX setting  
{0x5391, 0x94}, 	//CMX setting    
{0x5392, 0x00}, 	//CMX setting  
{0x5393, 0xa4}, 	//CMX setting  
{0x5394, 0x18}, 	//CMX setting  
// Contrast
{0x5401, 0x2c},	// Contrast setting
{0x5403, 0x28},	// Contrast setting
{0x5404, 0x06},	// Contrast setting	
{0x5405, 0xe0},	// Contrast setting
//Y Gamma
{0x5480, 0x04}, 	//Y Gamma setting  
{0x5481, 0x12}, 	//Y Gamma setting 
{0x5482, 0x27}, 	//Y Gamma setting  
{0x5483, 0x49}, 	//Y Gamma setting  
{0x5484, 0x57}, 	//Y Gamma setting  
{0x5485, 0x66}, 	//Y Gamma setting  
{0x5486, 0x75}, 	//Y Gamma setting  
{0x5487, 0x81}, 	//Y Gamma setting 
{0x5488, 0x8c}, 	//Y Gamma setting 
{0x5489, 0x95}, 	//Y Gamma setting 
{0x548a, 0xa5}, 	//Y Gamma setting 
{0x548b, 0xb2}, 	//Y Gamma setting 
{0x548c, 0xc8}, 	//Y Gamma setting 
{0x548d, 0xd9}, 	//Y Gamma setting 
{0x548e, 0xec}, 	//Y Gamma setting 
//UV Gamma
{0x5490, 0x01},  	//UV Gamma setting 
{0x5491, 0xc0},  	//UV Gamma setting 
{0x5492, 0x03},  	//UV Gamma setting 
{0x5493, 0x00},  	//UV Gamma setting 
{0x5494, 0x03},  	//UV Gamma setting 
{0x5495, 0xe0},  	//UV Gamma setting 
{0x5496, 0x03},  	//UV Gamma setting 
{0x5497, 0x10},  	//UV Gamma setting 
{0x5498, 0x02},  	//UV Gamma setting 
{0x5499, 0xac},  	//UV Gamma setting 
{0x549a, 0x02},  	//UV Gamma setting 
{0x549b, 0x75},  	//UV Gamma setting 
{0x549c, 0x02},   	//UV Gamma setting 
{0x549d, 0x44},  	//UV Gamma setting 
{0x549e, 0x02},  	//UV Gamma setting 
{0x549f, 0x20},  	//UV Gamma setting 
{0x54a0, 0x02},  	//UV Gamma setting 
{0x54a1, 0x07},  	//UV Gamma setting 
{0x54a2, 0x01},  	//UV Gamma setting 
{0x54a3, 0xec},  	//UV Gamma setting 
{0x54a4, 0x01},  	//UV Gamma setting 
{0x54a5, 0xc0},  	//UV Gamma setting 
{0x54a6, 0x01},  	//UV Gamma setting 
{0x54a7, 0x9b},  	//UV Gamma setting 
{0x54a8, 0x01},  	//UV Gamma setting 
{0x54a9, 0x63},  	//UV Gamma setting 
{0x54aa, 0x01},  	//UV Gamma setting 
{0x54ab, 0x2b},  	//UV Gamma setting 
{0x54ac, 0x01},  	//UV Gamma setting 
{0x54ad, 0x22},  	//UV Gamma setting 
// UV adjust
{0x5501, 0x1c},  	//UV adjust setting 
{0x5502, 0x00},   	//UV adjust setting 
{0x5503, 0x40},  	//UV adjust setting 
{0x5504, 0x00},  	//UV adjust setting 
{0x5505, 0x80},  	//UV adjust setting 
// Lens correction
{0x5800, 0x1c},	// Lens correction setting
{0x5801, 0x16},	// Lens correction setting
{0x5802, 0x15},	// Lens correction setting
{0x5803, 0x16},	// Lens correction setting
{0x5804, 0x18},	// Lens correction setting
{0x5805, 0x1a},	// Lens correction setting
{0x5806, 0x0c},	// Lens correction setting
{0x5807, 0x0a},	// Lens correction setting
{0x5808, 0x08},	// Lens correction setting
{0x5809, 0x08},	// Lens correction setting
{0x580a, 0x0a},	// Lens correction setting
{0x580b, 0x0b},	// Lens correction setting
{0x580c, 0x05},	// Lens correction setting
{0x580d, 0x02},	// Lens correction setting
{0x580e, 0x00},	// Lens correction setting
{0x580f, 0x00},	// Lens correction setting
{0x5810, 0x02},	// Lens correction setting
{0x5811, 0x05},	// Lens correction setting
{0x5812, 0x04},	// Lens correction setting
{0x5813, 0x01},	// Lens correction setting
{0x5814, 0x00},	// Lens correction setting
{0x5815, 0x00},	// Lens correction setting
{0x5816, 0x02},	// Lens correction setting
{0x5817, 0x03},	// Lens correction setting
{0x5818, 0x0a},	// Lens correction setting
{0x5819, 0x07},	// Lens correction setting
{0x581a, 0x05},	// Lens correction setting
{0x581b, 0x05},	// Lens correction setting
{0x581c, 0x08},	// Lens correction setting
{0x581d, 0x0b},	// Lens correction setting
{0x581e, 0x15},	// Lens correction setting
{0x581f, 0x14},	// Lens correction setting
{0x5820, 0x14},	// Lens correction setting
{0x5821, 0x13},	// Lens correction setting
{0x5822, 0x17},	// Lens correction setting
{0x5823, 0x16},	// Lens correction setting
{0x5824, 0x46},	// Lens correction setting
{0x5825, 0x4c},	// Lens correction setting
{0x5826, 0x6c},	// Lens correction setting
{0x5827, 0x4c},	// Lens correction setting
{0x5828, 0x80},	// Lens correction setting
{0x5829, 0x2e},	// Lens correction setting
{0x582a, 0x48},	// Lens correction setting
{0x582b, 0x46},	// Lens correction setting
{0x582c, 0x2a},	// Lens correction setting
{0x582d, 0x68},	// Lens correction setting
{0x582e, 0x08},	// Lens correction setting
{0x582f, 0x26},	// Lens correction setting
{0x5830, 0x44},	// Lens correction setting
{0x5831, 0x46},	// Lens correction setting
{0x5832, 0x62},	// Lens correction setting
{0x5833, 0x0c},	// Lens correction setting
{0x5834, 0x28},	// Lens correction setting
{0x5835, 0x46},	// Lens correction setting
{0x5836, 0x28},	// Lens correction setting
{0x5837, 0x88},	// Lens correction setting
{0x5838, 0x0e},	// Lens correction setting
{0x5839, 0x0e},	// Lens correction setting
{0x583a, 0x2c},	// Lens correction setting
{0x583b, 0x2e},	// Lens correction setting
{0x583c, 0x46},	// Lens correction setting
{0x583d, 0xca},	// Lens correction setting
{0x583e, 0xf0},	// Lens correction setting
{0x5842, 0x02},	// Lens correction setting
{0x5843, 0x5e},	// Lens correction setting
{0x5844, 0x04},	// Lens correction setting
{0x5845, 0x32},	// Lens correction setting
{0x5846, 0x03},	// Lens correction setting
{0x5847, 0x29},	// Lens correction setting
{0x5848, 0x02},	// Lens correction setting
{0x5849, 0xcc},	// Lens correction setting
// Start streaming
{0x0100, 0x01},	// start streaming
};

static const struct ov9740_reg ov9740_mipi_1lane_1280_720_30[] = {
// OV9740 MIPI 1280x720 Full at 30FPS in YUV format
// 24Mhz input, system clock 76Mhz, MIPI clock 304Mhz
// MIPI output; single lane, Lane 1 output


{0x0103, 0x01},      // Software RESET

// Orientation
{0x0101, 0x10},      // Orientation

// PLL setting
{0x3104, 0x20},      // PLL mode control
{0x0305, 0x03},      // PLL control
{0x0307, 0x4c},      // PLL control
{0x0303, 0x01},      // PLL control
{0x0301, 0x08},      // PLL control
{0x3010, 0x01},      // PLL control

{0x4300, 0x30},

//	{0x3104, 0x20},
//	{0x0305, 0x04},
//	{0x0307, 0x46},
//	{0x0303, 0x01},
//	{0x0301, 0x0a},
//	{0x3010, 0x01},


// Timing setting
{0x0340, 0x03},      // VTS
{0x0341, 0x07},      // VTS
{0x0342, 0x06},      // HTS
{0x0343, 0x62},      // HTS
{0x0344, 0x00},      // X start
{0x0345, 0x08},      // X start
{0x0346, 0x00},      // Y start
{0x0347, 0x04},      // Y start
{0x0348, 0x05},      // X end
{0x0349, 0x0c},      // X end
{0x034a, 0x02},      // Y end
{0x034b, 0xd8},      // Y end
{0x034c, 0x05},      // H output size
{0x034d, 0x00},      // H output size
{0x034e, 0x02},      // V output size
{0x034f, 0xd0},      // V output size

// Output select 
{0x3002, 0x00},      // IO control
{0x3004, 0x00},      // IO control
{0x3005, 0x00},      // IO control
{0x3012, 0x70},      // MIPI control
{0x3013, 0x60},      // MIPI control
{0x3014, 0x01},      // MIPI control
{0x301f, 0x43},      // MIPI control
{0x3026, 0x00},      // Output select
{0x3027, 0x00},      // Output select

// Analog control
{0x3601, 0x40},      // Analog control
{0x3602, 0x16},      // Analog control
{0x3603, 0xaa},      // Analog control
{0x3604, 0x0c},      // Analog control
{0x3610, 0xa1},      // Analog control
{0x3612, 0x24},      // Analog control
{0x3620, 0x66},      // Analog control
{0x3621, 0xc0},      // Analog control
{0x3622, 0x9f},      // Analog control
{0x3630, 0xd2},      // Analog control
{0x3631, 0x5e},      // Analog control
{0x3632, 0x27},      // Analog control
{0x3633, 0x50},      // Analog control

// Sensor control
{0x3703, 0x42},      // Sensor control 
{0x3704, 0x10},      // Sensor control 
{0x3705, 0x45},      // Sensor control 
{0x3707, 0x11},      // Sensor control 

// Timing control
{0x3817, 0x94},      // Internal timing control
{0x3819, 0x6e},      // Internal timing control
{0x3831, 0x40},      // Digital gain enable
{0x3833, 0x04},      // Internal timing control
{0x3835, 0x04},      // Internal timing control
{0x3837, 0x01},    // Internal timing control

// AEC/AGC control
{0x3503, 0x10},      // AEC/AGC control
{0x3a18, 0x01},      // Gain ceiling
{0x3a19, 0xB5},      // Gain ceiling
{0x3a1a, 0x05},      // Max diff
{0x3a11, 0x90},      // High threshold
{0x3a1b, 0x4a},      // WPT 2 
{0x3a0f, 0x48},      // WPT  
{0x3a10, 0x44},      // BPT  
{0x3a1e, 0x42},      // BPT 2 
{0x3a1f, 0x22},      // Low threshold 

// Banding filter

{0x3a08, 0x00},      // 50Hz banding step
{0x3a09, 0xe8},      // 50Hz banding step    
{0x3a0e, 0x03},      // 50Hz banding Max
{0x3a14, 0x15},      // 50Hz Max exposure
{0x3a15, 0xc6},      // 50Hz Max exposure
{0x3a0a, 0x00},      // 60Hz banding step
{0x3a0b, 0xc0},      // 60Hz banding step
{0x3a0d, 0x04},      // 60Hz banding Max
{0x3a02, 0x18},      // 60Hz Max exposure
{0x3a03, 0x20},      // 60Hz Max exposure

//50/60 detection
{0x3c0a, 0x9c},      // Number of samples
{0x3c0b, 0x3f},      // Number of samples
{0x3c01, 0x80},      // 50-60Hz manual
{0x3c0c, 0x00},      // 60Hz

// BLC control
{0x4002, 0x45},      // BLC auto enable
{0x4005, 0x18},      // BLC mode

// VFIFO control
{0x4601, 0x16},      // VFIFO control
{0x460e, 0x82},      // VFIFO control

// DVP control
{0x4702, 0x04},      // Vsync control
{0x4704, 0x00},      // Vsync mode 
{0x4706, 0x08},      // Vsync control

// MIPI control
{0x4800, 0x44},      // MIPI control
{0x4801, 0x0f},    // MIPI control
{0x4803, 0x05},    // MIPI control
{0x4805, 0x10},    // MIPI control
{0x4837, 0x20},    // MIPI control

// ISP control
{0x5000, 0xff},      // [7] LC [6] Gamma [3] DNS [2] BPC [1] WPC [0] CIP
{0x5001, 0xff},      // [7] SDE [6] UV adjust [4] scale [3] contrast [2] UV average [1] CMX [0] AWB
{0x5003, 0xff},      // [7] PAD [5] Buffer [3] Vario [1] BLC [0] AWB gain

//Color bar
{0x501a, 0x00},
//{0x5004, 0x00},
{0x0601, 0x00},

//AWB
{0x5180, 0xf0},      //AWB setting

{0x5181, 0x00},      //AWB setting
{0x5182, 0x41},      //AWB setting 
{0x5183, 0x42},      //AWB setting
{0x5184, 0x80},      //AWB setting
{0x5185, 0x68},      //AWB setting
{0x5186, 0x93},      //AWB setting 
{0x5187, 0xa8},      //AWB setting
{0x5188, 0x17},      //AWB setting
{0x5189, 0x45},      //AWB setting
{0x518a, 0x27},      //AWB setting
{0x518b, 0x41},      //AWB setting
{0x518c, 0x2d},      //AWB setting
{0x518d, 0xf0},      //AWB setting
{0x518e, 0x10},      //AWB setting
{0x518f, 0xff},      //AWB setting
{0x5190, 0x0 },      //AWB setting
{0x5191, 0xff},      //AWB setting 
{0x5192, 0x00},      //AWB setting
{0x5193, 0xff},      //AWB setting 
{0x5194, 0x00},      //AWB setting 

// DNS
{0x529a, 0x02},      //DNS setting
{0x529b, 0x08},      //DNS setting
{0x529c, 0x0a},      //DNS setting
{0x529d, 0x10},      //DNS setting
{0x529e, 0x10},      //DNS setting
{0x529f, 0x28},      //DNS setting
{0x52a0, 0x32},      //DNS setting
{0x52a2, 0x00},      //DNS setting 
{0x52a3, 0x02},      //DNS setting 
{0x52a4, 0x00},      //DNS setting 
{0x52a5, 0x04},      //DNS setting  
{0x52a6, 0x00},      //DNS setting  
{0x52a7, 0x08},      //DNS setting  
{0x52a8, 0x00},      //DNS setting  
{0x52a9, 0x10},      //DNS setting
{0x52aa, 0x00},      //DNS setting  
{0x52ab, 0x38},      //DNS setting
{0x52ac, 0x00},      //DNS setting  
{0x52ad, 0x3c},      //DNS setting 
{0x52ae, 0x00},      //DNS setting   

{0x52af, 0x4c},      //DNS setting

//CIP
{0x530d, 0x06},      //CIP setting

//CMX
{0x5380, 0x01},      //CMX setting  
{0x5381, 0x00},      //CMX setting  
{0x5382, 0x00},      //CMX setting   
{0x5383, 0x0d},      //CMX setting  
{0x5384, 0x00},      //CMX setting  
{0x5385, 0x2f},      //CMX setting   
{0x5386, 0x00},      //CMX setting  
{0x5387, 0x00},      //CMX setting   
{0x5388, 0x00},      //CMX setting   
{0x5389, 0xd3},      //CMX setting  
{0x538a, 0x00},      //CMX setting   
{0x538b, 0x0f},      //CMX setting   
{0x538c, 0x00},      //CMX setting   
{0x538d, 0x00},      //CMX setting  
{0x538e, 0x00},      //CMX setting  
{0x538f, 0x32},      //CMX setting   
{0x5390, 0x00},      //CMX setting  
{0x5391, 0x94},      //CMX setting    
{0x5392, 0x00},      //CMX setting  
{0x5393, 0xa4},      //CMX setting  
{0x5394, 0x18},      //CMX setting  

// Contrast
{0x5401, 0x2c},      // Contrast setting
{0x5403, 0x28},      // Contrast setting
{0x5404, 0x06},      // Contrast setting     
{0x5405, 0xe0},      // Contrast setting

//Y Gamma
{0x5480, 0x04},      //Y Gamma setting  
{0x5481, 0x12},      //Y Gamma setting 
{0x5482, 0x27},      //Y Gamma setting  
{0x5483, 0x49},      //Y Gamma setting  
{0x5484, 0x57},      //Y Gamma setting  
{0x5485, 0x66},      //Y Gamma setting  
{0x5486, 0x75},      //Y Gamma setting  

{0x5487, 0x81},      //Y Gamma setting 
{0x5488, 0x8c},      //Y Gamma setting 
{0x5489, 0x95},      //Y Gamma setting 
{0x548a, 0xa5},      //Y Gamma setting 
{0x548b, 0xb2},      //Y Gamma setting 
{0x548c, 0xc8},      //Y Gamma setting 
{0x548d, 0xd9},      //Y Gamma setting 
{0x548e, 0xec},      //Y Gamma setting 

//UV Gamma
{0x5490, 0x01},      //UV Gamma setting 
{0x5491, 0xc0},      //UV Gamma setting 
{0x5492, 0x03},      //UV Gamma setting 
{0x5493, 0x00},      //UV Gamma setting 
{0x5494, 0x03},      //UV Gamma setting 
{0x5495, 0xe0},      //UV Gamma setting 
{0x5496, 0x03},      //UV Gamma setting 
{0x5497, 0x10},      //UV Gamma setting 
{0x5498, 0x02},      //UV Gamma setting 
{0x5499, 0xac},      //UV Gamma setting 
{0x549a, 0x02},      //UV Gamma setting 
{0x549b, 0x75},      //UV Gamma setting 
{0x549c, 0x02},      //UV Gamma setting 
{0x549d, 0x44},      //UV Gamma setting 
{0x549e, 0x02},      //UV Gamma setting 
{0x549f, 0x20},      //UV Gamma setting 
{0x54a0, 0x02},      //UV Gamma setting 
{0x54a1, 0x07},      //UV Gamma setting 
{0x54a2, 0x01},      //UV Gamma setting 
{0x54a3, 0xec},      //UV Gamma setting 
{0x54a4, 0x01},      //UV Gamma setting 
{0x54a5, 0xc0},      //UV Gamma setting 
{0x54a6, 0x01},      //UV Gamma setting 
{0x54a7, 0x9b},      //UV Gamma setting 
{0x54a8, 0x01},      //UV Gamma setting 
{0x54a9, 0x63},      //UV Gamma setting 
{0x54aa, 0x01},      //UV Gamma setting 
{0x54ab, 0x2b},      //UV Gamma setting 
{0x54ac, 0x01},      //UV Gamma setting 
{0x54ad, 0x22},      //UV Gamma setting 
 
// UV adjust

{0x5501, 0x1c},      //UV adjust setting 
{0x5502, 0x00},      //UV adjust setting 
{0x5503, 0x40},      //UV adjust setting 
{0x5504, 0x00},      //UV adjust setting 
{0x5505, 0x80},      //UV adjust setting 

// Lens correction
{0x5800, 0x1c},      // Lens correction setting
{0x5801, 0x16},      // Lens correction setting
{0x5802, 0x15},      // Lens correction setting
{0x5803, 0x16},      // Lens correction setting
{0x5804, 0x18},      // Lens correction setting
{0x5805, 0x1a},      // Lens correction setting
{0x5806, 0x0c},      // Lens correction setting
{0x5807, 0x0a},      // Lens correction setting
{0x5808, 0x08},      // Lens correction setting
{0x5809, 0x08},      // Lens correction setting
{0x580a, 0x0a},      // Lens correction setting
{0x580b, 0x0b},      // Lens correction setting
{0x580c, 0x05},      // Lens correction setting
{0x580d, 0x02},      // Lens correction setting
{0x580e, 0x00},      // Lens correction setting
{0x580f, 0x00},      // Lens correction setting
{0x5810, 0x02},      // Lens correction setting
{0x5811, 0x05},      // Lens correction setting
{0x5812, 0x04},      // Lens correction setting
{0x5813, 0x01},      // Lens correction setting
{0x5814, 0x00},      // Lens correction setting
{0x5815, 0x00},      // Lens correction setting
{0x5816, 0x02},      // Lens correction setting
{0x5817, 0x03},      // Lens correction setting
{0x5818, 0x0a},      // Lens correction setting
{0x5819, 0x07},      // Lens correction setting
{0x581a, 0x05},      // Lens correction setting
{0x581b, 0x05},      // Lens correction setting
{0x581c, 0x08},      // Lens correction setting
{0x581d, 0x0b},      // Lens correction setting
{0x581e, 0x15},      // Lens correction setting
{0x581f, 0x14},      // Lens correction setting
{0x5820, 0x14},      // Lens correction setting
{0x5821, 0x13},      // Lens correction setting
{0x5822, 0x17},      // Lens correction setting

{0x5823, 0x16},      // Lens correction setting
{0x5824, 0x46},      // Lens correction setting
{0x5825, 0x4c},      // Lens correction setting
{0x5826, 0x6c},      // Lens correction setting
{0x5827, 0x4c},      // Lens correction setting
{0x5828, 0x80},      // Lens correction setting
{0x5829, 0x2e},      // Lens correction setting
{0x582a, 0x48},      // Lens correction setting
{0x582b, 0x46},      // Lens correction setting
{0x582c, 0x2a},      // Lens correction setting
{0x582d, 0x68},      // Lens correction setting
{0x582e, 0x08},      // Lens correction setting
{0x582f, 0x26},      // Lens correction setting
{0x5830, 0x44},      // Lens correction setting
{0x5831, 0x46},      // Lens correction setting
{0x5832, 0x62},      // Lens correction setting
{0x5833, 0x0c},      // Lens correction setting
{0x5834, 0x28},      // Lens correction setting
{0x5835, 0x46},      // Lens correction setting
{0x5836, 0x28},      // Lens correction setting
{0x5837, 0x88},      // Lens correction setting
{0x5838, 0x0e},      // Lens correction setting
{0x5839, 0x0e},      // Lens correction setting
{0x583a, 0x2c},      // Lens correction setting
{0x583b, 0x2e},      // Lens correction setting
{0x583c, 0x46},      // Lens correction setting
{0x583d, 0xca},      // Lens correction setting
{0x583e, 0xf0},      // Lens correction setting
{0x5842, 0x02},      // Lens correction setting
{0x5843, 0x5e},      // Lens correction setting
{0x5844, 0x04},      // Lens correction setting
{0x5845, 0x32},      // Lens correction setting
{0x5846, 0x03},      // Lens correction setting
{0x5847, 0x29},      // Lens correction setting
{0x5848, 0x02},      // Lens correction setting
{0x5849, 0xcc},      // Lens correction setting

{ OV9740_ISP_CTRL19,		0x02 },

// Start streaming
//{0x0100, 0x01},      // start streaming


}; 
static const struct ov9740_reg ov9740_defaults[] = {
	/* Software Reset */
	{ OV9740_SOFTWARE_RESET,	0x01 },

	/* Banding Filter */
	{ OV9740_AEC_B50_STEP_HI,	0x00 },
	{ OV9740_AEC_B50_STEP_LO,	0xe8 },
	{ OV9740_AEC_CTRL0E,		0x03 },
	{ OV9740_AEC_MAXEXPO_50_H,	0x15 },
	{ OV9740_AEC_MAXEXPO_50_L,	0xc6 },
	{ OV9740_AEC_B60_STEP_HI,	0x00 },
	{ OV9740_AEC_B60_STEP_LO,	0xc0 },
	{ OV9740_AEC_CTRL0D,		0x04 },
	{ OV9740_AEC_MAXEXPO_60_H,	0x18 },
	{ OV9740_AEC_MAXEXPO_60_L,	0x20 },

	/* LC */
	{ 0x5842, 0x02 }, { 0x5843, 0x5e }, { 0x5844, 0x04 }, { 0x5845, 0x32 },
	{ 0x5846, 0x03 }, { 0x5847, 0x29 }, { 0x5848, 0x02 }, { 0x5849, 0xcc },

	/* Un-documented OV9740 registers */
	{ 0x5800, 0x29 }, { 0x5801, 0x25 }, { 0x5802, 0x20 }, { 0x5803, 0x21 },
	{ 0x5804, 0x26 }, { 0x5805, 0x2e }, { 0x5806, 0x11 }, { 0x5807, 0x0c },
	{ 0x5808, 0x09 }, { 0x5809, 0x0a }, { 0x580a, 0x0e }, { 0x580b, 0x16 },
	{ 0x580c, 0x06 }, { 0x580d, 0x02 }, { 0x580e, 0x00 }, { 0x580f, 0x00 },
	{ 0x5810, 0x04 }, { 0x5811, 0x0a }, { 0x5812, 0x05 }, { 0x5813, 0x02 },
	{ 0x5814, 0x00 }, { 0x5815, 0x00 }, { 0x5816, 0x03 }, { 0x5817, 0x09 },
	{ 0x5818, 0x0f }, { 0x5819, 0x0a }, { 0x581a, 0x07 }, { 0x581b, 0x08 },
	{ 0x581c, 0x0b }, { 0x581d, 0x14 }, { 0x581e, 0x28 }, { 0x581f, 0x23 },
	{ 0x5820, 0x1d }, { 0x5821, 0x1e }, { 0x5822, 0x24 }, { 0x5823, 0x2a },
	{ 0x5824, 0x4f }, { 0x5825, 0x6f }, { 0x5826, 0x5f }, { 0x5827, 0x7f },
	{ 0x5828, 0x9f }, { 0x5829, 0x5f }, { 0x582a, 0x8f }, { 0x582b, 0x9e },
	{ 0x582c, 0x8f }, { 0x582d, 0x9f }, { 0x582e, 0x4f }, { 0x582f, 0x87 },
	{ 0x5830, 0x86 }, { 0x5831, 0x97 }, { 0x5832, 0xae }, { 0x5833, 0x3f },
	{ 0x5834, 0x8e }, { 0x5835, 0x7c }, { 0x5836, 0x7e }, { 0x5837, 0xaf },
	{ 0x5838, 0x8f }, { 0x5839, 0x8f }, { 0x583a, 0x9f }, { 0x583b, 0x7f },
	{ 0x583c, 0x5f },

	/* Y Gamma */
	{ 0x5480, 0x07 }, { 0x5481, 0x18 }, { 0x5482, 0x2c }, { 0x5483, 0x4e },
	{ 0x5484, 0x5e }, { 0x5485, 0x6b }, { 0x5486, 0x77 }, { 0x5487, 0x82 },
	{ 0x5488, 0x8c }, { 0x5489, 0x95 }, { 0x548a, 0xa4 }, { 0x548b, 0xb1 },
	{ 0x548c, 0xc6 }, { 0x548d, 0xd8 }, { 0x548e, 0xe9 },

	/* UV Gamma */
	{ 0x5490, 0x0f }, { 0x5491, 0xff }, { 0x5492, 0x0d }, { 0x5493, 0x05 },
	{ 0x5494, 0x07 }, { 0x5495, 0x1a }, { 0x5496, 0x04 }, { 0x5497, 0x01 },
	{ 0x5498, 0x03 }, { 0x5499, 0x53 }, { 0x549a, 0x02 }, { 0x549b, 0xeb },
	{ 0x549c, 0x02 }, { 0x549d, 0xa0 }, { 0x549e, 0x02 }, { 0x549f, 0x67 },
	{ 0x54a0, 0x02 }, { 0x54a1, 0x3b }, { 0x54a2, 0x02 }, { 0x54a3, 0x18 },
	{ 0x54a4, 0x01 }, { 0x54a5, 0xe7 }, { 0x54a6, 0x01 }, { 0x54a7, 0xc3 },
	{ 0x54a8, 0x01 }, { 0x54a9, 0x94 }, { 0x54aa, 0x01 }, { 0x54ab, 0x72 },
	{ 0x54ac, 0x01 }, { 0x54ad, 0x57 },

	/* AWB */
	{ OV9740_AWB_CTRL00,		0xf0 },
	{ OV9740_AWB_CTRL01,		0x00 },
	{ OV9740_AWB_CTRL02,		0x41 },
	{ OV9740_AWB_CTRL03,		0x42 },
	{ OV9740_AWB_ADV_CTRL01,	0x8a },
	{ OV9740_AWB_ADV_CTRL02,	0x61 },
	{ OV9740_AWB_ADV_CTRL03,	0xce },
	{ OV9740_AWB_ADV_CTRL04,	0xa8 },
	{ OV9740_AWB_ADV_CTRL05,	0x17 },
	{ OV9740_AWB_ADV_CTRL06,	0x1f },
	{ OV9740_AWB_ADV_CTRL07,	0x27 },
	{ OV9740_AWB_ADV_CTRL08,	0x41 },
	{ OV9740_AWB_ADV_CTRL09,	0x34 },
	{ OV9740_AWB_ADV_CTRL10,	0xf0 },
	{ OV9740_AWB_ADV_CTRL11,	0x10 },
	{ OV9740_AWB_CTRL0F,		0xff },
	{ OV9740_AWB_CTRL10,		0x00 },
	{ OV9740_AWB_CTRL11,		0xff },
	{ OV9740_AWB_CTRL12,		0x00 },
	{ OV9740_AWB_CTRL13,		0xff },
	{ OV9740_AWB_CTRL14,		0x00 },

	/* CIP */
	{ 0x530d, 0x12 },

	/* CMX */
	{ 0x5380, 0x01 }, { 0x5381, 0x00 }, { 0x5382, 0x00 }, { 0x5383, 0x17 },
	{ 0x5384, 0x00 }, { 0x5385, 0x01 }, { 0x5386, 0x00 }, { 0x5387, 0x00 },
	{ 0x5388, 0x00 }, { 0x5389, 0xe0 }, { 0x538a, 0x00 }, { 0x538b, 0x20 },
	{ 0x538c, 0x00 }, { 0x538d, 0x00 }, { 0x538e, 0x00 }, { 0x538f, 0x16 },
	{ 0x5390, 0x00 }, { 0x5391, 0x9c }, { 0x5392, 0x00 }, { 0x5393, 0xa0 },
	{ 0x5394, 0x18 },

	/* 50/60 Detection */
	{ 0x3c0a, 0x9c }, { 0x3c0b, 0x3f },

	/* Output Select */
	{ OV9740_IO_OUTPUT_SEL01,	0x00 },
	{ OV9740_IO_OUTPUT_SEL02,	0x00 },
	{ OV9740_IO_CREL00,		0xe8 },
	{ OV9740_IO_CREL01,		0x03 },
	{ OV9740_IO_CREL02,		0xff },
	{ OV9740_IO_MIPI_CHAN,	0x43 },
	//{ OV9740_IO_MIPI_DVP,	0x1D },

	/* AWB Control */
	{ OV9740_AWB_MANUAL_CTRL,	0x00 },

	/* Analog Control */
	{ OV9740_ANALOG_CTRL03,		0xaa },
	{ OV9740_ANALOG_CTRL32,		0x2f },
	{ OV9740_ANALOG_CTRL20,		0x66 },
	{ OV9740_ANALOG_CTRL21,		0xc0 },
	{ OV9740_ANALOG_CTRL31,		0x52 },
	{ OV9740_ANALOG_CTRL33,		0x50 },
	{ OV9740_ANALOG_CTRL30,		0xca },
	{ OV9740_ANALOG_CTRL04,		0x0c },
	{ OV9740_ANALOG_CTRL01,		0x40 },
	{ OV9740_ANALOG_CTRL02,		0x16 },
	{ OV9740_ANALOG_CTRL10,		0xa1 },
	{ OV9740_ANALOG_CTRL12,		0x24 },
	{ OV9740_ANALOG_CTRL22,		0x9f },
	{ OV9740_ANALOG_CTRL15,		0xf0 },

	/* Sensor Control */
	{ OV9740_SENSOR_CTRL03,		0x42 },
	{ OV9740_SENSOR_CTRL04,		0x10 },
	{ OV9740_SENSOR_CTRL05,		0x45 },
	{ OV9740_SENSOR_CTRL07,		0x14 },

	/* Timing Control */
	{ OV9740_TIMING_CTRL33,		0x04 },
	{ OV9740_TIMING_CTRL35,		0x02 },
	{ OV9740_TIMING_CTRL19,		0x6e },
	{ OV9740_TIMING_CTRL17,		0x94 },

	/* AEC/AGC Control */
	{ OV9740_AEC_ENABLE,		0x10 },
	{ OV9740_GAIN_CEILING_01,	0x00 },
	{ OV9740_GAIN_CEILING_02,	0x7f },
	{ OV9740_AEC_HI_THRESHOLD,	0xa0 },
	{ OV9740_AEC_3A1A,		0x05 },
	{ OV9740_AEC_CTRL1B_WPT2,	0x50 },
	{ OV9740_AEC_CTRL0F_WPT,	0x50 },
	{ OV9740_AEC_CTRL10_BPT,	0x4c },
	{ OV9740_AEC_CTRL1E_BPT2,	0x4c },
	{ OV9740_AEC_LO_THRESHOLD,	0x26 },

	/* BLC Control */
	{ OV9740_BLC_AUTO_ENABLE,	0x45 },
	{ OV9740_BLC_MODE,		0x18 },

	/* DVP Control */
	{ OV9740_DVP_VSYNC_CTRL02,	0x04 },
	{ OV9740_DVP_VSYNC_MODE,	0x00 },
	{ OV9740_DVP_VSYNC_CTRL06,	0x08 },

	/* PLL Setting */
	{ OV9740_PLL_MODE_CTRL01,	0x20 },
	{ OV9740_PRE_PLL_CLK_DIV,	0x03 },
	{ OV9740_PLL_MULTIPLIER,	0x4c },
	{ OV9740_VT_SYS_CLK_DIV,	0x01 },
	{ OV9740_VT_PIX_CLK_DIV,	0x08 },
	{ OV9740_PLL_CTRL3010,		0x01 },
	{ OV9740_VFIFO_CTRL00,		0x82 },

	/* Timing Setting */
	/* VTS */
	{ OV9740_FRM_LENGTH_LN_HI,	0x03 },
	{ OV9740_FRM_LENGTH_LN_LO,	0x07 },
	/* HTS */
	{ OV9740_LN_LENGTH_PCK_HI,	0x06 },
	{ OV9740_LN_LENGTH_PCK_LO,	0x62 },

	/* MIPI Control */
	{ OV9740_MIPI_CTRL00,		0x44 }, /* 0x64 for discontinuous clk */
	{ OV9740_MIPI_3837,		0x01 },
	{ OV9740_MIPI_CTRL01,		0x0f },
	{ OV9740_MIPI_CTRL03,		0x05 },
	{ OV9740_MIPI_CTRL05,		0x10 },
	{ OV9740_VFIFO_RD_CTRL,		0x16 },
	{ OV9740_MIPI_CTRL_3012,	0x70 },
	{ OV9740_SC_CMMM_MIPI_CTR,	0x01 },

	/* YUYV order */
	{ OV9740_ISP_CTRL19,		0x02 },
};

static enum v4l2_mbus_pixelcode ov9740_codes[] = {
	V4L2_MBUS_FMT_YUYV8_2X8,
	V4L2_MBUS_FMT_SBGGR8_1X8,
};

static const struct v4l2_queryctrl ov9740_controls[] = {
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Horizontally",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
};

/* read a register */
static int ov9740_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= (u8 *)&reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= val,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

/* write a register */
static int ov9740_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	struct i2c_msg msg;
	struct {
		u16 reg;
		u8 val;
	} __packed buf;
	int ret;

	reg = swab16(reg);

	buf.reg = reg;
	buf.val = val;

	msg.addr	= client->addr;
	msg.flags	= 0;
	msg.len		= 3;
	msg.buf		= (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}


/* Read a register, alter its bits, write it back */
static int ov9740_reg_rmw(struct i2c_client *client, u16 reg, u8 set, u8 unset)
{
	u8 val;
	int ret;

	ret = ov9740_reg_read(client, reg, &val);
	if (ret < 0) {
		dev_err(&client->dev,
			"[Read]-Modify-Write of register 0x%04x failed!\n",
			reg);
		return ret;
	}

	val |= set;
	val &= ~unset;

	ret = ov9740_reg_write(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev,
			"Read-Modify-[Write] of register 0x%04x failed!\n",
			reg);
		return ret;
	}

	return 0;
}

static int ov9740_reg_write_array(struct i2c_client *client,
				  const struct ov9740_reg *regarray,
				  int regarraylen)
{
	int i;
	int ret;

	for (i = 0; i < regarraylen; i++) {
		ret = ov9740_reg_write(client,
				       regarray[i].reg, regarray[i].val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* Start/Stop streaming from the device */
static int ov9740_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9740_priv *priv = to_ov9740(sd);
	int ret;

	/* Program orientation register. */
	if (priv->flag_vflip)
		ret = ov9740_reg_rmw(client, OV9740_IMAGE_ORT, 0x2, 0);
	else
		ret = ov9740_reg_rmw(client, OV9740_IMAGE_ORT, 0, 0x2);
	if (ret < 0)
		return ret;

	if (priv->flag_hflip)
		ret = ov9740_reg_rmw(client, OV9740_IMAGE_ORT, 0x1, 0);
	else
		ret = ov9740_reg_rmw(client, OV9740_IMAGE_ORT, 0, 0x1);
	if (ret < 0)
		return ret;

	if (enable) {
		dev_dbg(&client->dev, "Enabling Streaming\n");
		/* Start Streaming */
		ret = ov9740_reg_write(client, OV9740_MODE_SELECT, 0x01);

	} else {
		dev_dbg(&client->dev, "Disabling Streaming\n");
		/* Software Reset */
		ret = ov9740_reg_write(client, OV9740_SOFTWARE_RESET, 0x01);
		if (!ret)
			/* Setting Streaming to Standby */
			ret = ov9740_reg_write(client, OV9740_MODE_SELECT,
					       0x00);
	}

	priv->current_enable = enable;

	return ret;
}

/* Alter bus settings on camera side */
static int ov9740_set_bus_param(struct soc_camera_device *icd,
				unsigned long flags)
{
	return 0;
}

/* Request bus settings on camera side */
static unsigned long ov9740_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	unsigned long flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8;

	return soc_camera_apply_sensor_flags(icl, flags);
}

/* select nearest higher resolution for capture */
static void ov9740_res_roundup(u32 *width, u32 *height)
{
	/* Width must be a multiple of 4 pixels. */
	*width = ALIGN(*width, 4);

	/* Max resolution is 1280x720 (720p). */
	if (*width > OV9740_MAX_WIDTH)
		*width = OV9740_MAX_WIDTH;

	if (*height > OV9740_MAX_HEIGHT)
		*height = OV9740_MAX_HEIGHT;
}

/* Setup registers according to resolution and color encoding */
static int ov9740_set_res(struct i2c_client *client, u32 width, u32 height)
{
	u32 x_start;
	u32 y_start;
	u32 x_end;
	u32 y_end;
	bool scaling = 0;
	u32 scale_input_x;
	u32 scale_input_y;
	int ret;

	if ((width != OV9740_MAX_WIDTH) || (height != OV9740_MAX_HEIGHT))
		scaling = 1;

	/*
	 * Try to use as much of the sensor area as possible when supporting
	 * smaller resolutions.  Depending on the aspect ratio of the
	 * chosen resolution, we can either use the full width of the sensor,
	 * or the full height of the sensor (or both if the aspect ratio is
	 * the same as 1280x720.
	 */
	if ((OV9740_MAX_WIDTH * height) > (OV9740_MAX_HEIGHT * width)) {
		scale_input_x = (OV9740_MAX_HEIGHT * width) / height;
		scale_input_y = OV9740_MAX_HEIGHT;
	} else {
		scale_input_x = OV9740_MAX_WIDTH;
		scale_input_y = (OV9740_MAX_WIDTH * height) / width;
	}

	/* These describe the area of the sensor to use. */
	x_start = (OV9740_MAX_WIDTH - scale_input_x) / 2;
	y_start = (OV9740_MAX_HEIGHT - scale_input_y) / 2;
	x_end = x_start + scale_input_x - 1;
	y_end = y_start + scale_input_y - 1;

	ret = ov9740_reg_write(client, OV9740_X_ADDR_START_HI, x_start >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_X_ADDR_START_LO, x_start & 0xff);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_Y_ADDR_START_HI, y_start >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_Y_ADDR_START_LO, y_start & 0xff);
	if (ret)
		goto done;

	ret = ov9740_reg_write(client, OV9740_X_ADDR_END_HI, x_end >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_X_ADDR_END_LO, x_end & 0xff);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_Y_ADDR_END_HI, y_end >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_Y_ADDR_END_LO, y_end & 0xff);
	if (ret)
		goto done;

	ret = ov9740_reg_write(client, OV9740_X_OUTPUT_SIZE_HI, width >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_X_OUTPUT_SIZE_LO, width & 0xff);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_Y_OUTPUT_SIZE_HI, height >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_Y_OUTPUT_SIZE_LO, height & 0xff);
	if (ret)
		goto done;

	ret = ov9740_reg_write(client, OV9740_ISP_CTRL1E, scale_input_x >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_ISP_CTRL1F, scale_input_x & 0xff);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_ISP_CTRL20, scale_input_y >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_ISP_CTRL21, scale_input_y & 0xff);
	if (ret)
		goto done;

	ret = ov9740_reg_write(client, OV9740_VFIFO_READ_START_HI,
			       (scale_input_x - width) >> 8);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_VFIFO_READ_START_LO,
			       (scale_input_x - width) & 0xff);
	if (ret)
		goto done;

	ret = ov9740_reg_write(client, OV9740_ISP_CTRL00, 0xff);
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_ISP_CTRL01, 0xef |
							  (scaling << 4));
	if (ret)
		goto done;
	ret = ov9740_reg_write(client, OV9740_ISP_CTRL03, 0xff);

done:
	return ret;
}

/* set the format we will capture in */
static int ov9740_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9740_priv *priv = to_ov9740(sd);
	enum v4l2_colorspace cspace;
	enum v4l2_mbus_pixelcode code = mf->code;
	int ret;

	ov9740_res_roundup(&mf->width, &mf->height);

	switch (code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		cspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		cspace = V4L2_COLORSPACE_SRGB;
		break;
	default:
		return -EINVAL;
	}
	
	if ( mf->width == 640 && mf->height == 360 ) {
		ret = ov9740_reg_write_array(client, ov9740_mipi_2lane_640_360_60,
				     ARRAY_SIZE(ov9740_mipi_2lane_640_360_60));
	} else if ( mf->width == 640 && mf->height == 480 ) {
		ret = ov9740_reg_write_array(client, ov9740_mipi_2lane_640_480_30,
				     ARRAY_SIZE(ov9740_mipi_2lane_640_480_30));
		//code = V4L2_MBUS_FMT_UYVY8_2X8;
	} else if ( mf->width == 1280 && mf->height == 720 ) {
		ret = ov9740_reg_write_array(client, ov9740_mipi_1lane_1280_720_30,
				     ARRAY_SIZE(ov9740_mipi_1lane_1280_720_30));
	} else {
		ret = -1;
	}
//	ret = ov9740_reg_write_array(client, ov9740_defaults,
//				     ARRAY_SIZE(ov9740_defaults));
	if (ret < 0)
		return ret;

	ret = ov9740_set_res(client, mf->width, mf->height);
	if (ret < 0)
		return ret;

	mf->code	= code;
	mf->colorspace	= cspace;

	memcpy(&priv->current_mf, mf, sizeof(struct v4l2_mbus_framefmt));

	return ret;
}

static int ov9740_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	ov9740_res_roundup(&mf->width, &mf->height);

	mf->field = V4L2_FIELD_NONE;
	mf->code = V4L2_MBUS_FMT_SBGGR8_1X8; //V4L2_MBUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static int ov9740_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov9740_codes))
		return -EINVAL;

	*code = ov9740_codes[index];

	return 0;
}

static int ov9740_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left		= 0;
	a->bounds.top		= 0;
	a->bounds.width		= OV9740_MAX_WIDTH;
	a->bounds.height	= OV9740_MAX_HEIGHT;
	a->defrect		= a->bounds;
	a->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int ov9740_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	a->c.left		= 0;
	a->c.top		= 0;
	a->c.width		= OV9740_MAX_WIDTH;
	a->c.height		= OV9740_MAX_HEIGHT;
	a->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

/* Get status of additional camera capabilities */
static int ov9740_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov9740_priv *priv = to_ov9740(sd);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		ctrl->value = priv->flag_vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->value = priv->flag_hflip;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Set status of additional camera capabilities */
static int ov9740_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov9740_priv *priv = to_ov9740(sd);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		priv->flag_vflip = ctrl->value;
		break;
	case V4L2_CID_HFLIP:
		priv->flag_hflip = ctrl->value;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Get chip identification */
static int ov9740_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct ov9740_priv *priv = to_ov9740(sd);

	id->ident = priv->ident;
	id->revision = priv->revision;

	return 0;
}

static int ov9740_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov9740_priv *priv = to_ov9740(sd);

	if (!priv->current_enable)
		return 0;

	if (on) {
		ov9740_s_fmt(sd, &priv->current_mf);
		ov9740_s_stream(sd, priv->current_enable);
	} else {
		ov9740_s_stream(sd, 0);
		priv->current_enable = true;
	}

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov9740_get_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 2;

	ret = ov9740_reg_read(client, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = (__u64)val;

	return ret;
}

static int ov9740_set_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ov9740_reg_write(client, reg->reg, reg->val);
}
#endif

static int ov9740_video_probe(struct soc_camera_device *icd,
			      struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9740_priv *priv = to_ov9740(sd);
	u8 modelhi, modello;
	int ret;

	/* We must have a parent by now. And it cannot be a wrong one. */
	BUG_ON(!icd->parent ||
	       to_soc_camera_host(icd->parent)->nr != icd->iface);

	/*
	 * check and show product ID and manufacturer ID
	 */
	ret = ov9740_reg_read(client, OV9740_MODEL_ID_HI, &modelhi);
	if (ret < 0)
		goto err;

	ret = ov9740_reg_read(client, OV9740_MODEL_ID_LO, &modello);
	if (ret < 0)
		goto err;

	priv->model = (modelhi << 8) | modello;

	ret = ov9740_reg_read(client, OV9740_REVISION_NUMBER, &priv->revision);
	if (ret < 0)
		goto err;

	ret = ov9740_reg_read(client, OV9740_MANUFACTURER_ID, &priv->manid);
	if (ret < 0)
		goto err;

	ret = ov9740_reg_read(client, OV9740_SMIA_VERSION, &priv->smiaver);
	if (ret < 0)
		goto err;

	if (priv->model != 0x9740) {
		ret = -ENODEV;
		goto err;
	}

	priv->ident = V4L2_IDENT_OV9740;

	dev_info(&client->dev, "ov9740 Model ID 0x%04x, Revision 0x%02x, "
		 "Manufacturer 0x%02x, SMIA Version 0x%02x\n",
		 priv->model, priv->revision, priv->manid, priv->smiaver);

err:
	return ret;
}

static struct soc_camera_ops ov9740_ops = {
	.set_bus_param		= ov9740_set_bus_param,
	.query_bus_param	= ov9740_query_bus_param,
	.controls		= ov9740_controls,
	.num_controls		= ARRAY_SIZE(ov9740_controls),
};

static struct v4l2_subdev_video_ops ov9740_video_ops = {
	.s_stream		= ov9740_s_stream,
	.s_mbus_fmt		= ov9740_s_fmt,
	.try_mbus_fmt		= ov9740_try_fmt,
	.enum_mbus_fmt		= ov9740_enum_fmt,
	.cropcap		= ov9740_cropcap,
	.g_crop			= ov9740_g_crop,
};

static struct v4l2_subdev_core_ops ov9740_core_ops = {
	.g_ctrl			= ov9740_g_ctrl,
	.s_ctrl			= ov9740_s_ctrl,
	.g_chip_ident		= ov9740_g_chip_ident,
	.s_power		= ov9740_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= ov9740_get_register,
	.s_register		= ov9740_set_register,
#endif
};

static struct v4l2_subdev_ops ov9740_subdev_ops = {
	.core			= &ov9740_core_ops,
	.video			= &ov9740_video_ops,
};

/*
 * i2c_driver function
 */
static int ov9740_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov9740_priv *priv;
	struct soc_camera_device *icd	= client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "Missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(struct ov9740_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "Failed to allocate private data!\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov9740_subdev_ops);
	priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	icd->ops = &ov9740_ops;

	ret = ov9740_video_probe(icd, client);
	if (ret < 0) {
		icd->ops = NULL;
		kfree(priv);
	}

	return ret;
}

static int ov9740_remove(struct i2c_client *client)
{
	struct ov9740_priv *priv = i2c_get_clientdata(client);

	kfree(priv);

	return 0;
}

static const struct i2c_device_id ov9740_id[] = {
	{ "ov9740", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov9740_id);

static struct i2c_driver ov9740_i2c_driver = {
	.driver = {
		.name = "ov9740",
	},
	.probe    = ov9740_probe,
	.remove   = ov9740_remove,
	.id_table = ov9740_id,
};

static int __init ov9740_module_init(void)
{
	return i2c_add_driver(&ov9740_i2c_driver);
}

static void __exit ov9740_module_exit(void)
{
	i2c_del_driver(&ov9740_i2c_driver);
}

module_init(ov9740_module_init);
module_exit(ov9740_module_exit);

MODULE_DESCRIPTION("SoC Camera driver for OmniVision OV9740");
MODULE_AUTHOR("Andrew Chew <achew@nvidia.com>");
MODULE_LICENSE("GPL v2");
