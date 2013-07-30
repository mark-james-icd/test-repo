/*
 * Driver for OV2710 CMOS Image Sensor from Omnivision
 *
 * Copyright (C) 2013, Alex Feinman <alexfeinman@gmail.com>
 *
 * Based on OV5642 Camera Driver
 * Copyright (C) 2011, Bastian Hecht <hechtb@gmail.com>
 *
 * Based on Sony IMX074 Camera Driver
 * Copyright (C) 2010, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG 1
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-subdev.h>

#define OV2710_TABLE_WAIT_MS 0
#define OV2710_TABLE_END 1
#define OV2710_MAX_RETRIES 3
#define SIZEOF_I2C_TRANSBUF 32
#define DRIVER_NAME "ov2710"

#define OV2710_MAX_WIDTH		1920
#define OV2710_MAX_HEIGHT		1080

#define OV2710_PID_H 0x300A
#define OV2710_PID_L 0x300B

struct ov2710_reg {
	u16 addr;
	u16 val;
};

static struct ov2710_reg mode_1920x1080_ovt_10bit[] = {
{ 0x3103, 0x93 },
{ 0x3008, 0x82 },
	{OV2710_TABLE_WAIT_MS, 5},
{ 0x3008, 0x02 },
	{OV2710_TABLE_WAIT_MS, 5},
{ 0x3017, 0x7F },
{ 0x3018, 0xFC },
{ 0x3706, 0x61 },
{ 0x3712, 0x0C },
{ 0x3630, 0x6D },
{ 0x3801, 0xB4 },
{ 0x3621, 0x04 },
{ 0x3604, 0x60 },
{ 0x3603, 0xA7 },
{ 0x3631, 0x26 },
{ 0x3600, 0x04 },
{ 0x3620, 0x37 },
{ 0x3623, 0x00 },
{ 0x3702, 0x9E },
{ 0x3703, 0x5C },
{ 0x3704, 0x40 },
{ 0x370D, 0x0F },
{ 0x3713, 0x9F },
{ 0x3714, 0x4C },
{ 0x3710, 0x9E },
{ 0x3801, 0xC4 },
{ 0x3605, 0x05 },
{ 0x3606, 0x3F },
{ 0x302D, 0x90 },
{ 0x370B, 0x40 },
{ 0x3716, 0x31 },
{ 0x380D, 0x74 },
{ 0x5181, 0x20 },
{ 0x518F, 0x00 },
{ 0x4301, 0xFF },
{ 0x4303, 0x00 },
{ 0x3A00, 0x78 },
{ 0x300F, 0x88 },
{ 0x3011, 0x28 },
{ 0x3A1A, 0x06 },
{ 0x3A18, 0x00 },
{ 0x3A19, 0x7A },
{ 0x3A13, 0x54 },
{ 0x382E, 0x0F },
{ 0x381A, 0x1A },
{ 0x401D, 0x02 },
{ 0x5688, 0x03 },
{ 0x5684, 0x07 },
{ 0x5685, 0xA0 },
{ 0x5686, 0x04 },
{ 0x5687, 0x43 },
{ 0x3011, 0x0A },
{ 0x300F, 0x8A },
{ 0x3017, 0x00 },
{ 0x3018, 0x00 },
{ 0x300E, 0x04 },
{ 0x4801, 0x0F },
{ 0x300F, 0xC3 },
{ 0x3A0F, 0x40 },
{ 0x3A10, 0x38 },
{ 0x3A1B, 0x48 },
{ 0x3A1E, 0x30 },
{ 0x3A11, 0x90 },
{ 0x3A1F, 0x10 },
//{ 0x503d, 0x00 },
};

#if 0
static struct ov2710_reg mode_1280x720_ovt[] = {
	{ 0x3103, 0x93 },
{ 0x3008, 0x82 },
	{OV2710_TABLE_WAIT_MS, 5},
{ 0x3008, 0x02 },
	{OV2710_TABLE_WAIT_MS, 5},
	{ 0x3017, 0x7F },
	{ 0x3018, 0xFC },
	{ 0x3706, 0x61 },
	{ 0x3712, 0x0C },
	{ 0x3630, 0x6D },
	{ 0x3801, 0xB4 },
	{ 0x3621, 0x04 },
	{ 0x3604, 0x60 },
	{ 0x3603, 0xA7 },
	{ 0x3631, 0x26 },
	{ 0x3600, 0x04 },
	{ 0x3620, 0x37 },
	{ 0x3623, 0x00 },
	{ 0x3702, 0x9E },
	{ 0x3703, 0x5C },
	{ 0x3704, 0x40 },
	{ 0x370D, 0x0F },
	{ 0x3713, 0x9F },
	{ 0x3714, 0x4C },
	{ 0x3710, 0x9E },
	{ 0x3801, 0xC4 },
	{ 0x3605, 0x05 },
	{ 0x3606, 0x3F },
	{ 0x302D, 0x90 },
	{ 0x370B, 0x40 },
	{ 0x3716, 0x31 },
	{ 0x380D, 0x74 },
	{ 0x5181, 0x20 },
	{ 0x518F, 0x00 },
	{ 0x4301, 0xFF },
	{ 0x4303, 0x00 },
	{ 0x3A00, 0x78 },
	{ 0x300F, 0x88 },
	{ 0x3011, 0x28 },
	{ 0x3A1A, 0x06 },
	{ 0x3A18, 0x00 },
	{ 0x3A19, 0x7A },
	{ 0x3A13, 0x54 },
	{ 0x382E, 0x0F },
	{ 0x381A, 0x1A },
	{ 0x401D, 0x02 },
	{ 0x5688, 0x03 },
	{ 0x5684, 0x07 },
	{ 0x5685, 0xA0 },
	{ 0x5686, 0x04 },
	{ 0x5687, 0x43 },
	{ 0x3A0F, 0x40 },
	{ 0x3A10, 0x38 },
	{ 0x3A1B, 0x48 },
	{ 0x3A1E, 0x30 },
	{ 0x3A11, 0x90 },
	{ 0x3A1F, 0x10 },
	{ 0x3010, 0x20 },
{ 0x503d, 0xA0 },
};
#endif
#if 0
static struct ov2710_reg mode_1920x1080[] = {
 { 0x3103, 0x93 },
 { 0x3008, 0x82 },
	{OV2710_TABLE_WAIT_MS, 5},
 { 0x3008, 0x42 },
	{OV2710_TABLE_WAIT_MS, 5},
 { 0x3017, 0x7f },
 { 0x3018, 0xfc },
 { 0x3706, 0x61 },
 { 0x3712, 0x0c },
 { 0x3630, 0x6d },
 { 0x3801, 0xb4 },
 { 0x3621, 0x04 },
 { 0x3604, 0x60 },
 { 0x3603, 0xa7 },
 { 0x3631, 0x26 },
 { 0x3600, 0x04 },
 { 0x3620, 0x37 },
 { 0x3623, 0x00 },
 { 0x3702, 0x9e },
 { 0x3703, 0x5c },
 { 0x3704, 0x40 },
 { 0x370d, 0x0f },
 { 0x3713, 0x9f },
 { 0x3714, 0x4c },
 { 0x3710, 0x9e },
 { 0x3801, 0xc4 },
 { 0x3605, 0x05 },
 { 0x3606, 0x3f },
 { 0x302d, 0x90 },
 { 0x370b, 0x40 },
 { 0x3716, 0x31 },
 { 0x3707, 0x52 },
 { 0x380d, 0x74 },
 { 0x5181, 0x20 },
 { 0x518f, 0x00 },
 { 0x4301, 0xff },
 { 0x4303, 0x00 },
 { 0x3a00, 0x78 },
 { 0x300f, 0x88 },
 { 0x3011, 0x28 },
 { 0x3a1a, 0x06 },
 { 0x3a18, 0x00 },
 { 0x3a19, 0x7a },
 { 0x3a13, 0x54 },
 { 0x382e, 0x0f },
 { 0x381a, 0x1a },
 { 0x401d, 0x02 },
 { 0x381c, 0x00 },
 { 0x381d, 0x02 },
 { 0x381e, 0x04 },
 { 0x381f, 0x38 },
 { 0x3820, 0x00 },
 { 0x3821, 0x98 },
 { 0x3800, 0x01 },
 { 0x3802, 0x00 },
 { 0x3803, 0x0a },
 { 0x3804, 0x07 },
 { 0x3805, 0x90 },
 { 0x3806, 0x04 },
 { 0x3807, 0x40 },
 { 0x3808, 0x07 },
 { 0x3809, 0x90 },
 { 0x380a, 0x04 },
 { 0x380b, 0x40 },
 { 0x380e, 0x04 },
 { 0x380f, 0x50 },
 { 0x380c, 0x09 },
 { 0x380d, 0x74 },
 { 0x3810, 0x08 },
 { 0x3811, 0x02 },
 { 0x5688, 0x03 },
 { 0x5684, 0x07 },
 { 0x5685, 0xa0 },
 { 0x5686, 0x04 },
 { 0x5687, 0x43 },
 { 0x3011, 0x0a },
 { 0x300f, 0x8a },
 { 0x3017, 0x00 },
 { 0x3018, 0x00 },
 { 0x4800, 0x24 },
 { 0x300e, 0x04 },
 { 0x4801, 0x0f },
 { 0x300f, 0xc3 },
 { 0x3010, 0x00 },
 { 0x3011, 0x0a },
 { 0x3012, 0x01 },
 { 0x3a0f, 0x40 },
 { 0x3a10, 0x38 },
 { 0x3a1b, 0x48 },
 { 0x3a1e, 0x30 },
 { 0x3a11, 0x90 },
 { 0x3a1f, 0x10 },
 { 0x3a0e, 0x03 },
 { 0x3a0d, 0x04 },
 { 0x3a08, 0x14 },
 { 0x3a09, 0xc0 },
 { 0x3a0a, 0x11 },
 { 0x3a0b, 0x40 },
 { 0x300f, 0xc3 },
 { 0x3010, 0x00 },
 { 0x3011, 0x0e },
 { 0x3012, 0x02 },
 { 0x380c, 0x09 },
 { 0x380d, 0xec },
 { 0x3703, 0x61 },
 { 0x3704, 0x44 },
 { 0x3801, 0xd2 },
 { 0x3503, 0x33 },
 { 0x3500, 0x00 },
 { 0x3501, 0x44 },
 { 0x3502, 0xa0 },
 { 0x350a, 0x00 },
 { 0x350b, 0x00 },
 { 0x5001, 0x4e },
 { 0x5000, 0x5f },
 { 0x3008, 0x02 },
 { 0x3212, 0x01 },
 { 0x350b, 0x0c },
 { 0x3500, 0x00 },
 { 0x4850, 0x00 },
 { 0x380e, 0x04 },
 { 0x8b50, 0x00 },
 { 0x3212, 0x11 },
 { 0x3212, 0xa1 },
 { 0x350b, 0x08 },
 { 0x350b, 0x05 },
 { 0x350b, 0x02 },
 { 0x350b, 0x00 },
 { 0x3212, 0x01 },
 { 0x3500, 0x00 },
 { 0x3ef0, 0x00 },
 { 0x380e, 0x04 },
 { 0x50f0, 0x00 },
 { 0x3212, 0x11 },
 { 0x3212, 0xa1 },
 { 0x3212, 0x01 },
 { 0x350b, 0x00 },
 { 0x3500, 0x00 },
 { 0x3630, 0x00 },
 { 0x3212, 0x11 },
 { 0x3212, 0xa1 },
 { 0x3212, 0x01 },
 { 0x350b, 0x05 },
 { 0x3500, 0x00 },
 { 0x2420, 0x00 },
 { 0x3212, 0x11 },
 { 0x3212, 0xa1 },
 { 0x350b, 0x02 },
 { 0x350b, 0x00 },
 { 0x3212, 0x01 },
 { 0x350b, 0x0c },
 { 0x3500, 0x00 },
 { 0x1210, 0x00 },
 { 0x3212, 0x11 },
 { 0x3212, 0xa1 },
 { 0x350b, 0x09 },
 { 0x350b, 0x06 },
 { 0x350b, 0x04 },
 { 0x350b, 0x02 },
 { 0x350b, 0x00 },
 { 0x3500, 0x00 },
 { 0x0e80, 0x00 },
 { 0x3500, 0x00 },
 { 0x0d20, 0x00 },
 { 0x3500, 0x00 },
 { 0x0bf0, 0x00 },
 { 0x3500, 0x00 },
 { 0x0ae0, 0x00 },
 { 0x3500, 0x00 },
 { 0x09f0, 0x00 },
 { 0x3500, 0x00 },
 { 0x0920, 0x00 },
 { 0x3500, 0x00 },
 { 0x0860, 0x00 },
 { 0x3500, 0x00 },
 { 0x07b0, 0x00 },
 { 0x3500, 0x00 },
 { 0x0720, 0x00 },
 { 0x3500, 0x00 },
 { 0x0690, 0x00 },
 { 0x3500, 0x00 },
 { 0x0620, 0x00 },
 { 0x3500, 0x00 },
 { 0x05b0, 0x00 },
 { 0x3500, 0x00 },
 { 0x0550, 0x00 },
 { 0x3500, 0x00 },
 { 0x04f0, 0x00 },
 { 0x3500, 0x00 },
 { 0x04a0, 0x00 },
 { 0x3500, 0x00 },
 { 0x0450, 0x00 },
 { 0x3500, 0x00 },
 { 0x0410, 0x00 },
 { 0x3500, 0x00 },
 { 0x03d0, 0x00 },
 { 0x3500, 0x00 },
 { 0x0390, 0x00 },
 { 0x3500, 0x00 },
 { 0x0360, 0x00 },
 { 0x3500, 0x00 },
 { 0x0330, 0x00 },
 { 0x3500, 0x00 },
 { 0x0300, 0x00 },
 { 0x3500, 0x00 },
 { 0x02e0, 0x00 },
 { 0x3500, 0x00 },
 { 0x02c0, 0x00 },
 { 0x3500, 0x00 },
 { 0x02a0, 0x00 },
 { 0x3500, 0x00 },
 { 0x0280, 0x00 },
 { 0x3500, 0x00 },
 { 0x0260, 0x00 },
 { 0x3500, 0x00 },
 { 0x0250, 0x00 },
 { 0x3500, 0x00 },
 { 0x0230, 0x00 },
 { 0x3500, 0x00 },
 { 0x0220, 0x00 },
 { 0x3500, 0x00 },
 { 0x0200, 0x00 },
 { 0x3500, 0x00 },
 { 0x01f0, 0x00 },
 { 0x3500, 0x00 },
 { 0x01e0, 0x00 },
 { 0x3500, 0x00 },
 { 0x01d0, 0x00 },
 { 0x3500, 0x00 },
 { 0x01c0, 0x00 },
 { 0x3500, 0x00 },
 { 0x01b0, 0x00 },
 { 0x3500, 0x00 },
 { 0x01a0, 0x00 },
 { 0x3500, 0x00 },
 { 0x0190, 0x00 },
 { 0x3500, 0x00 },
 { 0x0180, 0x00 },
 { 0x3500, 0x00 },
 { 0x0170, 0x00 },
 { 0x3500, 0x00 },
 { 0x0160, 0x00 },
 { 0x3500, 0x00 },
 { 0x0150, 0x00 },
 { 0x3500, 0x00 },
 { 0x0160, 0x00 },
 { 0x3500, 0x00 },
 { 0x0180, 0x00 },
 { 0x3500, 0x00 },
 { 0x0190, 0x00 },
 { 0x3500, 0x00 },
 { 0x01a0, 0x00 },
 { 0x3500, 0x00 },
 { 0x01b0, 0x00 },
 { 0x503d, 0xA0 },

	{OV2710_TABLE_END, 0x0000}
};
#endif

static struct ov2710_reg mode_1280x720[] = {
	{0x3103, 0x93},
	{0x3008, 0x82},
	{OV2710_TABLE_WAIT_MS, 5},
	{0x3008, 0x42},
	{OV2710_TABLE_WAIT_MS, 5},
	{0x3017, 0x7f},
	{0x3018, 0xfc},

	{0x3706, 0x61},
	{0x3712, 0x0c},
	{0x3630, 0x6d},
	{0x3801, 0xb4},
	{0x3621, 0x04},
	{0x3604, 0x60},
	{0x3603, 0xa7},
	{0x3631, 0x26},
	{0x3600, 0x04},
	{0x3620, 0x37},
	{0x3623, 0x00},
	{0x3702, 0x9e},
	{0x3703, 0x5c},
	{0x3704, 0x40},
	{0x370d, 0x0f},
	{0x3713, 0x9f},
	{0x3714, 0x4c},
	{0x3710, 0x9e},
	{0x3801, 0xc4},
	{0x3605, 0x05},
	{0x3606, 0x3f},
	{0x302d, 0x90},
	{0x370b, 0x40},
	{0x3716, 0x31},
	{0x3707, 0x52},
	{0x380d, 0x74},
	{0x5181, 0x20},
	{0x518f, 0x00},
	{0x4301, 0xff},
	{0x4303, 0x00},
	{0x3a00, 0x78},
	{0x300f, 0x88},
	{0x3011, 0x28},
	{0x3a1a, 0x06},
	{0x3a18, 0x00},
	{0x3a19, 0x7a},
	{0x3a13, 0x54},
	{0x382e, 0x0f},
	{0x381a, 0x1a},
	{0x401d, 0x02},

	{0x381c, 0x10},
	{0x381d, 0xb0},
	{0x381e, 0x02},
	{0x381f, 0xec},
	{0x3800, 0x01},
	{0x3820, 0x0a},
	{0x3821, 0x2a},
	{0x3804, 0x05},
	{0x3805, 0x10},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3806, 0x02},
	{0x3807, 0xe0},
	{0x3808, 0x05},
	{0x3809, 0x10},
	{0x380a, 0x02},
	{0x380b, 0xe0},
	{0x380e, 0x02},
	{0x380f, 0xf0},
	{0x380c, 0x07},
	{0x380d, 0x00},
	{0x3810, 0x10},
	{0x3811, 0x06},

	{0x5688, 0x03},
	{0x5684, 0x05},
	{0x5685, 0x00},
	{0x5686, 0x02},
	{0x5687, 0xd0},

	{0x3a08, 0x1b},
	{0x3a09, 0xe6},
	{0x3a0a, 0x17},
	{0x3a0b, 0x40},
	{0x3a0e, 0x01},
	{0x3a0d, 0x02},
	{0x3011, 0x0a},
	{0x300f, 0x8a},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x4800, 0x24},
	{0x300e, 0x04},
	{0x4801, 0x0f},
	{0x300f, 0xc3},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},

	{0x3010, 0x10},
	{0x3a0e, 0x02},
	{0x3a0d, 0x03},
	{0x3a08, 0x0d},
	{0x3a09, 0xf3},
	{0x3a0a, 0x0b},
	{0x3a0b, 0xa0},

	{0x300f, 0xc2},
	{0x3011, 0x0e},
	{0x3012, 0x02},
	{0x380c, 0x07},
	{0x380d, 0x6a},
	{0x3703, 0x5c},
	{0x3704, 0x40},
	{0x3801, 0xbc},

	{0x3503, 0x33},
	{0x3500, 0x00},
	{0x3501, 0x00},
	{0x3502, 0x00},
	{0x350a, 0x00},
	{0x350b, 0x00},
	{0x5001, 0x4e},
	{0x5000, 0x5f},
	{0x3008, 0x02},
    { 0x503d, 0xA0 },

	{OV2710_TABLE_END, 0x0000}
};

enum {
	OV2710_MODE_1920x1080,
//	OV2710_MODE_1280x720,
};


static struct ov2710_reg *mode_table[] = {
	[OV2710_MODE_1920x1080] = mode_1920x1080_ovt_10bit,
//	[OV2710_MODE_1280x720] = mode_1280x720,
};

struct ov2710_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct ov2710 {
	int mode;
	struct v4l2_mbus_framefmt mf;
	struct v4l2_subdev	subdev;
	//struct i2c_client 	*i2c_client;
	const struct ov2710_datafmt	*fmt;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
};

static const struct ov2710_datafmt ov2710_colour_fmts[] = {
	{V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_COLORSPACE_SRGB},
//	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct ov2710 *to_ov2710(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov2710, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct ov2710_datafmt
			*ov2710_find_datafmt(enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov2710_colour_fmts); i++)
		if (ov2710_colour_fmts[i].code == code)
			return ov2710_colour_fmts + i;

	return NULL;
}

static int ov2710_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)

		return -EINVAL;

	*val = data[2];

	return 0;
}

static int ov2710_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("ov2710: i2c transfer failed, retrying %x %x\n",
		       addr, val);

		msleep(3);
	} while (retry <= OV2710_MAX_RETRIES);

	return err;
}

static int ov2710_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	
	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	pr_err("ov2710: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int ov2710_write_table(struct i2c_client* client,
			      const struct ov2710_reg table[])
{
	int err;
	const struct ov2710_reg *next, *n_next;
	struct ov2710* info = to_ov2710(client);
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	unsigned int i;
	u16 val;

	for (next = table; next->addr != OV2710_TABLE_END; next++) {
		if (next->addr == OV2710_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		if (!buf_filled) {
			b_ptr = info->i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != OV2710_TABLE_END &&
			n_next->addr != OV2710_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = ov2710_write_bulk_reg(client,
			info->i2c_trans_buf, buf_filled);
		if (err)
			return err;
		buf_filled = 0;
	}
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2710_get_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = ov2710_read_reg(client, reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ov2710_set_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ov2710_write_reg(client, reg->reg, reg->val);
}
#endif

static int ov2710_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf);
			
#if 0			
static void ov2710_set_default_fmt(struct ov2710 *priv)
{
	struct v4l2_mbus_framefmt *mf = &priv->mf;

	mf->width = 1920;
	mf->height = 1080;
	mf->code = V4L2_MBUS_FMT_SBGGR10_1X10;
	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_SRGB;
}
#endif			

static int ov2710_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	const struct ov2710_datafmt *fmt = ov2710_find_datafmt(mf->code);
/*
	dev_dbg(sd->v4l2_dev->dev, "%s(%u) width: %u heigth: %u\n",
			__func__, mf->code, mf->width, mf->height);
*/
	if (!fmt) {
		mf->code	= ov2710_colour_fmts[0].code;
		mf->colorspace	= ov2710_colour_fmts[0].colorspace;
	    mf->field = V4L2_FIELD_NONE;
	}
	
//	if ( mf->width <= 1280 && mf->height <= 720 ) {
//		mf->width = 1280;
//		mf->height = 720;
//	} else {
		mf->width = 1920;
		mf->height = 1080;
		mf->field = V4L2_FIELD_NONE;
	    mf->colorspace = V4L2_COLORSPACE_SRGB;
	    mf->code = V4L2_MBUS_FMT_SBGGR10_1X10;
//	}

	mf->field	= V4L2_FIELD_NONE;
	ov2710_s_fmt(sd, mf);
	return 0;
}

static int ov2710_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2710 *priv = to_ov2710(client);
	struct ov2710_reg* mode_data = mode_1920x1080_ovt_10bit;

	dev_dbg(sd->v4l2_dev->dev, "%s(%u) - %dx%d\n", __func__, mf->code, mf->width, mf->height);

	/* MIPI CSI could have changed the format, double-check */
	if (!ov2710_find_datafmt(mf->code))
		return -EINVAL;

	if ( mf->width == 1920 )
		mode_data = mode_1920x1080_ovt_10bit;
#if 0		
	else if ( mf->width == 1280 )
		mode_data = mode_1280x720_ovt;
#endif	

	priv->fmt = ov2710_find_datafmt(mf->code);
	priv->mf = *mf;
	

	return 0;
}

static int ov2710_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2710 *priv = to_ov2710(client);

	const struct ov2710_datafmt *fmt = priv->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= OV2710_MAX_WIDTH;
	mf->height	= OV2710_MAX_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov2710_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov2710_colour_fmts))
		return -EINVAL;

	*code = ov2710_colour_fmts[index].code;
	return 0;
}

static int ov2710_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident	= V4L2_IDENT_OV2710;
	id->revision	= 0;

	return 0;
}

static int ov2710_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct v4l2_rect *rect = &a->c;

	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	rect->top	= 0;
	rect->left	= 0;
	rect->width	= OV2710_MAX_WIDTH;
	rect->height	= OV2710_MAX_HEIGHT;

	return 0;
}

static int ov2710_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= OV2710_MAX_WIDTH;
	a->bounds.height		= OV2710_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int ov2710_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2710 *priv = to_ov2710(client);
printk("%s\n", __func__);
	if (on) {
		ov2710_s_fmt(sd, &priv->mf);
		ov2710_write_table(client, mode_1920x1080_ovt_10bit);
	} //else
		//ov5650_s_stream(sd, 0);

	return 0;
}


static struct v4l2_subdev_video_ops ov2710_subdev_video_ops = {
	.s_mbus_fmt	= ov2710_s_fmt,
	.g_mbus_fmt	= ov2710_g_fmt,
	.try_mbus_fmt	= ov2710_try_fmt,
	.enum_mbus_fmt	= ov2710_enum_fmt,
	.g_crop		= ov2710_g_crop,
	.cropcap	= ov2710_cropcap,
};

static struct v4l2_subdev_core_ops ov2710_subdev_core_ops = {
	.g_chip_ident	= ov2710_g_chip_ident,
	.s_power = ov2710_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov2710_get_register,
	.s_register	= ov2710_set_register,
#endif
};

static struct v4l2_subdev_ops ov2710_subdev_ops = {
	.core	= &ov2710_subdev_core_ops,
	.video	= &ov2710_subdev_video_ops,
};

/*
 * We have to provide soc-camera operations, but we don't have anything to say
 * there. The MIPI CSI2 driver will provide .query_bus_param and .set_bus_param
 */
static unsigned long soc_ov2710_query_bus_param(struct soc_camera_device *icd)
{
	return 0;
}

static int soc_ov2710_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	return -EINVAL;
}

static struct soc_camera_ops soc_ov2710_ops = {
	.query_bus_param	= soc_ov2710_query_bus_param,
	.set_bus_param		= soc_ov2710_set_bus_param,
};

static int ov2710_video_probe(struct soc_camera_device *icd,
			      struct i2c_client *client)
{
	int ret;
	u8 id_high, id_low;
	u16 id;

	/* Read sensor Model ID */
	ret = ov2710_read_reg(client, OV2710_PID_H, &id_high);
	if (ret < 0)
		return ret;

	id = id_high << 8;

	ret = ov2710_read_reg(client, OV2710_PID_L, &id_low);
	if (ret < 0)
		return ret;

	id |= id_low;

	dev_info(&client->dev, "Chip ID 0x%04x detected\n", id);

//	if (id != 0x5642)
//		return -ENODEV;

	return 0;
}

static int ov2710_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov2710 *priv;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "OV5642: missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "OV5642: missing platform data!\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(struct ov2710), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

        //strlcpy(priv->subdev.name, DRIVER_NAME, sizeof(priv->subdev.name));
	v4l2_i2c_subdev_init(&priv->subdev, client, &ov2710_subdev_ops);

	icd->ops	= &soc_ov2710_ops;
	priv->fmt	= &ov2710_colour_fmts[0];

	ret = ov2710_video_probe(icd, client);
	if (ret < 0)
		goto error;

	//i2c_set_clientdata(client, priv);

	return 0;

error:
	icd->ops = NULL;
	kfree(priv);
	return ret;
}

static int ov2710_remove(struct i2c_client *client)
{
	struct ov2710 *priv = to_ov2710(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	icd->ops = NULL;
	if (icl->free_bus)
		icl->free_bus(icl);
	kfree(priv);

	return 0;
}

static const struct i2c_device_id ov2710_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2710_id);

static struct i2c_driver ov2710_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe		= ov2710_probe,
	.remove		= ov2710_remove,
	.id_table	= ov2710_id,
};

static int __init ov2710_mod_init(void)
{
	return i2c_add_driver(&ov2710_i2c_driver);
}

static void __exit ov2710_mod_exit(void)
{
	i2c_del_driver(&ov2710_i2c_driver);
}

module_init(ov2710_mod_init);
module_exit(ov2710_mod_exit);

MODULE_DESCRIPTION("Omnivision OV2710 Camera driver");
MODULE_AUTHOR("Bastian Hecht <hechtb@gmail.com>");
MODULE_LICENSE("GPL v2");
