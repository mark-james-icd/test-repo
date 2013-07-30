/*
 * arch/arm/mach-tegra/board-cardhu.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.  All rights reserved.
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/pca954x.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/rfkill-gpio.h>

#include <sound/wm8903.h>
#include <media/tegra_camera.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io_dpd.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/tegra_wm8903_pdata.h>
//#include <mach/tegra_dmic_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/thermal.h>
#include <mach/pci.h>
#include <mach/tegra_fiq_debugger.h>

#include "board.h"
#include "clock.h"
#include "board-houdini.h"
#include "board-touch.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "wdt-recovery.h"

static struct balanced_throttle throttle_list[] = {
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_TJ,
		.throt_tab_size = 10,
		.throt_tab = {
			{      0, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 760000, 1000 },
			{ 760000, 1050 },
			{1000000, 1050 },
			{1000000, 1100 },
		},
	},
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_SKIN,
		.throt_tab_size = 6,
		.throt_tab = {
			{ 640000, 1200 },
			{ 640000, 1200 },
			{ 760000, 1200 },
			{ 760000, 1200 },
			{1000000, 1200 },
			{1000000, 1200 },
		},
	},
#endif
};

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.shutdown_device_id = THERMAL_DEVICE_ID_NCT_EXT,
	.temp_shutdown = 90000,

#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	.throttle_edp_device_id = THERMAL_DEVICE_ID_NCT_EXT,
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	.temp_throttle = 85000,
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	.skin_device_id = THERMAL_DEVICE_ID_SKIN,
	.temp_throttle_skin = 43000,
	.tc1_skin = 0,
	.tc2_skin = 1,
	.passive_delay_skin = 5000,

	.skin_temp_offset = 9793,
	.skin_period = 1100,
	.skin_devs_size = 2,
	.skin_devs = {
		{
			THERMAL_DEVICE_ID_NCT_EXT,
			{
				2, 1, 1, 1,
				1, 1, 1, 1,
				1, 1, 1, 0,
				1, 1, 0, 0,
				0, 0, -1, -7
			}
		},
		{
			THERMAL_DEVICE_ID_NCT_INT,
			{
				-11, -7, -5, -3,
				-3, -2, -1, 0,
				0, 0, 1, 1,
				1, 2, 2, 3,
				4, 6, 11, 18
			}
		},
	},
#endif
};

static struct rfkill_gpio_platform_data cardhu_bt_rfkill_pdata[] = {
	{
		.name           = "bt_rfkill",
		.shutdown_gpio  = TEGRA_GPIO_PU0,
		.reset_gpio     = TEGRA_GPIO_INVALID,
		.type           = RFKILL_TYPE_BLUETOOTH,
	},
};

static struct platform_device cardhu_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id             = -1,
	.dev = {
		.platform_data = &cardhu_bt_rfkill_pdata,
	},
};

static struct resource cardhu_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PU1,
			.end    = TEGRA_GPIO_PU1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device cardhu_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(cardhu_bluesleep_resources),
	.resource       = cardhu_bluesleep_resources,
};

static noinline void __init cardhu_setup_bluesleep(void)
{
	platform_device_register(&cardhu_bluesleep_device);
	return;
}

static __initdata struct tegra_clk_init_table cardhu_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s0",	"pll_a_out0",	0,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio2",	"i2s2_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "vi",		"pll_p",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static struct pca954x_platform_mode pca954x_modes_mux0[] = {
            {
                .adap_id        = 5,
            },
            {
                .adap_id        = 6,
            },
            {
                .adap_id        = 7,
            },
            {
                .adap_id        = 8,
            },
	};
static struct pca954x_platform_data pca9546_data_mux0 = {
        .modes          = pca954x_modes_mux0,
        .num_modes      = ARRAY_SIZE(pca954x_modes_mux0),
};

static struct i2c_board_info __initdata cardhu_i2c_bus3_board_info[] = {
	{
		/* I2C mux */
		I2C_BOARD_INFO("pca954x", 0x70),
        	.platform_data = &pca9546_data_mux0,
	},
};
static struct tegra_i2c_platform_data cardhu_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 10000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};


#if 0
struct tegra_wired_jack_conf audio_wr_jack_conf = {
	.hp_det_n = TEGRA_GPIO_PW2,
	.en_mic_ext = TEGRA_GPIO_PX1,
	.en_mic_int = TEGRA_GPIO_PX0,
};
#endif

static struct wm8903_platform_data cardhu_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = CARDHU_GPIO_WM8903(0),
	.gpio_cfg = {
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP1_FN_SHIFT),
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP2_FN_SHIFT) |
			WM8903_GP2_DIR,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
};

static struct i2c_board_info __initdata cardhu_codec_wm8903_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
	.platform_data = &cardhu_wm8903_pdata,
};

#if 0
static struct tegra_dmic_platform_data houdini_dmic_pdata = {
	.codec_info = {
		[0] = {
			.codec_name = "dmic-codec.0",
			.cpu_dai_name = "tegra30-i2s.2",
			.codec_dai_name = "dmic-hifi",
			.name = "I2S-TDM-1",
			.pcm_driver = "tegra-pcm-audio",
		},
	},
};

struct platform_device houdini_dmic_device = {
        .name           = "tegra-snd-dmic",
        .id             = 0,
        .dev = {
                .platform_data = &houdini_dmic_pdata,
        },
};

struct platform_device houdini_dmic_codec = {
        .name           = "dmic-codec",
        .id             = 0,
};
#endif

static void cardhu_i2c_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	tegra_i2c_device1.dev.platform_data = &cardhu_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &cardhu_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &cardhu_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &cardhu_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &cardhu_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	i2c_register_board_info(4, &cardhu_codec_wm8903_info, 1);

	i2c_register_board_info(3, cardhu_i2c_bus3_board_info, 1);
}

static struct platform_device *cardhu_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data cardhu_uart_pdata;
static struct tegra_uart_platform_data cardhu_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	struct board_info board_info;
	int debug_port_id;

	tegra_get_board_info(&board_info);

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 0;
			/* UARTB is debug port
			 *       for SLT - E1186/E1187/PM269
			 *       for E1256/E1257
			 */
		if (((board_info.sku & SKU_SLT_ULPI_SUPPORT) &&
			((board_info.board_id == BOARD_E1186) ||
			(board_info.board_id == BOARD_E1187) ||
			(board_info.board_id == BOARD_PM269))) ||
			(board_info.board_id == BOARD_E1256) ||
			(board_info.board_id == BOARD_E1257))
				debug_port_id = 1;
	}

#ifdef CONFIG_TEGRA_IRDA
	if ((board_info.board_id == BOARD_E1186) ||
		(board_info.board_id == BOARD_E1198)) {
		if (debug_port_id == 1) {
			cardhu_irda_pdata.is_irda = false;
			pr_err("UARTB is not available for IrDA\n");
		}
	}
#endif

	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		cardhu_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		cardhu_uart_devices[1] = &debug_uartb_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 2:
		/* UARTC is the debug port. */
		pr_info("Selecting UARTC as the debug console\n");
		cardhu_uart_devices[2] = &debug_uartc_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartc");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartc_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		cardhu_uart_devices[3] = &debug_uartd_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	case 4:
		/* UARTE is the debug port. */
		pr_info("Selecting UARTE as the debug console\n");
		cardhu_uart_devices[4] = &debug_uarte_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uarte");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarte_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA", debug_port_id);
		cardhu_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
	return;
}

static void __init cardhu_uart_init(void)
{
	struct clk *c;
	int i;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	cardhu_uart_pdata.parent_clk_list = uart_parent_clk;
	cardhu_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	cardhu_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	cardhu_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	cardhu_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uartb_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uartc_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uartd_device.dev.platform_data = &cardhu_uart_pdata;
	/* UARTE is used for loopback test purpose */
	tegra_uarte_device.dev.platform_data = &cardhu_loopback_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

#ifdef CONFIG_TEGRA_IRDA
	if (((board_info.board_id == BOARD_E1186) ||
		(board_info.board_id == BOARD_E1198)) &&
			cardhu_irda_pdata.is_irda) {
		cardhu_irda_pdata.parent_clk_list = uart_parent_clk;
		cardhu_irda_pdata.parent_clk_count =
					ARRAY_SIZE(uart_parent_clk);

		tegra_uartb_device.dev.platform_data = &cardhu_irda_pdata;
	}
#endif

	platform_add_devices(cardhu_uart_devices,
				ARRAY_SIZE(cardhu_uart_devices));
}

static struct tegra_camera_platform_data tegra_camera_pdata = {
	.limit_3d_emc_clk = false,
};

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.dev = {
		.platform_data = &tegra_camera_pdata,
	},
	.id = -1,
};

static void tegra_camera_init(void)
{
	/* For AP37 platform, limit 3d and emc freq when camera is ON */
	if (TEGRA_REVISION_A03 == tegra_get_revision() &&
		0xA0 == tegra_sku_id())
		tegra_camera_pdata.limit_3d_emc_clk = true;
	else
		tegra_camera_pdata.limit_3d_emc_clk = false;
}

static struct platform_device *cardhu_spi_devices[] __initdata = {
	&tegra_spi_device4,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data cardhu_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init cardhu_spi_init(void)
{
	int i;
	struct clk *c;
	struct board_info board_info, display_board_info;

	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	cardhu_spi_pdata.parent_clk_list = spi_parent_clk;
	cardhu_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device4.dev.platform_data = &cardhu_spi_pdata;
	platform_add_devices(cardhu_spi_devices,
				ARRAY_SIZE(cardhu_spi_devices));
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct tegra_asoc_platform_data cardhu_audio_wm8903_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
	.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 0,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
	},
	.i2s_param[BASEBAND]	= {
		.audio_port_id	= -1,
	},

	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 2,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S, //TEGRA_DAIFMT_DSP_A,
	},

};

static struct platform_device cardhu_audio_wm8903_device = {
	.name	= "tegra-snd-wm8903",
	.id	= 0,
	.dev	= {
		.platform_data = &cardhu_audio_wm8903_pdata,
	},
};

static struct platform_device *cardhu_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) ||  defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
//	&houdini_dmic_codec,
//	&houdini_dmic_device,
	&cardhu_bt_rfkill_device,
	&tegra_pcm_device,
	&tegra_hda_device,
	&tegra_cec_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

#define E1506_MXT_CONFIG_CRC 0x62F903
static const u8 e1506_config[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x05, 0x01, 0x00,
	0x00, 0x1E, 0x0A, 0x8B, 0x00, 0x00, 0x13, 0x0B,
	0x00, 0x10, 0x32, 0x03, 0x03, 0x00, 0x03, 0x01,
	0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0xBF, 0x03, 0x1B,
	0x02, 0x00, 0x00, 0x37, 0x37, 0x00, 0x00, 0x00,
	0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xA9, 0x7F, 0x9A, 0x0E, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0x23, 0x00, 0x00, 0x00, 0x0A,
	0x0F, 0x14, 0x19, 0x03, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x08, 0x10,
	0x00
};

#define MXT_CONFIG_CRC  0xD62DE8
static const u8 config[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x14, 0x14, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x00, 0x00,
	0x1B, 0x2A, 0x00, 0x20, 0x3C, 0x04, 0x05, 0x00,
	0x02, 0x01, 0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0xFF,
	0x02, 0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x64, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x00, 0x00, 0x00, 0x05, 0x0A, 0x15, 0x1E, 0x00,
	0x00, 0x04, 0xFF, 0x03, 0x3F, 0x64, 0x64, 0x01,
	0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x10, 0x3C, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define MXT_CONFIG_CRC_SKU2000  0xA24D9A
static const u8 config_sku2000[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x14, 0x14, 0x19,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x00, 0x00,
	0x1B, 0x2A, 0x00, 0x20, 0x3A, 0x04, 0x05, 0x00,  //23=thr  2 di
	0x04, 0x04, 0x41, 0x0A, 0x0A, 0x0A, 0x0A, 0xFF,
	0x02, 0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,  //0A=limit
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x00, 0x00, 0x00, 0x05, 0x0A, 0x15, 0x1E, 0x00,
	0x00, 0x04, 0x00, 0x03, 0x3F, 0x64, 0x64, 0x01,
	0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x10, 0x3C, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static struct platform_device *cardhu_audio_devices[] __initdata = {
		&cardhu_audio_wm8903_device,

};

#if 0
static __initdata struct tegra_clk_init_table spi_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "sbc1",       "pll_p",        52000000,       true},
	{ NULL,         NULL,           0,              0},
};
#endif

#if defined(CONFIG_USB_SUPPORT)

static void cardu_usb_hsic_postsupend(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
//	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
}

static void cardu_usb_hsic_preresume(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
//	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
#endif
}

static void cardu_usb_hsic_phy_ready(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
//	baseband_xmm_set_power_status(BBXMM_PS_L0);
#endif
}

static void cardu_usb_hsic_phy_off(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
//	baseband_xmm_set_power_status(BBXMM_PS_L3);
#endif
}

static struct tegra_usb_phy_platform_ops hsic_xmm_plat_ops = {
	.post_suspend = cardu_usb_hsic_postsupend,
	.pre_resume = cardu_usb_hsic_preresume,
	.port_power = cardu_usb_hsic_phy_ready,
	.post_phy_off = cardu_usb_hsic_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_xmm_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
	},
	.ops = &hsic_xmm_plat_ops,
};
#endif

static int hsic_enable_gpio = -1;
static int hsic_reset_gpio = -1;

void hsic_platform_open(void)
{
	int reset_gpio = 0, enable_gpio = 0;

	if (hsic_enable_gpio != -1)
		enable_gpio = gpio_request(hsic_enable_gpio, "uhsic_enable");
	if (hsic_reset_gpio != -1)
		reset_gpio = gpio_request(hsic_reset_gpio, "uhsic_reset");
	/* hsic enable signal deasserted, hsic reset asserted */
	if (!enable_gpio)
		gpio_direction_output(hsic_enable_gpio, 0 /* deasserted */);
	if (!reset_gpio)
		gpio_direction_output(hsic_reset_gpio, 0 /* asserted */);
	/* keep hsic reset asserted for 1 ms */
	udelay(1000);
	/* enable (power on) hsic */
	if (!enable_gpio)
		gpio_set_value_cansleep(hsic_enable_gpio, 1);
	udelay(1000);
	/* deassert reset */
	if (!reset_gpio)
		gpio_set_value_cansleep(hsic_reset_gpio, 1);

}

void hsic_platform_close(void)
{
	if (hsic_enable_gpio != -1) {
		gpio_set_value(hsic_enable_gpio, 0);
		gpio_free(hsic_enable_gpio);
	}
	if (hsic_reset_gpio != -1) {
		gpio_set_value(hsic_reset_gpio, 0);
		gpio_free(hsic_reset_gpio);
	}
}

void hsic_power_on(void)
{
	if (hsic_enable_gpio != -1) {
		gpio_set_value_cansleep(hsic_enable_gpio, 1);
		udelay(1000);
	}
}

void hsic_power_off(void)
{
	if (hsic_enable_gpio != -1) {
		gpio_set_value_cansleep(hsic_enable_gpio, 0);
		udelay(1000);
	}
}

#if defined(CONFIG_USB_SUPPORT)
static struct tegra_usb_phy_platform_ops hsic_plat_ops = {
	.open = hsic_platform_open,
	.close = hsic_platform_close,
	.pre_phy_on = hsic_power_on,
	.post_phy_off = hsic_power_off,
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
	},
	.ops = &hsic_plat_ops,
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode        = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "vdd_vbus_typea_usb",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "vdd_vbus_micro_usb",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};
#endif

#if defined(CONFIG_USB_SUPPORT)
static void cardhu_usb_init(void)
{
	struct board_info bi;

	tegra_get_board_info(&bi);

	/* OTG should be the first to be registered */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	/* setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	if (bi.board_id == BOARD_PM267) {
		hsic_enable_gpio = EN_HSIC_GPIO;
		hsic_reset_gpio = PM267_SMSC4640_HSIC_HUB_RESET_GPIO;
		tegra_ehci2_device.dev.platform_data = &tegra_ehci2_hsic_pdata;
		platform_device_register(&tegra_ehci2_device);
	} else if (bi.board_id == BOARD_E1256) {
		hsic_enable_gpio = EN_HSIC_GPIO;
		tegra_ehci2_device.dev.platform_data = &tegra_ehci2_hsic_pdata;
		platform_device_register(&tegra_ehci2_device);
	} else if (bi.board_id == BOARD_E1186) {
		tegra_ehci2_device.dev.platform_data =
						&tegra_ehci2_hsic_xmm_pdata;
		/* ehci2 registration happens in baseband-xmm-power  */
	} else {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
		platform_device_register(&tegra_ehci2_device);
	}

	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);

}
#else
static void cardhu_usb_init(void) { }
#endif

static struct tegra_pci_platform_data cardhu_pci_platform_data = {
	.port_status[0]	= 1,
	.port_status[1]	= 1,
	.port_status[2]	= 1,
	.use_dock_detect	= 0,
	.gpio		= 0,
};

static void cardhu_pci_init(void)
{
	struct board_info board_info;

//	tegra_gpio_enable(TEGRA_GPIO_PC6);
//	tegra_gpio_enable(TEGRA_GPIO_PK3);
//	tegra_gpio_enable(TEGRA_GPIO_PK4);
	tegra_get_board_info(&board_info);
	cardhu_pci_platform_data.port_status[0] = 0;
	cardhu_pci_platform_data.port_status[1] = 0;
	cardhu_pci_platform_data.port_status[2] = 1;
	cardhu_pci_platform_data.use_dock_detect = 0;
	cardhu_pci_platform_data.gpio = -1;
	cardhu_pci_platform_data.gpio = 0;
	
	if (gpio_request(TEGRA_GPIO_PC6, "en-vdd-pex-slot") == 0) {
		gpio_direction_output(TEGRA_GPIO_PC6, 1);
		gpio_set_value(TEGRA_GPIO_PC6, 1);
		mdelay(100);
		gpio_set_value(TEGRA_GPIO_PC6, 0);
	} else {
		pr_err("%s: GPIO_request failed for en-vdd-pex-slot\n",
				__func__);
	}

/*
        if (gpio_request(TEGRA_GPIO_PV2, "en-vdd-pex-slot_backup") == 0) {
                gpio_direction_output(TEGRA_GPIO_PV2, 1);
                gpio_set_value(TEGRA_GPIO_PV2, 0);
                mdelay(100);
                gpio_set_value(TEGRA_GPIO_PV2, 1);
        } else {
                pr_err("%s: GPIO_request failed for en-vdd-pex-slot2\n",
                                __func__);
	}
*/	
	if (gpio_request(TEGRA_GPIO_PK3, "lan1-ena") == 0) {
		gpio_direction_output(TEGRA_GPIO_PK3, 1);
		gpio_set_value(TEGRA_GPIO_PK3, 1);
	} else {
		pr_err("%s: GPIO_request failed for lan1-ena\n",
				__func__);
 	}

	if (gpio_request(TEGRA_GPIO_PK4, "lan2-ena") == 0) {
		gpio_direction_output(TEGRA_GPIO_PK4, 1);
		gpio_set_value(TEGRA_GPIO_PK4, 1);
	} else {
		pr_err("%s: GPIO_request failed for lan2-ena\n",
				__func__);
	}

	if ((board_info.board_id == BOARD_E1186) ||
			(board_info.board_id == BOARD_E1187) ||
			(board_info.board_id == BOARD_E1291) ||
			(board_info.board_id == BOARD_PM315)) {
printk("Registering PCI device\n");
		tegra_pci_device.dev.platform_data = &cardhu_pci_platform_data;
		platform_device_register(&tegra_pci_device);
	}
}

static void __init tegra_cardhu_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	tegra_thermal_init(&thermal_data,
				throttle_list,
				ARRAY_SIZE(throttle_list));
	tegra_clk_init_from_table(cardhu_clk_init_table);
	cardhu_pinmux_init();
	cardhu_i2c_init();
	cardhu_spi_init();
	cardhu_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	cardhu_edp_init();
#endif
	cardhu_uart_init();
	tegra_camera_init();
	platform_add_devices(cardhu_devices, ARRAY_SIZE(cardhu_devices));
	platform_add_devices(cardhu_audio_devices, ARRAY_SIZE(cardhu_audio_devices));
	tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	cardhu_sdhci_init();
	cardhu_regulator_init();
	cardhu_suspend_init();
	cardhu_panel_init();
	cardhu_sensors_init();
	cardhu_setup_bluesleep();
	cardhu_pins_state_init();
	cardhu_emc_init();
	tegra_release_bootloader_fb();
	cardhu_pci_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
}

static void __init tegra_cardhu_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* Support 1920X1080 32bpp,double buffered on HDMI*/
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_16M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

static const char *cardhu_dt_board_compat[] = {
	"nvidia,cardhu",
	NULL
};

MACHINE_START(HOUDINI, "houdini")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_cardhu_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_cardhu_init,
	.dt_compat	= cardhu_dt_board_compat,
MACHINE_END
