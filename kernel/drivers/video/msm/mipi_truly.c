/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define DEBUG
#include <linux/leds.h>
#include <mach/gpio.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_truly.h"

#define GPIO_TRULY_LCD_DSI_SEL 125
#define GPIO_TRULY_LCD_RESET 129

static int prev_bl = 17;
static int gpio_backlight_en = 0xff;

static int first_on = 1;
extern int boot_splash;

static struct msm_panel_common_pdata *mipi_truly_pdata;
static struct dsi_buf truly_tx_buf;
static struct dsi_buf truly_rx_buf;

#ifndef MIPI_TRULY_FAKE_PANEL

#define TRULY_CMD_DELAY 0
#define MIPI_SETTING_DELAY 0
#define TRULY_SLEEP_OFF_DELAY 120
#define TRULY_DISPLAY_ON_DELAY 0

/* common setting */
static char extend_cmd_enable[4] = {0xB9, 0xFF, 0x83, 0x69};
static char exit_sleep[2] = {0x11, 0x00};
static char display_on[2] = {0x29, 0x00};
static char display_off[2] = {0x28, 0x00};
static char enter_sleep[2] = {0x10, 0x00};
static char write_memory_start[2] = {0x2C, 0x00};
static char mipi_setting[14] = {
	0xBA, 0x00, 0xA0, 0xC6,
	0x00, 0x0A, 0x00, 0x10,
	0x30, 0x6F, 0x02, 0x11,
	0x18, 0x40, 
};
static struct dsi_cmd_desc truly_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(enter_sleep), enter_sleep}
};
/* video mode setting */
static char video_display_setting[16] = {
	0xB2, 0x00, 0x23, 0x62, 
	0x62, 0x70, 0x00, 0xFF, 
	0x00, 0x00, 0x00, 0x00, 
	0x03, 0x03, 0x00, 0x01,
};
static char video_wave_cycle_setting[6] = {0xB4, 0x00, 0x1D, 0x5F, 0x0E, 0x06};
static char video_gip_setting[27] = {
	0xD5, 0x00, 0x04, 0x03,
	0x00, 0x01, 0x05, 0x1C,
	0x70, 0x01, 0x03, 0x00,
	0x00, 0x40, 0x06, 0x51,
	0x07, 0x00, 0x00, 0x41,
	0x06, 0x50, 0x07, 0x07,
	0x0F, 0x04, 0x00, 
};
static char video_power_setting[20] = {
	0xB1, 0x01, 0x00, 0x34,
	0x06, 0x00, 0x0F, 0x0F,
	0x2A, 0x32, 0x3F, 0x3F,
	0x07, 0x3A, 0x01, 0xE6,
	0xE6, 0xE6, 0xE6, 0xE6,
};
static char video_vcom_setting[3] = {0xB6, 0x56, 0x56};
static char video_pannel_setting[2] = {0xCC, 0x02};
static char video_gamma_setting[35] = {
	0xE0, 0x00, 0x1D, 0x22,
	0x38, 0x3D, 0x3F, 0x2E,
	0x4A, 0x06, 0x0D, 0x0F,
	0x13, 0x15, 0x13, 0x16,
	0x10, 0x19, 0x00, 0x1D,
	0x22, 0x38, 0x3D, 0x3F,
	0x2E, 0x4A, 0x06, 0x0D,
	0x0F, 0x13, 0x15, 0x13,
	0x16, 0x10, 0x19, 
};

static struct dsi_cmd_desc truly_video_display_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(extend_cmd_enable), extend_cmd_enable},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(video_display_setting), video_display_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(video_wave_cycle_setting), video_wave_cycle_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(video_gip_setting), video_gip_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(video_power_setting), video_power_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(video_vcom_setting), video_vcom_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(video_pannel_setting), video_pannel_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(video_gamma_setting), video_gamma_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(mipi_setting), mipi_setting},
	{DTYPE_DCS_WRITE, 1, 0, 0, TRULY_SLEEP_OFF_DELAY, sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, TRULY_DISPLAY_ON_DELAY, sizeof(display_on), display_on},
};

/* command mode setting */
static char cmd_oscillator_setting[3] = {
	0xB0, 0x01, 0x09,
};
static char cmd_power_setting[20] = {
	0xB1, 0x85, 0x00, 0x34, 
	0x07, 0x00, 0x0F, 0x0F, 
	0x2A, 0x32, 0x3F, 0x3F, 
	0x01, 0x3A, 0x01, 0xE6, 
	0xE6, 0xE6, 0xE6, 0xE6
};
static char cmd_display_setting[16] = {
	0xB2, 0x00, 0x20, 0x05, 
	0x05, 0x70, 0x00, 0xFF, 
	0x00, 0x00, 0x00, 0x00, 
	0x03, 0x03, 0x00, 0x01
};
static char cmd_wave_cycle_setting[6] = {0xB4, 0x00, 0x18, 0x80, 0x06, 0x02};
static char cmd_vcom_setting[3] = {0xB6, 0x42, 0x42};
static char cmd_gip_setting[27] = {
	0xD5, 0x00, 0x04, 0x03, 
	0x00, 0x01, 0x05, 0x28, 
	0x70, 0x01, 0x03, 0x00, 
	0x00, 0x40, 0x06, 0x51, 
	0x07, 0x00, 0x00, 0x41, 
	0x06, 0x50, 0x07, 0x07, 
	0x0F, 0x04, 0x00
};
static char cmd_gamma_setting[35] = {
	0xE0, 0x00, 0x13, 0x19, 
	0x38, 0x3D, 0x3F, 0x28, 
	0x46, 0x07, 0x0D, 0x0E, 
	0x12, 0x15, 0x12, 0x14, 
	0x0F, 0x17, 0x00, 0x13, 
	0x19, 0x38, 0x3D, 0x3F, 
	0x28, 0x46, 0x07, 0x0D, 
	0x0E, 0x12, 0x15, 0x12, 
	0x14, 0x0F, 0x17
};
static char cmd_dgc_lut_setting[128] = {
	0xC1, 0x01,
	//R
	0x04, 0x13, 0x1A, 0x20, 
	0x27, 0x2C, 0x32, 0x36, 
	0x3F, 0x47, 0x50, 0x59, 
	0x60, 0x68, 0x71, 0x7B, 
	0x82, 0x89, 0x91, 0x98, 
	0xA0, 0xA8, 0xB0, 0xB8, 
	0xC1, 0xC9, 0xD0, 0xD7, 
	0xE0, 0xE7, 0xEF, 0xF7, 
	0xFE, 0xCF, 0x52, 0x34, 
	0xF8, 0x51, 0xF5, 0x9D, 
	0x75, 0x00,
	//G
	0x04, 0x13, 0x1A, 0x20, 
	0x27, 0x2C, 0x32, 0x36, 
	0x3F, 0x47, 0x50, 0x59, 
	0x60, 0x68, 0x71, 0x7B, 
	0x82, 0x89, 0x91, 0x98, 
	0xA0, 0xA8, 0xB0, 0xB8, 
	0xC1, 0xC9, 0xD0, 0xD7, 
	0xE0, 0xE7, 0xEF, 0xF7, 
	0xFE, 0xCF, 0x52, 0x34, 
	0xF8, 0x51, 0xF5, 0x9D, 
	0x75, 0x00,
	//B
	0x04, 0x13, 0x1A, 0x20, 
	0x27, 0x2C, 0x32, 0x36, 
	0x3F, 0x47, 0x50, 0x59, 
	0x60, 0x68, 0x71, 0x7B, 
	0x82, 0x89, 0x91, 0x98, 
	0xA0, 0xA8, 0xB0, 0xB8, 
	0xC1, 0xC9, 0xD0, 0xD7, 
	0xE0, 0xE7, 0xEF, 0xF7, 
	0xFE, 0xCF, 0x52, 0x34, 
	0xF8, 0x51, 0xF5, 0x9D, 
	0x75, 0x00,
};
static char cmd_pixel_format_setting[2] = {0x3A, 0x77};

static struct dsi_cmd_desc truly_cmd_display_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(extend_cmd_enable), extend_cmd_enable},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(cmd_oscillator_setting), cmd_oscillator_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(cmd_power_setting), cmd_power_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(cmd_display_setting), cmd_display_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(cmd_wave_cycle_setting), cmd_wave_cycle_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(cmd_vcom_setting), cmd_vcom_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(cmd_gip_setting), cmd_gip_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, MIPI_SETTING_DELAY, sizeof(cmd_gamma_setting), cmd_gamma_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(cmd_dgc_lut_setting), cmd_dgc_lut_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, MIPI_SETTING_DELAY, sizeof(mipi_setting), mipi_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(cmd_pixel_format_setting), cmd_pixel_format_setting},
	{DTYPE_DCS_WRITE, 1, 0, 0, TRULY_SLEEP_OFF_DELAY, sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(display_on), display_on},
	{DTYPE_DCS_WRITE, 1, 0, 0, TRULY_CMD_DELAY, sizeof(write_memory_start), write_memory_start},
};

void mipi_truly_sel_mode(int mode)
{
	gpio_set_value_cansleep(GPIO_TRULY_LCD_DSI_SEL, (mode == DSI_VIDEO_MODE) ? 1 : 0);
	return;
}

static int mipi_truly_lcd_reset(void)
{
    int rc = 0;

    pr_debug("%s\n", __func__);

    gpio_set_value_cansleep(GPIO_TRULY_LCD_RESET, 1);
    udelay(1);
    gpio_set_value_cansleep(GPIO_TRULY_LCD_RESET, 0);
    udelay(15);
    gpio_set_value_cansleep(GPIO_TRULY_LCD_RESET, 1);
    msleep(5);

    return rc;
}

#endif //MIPI_TRULY_FAKE_PANEL

extern struct list_head leds_list;

static int late_display_set_brightness(enum led_brightness brightness)
{
    struct led_classdev *led_cdev;
	int found = 0;

    //pr_debug("%s: brightness = %d\n", __func__, brightness);

    list_for_each_entry(led_cdev, &leds_list, node) {
        if (!strncmp(led_cdev->name, "lcd-backlight", 13)) {
			found = 1;
            break;
        }
    }

    if (found) {
        led_brightness_set(led_cdev, brightness);
    } else {
        pr_err("%s: set brightness failed, not found lcd-backlight\n", __func__);
        return -1;
    }

    return 0;
}

static int mipi_truly_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;

	pr_debug("mipi_truly_lcd_on E\n");

	mfd = platform_get_drvdata(pdev);

	if (!mfd) {
		return -ENODEV;
	}

	if (mfd->key != MFD_KEY) {
		return -EINVAL;
	}

	mipi  = &mfd->panel_info.mipi;
	pr_debug("%s: mode = %d\n", __func__, mipi->mode);
#ifndef MIPI_TRULY_FAKE_PANEL
	/* select DSI mode */
	mipi_truly_sel_mode(mipi->mode);

	if (mipi_truly_lcd_reset() < 0) {
		pr_err("mipi_truly_lcd_reset error\n");
		return -EINVAL;
	}

	if (mipi->mode == DSI_VIDEO_MODE) {
		mipi_dsi_cmds_tx(mfd, &truly_tx_buf, truly_video_display_on_cmds,
			ARRAY_SIZE(truly_video_display_on_cmds));
	} else if (mipi->mode == DSI_CMD_MODE) {
		mipi_dsi_cmds_tx(mfd, &truly_tx_buf, truly_cmd_display_on_cmds,
			ARRAY_SIZE(truly_cmd_display_on_cmds));
	}

	if (boot_splash && first_on) {
		late_display_set_brightness(LED_FULL);
		first_on = 0;
	}
#endif

	pr_debug("mipi_truly_lcd_on X\n");

	return 0;
}

static int mipi_truly_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	pr_debug("mipi_truly_lcd_off E\n");

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

#ifndef MIPI_TRULY_FAKE_PANEL
	mipi_dsi_cmds_tx(mfd, &truly_tx_buf, truly_display_off_cmds,
			ARRAY_SIZE(truly_display_off_cmds));
#endif

	pr_debug("mipi_truly_lcd_off X\n");

	return 0;
}

static int __devinit mipi_truly_lcd_probe(struct platform_device *pdev)
{
	int rc = 0;
	pr_debug("%s\n", __func__);

	if (pdev->id == 0) {
		mipi_truly_pdata = pdev->dev.platform_data;
	    if (mipi_truly_pdata) {
			gpio_backlight_en = mipi_truly_pdata->gpio;
	    }

		return rc;
	}

	msm_fb_add_device(pdev);

	return rc;
}

static struct platform_driver this_driver = {
	.probe  = mipi_truly_lcd_probe,
	.driver = {
		.name   = "mipi_truly",
	},
};

static void mipi_truly_set_backlight(struct msm_fb_data_type *mfd)
{
	int step = 0, i = 0;
	unsigned long flags;
	int bl_level = mfd->bl_level;

	/* real backlight level, 1 - max, 16 - min, 17 - off */
	bl_level = 17 - bl_level;

	if (bl_level > prev_bl) {
		step = bl_level - prev_bl;
		if (bl_level == 17) {
			step--;
		}
	} else if (bl_level < prev_bl) {
		step = bl_level + 16 - prev_bl;
	} else {
		pr_debug("%s: no change\n", __func__);
		return;
	}

	if (bl_level == 17) {
		/* turn off backlight */
		pr_debug("%s: turn off backlight\n", __func__);
		gpio_set_value(gpio_backlight_en, 0);
	} else {
		local_irq_save(flags);

		if (prev_bl == 17) {
			/* turn on backlight */
			pr_debug("%s: turn on backlight\n", __func__);
			gpio_set_value(gpio_backlight_en, 1);
			udelay(30);
		}
		
		/* adjust backlight level */
		for (i = 0; i < step; i++) {
			gpio_set_value(gpio_backlight_en, 0);
			udelay(1);
			gpio_set_value(gpio_backlight_en, 1);
			udelay(1);
		}

		local_irq_restore(flags);
	}
	msleep(1);
	
	prev_bl = bl_level;

	return;
}

static struct msm_fb_panel_data truly_panel_data = {
	.on	= mipi_truly_lcd_on,
	.off = mipi_truly_lcd_off,
	.set_backlight = mipi_truly_set_backlight,
};

static int ch_used[3];

int mipi_truly_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	pr_debug("mipi_truly_device_register\n");

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_truly", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	truly_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &truly_panel_data,
		sizeof(truly_panel_data));
	if (ret) {
		pr_debug("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_debug("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

int mipi_truly_lcd_late_init(void)
{
	pr_debug("mipi_truly_lcd_late_init\n");
	
	mipi_dsi_buf_alloc(&truly_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&truly_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

EXPORT_SYMBOL(mipi_truly_lcd_late_init);

static int __init mipi_truly_lcd_init(void)
{
  if (boot_splash) {
    /* boot splash, do nothing */
  } else {
    mipi_truly_lcd_late_init();
  }

  return 0;
}
module_init(mipi_truly_lcd_init);
