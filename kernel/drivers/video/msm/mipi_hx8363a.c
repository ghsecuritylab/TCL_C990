/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_hx8363a.h"

#define HX8363A_BL_CONTROL
#define GPIO_HX8363A_LCD_RESET 85

#ifdef GPIO_HX8363A_LCD_RESET
#include <linux/gpio.h>
#endif

extern int boot_splash;

struct hx8363a_state_type {
	boolean disp_powered_up;
	boolean disp_initialized;
	boolean display_on;
};

static struct hx8363a_state_type hx8363a_state = {1, 0, 0};

static struct msm_panel_common_pdata *mipi_hx8363a_pdata;
static struct dsi_buf hx8363a_tx_buf;
static struct dsi_buf hx8363a_rx_buf;

#define HX8363A_CMD_DELAY		0
#define HX8363A_SLEEP_OFF_DELAY	150
#define HX8363A_DISPLAY_ON_DELAY	150

//#define USES_DBG_PARA


#ifndef USES_DBG_PARA
static char hx8363a_video_vcom_setting[] = {0xB6, 0x15}; //{0xB6, 0x2d}; //
#else
static char hx8363a_video_vcom_setting[] = {0xB6, 0x10};
#endif
static char hx8363a_video_pannel_setting[] = {0xCC, 0x09 }; //{0xCC, 0x01 }; //

static char hx8363a_exit_sleep[] = {0x11};
static char hx8363a_display_on[] = {0x29};
static char hx8363a_display_off[] = {0x28};
static char hx8363a_enter_sleep[] = {0x10};

static char hx8363a_video_wave_cycle_setting[] = 
{
	/* Set_CYC CPT   */

	0xB4, 0x08, 0x12, 0x72, 
	0x12, 0x06, 0x03, 0x54, 
	0x03, 0x4e
};

static char hx8363a_video_power_setting[] = {
#ifndef USES_DBG_PARA
	0xB1, 0x81, 0x34, 0x08,
	0x34, 0x02, 0x13, 0x11,
	0x11, 0x2D, 0x35, 0x3F,
	0x3F,
#else
	0xB1, 0x81, 0x30, 0x08,
	0x34, 0x01, 0x13, 0x11,
	0x00, 0x35, 0x3e, 0x16,
	0x16,
#endif
};

static char hx8363a_video_setrgbif[]=
{
	0xB3, 0x00, 
};

static char hx8363a_video_gamma_setting[] = {
	0xE0, 
	0x01, 0x1D, 0x22,0x33, 
	0x3A, 0x3F, 0x06,0x8D, 
	0x8E, 0x91, 0x54,0x12, 
	0x14, 0x4f, 0x18,
	
	0x01, 0x1D, 0x22, 0x33,
	0x3A, 0x3F, 0x06, 0x8D,
	0x8E, 0x91, 0x54, 0x12,
	0x14, 0x4f, 0x18
};

// the following are newly defined! 
static char hx8363a_video_colmod[]=
{
#ifndef USES_DBG_PARA
	0x3A, 0x70,
#else
	0x3A, 0x77,
#endif
};

static char hx8363a_video_ptba[]=
{
	0xBF, 0x00, 0x10
};

static char hx8363a_video_set_disp[]=
{
	0xB2, 0x33, 0x33, 0x22
};

static char hx8363a_extend_cmd_enable[] = 
{
	0xB9, 0xFF, 0x83, 0x63
};

static char hx8363a_video_mipi_setting[] = {
	0xBA, 0x80, 0x00, 0x10,
	0x08, 0x08, 0x10, 0x7C,
#if defined(CONFIG_PROJECT_QRD) || defined(CONFIG_PROJECT_ES02)
	0x6E, 0x6D, 0x0A, 0x01,        // 01: 2 lanes, 00: 1 lane event mode 
#else
	0x6E, 0x6D, 0x0A, 0x00,        // 01: 2 lanes, 00: 1 lane event mode 
#endif
	0x84, 0x43                     // 0x84: burst mode, 0x80: non-burst mode  
};

/* DGC: dynamic gain control.  */
static char hx8363a_video_dgc_settings[]=
{
	0xC1,0x01,	
	0x00,0x04,0x0D,0x17,0x1F,0x28,0x30,0x37,
	0x3F,0x47,0x4F,0x56,0x5D,0x63,0x6A,0x70,
	0x76,0x7C,0x81,0x88,0x8F,0x96,0x9D,0xA2,
	0xA8,0xAE,0xB5,0xBD,0xC5,0xCB,0xD3,0xDF,
	0xFF,0x45,0x3C,0x90,0x21,0x4E,0xA3,0xF4,
	0x63,0x00,                    // R	
	
	0x00,0x04,0x0D,0x17,0x1F,0x28,0x30,0x37,
	0x3F,0x47,0x4F,0x56,0x5D,0x63,0x6A,0x70,
	0x76,0x7C,0x81,0x88,0x8F,0x96,0x9D,0xA2,
	0xA8,0xAE,0xB5,0xBD,0xC5,0xCB,0xD3,0xDF,
	0xFF,0x45,0x3C,0x90,0x21,0x4E,0xA3,0xF4,
	0x63,0x00,                             // G
	
	0x00,0x04,0x0D,0x17,0x1F,0x28,0x30,0x37,
	0x3F,0x47,0x4F,0x56,0x5D,0x63,0x6A,0x70,
	0x76,0x7C,0x81,0x88,0x8F,0x96,0x9D,0xA2,
	0xA8,0xAE,0xB5,0xBD,0xC5,0xCB,0xD3,0xDF,
	0xFF,0x45,0x3C,0x90,0x21,0x4E,0xA3,0xF4,
	0x63,0x00                               // B        
};

#ifdef HX8363A_BL_CONTROL
static char hx8363a_video_cabc_setting[] = {0x55, 0x00};
static char hx8363a_video_bctrl_bl_on_setting[] = {0x53, 0x24};
static char hx8363a_video_bl_value[] = {0x51, 0x80};
#endif

static struct dsi_cmd_desc hx8363a_video_display_init_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_extend_cmd_enable), hx8363a_extend_cmd_enable},			
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_power_setting), hx8363a_video_power_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_SLEEP_OFF_DELAY,
        sizeof(hx8363a_exit_sleep), hx8363a_exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_set_disp), hx8363a_video_set_disp},
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_colmod), hx8363a_video_colmod},		
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_wave_cycle_setting), hx8363a_video_wave_cycle_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_ptba), hx8363a_video_ptba},	
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_vcom_setting), hx8363a_video_vcom_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_pannel_setting), hx8363a_video_pannel_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_gamma_setting), hx8363a_video_gamma_setting},	
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_dgc_settings), hx8363a_video_dgc_settings},
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_mipi_setting), hx8363a_video_mipi_setting},	
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_setrgbif), hx8363a_video_setrgbif},
#ifdef HX8363A_BL_CONTROL
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_cabc_setting), hx8363a_video_cabc_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_bctrl_bl_on_setting), hx8363a_video_bctrl_bl_on_setting},
#endif
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_DISPLAY_ON_DELAY,
        sizeof(hx8363a_display_on), hx8363a_display_on},
};

#ifdef HX8363A_BL_CONTROL
static struct dsi_cmd_desc hx8363a_video_bl_value_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_video_bl_value), hx8363a_video_bl_value},
};
#endif

#ifndef GPIO_HX8363A_LCD_RESET
static struct dsi_cmd_desc hx8363a_video_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, HX8363A_SLEEP_OFF_DELAY,
        sizeof(hx8363a_exit_sleep), hx8363a_exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, HX8363A_DISPLAY_ON_DELAY,
        sizeof(hx8363a_display_on), hx8363a_display_on},
};
#endif

static struct dsi_cmd_desc hx8363a_video_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 150,
        sizeof(hx8363a_enter_sleep), hx8363a_enter_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 100,
        sizeof(hx8363a_display_off), hx8363a_display_off},
};

#if 0
static char hx8363a_manufacture_id[2] = {0x0A, 0x00}; /* DTYPE_DCS_READ */

static struct dsi_cmd_desc hx8363a_manufacture_id_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY,
        sizeof(hx8363a_extend_cmd_enable), hx8363a_extend_cmd_enable},			
	{DTYPE_DCS_READ, 1, 0, 1, 100,
	 sizeof(hx8363a_manufacture_id), hx8363a_manufacture_id}
};

static uint32 mipi_hx8363a_manufacture_id(struct msm_fb_data_type *mfd)
{
	struct dsi_buf *rp, *tp;
	//struct dsi_cmd_desc *cmd;
	uint32 *lp;

	tp = &hx8363a_tx_buf;
	rp = &hx8363a_rx_buf;
	//cmd = &hx8363a_manufacture_id_cmd;
	mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_manufacture_id_cmds,
			ARRAY_SIZE(hx8363a_manufacture_id_cmds));
	lp = (uint32 *)rp->data;
	pr_info("%s: manufacture_id=%x", __func__, *lp);
	return *lp;
}
#endif

#ifdef GPIO_HX8363A_LCD_RESET
static void mipi_hx8363a_lcd_reset(void)
{
	gpio_set_value_cansleep(GPIO_HX8363A_LCD_RESET, 1);
	msleep(10);
	gpio_set_value_cansleep(GPIO_HX8363A_LCD_RESET, 0);
	msleep(10);
	gpio_set_value_cansleep(GPIO_HX8363A_LCD_RESET, 1);
	msleep(50);
}
#endif


static int curr_bl_level = 0;
static void mipi_hx8363a_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level; // mfd.bl_max = 15;

	//pr_info("%s, bl_level:%d", __func__, bl_level);

	if (hx8363a_state.disp_initialized) {
		if (curr_bl_level == bl_level)
			return;

		curr_bl_level = bl_level;
#ifdef HX8363A_BL_CONTROL
		// 0-15 to 0-255
		hx8363a_video_bl_value[1] = (!!bl_level) ? ((bl_level * 16) + 15) : 0;
		mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_video_bl_value_cmds,
			ARRAY_SIZE(hx8363a_video_bl_value_cmds));
#endif
		mipi_hx8363a_pdata->pmic_backlight(!!bl_level); // For the backlight control if hx8363a LED_PWD is not connected
		//msleep(20);
	}

	return;
}

static int mipi_hx8363a_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
    
	if (mfd->key != MFD_KEY)
		return -EINVAL;
    
#ifdef GPIO_HX8363A_LCD_RESET
	mipi_hx8363a_lcd_reset();
#endif
            
#if 0
	msleep(20);
	//mipi_dsi_cmd_bta_sw_trigger(); /* clean up ack_err_status */    
	mipi_hx8363a_manufacture_id(mfd);
	pr_info("%s:  read hx8363a ID.\n", __func__);
#endif
    
#ifdef GPIO_HX8363A_LCD_RESET
	mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_video_display_init_cmds,
		ARRAY_SIZE(hx8363a_video_display_init_cmds));

	hx8363a_state.disp_initialized = TRUE;
#else
	if (!hx8363a_state.disp_initialized) {
		pr_info("%s, initialize...", __func__);
		mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_video_display_init_cmds,
			ARRAY_SIZE(hx8363a_video_display_init_cmds));
		hx8363a_state.disp_initialized = TRUE;
		hx8363a_state.display_on = TRUE;
	}
	else 
	if (!hx8363a_state.display_on) {
		pr_info("%s", __func__);
		mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_video_display_on_cmds,
			ARRAY_SIZE(hx8363a_video_display_on_cmds));
		hx8363a_state.display_on = TRUE;
	}
#endif

	return 0;
}

static int mipi_hx8363a_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (0 != curr_bl_level) {
		curr_bl_level = 0;
#ifdef HX8363A_BL_CONTROL
		hx8363a_video_bl_value[1] = 0;
		mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_video_bl_value_cmds,
			ARRAY_SIZE(hx8363a_video_bl_value_cmds));
#endif
		mipi_hx8363a_pdata->pmic_backlight(0); // For the backlight control if hx8363a LED_PWD is not connected

		pr_info("%s, set backlight level to 0.\n", __func__);
		msleep(20);
	}

#ifdef GPIO_HX8363A_LCD_RESET
	if (hx8363a_state.disp_initialized) {
		hx8363a_state.disp_initialized = FALSE;

		mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_video_display_off_cmds, 1);
#if 0
		msleep(250);
		mipi_hx8363a_lcd_reset();
#else
		gpio_set_value_cansleep(GPIO_HX8363A_LCD_RESET, 0);
#endif

		pr_info("%s", __func__);
	}
#else
	if (hx8363a_state.display_on) {
		pr_info("%s", __func__);
		mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_video_display_off_cmds,
			ARRAY_SIZE(hx8363a_video_display_off_cmds));
			//1);
		hx8363a_state.display_on = FALSE;
		msleep(20);
	}
#endif

	return 0;
}


static int __devinit mipi_hx8363a_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_hx8363a_pdata = pdev->dev.platform_data;
		return 0;
	}
    
	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_hx8363a_lcd_probe,
	.driver = {
		.name   = "mipi_truly_hx8363a",
	},
};

static struct msm_fb_panel_data hx8363a_panel_data = {
	.on		= mipi_hx8363a_lcd_on,
	.off		= mipi_hx8363a_lcd_off,
	.set_backlight	= mipi_hx8363a_set_backlight,
};

static int ch_used[3];

int mipi_hx8363a_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_truly_hx8363a", (panel << 8)|channel);

	if (!pdev)
		return -ENOMEM;

	hx8363a_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &hx8363a_panel_data,
				sizeof(hx8363a_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);

	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

int mipi_hx8363a_lcd_late_init(void)
{
	mipi_dsi_buf_alloc(&hx8363a_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&hx8363a_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

EXPORT_SYMBOL(mipi_hx8363a_lcd_late_init);

static int __init mipi_hx8363a_lcd_init(void)
{
  if (boot_splash) {
    /* boot splash, do nothing */
  } else {
    mipi_hx8363a_lcd_late_init();
  }

  return 0;
}
module_init(mipi_hx8363a_lcd_init);

