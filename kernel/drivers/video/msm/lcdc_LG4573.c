/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/* 
 Created by X.110311
 Modified by X.110316
 Modified by X.110329, for ILITEK 9481b driver IC
 Modified by X.111016, LG4573

*/



#include <linux/delay.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/msm_iomap.h>
#include <mach/mpp.h>
#include <mach/pmic.h>
#include <mach/vreg.h>
#include <linux/moduleparam.h>
#include <linux/kthread.h>

#define RESET_USING__GPIO
#define BACKLIGHT_USING__GPIO
//#define BACLKIGHT_USING__GPIO_PWM

#define LCDC2_SPI_SDO 90 //90
#define LCDC2_SPI_SDI 91 //91
#define LCDC2_SPI_CS  88
#define LCDC2_SPI_SCK 89
#define RESET_GPIO    85
#define BACKLIGHT_GPIO 84
#define BACKLIGHT_GPIO_HIGH gpio_set_value(BACKLIGHT_GPIO, 1)
#define BACKLIGHT_GPIO_LOW  gpio_set_value(BACKLIGHT_GPIO, 0)
#define LCDC2_SPI_SDO_HIGH gpio_set_value(LCDC2_SPI_SDO, 1)
#define LCDC2_SPI_SDO_LOW  gpio_set_value(LCDC2_SPI_SDO, 0)
#define LCDC2_SPI_SDI_READ  gpio_get_value(LCDC2_SPI_SDI)

#define LCDC2_SPI_SDI_LOW	gpio_set_value(LCDC2_SPI_SDI,1)
#define LCDC2_SPI_SDI_HIGH	gpio_set_value(LCDC2_SPI_SDI,0)

#define LCDC2_SPI_CS_HIGH gpio_set_value(LCDC2_SPI_CS, 1)
#define LCDC2_SPI_CS_LOW  gpio_set_value(LCDC2_SPI_CS, 0)
#define LCDC2_SPI_SCK_HIGH gpio_set_value(LCDC2_SPI_SCK, 1)
#define LCDC2_SPI_SCK_LOW  gpio_set_value(LCDC2_SPI_SCK, 0)
#define LCDC2_SPI_NDELAY //ndelay(50)

struct LG4573_state_type {
	bool	disp_initialized;/* disp initialized */
	bool	display_on;	/* lcdc is on */
	bool	disp_powered_up;/* lcdc power and lcd bl power on */
};

typedef struct LG4573_state_type	 LG4573_state_t;
static LG4573_state_t	LG4573_state = { 0 };

extern int power_vreg_gp6(int on);

#if defined(BACLKIGHT_USING__GPIO_PWM)

#define PERF_GP_CLK_RATE	19200000
static struct clk *gp_clk; // GPIO_27
extern int clk_enable(struct clk *clk);
extern void clk_disable(struct clk *clk);
extern struct clk *clk_get(struct device *dev, const char *id);
extern int clk_set_rate(struct clk *clk, unsigned long rate);
#endif

static int LG4573_spi_init(void);
static void lcdc_LG4573_intersleep(void);
static void lcdc_LG4573_exitsleep(void);
static int lcdc_LG4573_panel_off(struct platform_device *pdev);

static void LG4573_spi_write(bool bReg, unsigned char uc);
static void write_reg_LG4573(unsigned char uc);
static void write_data_LG4573(unsigned char uc);
//static void read_reg_LG4573(unsigned char uc);
//static void read_data_LG4573(unsigned char *uc);
static void lcdc_LG4573_set_backlight(struct msm_fb_data_type *mfd);


static void LG4573_disp_on(void);
static void LG4573_disp_on2(unsigned char polarity);



#ifdef DEBUG_SYSFS

static ssize_t lcd_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ok.\n");
}

static ssize_t lcd_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int error;
	unsigned long data;
	unsigned char val;

	if (NULL == buf) {
		printk(KERN_ERR "buf is null, count:%d\n", count);
		return 1;
	}
	
#if 1
	error = strict_strtoul(buf, 10, &data);
	if (error) {
		return error;
	}
	val = (unsigned char) (data & 0xff);
	
	//LG4573_disp_reset();
	LG4573_disp_on2(val);
#endif

	return count;
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		lcd_debug_show, lcd_debug_store);

static struct attribute *lcd_attributes[] = {
	&dev_attr_debug.attr,
	NULL
};

static struct attribute_group lcd_attribute_group = {
	.attrs = lcd_attributes
};


#endif



static unsigned char bitmap[8]=
{	(1<<7),	(1<<6),	(1<<5),	(1<<4),
	(1<<3),	(1<<2),	(1<<1),	(1<<0)
};


static void LG4573_disp_on()
{
	LG4573_disp_on2(0x0f);
}

static void LG4573_disp_on2(unsigned char polarity)
{
    printk("%s lcd, disp_on\n", __func__);

	/* init */
	write_reg_LG4573(0xB9); //Set_EXTC 
	write_data_LG4573(0xFF); 
	write_data_LG4573(0x83); 
	write_data_LG4573(0x69); 

#if 0
	write_reg_LG4573(0xB0); //Set osc 
	write_data_LG4573(0x01); 
	write_data_LG4573(0x0a); // 0b
#endif

	write_reg_LG4573(0xB1); //Set Power 
	write_data_LG4573(0x01); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x34); 
	write_data_LG4573(0x06); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x0F); 
	write_data_LG4573(0x0F); 
	write_data_LG4573(0x1A); 
	write_data_LG4573(0x21); 
	write_data_LG4573(0x3F); 
	write_data_LG4573(0x3F); 
	write_data_LG4573(0x07); 
	write_data_LG4573(0x23); 
	write_data_LG4573(0x01); 
	write_data_LG4573(0xE6); 
	write_data_LG4573(0xE6); 
	write_data_LG4573(0xE6); 
	write_data_LG4573(0xE6); 
	write_data_LG4573(0xE6);
	 
	write_reg_LG4573(0xB2); // SET Display 480x800 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x2B); 
	write_data_LG4573(0x0A); 
	write_data_LG4573(0x0A); 
	write_data_LG4573(0x70); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0xFF); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x03); 
	write_data_LG4573(0x03); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x01); 

	write_reg_LG4573(0xB3); // SETRGBIF 
	write_data_LG4573(polarity);//0x0f;//0x09); 

	write_reg_LG4573(0xB4); // SET Display CYC 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x0C); 
	write_data_LG4573(0x84); 
	write_data_LG4573(0x0C); 
	write_data_LG4573(0x01); 

	write_reg_LG4573(0xB6); // SET VCOM 

#if 1
	write_data_LG4573(0x23); 
	write_data_LG4573(0x23); 
#else 
// for debug
	write_data_LG4573(vcom); 
	write_data_LG4573(vcom); 
	printk(KERN_ERR "vcom:0x%02x", vcom); // 10  20
	vcom=vcom+2;
#endif	

	write_reg_LG4573(0xD5); //SET GIP 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x05); 
	write_data_LG4573(0x03); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x01); 
	write_data_LG4573(0x09); 
	write_data_LG4573(0x10); 
	write_data_LG4573(0x80); 
	write_data_LG4573(0x37); 
	write_data_LG4573(0x37); 
	write_data_LG4573(0x20); 
	write_data_LG4573(0x31); 
	write_data_LG4573(0x46); 
	write_data_LG4573(0x8A); 
	write_data_LG4573(0x57); 
	write_data_LG4573(0x9B); 
	write_data_LG4573(0x20); 
	write_data_LG4573(0x31); 
	write_data_LG4573(0x46); 
	write_data_LG4573(0x8A); 
	write_data_LG4573(0x57); 
	write_data_LG4573(0x9B); 
	write_data_LG4573(0x07); 
	write_data_LG4573(0x0F); 
	write_data_LG4573(0x07); 
	write_data_LG4573(0x00); 

	write_reg_LG4573(0xE0); //SET GAMMA 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x01); 
	write_data_LG4573(0x03); 
	write_data_LG4573(0x2B); 
	write_data_LG4573(0x33); 
	write_data_LG4573(0x3F); 
	write_data_LG4573(0x0D); 
	write_data_LG4573(0x30); 
	write_data_LG4573(0x06); 
	write_data_LG4573(0x0B); 
	write_data_LG4573(0x0D); 
	write_data_LG4573(0x10); 
	write_data_LG4573(0x13); 
	write_data_LG4573(0x11); 
	write_data_LG4573(0x13); 
	write_data_LG4573(0x11); 
	write_data_LG4573(0x17); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x01); 
	write_data_LG4573(0x03); 
	write_data_LG4573(0x2B); 
	write_data_LG4573(0x33); 
	write_data_LG4573(0x3F); 
	write_data_LG4573(0x0D); 
	write_data_LG4573(0x30); 
	write_data_LG4573(0x06); 
	write_data_LG4573(0x0B); 
	write_data_LG4573(0x0D); 
	write_data_LG4573(0x10); 
	write_data_LG4573(0x13); 
	write_data_LG4573(0x11); 
	write_data_LG4573(0x13); 
	write_data_LG4573(0x11); 
	write_data_LG4573(0x17); 

	write_reg_LG4573(0x2A); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x01); 
	write_data_LG4573(0xDF);
	        
	write_reg_LG4573(0x2B);  
	write_data_LG4573(0x00); 
	write_data_LG4573(0x00); 
	write_data_LG4573(0x03); 
	write_data_LG4573(0x1F);

	write_reg_LG4573(0x36); //Set_address_mode 
	write_data_LG4573(0x04);
	write_reg_LG4573(0x3A); //Set COLMOD
	write_data_LG4573(0x77); //  24bit: 0x77, 18bit: 0x66, 16bit: 0x55
	//////////////////////////
	write_reg_LG4573(0xCC); //SETPANEL
	write_data_LG4573(0x02);
	//////////////////////////
	write_reg_LG4573(0x11); //Sleep Out
	mdelay(130); //DelayX1ms(120);

	write_reg_LG4573(0x29); //Display On
	mdelay(20); //DelayX1ms(20);

	write_reg_LG4573(0x2C); //START WRITE GRAM
	mdelay(10);	//DelayX1ms(10);
	
    printk("lcd, disp_on done\n");

	LG4573_state.display_on = TRUE;

}

int LG4573_spi_init(void)
{
	int result;

#if 1
	result = gpio_tlmm_config(GPIO_CFG (LCDC2_SPI_SDI, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(result){
		goto __SPI_INIT_DONE;
	}
#endif

	result = gpio_tlmm_config(GPIO_CFG(LCDC2_SPI_SDO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(result){
		goto __SPI_INIT_DONE;
	}

	result = gpio_tlmm_config(GPIO_CFG(LCDC2_SPI_CS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(result){
		goto __SPI_INIT_DONE;
	}

	result = gpio_tlmm_config(GPIO_CFG(LCDC2_SPI_SCK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(result){
		goto __SPI_INIT_DONE;
	}

	LCDC2_SPI_CS_HIGH;
	LCDC2_SPI_SCK_HIGH;

	printk(KERN_ERR "%s spi init.\n", __func__);

__SPI_INIT_DONE:
	
	return result;
}

void write_reg_LG4573(unsigned char uc)
{
	LG4573_spi_write(TRUE, uc);
}

void write_data_LG4573(unsigned char uc)
{
	LG4573_spi_write(FALSE, uc);
}

void LG4573_spi_write(bool bReg, unsigned char uc)
{
	int i;


	/* D/CX bit */

	LCDC2_SPI_SCK_LOW;	
	LCDC2_SPI_CS_LOW;
	LCDC2_SPI_NDELAY;
	LCDC2_SPI_NDELAY;

	if(bReg) {
		LCDC2_SPI_SDO_LOW;
	} else {
		LCDC2_SPI_SDO_HIGH;
	}
	LCDC2_SPI_NDELAY;
	LCDC2_SPI_SCK_HIGH;
	LCDC2_SPI_NDELAY;
	

	/* data */
	
	for( i = 0; i < 8; i++) {
		LCDC2_SPI_SCK_LOW;
		if(bitmap[i] & uc) {
			LCDC2_SPI_SDO_HIGH;
		} else {
			LCDC2_SPI_SDO_LOW;
		}
		LCDC2_SPI_NDELAY;
		LCDC2_SPI_SCK_HIGH;
		LCDC2_SPI_NDELAY;
	}

	LCDC2_SPI_CS_HIGH;
};;


void LG4573_spi_read(unsigned char reg, unsigned char *puc, unsigned char c /* count to read */)
{
	int i;
	int c1;

	// start 
	
	LCDC2_SPI_SCK_LOW;
	LCDC2_SPI_CS_LOW;
	LCDC2_SPI_NDELAY;
	LCDC2_SPI_NDELAY;

	
	// write reg
	
	LCDC2_SPI_SDO_LOW;
	
	LCDC2_SPI_NDELAY;
	LCDC2_SPI_SCK_HIGH;
	LCDC2_SPI_NDELAY;
	
	for( i = 0; i < 8; i++) {
		LCDC2_SPI_SCK_LOW;
		if(bitmap[i] & reg) {
			LCDC2_SPI_SDO_HIGH;
		} else {
			LCDC2_SPI_SDO_LOW;
		}
		LCDC2_SPI_NDELAY;
		LCDC2_SPI_SCK_HIGH;
		LCDC2_SPI_NDELAY;
	}

	
	/* data */

	{
		//unsigned int  times = 4;

		for (c1 = 0; c1 < c; ++c1) {	
			unsigned char d1 = 0;
			for( i = 7; i >= 0 ; --i) {
				LCDC2_SPI_SCK_LOW;
				LCDC2_SPI_NDELAY;
				
				LCDC2_SPI_SCK_HIGH;
				LCDC2_SPI_NDELAY;			
				d1 |= (LCDC2_SPI_SDI_READ << i);
			}
			puc[c1] = d1;
		}
	}

	LCDC2_SPI_CS_HIGH;
}



void LG4573_disp_reset(void)
{
    printk(KERN_ERR "%s lcd, disp_reset\n", __func__);

#if defined(RESET_USING_PMIC__MPP) // reset in AMSS??, removed by X.110524
    printk(KERN_ERR "using MPP %d\n", RESET_MPP);
	mpp_config_digital_out(RESET_MPP, MPP_CFG(PM_MPP__DLOGIC__LVL_MSMP, PM_MPP__DLOGIC_OUT__CTRL_HIGH));
	mdelay(1); // ms
	mpp_config_digital_out(RESET_MPP, MPP_CFG(PM_MPP__DLOGIC__LVL_MSMP, PM_MPP__DLOGIC_OUT__CTRL_LOW));
	mdelay(50); // ms
	mpp_config_digital_out(RESET_MPP, MPP_CFG(PM_MPP__DLOGIC__LVL_MSMP, PM_MPP__DLOGIC_OUT__CTRL_HIGH));
	mdelay(100); // ms
	
#elif defined(RESET_USING__GPIO)
	{
	int result;
	result = gpio_tlmm_config(GPIO_CFG(RESET_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(result){ 
		printk(KERN_INFO "lcd, rst err! gpio cfg %d\n", RESET_GPIO);		
		return ;
	}	
	}
    printk(KERN_ERR "using GPIO %d\n", RESET_GPIO);
	gpio_set_value(RESET_GPIO, 1);
	mdelay(10); // ms	
	gpio_set_value(RESET_GPIO, 0);
	mdelay(10); // ms	
	gpio_set_value(RESET_GPIO, 1);
	mdelay(10); // ms
#endif
}

#if 0
static void LG4573_disp_powerup(void)
{
	if (!LG4573_state.disp_powered_up && !LG4573_state.display_on)
	{
		/* power */

		/*pmic_vreg_set_level(PM_VREG_MSMP_ID,2800);
		pmic_vreg_set_level(PM_VREG_LP_MSME_ID,2800);
		*/

		mdelay(100);

		/* reset */
		LG4573_disp_reset();

		LG4573_state.disp_powered_up = TRUE;
	}
}
#endif
#if 0
//extern int waiting_to_detect_the_number_of_panel;
static int /* 0 is not exist */ detect_panel_RY007(void)
{
return 0; // #9 todo, if other panels isn't exist, used this, please make this driver to link last in the MAKEFILE.

#if 1
	{
		unsigned char buff1[16];
		int i;
#define REG_TO_READ		0xFA
#define READ_COUNT			4

//		  LG4573_spi_write(TRUE, 0x01);
//		  mdelay(200);

		LG4573_spi_read(REG_TO_READ, buff1, READ_COUNT);
		printk(KERN_ERR "Read reg: 0x%02x, result: \n", REG_TO_READ); 	
		for (i = 0; i < READ_COUNT; ++i) {
			printk(KERN_ERR "0x%02x \n", buff1[i]);
		}

		if ((buff1[0] == 0xff) && (buff1[1] == 0xff) && (buff1[2] == 0xff) && (buff1[3] == 0xff)) {
			printk(KERN_ERR "%s detect panel, found..?\n", __func__);				
			return 1;
		}
	}
#endif

	return 0; 
}
#endif
static void lcdc_LG4573_intersleep(void)
{
   	printk(KERN_INFO "LCDC_LG4573:intersleep\n");

#if defined(RESET_USING_PMIC__MPP)// please remove/modify it on P1 if the reset pin is change to arm11 side	// config the RESET pin to reduce current.
	mpp_config_digital_out(RESET_MPP, MPP_CFG(PM_MPP__DLOGIC__LVL_MSMP, PM_MPP__DLOGIC_OUT__CTRL_LOW));		

#else

// solution 1
#if 0
	write_reg_LG4573(0x28); 		
	write_reg_LG4573(0x10); 		
	mdelay(150); //Wait  150 ms		
	// #3 ?? RGB IF Disable		
	mdelay(150); //Wait  150 ms		
	write_reg_LG4573(0xC1); 		
	write_data_LG4573(0x01);		
#endif	


/* solution 2 */
#if 0
	power_vreg_gp6(0);	
#endif

/* solution 3 */
#if 1
	//power_vreg_gp6(0);	
	LG4573_disp_reset();
#endif


#endif

}

static void lcdc_LG4573_exitsleep(void)
{
	printk(KERN_INFO "LCDC_LG4573:exitsleep\n");

#if defined(RESET_USING_PMIC__MPP) // please remove/modify it on P1 if the reset pin is change to arm11 side	// restore
	mpp_config_digital_out(RESET_MPP, MPP_CFG(PM_MPP__DLOGIC__LVL_MSMP, PM_MPP__DLOGIC_OUT__CTRL_HIGH));	
	mdelay(1); //?

	LG4573_disp_reset();
	LG4573_disp_on();	

#elif defined(RESET_USING__GPIO)

/* solution 1 */
#if 0
/* from FAE
	2 Times /CS pin toggle		
	Wait 2 ms		
	4 Times /CS pin toggle		
	Power-On Sequence NO1 22		
*/
	int i;

	for (i = 0; i < 2; ++i) {
		LCDC2_SPI_CS_LOW;
		LCDC2_SPI_CS_HIGH;
	}
	mdelay(2);

	for (i = 0; i < 4; ++i) {
		LCDC2_SPI_CS_LOW;
		LCDC2_SPI_CS_HIGH;
	}

	//LG4573_disp_reset(); // ???
	LG4573_disp_on();	
#endif

/* solution 2 */
#if 0
	power_vreg_gp6(1);	
	mdelay(10); // need ?
	LG4573_disp_reset(); 
	LG4573_disp_on();
#endif


/* solution 3 */
#if 1
	//LG4573_disp_reset(); 
	LG4573_disp_on();
#endif


#endif


}

static int lcdc_LG4573_panel_on(struct platform_device *pdev)
{
    int ret;

    printk(KERN_ERR "%s, initialized:%d\n", __func__, LG4573_state.disp_initialized);

	if (!LG4573_state.disp_initialized) {
		ret = LG4573_spi_init();
		if(ret) {
		     printk(KERN_DEBUG "failure init ssd_spi_init!!!\n");
		     return ret;
		}

		//LG4573_disp_powerup();
		
		//LG4573_disp_reset();
		LG4573_disp_on();

		LG4573_state.disp_powered_up = TRUE;
		LG4573_state.disp_initialized = TRUE;
        return 0;
	}

	if (!LG4573_state.disp_powered_up ) {
	   lcdc_LG4573_exitsleep();
	   LG4573_state.disp_powered_up = TRUE;
	}
	return 0;
}

static int lcdc_LG4573_panel_off(struct platform_device *pdev)
{
	printk(KERN_ERR "%s\n", __func__);

	if (LG4573_state.disp_powered_up ) {

		/* Main panel power off (Deep standby in) */
		lcdc_LG4573_intersleep(); 
		//mdelay(300); // removed by X.110816. 
	
		//LG4573_state.disp_initialized = FALSE;
		LG4573_state.disp_powered_up = FALSE;
	}

	return 0;
}

#ifdef BACLKIGHT_USING__GPIO_PWM

static void lcdc_LG4573_enable_gp_clock(int on)
{

	static int count = 0;

	if (on) {
		if (count <= 0) {
			clk_enable(gp_clk);
			printk(KERN_ERR "%s: on\n", __func__);

			count = 1;	
		}
	}
	else {
		if (count > 0) {
			clk_disable(gp_clk);
			printk(KERN_ERR "%s: off\n", __func__);

			count = 0;
		}
	}

}   

#endif
static void lcdc_LG4573_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level;
	int bl_max;
//	int ret;

	//printk(KERN_INFO "set bkl:%d\n", bl_level);
	if (bl_level > mfd->panel_info.bl_max) {
		bl_level = mfd->panel_info.bl_max;
	}
	bl_max = mfd->panel_info.bl_max;

#if defined(BACKLIGHT_USING_PMIC__LCD_DRV_N)
	int i = 0;

	while (i++ < 3) {
		ret = pmic_set_led_intensity(LED_LCD, bl_level);
		if (ret == 0)
			break;
		msleep(10);
	}
	
#elif defined(BACKLIGHT_USING__GPIO)
	if (bl_level > 0) {
		BACKLIGHT_GPIO_HIGH;
	} else {
		BACKLIGHT_GPIO_LOW;
	}

#elif defined(BACLKIGHT_USING__GPIO_PWM)
		if(bl_level > 0) {

#if 0 // 0.8192hz, please sync clk_set_rate(327680);
			const unsigned short m_val = 1U;
			const unsigned short n_val = 10;
			
			unsigned short duty = (n_val / bl_max) * bl_level;	
			
			clk_enable(gp_clk);
			writel((m_val << 16) | (~(2 * duty) & 0xffff), MSM_CLK_CTL_BASE + 0x58);

			/* using sleep clock: 32.768k /DIV 4 = 8.192k 
				8.192k / 10 = 0.8192hz
			*/
			writel(( ~(n_val - m_val) << 16) | 0xb5e, MSM_CLK_CTL_BASE + 0x5c);

#endif

#if 1 // 
			const unsigned short m_val = 1U;
			const unsigned short n_val = 256;
			//unsigned long cur_rate = 0;

			// according HW request, make max backlight current to ~18mA, not 20mA.
			
			//unsigned short duty = (n_val / bl_max) * bl_level;	// max: 20mA
			unsigned short duty = (n_val / (bl_max + 1)) * bl_level;	//max: ~18.33mA


			lcdc_LG4573_enable_gp_clock(1); //clk_enable(gp_clk);
			
			writel((m_val << 16) | (~(2 * duty) & 0xffff), MSM_CLK_CTL_BASE + 0x58);

			/* using TCXO clock: 19.2M /DIV 4 = 4.2M
				4.2M / 256 = 18.75 Khz
			*/
			writel(( ~(n_val - m_val) << 16) | 0xb58, MSM_CLK_CTL_BASE + 0x5c);
#endif

		}
		else {
			writel(0x0, MSM_CLK_CTL_BASE + 0x5c);

			lcdc_LG4573_enable_gp_clock(0); //clk_disable(gp_clk);
		}	
#endif
}





static int __devinit LG4573_probe(struct platform_device *pdev)
{
	int result = 0;
	printk(KERN_ERR "%s probe.\n", __func__);

	// detect the panel

	LG4573_spi_init();	// config in lk ??
#if 0
	if (0 == detect_panel_RY007()) { 
		// the panel is not exist
		--waiting_to_detect_the_number_of_panel; // PLEASE update the total number of panel, if you add a new LCD driver.

		if (waiting_to_detect_the_number_of_panel >= 1) {
			printk(KERN_ERR "%s, panel detect failed, r:%d.\n", __func__, waiting_to_detect_the_number_of_panel);

			return -1;//ENODEV; 
		}

		printk(KERN_ERR "%s, panel detect failed, but it's the last, so add the device ANYWAY.\n", __func__); 	
	}
#endif
	// power on
	
//	power_vreg_gp6(1);  // powered up in lk.  we call this for record vreg count only.

	msm_fb_add_device(pdev);	

#if defined(BACKLIGHT_USING__GPIO)
	//gpio_request(BACKLIGHT_GPIO, "lcd_backlight"); // need?
	result = gpio_tlmm_config(GPIO_CFG(BACKLIGHT_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

#elif defined(BACLKIGHT_USING__GPIO_PWM)

	result = gpio_tlmm_config(GPIO_CFG(27, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

	gp_clk = clk_get(NULL, "gp_clk"); 	// USING clk_put to release it?

	result = clk_set_rate(gp_clk, PERF_GP_CLK_RATE);//	32.768k: 327680, 19.2M: 19200000);
	
#endif
	if(result) {
		printk(KERN_ERR "Err: config backlight gpio!, result:%d\n", result); 
	}


#if defined(DEBUG_SYSFS)
		// path: /sys/devices/platform/lcdc_gordon_vga.1
		result = sysfs_create_group(&pdev->dev.kobj, &lcd_attribute_group); 
#endif

	return 0;
}


#define SZ_THIS_MODULE_NAME	"lcdc_wvga_RY007"		//"lcdc_gordon_vga"
static struct platform_driver this_driver = {
	.probe  = LG4573_probe,
	.driver = {
		.name   = SZ_THIS_MODULE_NAME,
	},
};

static struct msm_fb_panel_data LG4573_panel_data = {
	.on = lcdc_LG4573_panel_on,
	.off = lcdc_LG4573_panel_off,
	.set_backlight = lcdc_LG4573_set_backlight,
};

static struct platform_device this_device = {
	.name   = SZ_THIS_MODULE_NAME,
	.id	= 1,
	.dev	= {
		.platform_data = &LG4573_panel_data,
	}
};

static int __init lcdc_LG4573_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		return ret;
	}
	
	pinfo = &LG4573_panel_data.panel_info;
	pinfo->xres = 480; //320;
	pinfo->yres = 800; //480;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 1;
	pinfo->clk_rate = 27306667;//24576000;//27306667;//28086857; 25M: 24576000; // LG4573 wanna 27M
#if defined(BACKLIGHT_USING_PMIC__LCD_DRV_N)
	pinfo->bl_max = 10;//MAX_BACKLIGHT_LEVEL; // 0 is off , 10 ~ 100 mA
#elif defined (BACLKIGHT_USING__GPIO_PWM)
	pinfo->bl_max = 10;//MAX_BACKLIGHT_LEVEL;
#else
	pinfo->bl_max = 1;
#endif
	pinfo->bl_min = 1;

	pinfo->lcdc.h_back_porch = 10;	//59; // 3;
	pinfo->lcdc.h_front_porch = 10;	//10; // 3;
	pinfo->lcdc.h_pulse_width = 10;	//10; // 2;
	pinfo->lcdc.v_back_porch = 6;	//15; // 2;
	pinfo->lcdc.v_front_porch = 6;	//15; // 4;
	pinfo->lcdc.v_pulse_width = 6;	//15; // 2;

	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;
	// 9830400

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}


module_init(lcdc_LG4573_panel_init);


