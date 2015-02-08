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
 Created by Joe .110311
 Modified by Joe.110316
 Modified by Joe.110329, for ILITEK 9481b driver IC
 Modified by Joe.111016, ILI9481B

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
//#define BACKLIGHT_USING__GPIO //del by Joe 120408
//#define BACLKIGHT_USING__GPIO_PWM
#define FEATURE_SUPPORT_AW9656
extern int boot_splash;//late_display

#define LCDC2_SPI_SDO 33 //90
#define LCDC2_SPI_SDI 92 //91 //91
#define LCDC2_SPI_CS  31  //88
#define LCDC2_SPI_SCK 32  //89
#define RESET_GPIO    85
#define BACKLIGHT_GPIO 27//84
#define LCDC2_ID      82

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

struct ILI9481B_state_type {
	bool	disp_initialized;/* disp initialized */
	bool	display_on;	/* lcdc is on */
	bool	disp_powered_up;/* lcdc power and lcd bl power on */
};

typedef struct ILI9481B_state_type	 ILI9481B_state_t;
static ILI9481B_state_t	ILI9481B_state = { 0 };

extern int power_vreg_gp6(int on);

#if defined(BACLKIGHT_USING__GPIO_PWM)

#define PERF_GP_CLK_RATE	19200000
static struct clk *gp_clk; // GPIO_27
extern int clk_enable(struct clk *clk);
extern void clk_disable(struct clk *clk);
extern struct clk *clk_get(struct device *dev, const char *id);
extern int clk_set_rate(struct clk *clk, unsigned long rate);
#endif

static int ILI9481B_spi_init(void);
static void lcdc_ILI9481B_intersleep(void);
static void lcdc_ILI9481B_exitsleep(void);
static int lcdc_ILI9481B_panel_off(struct platform_device *pdev);

static void ILI9481B_spi_write(bool bReg, unsigned char uc);
static void write_reg_ILI9481B(unsigned char uc);
static void write_data_ILI9481B(unsigned char uc);
//static void read_reg_LG4573(unsigned char uc);
//static void read_data_LG4573(unsigned char *uc);
static void lcdc_ILI9481B_set_backlight(struct msm_fb_data_type *mfd);


static void ILI9481B_disp_on(void);
static void ILI9481B_disp_on2(unsigned char polarity);



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
	ILI9481B_disp_on2(val);
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


static void ILI9481B_disp_on()
{
	ILI9481B_disp_on2(0x0f);
}

static void ILI9481B_disp_on2(unsigned char polarity)
{
    printk("%s lcd, disp_on\n", __func__);
    {
	/* init */

	//VCI=2.8V
	// 	mdelay(50);		//Wait for Power stabilization
	//----------- LCM Reset before initialized ---------//
	//	LcmReset=1;
	//	mdelay(1);
	//	LcmReset=0;		//LCM Hardware Reset.
	//	mdelay(10);
	//	LcmReset=1;
	//	mdelay(100);		//delay
	//----------------------------------------------------------//
	//
	//---------------- Start Initial Sequence ------------//
	//
	write_reg_ILI9481B(0x11);
	mdelay(100);

/*currently, for power save, when the system is enter sleep mode, we pull down the RESET pin, so MUST re-send the params,
 	planning to change this mpp pin to AP(ARM11) gpio, it can avoid the problem.*/
 #if 1
  	write_reg_ILI9481B(0xD0);
  	write_data_ILI9481B(0x07);
  	write_data_ILI9481B(0x41);//45
  	write_data_ILI9481B(0x1a);// 1b  1d
  	write_reg_ILI9481B(0xD1);
  	write_data_ILI9481B(0x00);
  	write_data_ILI9481B(0x31);// #3 30);//31  30  2d  15
  	write_data_ILI9481B(0x14);// 15  14  13           1b

	write_reg_ILI9481B(0xD2);
	write_data_ILI9481B(0x01);
	write_data_ILI9481B(0x11);//02     12

	write_reg_ILI9481B(0xC0);
	write_data_ILI9481B(0x10);
	write_data_ILI9481B(0x3B);
	write_data_ILI9481B(0x00);
	write_data_ILI9481B(0x02);
	write_data_ILI9481B(0x11);

	write_reg_ILI9481B(0xC1);
	write_data_ILI9481B(0x10);
	write_data_ILI9481B(0x10);
	write_data_ILI9481B(0x22);//FP,BP

	write_reg_ILI9481B(0xC5);
	write_data_ILI9481B(0x00);//#3 3);//72Hz, 02:85Hz

	write_reg_ILI9481B(0xC6); // polarity
	write_data_ILI9481B(0x9b);//modified by X.110815.	01);//81

    write_reg_ILI9481B(0xC8);
    	write_data_ILI9481B(0x00);//KP1[2:0];KP0[2:0]
	write_data_ILI9481B(0X56);//KP3[2:0];KP2[2:0]
  	write_data_ILI9481B(0X05);//KP5[2:0];KP4[2:0]
  	write_data_ILI9481B(0X50);//21 RP1[2:0];RP0[2:0]
  	write_data_ILI9481B(0X0f);//0a 01VRP0[3:0]
  	write_data_ILI9481B(0X02);//04 16VRP1[4:0]
  	write_data_ILI9481B(0X27);//KN1[2:0];KN0[2:0]
  	write_data_ILI9481B(0X12);//KN3[2:0];KN2[2:0]
  	write_data_ILI9481B(0X77);//KN5[2:0];KN4[2:0]
  	write_data_ILI9481B(0X05);//13 RN1[2:0];RN0[2:0]
  	write_data_ILI9481B(0X01);//02 0a 06 06 VRN0[3:0]
  	write_data_ILI9481B(0X1e);//0c 00 VRN1[4:0]

  	write_reg_ILI9481B(0xF3);
  	write_data_ILI9481B(0x40);
  	write_data_ILI9481B(0x0A);

  	write_reg_ILI9481B(0xF6);
  	write_data_ILI9481B(0x80);

  	write_reg_ILI9481B(0xF7);
  	write_data_ILI9481B(0x80);

	write_reg_ILI9481B(0x2A);
	write_data_ILI9481B(0x00);
	write_data_ILI9481B(0x00);
	write_data_ILI9481B(0x01);
	write_data_ILI9481B(0x3F);

	write_reg_ILI9481B(0x2B);
	write_data_ILI9481B(0x00);
	write_data_ILI9481B(0x00);
	write_data_ILI9481B(0x01);
	write_data_ILI9481B(0xDF);

	write_reg_ILI9481B(0x3A);
	write_data_ILI9481B(0x66);

	write_reg_ILI9481B(0xB4);
	write_data_ILI9481B(0x11);

	write_reg_ILI9481B(0x36);
	write_data_ILI9481B(0x0a);//0A
	mdelay(20);

	write_reg_ILI9481B(0x13);
#endif
	write_reg_ILI9481B(0x29);
	mdelay(20);
	write_reg_ILI9481B(0x2c);



    }

	ILI9481B_state.display_on = TRUE;

}

int ILI9481B_spi_init(void)
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

void write_reg_ILI9481B(unsigned char uc)
{
	ILI9481B_spi_write(TRUE, uc);
}

void write_data_ILI9481B(unsigned char uc)
{
	ILI9481B_spi_write(FALSE, uc);
}

void ILI9481B_spi_write(bool bReg, unsigned char uc)
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
}


void ILI9481B_read_reg(byte data)
{
	int i;

	LCDC2_SPI_CS_HIGH;
	LCDC2_SPI_CS_LOW;

	/* D/CX bit */
	LCDC2_SPI_SCK_LOW;
	LCDC2_SPI_SDI_LOW;
	LCDC2_SPI_SCK_HIGH;

	/* data */
	for( i = 0; i < 8; i++) {
		LCDC2_SPI_SCK_LOW;
		if(bitmap[i] & data) {
			LCDC2_SPI_SDI_HIGH;
		} else {
			LCDC2_SPI_SDI_LOW;
		}
		
		LCDC2_SPI_SCK_HIGH;
	}
}

void ILI9481B_read_data(byte *data)
{
	int i;
	*data=0;
	LCDC2_SPI_CS_LOW;

	/* data */
	for( i = 0; i < 8; i++) {
		LCDC2_SPI_SCK_LOW;

		if(LCDC2_SPI_SDI_READ) {
			(*data)|=bitmap[i];
		}
		LCDC2_SPI_SCK_HIGH;
	}
}

#if 0
void ILI9481B_spi_read(unsigned char reg, unsigned char *puc, unsigned char c /* count to read */)
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
#endif


void ILI9481B_disp_reset(void)
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
static void lcdc_ILI9481B_intersleep(void)
{
   	printk(KERN_INFO "LCDC_ILI9481B:intersleep\n");
    write_reg_ILI9481B(0x28);
	write_reg_ILI9481B(0x10);
	mdelay(120);
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
	//ILI9481B_disp_reset(); //modify by Joe 120405
#endif


#endif

}

static void lcdc_ILI9481B_exitsleep(void)
{
	printk(KERN_INFO "LCDC_ILI9481B:exitsleep\n");

#if defined(RESET_USING_PMIC__MPP) // please remove/modify it on P1 if the reset pin is change to arm11 side	// restore
	mpp_config_digital_out(RESET_MPP, MPP_CFG(PM_MPP__DLOGIC__LVL_MSMP, PM_MPP__DLOGIC_OUT__CTRL_HIGH));	
	mdelay(1); //?

	ILI9481B_disp_reset();
	ILI9481B_disp_on();	

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
    //mdelay(10);
	//ILI9481B_disp_reset();
	ILI9481B_disp_on();
#endif


#endif


}

static int lcdc_ILI9481B_panel_on(struct platform_device *pdev)
{
    int ret;

    printk(KERN_ERR "%s, initialized:%d\n", __func__, ILI9481B_state.disp_initialized);

	if (!ILI9481B_state.disp_initialized) {
		ret = ILI9481B_spi_init();
		if(ret) {
		     printk(KERN_DEBUG "failure init ssd_spi_init!!!\n");
		     return ret;
		}

		//LG4573_disp_powerup();
		
		//ILI9481B_disp_reset();
		ILI9481B_disp_on();

		ILI9481B_state.disp_powered_up = TRUE;
		ILI9481B_state.disp_initialized = TRUE;
        return 0;
	}

	if (!ILI9481B_state.disp_powered_up ) {
	   lcdc_ILI9481B_exitsleep();
	   ILI9481B_state.disp_powered_up = TRUE;
	}
	return 0;
}

static int lcdc_ILI9481B_panel_off(struct platform_device *pdev)
{
	printk(KERN_ERR "%s\n", __func__);

	if (ILI9481B_state.disp_powered_up ) {

		/* Main panel power off (Deep standby in) */
		lcdc_ILI9481B_intersleep(); 
		//mdelay(300); // removed by X.110816. 
	
		//LG4573_state.disp_initialized = FALSE;
		ILI9481B_state.disp_powered_up = FALSE;
	}

	return 0;
}

#ifdef BACLKIGHT_USING__GPIO_PWM

void lcdc_ILI9481B_enable_gp_clock(int on)
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
static void lcdc_ILI9481B_set_backlight(struct msm_fb_data_type *mfd)
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


			lcdc_ILI9481B_enable_gp_clock(1); //clk_enable(gp_clk);
			
			writel((m_val << 16) | (~(2 * duty) & 0xffff), MSM_CLK_CTL_BASE + 0x58);

			/* using TCXO clock: 19.2M /DIV 4 = 4.2M
				4.2M / 256 = 18.75 Khz
			*/
			writel(( ~(n_val - m_val) << 16) | 0xb58, MSM_CLK_CTL_BASE + 0x5c);
#endif

		}
		else {
			writel(0x0, MSM_CLK_CTL_BASE + 0x5c);

			lcdc_ILI9481B_enable_gp_clock(0); //clk_disable(gp_clk);
		}	
#endif
}

static int detect_panel_ili9481b(void)
{
    //int result;
    #if 0
    char deviceID[6];
    ILI9481B_spi_init();	// config in lk ??
	write_reg_HX8357B(0xFE); 
	write_data_HX8357B(0xBF);

    HX8357B_read_reg(0xFF);
    rc = gpio_request(LCDC2_SPI_SDI, "lcdc_spi_sdi");

	if (!rc) {
		gpio_direction_input(LCDC2_SPI_SDI);
	} else {
		printk(KERN_ERR"%s: REQUEST  LCDC_SPI_GPIO103 FAILD",__func__);
		return -1;
	}
	HX8357B_read_data(&deviceID[0]);
	HX8357B_read_data(&deviceID[1]);
	HX8357B_read_data(&deviceID[2]);
	HX8357B_read_data(&deviceID[3]);
	HX8357B_read_data(&deviceID[4]);
	HX8357B_read_data(&deviceID[5]);
	gpio_direction_output(LCDC2_SPI_SDI,0);  

    printk(KERN_INFO "%s: HX8357B_ID: %02x%02x\n", __func__, deviceID[2], deviceID[3]);  

    if((deviceID[2] == 0x83)&&(deviceID[3] == 0x57))
    {
        return 1;
    }
    else
    {
        return 0;
    }
    #endif
    gpio_request(LCDC2_ID,"ili9481b id_in");
    //result = gpio_tlmm_config(GPIO_CFG(LCDC2_ID, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    gpio_direction_input(LCDC2_ID);
    
    if(gpio_get_value(LCDC2_ID))
    {
        printk(KERN_ERR "%s gpio_get_value is high\n", __func__);
        return 1;
    }
    printk(KERN_ERR "%s gpio_get_value is ow\n", __func__);
    return 0;
}




static int __devinit ILI9481B_probe(struct platform_device *pdev)
{
    //char deviceID[6];
	int result = 0;
    //int rc = 0;
	printk(KERN_ERR "%s probe.\n", __func__);

	// detect the panel

	
#if 0
    ILI9481B_spi_init();	// config in lk ??
	write_reg_ILI9481B(0xFE); 
	write_data_ILI9481B(0xBF);

    ILI9481B_read_reg(0xFF);
    rc = gpio_request(LCDC2_SPI_SDI, "lcdc_spi_sdi");

	if (!rc) {
		gpio_direction_input(LCDC2_SPI_SDI);
	} else {
		printk(KERN_ERR"%s: REQUEST  LCDC_SPI_GPIO103 FAILD",__func__);
		return -1;
	}
	ILI9481B_read_data(&deviceID[0]);
	ILI9481B_read_data(&deviceID[1]);
	ILI9481B_read_data(&deviceID[2]);
	ILI9481B_read_data(&deviceID[3]);
	ILI9481B_read_data(&deviceID[4]);
	ILI9481B_read_data(&deviceID[5]);
	gpio_direction_output(LCDC2_SPI_SDI,0);  

    printk(KERN_INFO "%s: ILI9481B_ID: %02x%02x\n", __func__, deviceID[2], deviceID[3]);
#endif
    //return -1;
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
    if(detect_panel_ili9481b())   //low is useful
    {
        ILI9481B_spi_init();	// config in lk ??
        msm_fb_add_device(pdev);
    }
    else
    {   
        printk(KERN_ERR "%s No device.\n", __func__);
        return -1;
    }
		

#if defined(BACKLIGHT_USING__GPIO)
	//gpio_request(BACKLIGHT_GPIO, "lcd_backlight"); // need?
	result = gpio_tlmm_config(GPIO_CFG(BACKLIGHT_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

#elif defined(BACLKIGHT_USING__GPIO_PWM)
	result = gpio_tlmm_config(GPIO_CFG(27, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

	gp_clk = clk_get(NULL, "gp_clk"); 	// USING clk_put to release it?
    if (IS_ERR(gp_clk)) {
        printk(KERN_ERR "trout_init_panel: could not get gp"
               "clock\n");
        gp_clk = NULL;
    }
    if(gp_clk)
	result = clk_set_rate(gp_clk, PERF_GP_CLK_RATE);//	32.768k: 327680, 19.2M: 19200000);
	
	if (result)
		printk(KERN_ERR "%s: set clock rate "
		       "failed\n",__func__);
#elif defined(FEATURE_SUPPORT_AW9656)
    result	=gpio_request(BACKLIGHT_GPIO, "ili9481b backlight");
    result = gpio_tlmm_config(GPIO_CFG(BACKLIGHT_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

	
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


#define SZ_THIS_MODULE_NAME	"lcdc_wvga_ILI9481B"		//"lcdc_gordon_vga"
static struct platform_driver this_driver = {
	.probe  = ILI9481B_probe,
	.driver = {
		.name   = SZ_THIS_MODULE_NAME,
	},
};

static struct msm_fb_panel_data ILI9481B_panel_data = {
	.on = lcdc_ILI9481B_panel_on,
	.off = lcdc_ILI9481B_panel_off,
	.set_backlight = lcdc_ILI9481B_set_backlight,
};

static struct platform_device this_device = {
	.name   = SZ_THIS_MODULE_NAME,
	.id	= 1,
	.dev	= {
		.platform_data = &ILI9481B_panel_data,
	}
};

int late_lcdc_ILI9481B_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		return ret;
	}
	
	pinfo = &ILI9481B_panel_data.panel_info;
	pinfo->xres = 320; //320;
	pinfo->yres = 480; //480;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 1;
	pinfo->clk_rate = 8000000;//8000000;//27306667;//28086857; 25M: 24576000; // ILI9481B wanna 27M
#if defined(BACKLIGHT_USING_PMIC__LCD_DRV_N)
	pinfo->bl_max = 10; // 0 is off , 10 ~ 100 mA
#elif defined (BACLKIGHT_USING__GPIO_PWM)||defined(FEATURE_SUPPORT_AW9656)
	pinfo->bl_max = 10;
#else
	pinfo->bl_max = 1;
#endif
	pinfo->bl_min = 1;

	pinfo->lcdc.h_back_porch = 3;	//59; // 3;
	pinfo->lcdc.h_front_porch = 3;	//10; // 3;
	pinfo->lcdc.h_pulse_width = 2;	//10; // 2;
	pinfo->lcdc.v_back_porch = 2;	//15; // 2;
	pinfo->lcdc.v_front_porch = 4;	//15; // 4;
	pinfo->lcdc.v_pulse_width = 2;	//15; // 2;

	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;
	// 9830400

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}
EXPORT_SYMBOL(late_lcdc_ILI9481B_panel_init);

static int __init lcdc_ILI9481B_panel_init(void)
{
    if (boot_splash) {
		/* boot splash, do nothing */
	} else {
		late_lcdc_ILI9481B_panel_init();
	}

	return 0;
}

module_init(lcdc_ILI9481B_panel_init);


