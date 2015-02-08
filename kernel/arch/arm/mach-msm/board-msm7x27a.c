/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <linux/atomic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/usbdiag.h>
#include <mach/usb_gadget_fserial.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/usb/android_composite.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/android_pmem.h>
#include <linux/msm-charger.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <mach/rpc_pmapp.h>

#include <mach/msm_battery.h>
#ifdef CONFIG_SMSC911X
#include <linux/smsc911x.h>
#endif
#include <linux/atmel_maxtouch.h>
#include <linux/i2c/bq27510.h>
#include "devices.h"
#include "timer.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>

#if defined(CONFIG_TOUCHSCREEN_FT5206) || defined(CONFIG_TOUCHSCREEN_FT5306)
#include <linux/input/ft5x06_ts.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
#include <linux/input/rmi_platformdata.h>
#include <linux/input/rmi_i2c.h>
#endif

#ifdef  CONFIG_INPUT_LTR502
#include <linux/input/ltr502.h>
#endif

#ifdef  CONFIG_INPUT_LIS3DH
#include <linux/input/lis3dh.h>
#endif
#ifdef  CONFIG_INPUT_L3G4200D
#include <linux/input/k3g.h>
#endif

#ifdef CONFIG_I2C_GPIO
#include <linux/i2c-gpio.h>
#endif // CONFIG_I2C_GPIO


#ifdef CONFIG_SENSORS_AK8975
#include <linux/akm8975.h>
#if defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03)
#define COMPASS_DRDY 18
#else
#define COMPASS_DRDY 93
#endif
#endif // CONFIG_SENSORS_AK8975

#ifdef CONFIG_TOUCHSCREEN_MCS6024_TS
struct mcs6024_ts_platform_data {
	int irq;		/* irq number   */
    int wake;       /*wake id*/
};
#endif


#define PMEM_KERNEL_EBI1_SIZE  0x3A000
#define MSM_PMEM_AUDIO_SIZE	0x5B000
#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B
#define BAHAMA_SLAVE_ID_FM_REG 0x02
#define FM_GPIO	83
#define GPIO_INVALID NR_MSM_GPIOS


#if defined(CONFIG_GPIO_SX150X)
enum {
	GPIO_EXPANDER_IRQ_BASE	= NR_MSM_IRQS + NR_GPIO_IRQS,
	GPIO_EXPANDER_GPIO_BASE	= NR_MSM_GPIOS,
	/* SURF expander */
	GPIO_CORE_EXPANDER_BASE	= GPIO_EXPANDER_GPIO_BASE,
	GPIO_BT_SYS_REST_EN	= GPIO_CORE_EXPANDER_BASE,
	GPIO_WLAN_EXT_POR_N,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_PRESSURE_XCLR,
	GPIO_VREG_S3_EXP,
	GPIO_UBM2M_PWRDWN,
	GPIO_ETM_MODE_CS_N,
	GPIO_HOST_VBUS_EN,
	GPIO_SPI_MOSI,
	GPIO_SPI_MISO,
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_CORE_EXPANDER_IO13,
	GPIO_CORE_EXPANDER_IO14,
	GPIO_CORE_EXPANDER_IO15,
	/* Camera expander */
	GPIO_CAM_EXPANDER_BASE	= GPIO_CORE_EXPANDER_BASE + 16,
	GPIO_CAM_GP_STROBE_READY	= GPIO_CAM_EXPANDER_BASE,
	GPIO_CAM_GP_AFBUSY,
	GPIO_CAM_GP_CAM_PWDN,
	GPIO_CAM_GP_CAM1MP_XCLR,
	GPIO_CAM_GP_CAMIF_RESET_N,
	GPIO_CAM_GP_STROBE_CE,
	GPIO_CAM_GP_LED_EN1,
	GPIO_CAM_GP_LED_EN2,
};


enum {
	SX150X_CORE,
	SX150X_CAM,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_CORE]	= {
		.gpio_base		= GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0xfef8,
		.irq_summary		= -1,
	},
	[SX150X_CAM]	= {
		.gpio_base		= GPIO_CAM_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0x23,
		.irq_summary		= -1,
	},
};
#endif


#if defined(CONFIG_MACH_MSM7X27A_SKU3)
enum {
    GPIO_BT_SYS_REST_EN     = 85,
    GPIO_DISPLAY_RESET	= 90,
	GPIO_DISPLAY_PWR_EN = GPIO_INVALID, //unused GPIO
    GPIO_BACKLIGHT_EN	= 89,
    GPIO_HOST_VBUS_EN	= GPIO_INVALID,
    GPIO_SPI_MOSI		= 19,
    GPIO_SPI_MISO		= GPIO_INVALID, //unused GPIO
    GPIO_SPI_CLK		= 20,
    GPIO_SPI_CS0_N		= 21,
    GPIO_CAM_GP_CAM_PWDN	= 93,
    GPIO_CAM_GP_CAM1MP_XCLR	= 91,
    GPIO_CAM_GP_CAMIF_RESET_N = 29,
    GPIO_CAM_GP_LED_EN1	= GPIO_INVALID,
    GPIO_CAM_GP_LED_EN2	= GPIO_INVALID,
};


#elif defined(CONFIG_MACH_MSM7X27A_SKU1)

#if defined(CONFIG_PROJECT_ES03) // #9 modified by X
enum { 
	GPIO_BT_SYS_REST_EN = 114,	// zq, BCM4330
    GPIO_DISPLAY_PWR_EN	= GPIO_INVALID, 
    GPIO_BACKLIGHT_EN	= GPIO_INVALID, 
    GPIO_HOST_VBUS_EN	= GPIO_INVALID, 
    GPIO_SPI_MOSI		= GPIO_INVALID, 
    GPIO_SPI_MISO		= 33,//91,
    GPIO_SPI_CLK		= 32,//89,
    GPIO_SPI_CS0_N		= 31,//88,
    GPIO_CAM_GP_CAM_PWDN	= GPIO_INVALID,
    GPIO_CAM_GP_CAM1MP_XCLR	= GPIO_INVALID,
    GPIO_CAM_GP_CAMIF_RESET_N = GPIO_INVALID,
    GPIO_CAM_GP_LED_EN1	= GPIO_INVALID,
    GPIO_CAM_GP_LED_EN2	= GPIO_INVALID,
};

#elif defined(CONFIG_PROJECT_ES02)
enum {
    GPIO_BT_SYS_REST_EN = 20,
    GPIO_DISPLAY_RESET	= 85,
    GPIO_DISPLAY_PWR_EN	= 0,
    GPIO_BACKLIGHT_EN	= 84,
    GPIO_HOST_VBUS_EN	= 107,
    GPIO_SPI_MOSI		= 90,
    GPIO_SPI_MISO		= 91,
    GPIO_SPI_CLK		= 89,
    GPIO_SPI_CS0_N		= 88,
    GPIO_CAM_GP_CAM_PWDN	= 108,
    GPIO_CAM_GP_CAM1MP_XCLR	= 109,
    GPIO_CAM_GP_CAMIF_RESET_N = 29,
    GPIO_CAM_GP_LED_EN1	= 35,
    GPIO_CAM_GP_LED_EN2	= 34,
};

#else
enum {
    GPIO_BT_SYS_REST_EN = 114,
    GPIO_DISPLAY_PWR_EN	= 116,
    GPIO_BACKLIGHT_EN	= 116,
    GPIO_HOST_VBUS_EN	= 107,
    GPIO_SPI_MOSI		= 109,
    GPIO_SPI_MISO		= 126,
    GPIO_SPI_CLK		= 127,
    GPIO_SPI_CS0_N		= 128,
    GPIO_CAM_GP_CAM_PWDN	= 120,
    GPIO_CAM_GP_CAM1MP_XCLR	= 118,
    GPIO_CAM_GP_CAMIF_RESET_N = 123,
    GPIO_CAM_GP_LED_EN1	= 96,
    GPIO_CAM_GP_LED_EN2	= 96,
};
#endif

#endif // CONFIG_MACH_MSM7X27A_SKU1



#ifndef CONFIG_PROJECT_ES03	// removed by X

#define GPIO_BT_SYS_REST_EN_SKU3_1 GPIO_BT_SYS_REST_EN 	// 20
static unsigned int GPIO_BT_SYS_REST_EN_ =  GPIO_BT_SYS_REST_EN;


#if defined(CONFIG_PROJECT_ES02)
#define GPIO_VDD33_EN ( (machine_is_msm7x27a_sku1()||machine_is_msm7x27a_sku3()) ? 119 : (GPIO_INVALID))
#else
#define GPIO_VDD33_EN (machine_is_msm7x27a_sku1()?119:(GPIO_INVALID))
#endif

#endif // !CONFIG_PROJECT_ES03 removed by X




#ifndef CONFIG_PROJECT_ES03 // removed by X

enum {
	GPIO_VOTE_VDD33_WLAN ,
	GPIO_VOTE_VDD33_BT,
};
#define GPIO_VOTE_VDD33_WLAN_MASK (1 << GPIO_VOTE_VDD33_WLAN)
#define GPIO_VOTE_VDD33_BT_MASK (1 << GPIO_VOTE_VDD33_BT)
#define GPIO_VOTE_VDD33_ALL_MASK (GPIO_VOTE_VDD33_WLAN_MASK | GPIO_VOTE_VDD33_BT_MASK)


static unsigned long gpio_voter_table[NR_MSM_GPIOS];
/**
 * @brief Vote for set GPIO value.
 *
 * @detailed Use this function if 1 or more driver need to use a single
 * GPIO. This function keeps track of which driver require
 * a specific GPIO. This function will only allow the GPIO
 * to be disable or enable, if all the required voters that
 * have previously enable/disable the GPIO agree to do this.
 *
 *
 * @param gpio     		Wchih gpio need to be voted
 * @param value    		Which value need to be set
 * @param voter_id 		Which voter(module) wants to enable/disable the gpio
 * @param condition_mask 	Mask to select the required voter(module)
 */
static void gpio_set_value_vote(unsigned int gpio,int value,int voter_id,int condition_mask)
{
	int all_vote_ok = 0;
	if(gpio >= NR_MSM_GPIOS) {
		pr_err("Invalid GPIO number %d\n", gpio);
		return;
	}
	if(value) {
		set_bit(voter_id,&gpio_voter_table[gpio]);
		all_vote_ok = ((gpio_voter_table[gpio] & condition_mask) == condition_mask);
	}else{
		clear_bit(voter_id,&gpio_voter_table[gpio]);
		/* If all bits is 0 then set value to 0*/
		all_vote_ok = !(gpio_voter_table[gpio] & condition_mask);
	}
	if(all_vote_ok){
		gpio_set_value(gpio, !!value);
	}
}

#endif // !CONFIG_PROJECT_ES03




#ifndef CONFIG_PROJECT_ES03		// removed by zq

	/* FM Platform power and shutdown routines */
#define FPGA_MSM_CNTRL_REG2 0x90008010

static void config_pcm_i2s_mode(int mode)
{
	void __iomem *cfg_ptr;
	u8 reg2;

	cfg_ptr = ioremap_nocache(FPGA_MSM_CNTRL_REG2, sizeof(char));

	if (!cfg_ptr)
		return;
	if (mode) {
		/*enable the pcm mode in FPGA*/
		reg2 = readb_relaxed(cfg_ptr);
		if (reg2 == 0) {
			reg2 = 1;
			writeb_relaxed(reg2, cfg_ptr);
		}
	} else {
		/*enable i2s mode in FPGA*/
		reg2 = readb_relaxed(cfg_ptr);
		if (reg2 == 1) {
			reg2 = 0;
			writeb_relaxed(reg2, cfg_ptr);
		}
	}
	iounmap(cfg_ptr);
}

static unsigned fm_i2s_config_power_on[] = {
	/*FM_I2S_SD*/
	GPIO_CFG(68, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*FM_I2S_WS*/
	GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*FM_I2S_SCK*/
	GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static unsigned fm_i2s_config_power_off[] = {
	/*FM_I2S_SD*/
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*FM_I2S_WS*/
	GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*FM_I2S_SCK*/
	GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};


static int config_i2s(int mode)
{
	int pin, rc = 0;

	if (mode == FM_I2S_ON) {
		if (machine_is_msm7x27a_surf())
			config_pcm_i2s_mode(0);
		pr_err("%s mode = FM_I2S_ON", __func__);
		for (pin = 0; pin < ARRAY_SIZE(fm_i2s_config_power_on);
			pin++) {
				rc = gpio_tlmm_config(
					fm_i2s_config_power_on[pin],
					GPIO_CFG_ENABLE
					);
				if (rc < 0)
					return rc;
			}
	} else if (mode == FM_I2S_OFF) {
		pr_err("%s mode = FM_I2S_OFF", __func__);
		for (pin = 0; pin < ARRAY_SIZE(fm_i2s_config_power_off);
			pin++) {
				rc = gpio_tlmm_config(
					fm_i2s_config_power_off[pin],
					GPIO_CFG_ENABLE
					);
				if (rc < 0)
					return rc;
			}
	}
	return rc;
}
static int config_pcm(int mode)
{
	int pin, rc = 0;

	if (mode == BT_PCM_ON) {
		if (machine_is_msm7x27a_surf())
			config_pcm_i2s_mode(1);
		pr_err("%s mode =BT_PCM_ON", __func__);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_on);
			pin++) {
				rc = gpio_tlmm_config(bt_config_pcm_on[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0)
					return rc;
			}
	} else if (mode == BT_PCM_OFF) {
		pr_err("%s mode =BT_PCM_OFF", __func__);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_off);
			pin++) {
				rc = gpio_tlmm_config(bt_config_pcm_off[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0)
					return rc;
			}

	}

	return rc;
}

static int msm_bahama_setup_pcm_i2s(int mode)
{
	int fm_state = 0, bt_state = 0;
	int rc = 0;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	fm_state = marimba_get_fm_status(&config);
	bt_state = marimba_get_bt_status(&config);

	switch (mode) {
	case BT_PCM_ON:
	case BT_PCM_OFF:
		if (!fm_state)
			rc = config_pcm(mode);
		break;
	case FM_I2S_ON:
		rc = config_i2s(mode);
		break;
	case FM_I2S_OFF:
		if (bt_state)
			rc = config_pcm(BT_PCM_ON);
		else
			rc = config_i2s(mode);
		break;
	default:
		rc = -EIO;
		pr_err("%s:Unsupported mode", __func__);
	}
	return rc;
}

static int bt_set_gpio(int on)
{
	int rc = 0;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	if (on) {

		rc = gpio_tlmm_config(GPIO_CFG(GPIO_BT_SYS_REST_EN_, 0,
								GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
								GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
				pr_err("%s: gpio_tlmm_config for %d failed\n",
						__func__, GPIO_BT_SYS_REST_EN_);
				goto gpioconfig_fail;
		}

		rc = gpio_direction_output(GPIO_BT_SYS_REST_EN_, 1);
		msleep(100);
	} else {
		if (!marimba_get_fm_status(&config) &&
				!marimba_get_bt_status(&config)) {
			gpio_set_value_cansleep(GPIO_BT_SYS_REST_EN_, 0);
			rc = gpio_direction_input(GPIO_BT_SYS_REST_EN_);
			msleep(100);
		}
	}

gpioconfig_fail:
	if (rc)
		pr_err("%s: BT sys_reset_en GPIO : Error", __func__);

	return rc;
}
static struct vreg *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc = 0;
	const char *id = "FMPW";
	uint32_t irqcfg;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};
	u8 value;

	/* Voting for 1.8V Regulator */
	fm_regulator = vreg_get(NULL , "msme1");
	if (IS_ERR(fm_regulator)) {
		pr_err("%s: vreg get failed with : (%ld)\n",
			__func__, PTR_ERR(fm_regulator));
		return -EINVAL;
	}

	/* Set the voltage level to 1.8V */
	rc = vreg_set_level(fm_regulator, 1800);
	if (rc < 0) {
		pr_err("%s: set regulator level failed with :(%d)\n",
			__func__, rc);
		goto fm_vreg_fail;
	}

	/* Enabling the 1.8V regulator */
	rc = vreg_enable(fm_regulator);
	if (rc) {
		pr_err("%s: enable regulator failed with :(%d)\n",
			__func__, rc);
		goto fm_vreg_fail;
	}

	/* Voting for 19.2MHz clock */
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
			PMAPP_CLOCK_VOTE_ON);
	if (rc < 0) {
		pr_err("%s: clock vote failed with :(%d)\n",
			 __func__, rc);
		goto fm_clock_vote_fail;
	}

	rc = bt_set_gpio(1);
	if (rc) {
		pr_err("%s: bt_set_gpio = %d", __func__, rc);
		goto fm_gpio_config_fail;
	}
	/*re-write FM Slave Id, after reset*/
	value = BAHAMA_SLAVE_ID_FM_ADDR;
	rc = marimba_write_bit_mask(&config,
			BAHAMA_SLAVE_ID_FM_REG, &value, 1, 0xFF);
	if (rc < 0) {
		pr_err("%s: FM Slave ID rewrite Failed = %d", __func__, rc);
		goto fm_gpio_config_fail;
	}
	/* Configuring the FM GPIO */
	irqcfg = GPIO_CFG(FM_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA);

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			 __func__, irqcfg, rc);
		goto fm_gpio_config_fail;
	}

	return 0;

fm_gpio_config_fail:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_OFF);
	bt_set_gpio(0);
fm_clock_vote_fail:
	vreg_disable(fm_regulator);

fm_vreg_fail:
	vreg_put(fm_regulator);

	return rc;
};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";

	/* Releasing the GPIO line used by FM */
	uint32_t irqcfg = GPIO_CFG(FM_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA);

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			 __func__, irqcfg, rc);

	/* Releasing the 1.8V Regulator */
	if (fm_regulator != NULL) {
		rc = vreg_disable(fm_regulator);
		if (rc)
			pr_err("%s: disable regulator failed:(%d)\n",
				__func__, rc);
		fm_regulator = NULL;
	}

	/* Voting off the clock */
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		pr_err("%s: voting off failed with :(%d)\n",
			__func__, rc);
	rc = bt_set_gpio(0);
	if (rc)
		pr_err("%s: bt_set_gpio = %d", __func__, rc);
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup = fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(FM_GPIO),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
	/* Configuring the FM SoC as I2S Master */
	.is_fm_soc_i2s_master = true,
	.config_i2s_gpio = msm_bahama_setup_pcm_i2s,
};


#endif // !CONFIG_PROJECT_ES03



static unsigned bt_config_power_on[] = {
	/*bt_wakes_msm*/
	GPIO_CFG(40, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*msm wakes bt*/
	GPIO_CFG(42, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*RFR*/
	GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*CTS*/
	GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*RX*/
	GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*TX*/
	GPIO_CFG(46, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static unsigned bt_config_pcm_on[] = {
	/*PCM_DOUT*/
	GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(70, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(71, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static unsigned bt_config_power_off[] = {
	/*bt_wakes_msm*/
	GPIO_CFG(40, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*msm wakes bt*/
	GPIO_CFG(42, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*RFR*/
	GPIO_CFG(43, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*CTS*/
	GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*RX*/
	GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*TX*/
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};
static unsigned bt_config_pcm_off[] = {
	/*PCM_DOUT*/
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(69, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};



#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

#if defined(CONFIG_PROJECT_ES03) //modified by zq, form 83 to 40
static struct resource bluesleep_resources[] = {			
	{
		.name	= "gpio_host_wake",
		.start	= 40,
		.end	= 40,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 42,
		.end	= 42,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(40),
		.end	= MSM_GPIO_TO_INT(40),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

#endif



#ifndef CONFIG_PROJECT_ES03 // removed by zq

struct bahama_config_register {
		u8 reg;
		u8 value;
		u8 mask;
};
struct bt_vreg_info {
	const char *name;
	unsigned int pmapp_id;
	unsigned int level;
	unsigned int is_pin_controlled;
	struct vreg *vregs;
};
static struct bt_vreg_info bt_vregs[] = {
	{"msme1", 2, 1800, 0, NULL},
	{"bt", 21, 2900, 1, NULL},
};

static int bahama_bt(int on)
{

	int rc = 0;
	int i;

	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	struct bahama_variant_register {
		const size_t size;
		const struct bahama_config_register *set;
	};

	const struct bahama_config_register *p;

	u8 version;

	const struct bahama_config_register v10_bt_on[] = {
		{ 0xE9, 0x00, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xE4, 0x00, 0xFF },
		{ 0xE5, 0x00, 0x0F },
#ifdef CONFIG_WLAN
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
		{ 0x01, 0x0C, 0x1F },
		{ 0x01, 0x08, 0x1F },
	};

	const struct bahama_config_register v20_bt_on_fm_off[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x00, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0x8E, 0x15, 0xFF },
		{ 0x8F, 0x15, 0xFF },
		{ 0x90, 0x15, 0xFF },

		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v20_bt_on_fm_on[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v10_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_config_register v20_bt_off_fm_off[] = {
		{ 0xF4, 0x84, 0xFF },
		{ 0xF0, 0x04, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_config_register v20_bt_off_fm_on[] = {
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};
	const struct bahama_variant_register bt_bahama[2][3] = {
	{
		{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
		{ ARRAY_SIZE(v20_bt_off_fm_off), v20_bt_off_fm_off },
		{ ARRAY_SIZE(v20_bt_off_fm_on), v20_bt_off_fm_on }
	},
	{
		{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
		{ ARRAY_SIZE(v20_bt_on_fm_off), v20_bt_on_fm_off },
		{ ARRAY_SIZE(v20_bt_on_fm_on), v20_bt_on_fm_on }
	}
	};

	u8 offset = 0; /* index into bahama configs */
	on = on ? 1 : 0;
	version = marimba_read_bahama_ver(&config);
	if ((int)version < 0 || version == BAHAMA_VER_UNSUPPORTED) {
		dev_err(&msm_bt_power_device.dev, "%s: Bahama \
				version read Error, version = %d \n",
				__func__, version);
		return -EIO;
	}

	if (version == BAHAMA_VER_2_0) {
		if (marimba_get_fm_status(&config))
			offset = 0x01;
	}

	p = bt_bahama[on][version + offset].set;

	dev_info(&msm_bt_power_device.dev,
		"%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_bahama[on][version + offset].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"%s: reg %x write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x write value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
		value = 0;
		rc = marimba_read_bit_mask(&config,
				(p+i)->reg, &value,
				sizeof((p+i)->value), (p+i)->mask);
		if (rc < 0)
			dev_err(&msm_bt_power_device.dev, "%s marimba_read_bit_mask- error",
					__func__);
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x read value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	/* Update BT Status */
	if (on)
		marimba_set_bt_status(&config, true);
	else
		marimba_set_bt_status(&config, false);
	return rc;
}
static int bluetooth_switch_regulators(int on)
{
	int i, rc = 0;
	const char *id = "BTPW";

	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
		if (!bt_vregs[i].vregs) {
			pr_err("%s: vreg_get %s failed(%d)\n",
			__func__, bt_vregs[i].name, rc);
			goto vreg_fail;
		}
		rc = on ? vreg_set_level(bt_vregs[i].vregs,
				bt_vregs[i].level) : 0;

		if (rc < 0) {
			pr_err("%s: vreg set level failed (%d)\n",
					__func__, rc);
			goto vreg_set_level_fail;
		}
		rc = on ? vreg_enable(bt_vregs[i].vregs) : 0;

		if (rc < 0) {
			pr_err("%s: vreg %s %s failed(%d)\n",
					__func__, bt_vregs[i].name,
					on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
		if (bt_vregs[i].is_pin_controlled == 1) {
			rc = pmapp_vreg_lpm_pincntrl_vote(id,
					bt_vregs[i].pmapp_id,
					PMAPP_CLOCK_ID_D1,
					on ? PMAPP_CLOCK_VOTE_ON :
					PMAPP_CLOCK_VOTE_OFF);
			if (rc < 0) {
				pr_err("%s: vreg %s pin ctrl failed(%d)\n",
						__func__, bt_vregs[i].name,
						rc);
				goto pincntrl_fail;
			}
		}

		rc = on ? 0 : vreg_disable(bt_vregs[i].vregs);
		if (rc < 0) {
			pr_err("%s: vreg %s %s failed(%d)\n",
					__func__, bt_vregs[i].name,
					on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}

	return rc;
pincntrl_fail:
	if (on)
		vreg_disable(bt_vregs[i].vregs);
vreg_fail:
	while (i) {
		if (on)
			vreg_disable(bt_vregs[--i].vregs);
		}
vreg_set_level_fail:
	vreg_put(bt_vregs[0].vregs);
	vreg_put(bt_vregs[1].vregs);
	return rc;
}

static unsigned int msm_bahama_setup_power(void)
{
	int rc = 0;
	struct vreg *vreg_s3 = NULL;

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_s3));
		return PTR_ERR(vreg_s3);
	}
	rc = vreg_set_level(vreg_s3, 1800);
	if (rc < 0) {
		pr_err("%s: vreg set level failed (%d)\n",
				__func__, rc);
		goto vreg_fail;
	}
	rc = vreg_enable(vreg_s3);
	if (rc < 0) {
		pr_err("%s: vreg enable failed (%d)\n",
		       __func__, rc);
		goto vreg_fail;
	}

	/*setup Bahama_sys_reset_n*/
	rc = gpio_request(GPIO_BT_SYS_REST_EN_, "bahama sys_rst_n");
	if (rc < 0) {
		pr_err("%s: gpio_request %d = %d\n", __func__,
			GPIO_BT_SYS_REST_EN_, rc);
		goto vreg_fail;
	}
	rc = bt_set_gpio(1);
	if (rc < 0) {
		pr_err("%s: bt_set_gpio %d = %d\n", __func__,
			GPIO_BT_SYS_REST_EN_, rc);
		goto gpio_fail;
	}
	return rc;

gpio_fail:
	gpio_free(GPIO_BT_SYS_REST_EN_);
vreg_fail:
	vreg_put(vreg_s3);
	return rc;
}

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;
	struct vreg *vreg_s3 = NULL;

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_s3));
		return PTR_ERR(vreg_s3);
	}
	rc = vreg_disable(vreg_s3);
	if (rc) {
		pr_err("%s: vreg disable failed (%d)\n",
		       __func__, rc);
		vreg_put(vreg_s3);
		return rc;
	}
	if (value == BAHAMA_ID) {
		rc = bt_set_gpio(0);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
		}
	}
	return rc;
}

static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {
		int i;
		struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};
		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};
		if (marimba_read_bahama_ver(&config) == BAHAMA_VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					pr_err("%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				pr_debug("%s: reg 0x%02x value 0x%02x"
					" mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	rc = bt_set_gpio(0);
	if (rc) {
		pr_err("%s: bt_set_gpio = %d\n",
		       __func__, rc);
	}
	pr_debug("core type: %d\n", type);
	return rc;
}

static int bluetooth_power(int on)
{
	int pin, rc = 0;
	const char *id = "BTPW";
	int cid = 0;

	cid = adie_get_detected_connectivity_type();
	if (cid != BAHAMA_ID) {
		pr_err("%s: unexpected adie connectivity type: %d\n",
					__func__, cid);
		return -ENODEV;
	}
	if (on) {
		/*setup power for BT SOC*/
		if(machine_is_msm7x27a_sku1()&&!hw_version_ge(3,3)){
			rc = gpio_tlmm_config(GPIO_CFG(GPIO_VDD33_EN, 0,
					GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			if (rc) {
				pr_err("%s: gpio_tlmm_config for %d failed\n",
					__func__, GPIO_VDD33_EN);
			}
			gpio_set_value_vote(GPIO_VDD33_EN,1,GPIO_VOTE_VDD33_BT,GPIO_VOTE_VDD33_BT_MASK);
		}
		rc = bt_set_gpio(on);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
			goto exit;
		}
		rc = bluetooth_switch_regulators(on);
		if (rc < 0) {
			pr_err("%s: bluetooth_switch_regulators rc = %d",
					__func__, rc);
			goto exit;
		}
		/*setup BT GPIO lines*/
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on);
			pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					GPIO_CFG_ENABLE);
			if (rc < 0) {
				pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
						__func__,
						bt_config_power_on[pin],
						rc);
				goto fail_power;
			}
		}
		/*Setup BT clocks*/
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
			PMAPP_CLOCK_VOTE_ON);
		if (rc < 0) {
			pr_err("Failed to vote for TCXO_D1 ON\n");
			goto fail_clock;
		}
		msleep(20);

		/*I2C config for Bahama*/
		rc = bahama_bt(1);
		if (rc < 0) {
			pr_err("%s: bahama_bt rc = %d", __func__, rc);
			goto fail_i2c;
		}
		msleep(20);

		/*setup BT PCM lines*/
		rc = msm_bahama_setup_pcm_i2s(BT_PCM_ON);
		if (rc < 0) {
			pr_err("%s: msm_bahama_setup_pcm_i2s , rc =%d\n",
				__func__, rc);
				goto fail_power;
			}
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
				  PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			pr_err("%s:Pin Control Failed, rc = %d",
					__func__, rc);

	} else {
		rc = bahama_bt(0);
		if (rc < 0)
			pr_err("%s: bahama_bt rc = %d", __func__, rc);
		if(machine_is_msm7x27a_sku1()&&!hw_version_ge(3,3)){
			rc = gpio_tlmm_config(GPIO_CFG(GPIO_VDD33_EN, 0,
								GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
								GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			if (rc) {
				pr_err("%s: gpio_tlmm_config for %d failed\n",__func__, GPIO_VDD33_EN);
			}
			gpio_set_value_vote(GPIO_VDD33_EN,0,GPIO_VOTE_VDD33_BT,GPIO_VOTE_VDD33_ALL_MASK);
		}
		rc = bt_set_gpio(on);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
		}
fail_i2c:
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
				  PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0)
			pr_err("%s: Failed to vote Off D1\n", __func__);
fail_clock:
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off);
			pin++) {
				rc = gpio_tlmm_config(bt_config_power_off[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0) {
					pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				}
			}
		rc = msm_bahama_setup_pcm_i2s(BT_PCM_OFF);
		if (rc < 0) {
			pr_err("%s: msm_bahama_setup_pcm_i2s, rc =%d\n",
					__func__, rc);
				}
fail_power:
		rc = bluetooth_switch_regulators(0);
		if (rc < 0) {
			pr_err("%s: switch_regulators : rc = %d",\
					__func__, rc);
			goto exit;
		}
	}
	return rc;
exit:
	pr_err("%s: failed with rc = %d", __func__, rc);
	return rc;
}

#else
#define GPIO_BT_SHUTDOWN 49

static int bluetooth_power(int on)
{
	int pin, rc;

	if (on) {
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin], GPIO_CFG_ENABLE);
			if (rc) {
				pr_err("bt config power on failed.");
				goto POWER_OFF_CONFIG;
			}
		}

		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_on); pin++) {
			rc = gpio_tlmm_config(bt_config_pcm_on[pin], GPIO_CFG_ENABLE);
			if (rc) {
				pr_err("bt config power on failed.");
				goto PCM_OFF_CONFIG;
			} 
		}
	
		//bt_rest_n
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_BT_SYS_REST_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc)
			pr_err("bt config bt_shutdown pin failed");
		gpio_set_value(GPIO_BT_SYS_REST_EN, 1);
				
		msleep(50);

		//bt_reg_on
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_BT_SHUTDOWN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc)
			pr_err("bt config bt_shutdown pin failed");
		gpio_set_value(GPIO_BT_SHUTDOWN, 1);

		pr_err("=======> bt power on");
	} else {

		//bt_shutdown off
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_BT_SHUTDOWN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc)
			pr_err("bt config bt_shutdown pin failed");
		gpio_set_value(GPIO_BT_SHUTDOWN, 0);
		
		msleep(50);
		
		//bt_reg_on off
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_BT_SYS_REST_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc)
			pr_err("bt config bt_shutdown pin failed");
		gpio_set_value(GPIO_BT_SYS_REST_EN, 0);

PCM_OFF_CONFIG:	
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_off); pin++) {
			rc = gpio_tlmm_config(bt_config_pcm_off[pin], GPIO_CFG_ENABLE);
			if (rc) {
				pr_err("bt config power on failed.");
				return -EIO;
			} 
		}

POWER_OFF_CONFIG:		
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin], GPIO_CFG_ENABLE);
			if (rc) {
				pr_err("bt config power on failed.");
				return -EIO;
			}
		}

		pr_err("=======> bt power off");
	}
	

	return rc;
}

#endif

static int __init bt_power_init(void)
{
#if defined(CONFIG_PROJECT_ES03)
	msm_bt_power_device.dev.platform_data = &bluetooth_power;
	return 0;
#else
	int i, rc = 0;
	if( machine_is_msm7x27a_sku3() || ( machine_is_msm7x27a_sku1() && hw_version_ge(3,3)) ){
		bt_vregs[1].level = 3300;
	}
	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
			bt_vregs[i].vregs = vreg_get(NULL,
					bt_vregs[i].name);
			if (IS_ERR(bt_vregs[i].vregs)) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, bt_vregs[i].name,
				       PTR_ERR(bt_vregs[i].vregs));
				rc = PTR_ERR(bt_vregs[i].vregs);
				goto vreg_get_fail;
			}
		}

	msm_bt_power_device.dev.platform_data = &bluetooth_power;

	return rc;

vreg_get_fail:
	while (i)
		vreg_put(bt_vregs[--i].vregs);
	return rc;
#endif	
}

#ifndef CONFIG_PROJECT_ES03	// removed by zq
static struct marimba_platform_data marimba_pdata = {
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.bahama_setup                        = msm_bahama_setup_power,
	.bahama_shutdown                     = msm_bahama_shutdown_power,
	.bahama_core_config                  = msm_bahama_core_config,
	.fm				     = &marimba_fm_pdata,
};
#endif

#endif // defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)




#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static struct i2c_board_info core_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
	},
};
static struct i2c_board_info cam_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data	= &sx150x_data[SX150X_CAM],
	},
};
#endif

#if  0 // #9 removed by X		defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static struct i2c_board_info bahama_devices[] = {
{
	I2C_BOARD_INFO("marimba", 0xc),
	.platform_data = &marimba_pdata,
},
};
#endif



#ifdef CONFIG_EMUL_I2C__GPIO_61_60	// #9
static struct i2c_gpio_platform_data gpio_i2c_61_60_data = {
	.sda_pin = 60, //proto1: 60,
	.scl_pin = 61, //proto1: 61,
	.udelay  = 5, // 2
	.timeout = HZ/10, // 100
};

static struct platform_device gpio_i2c_61_60_dev = {
	.name = "i2c-gpio",
	.id = 10, //(MSM_GSBI1_QUP_I2C_BUS_ID + ?),
	.dev = {
		.platform_data = &gpio_i2c_61_60_data,
	},
};
#endif



#if defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03) 

//#define DEBUG_ACC_SENSOR
//#define DEBUG_COMPASS_SENSOR
//#define DEBUG_LIGHT_SENSOR

#ifdef DEBUG_ACC_SENSOR
static struct i2c_gpio_platform_data acc_sensor_gpio_i2c_data = {
	.sda_pin = 122,
	.scl_pin = 123,
	.udelay  = 5,
	.timeout = HZ/10,
};

static struct platform_device acc_sensor_gpio_i2c_device = {
	.name = "i2c-gpio",
	.id = (MSM_GSBI1_QUP_I2C_BUS_ID + 1),
	.dev = {
		.platform_data = &acc_sensor_gpio_i2c_data,
	},
};
#endif // DEBUG_ACC_SENSOR

#ifdef DEBUG_COMPASS_SENSOR
static struct i2c_gpio_platform_data compass_sensor_gpio_i2c_data = {
	.sda_pin = 122,
	.scl_pin = 123,
	.udelay  = 5,
	.timeout = HZ/10,
};

static struct platform_device compass_sensor_gpio_i2c_device = {
	.name = "i2c-gpio",
	.id = (MSM_GSBI1_QUP_I2C_BUS_ID + 2),
	.dev = {
		.platform_data = &compass_sensor_gpio_i2c_data,
	},
};
#endif // DEBUG_COMPASS_SENSOR

#ifdef DEBUG_LIGHT_SENSOR
static struct i2c_gpio_platform_data light_sensor_gpio_i2c_data = {
	.sda_pin = 122,
	.scl_pin = 123,
	.udelay  = 5,
	.timeout = HZ/10,
};

static struct platform_device light_sensor_gpio_i2c_device = {
	.name = "i2c-gpio",
	.id = (MSM_GSBI1_QUP_I2C_BUS_ID + 3),
	.dev = {
		.platform_data = &light_sensor_gpio_i2c_data,
	},
};
#endif // DEBUG_LIGHT_SENSOR

//static unsigned int acc_sensor_i2c_bus_id = (MSM_GSBI1_QUP_I2C_BUS_ID + 1);
//static unsigned int compass_sensor_i2c_bus_id = (MSM_GSBI1_QUP_I2C_BUS_ID + 2);
//static unsigned int light_sensor_i2c_bus_id = (MSM_GSBI1_QUP_I2C_BUS_ID + 3);


#else

static unsigned int gsensor_sensor_i2c_bus_id = MSM_GSBI1_QUP_I2C_BUS_ID;
static unsigned int ecompass_sensor_i2c_bus_id = MSM_GSBI1_QUP_I2C_BUS_ID;
static unsigned int light_sensor_i2c_bus_id = MSM_GSBI1_QUP_I2C_BUS_ID;

#endif // CONFIG_PROJECT_ES02



#ifdef  CONFIG_INPUT_L3G4200D

static struct k3g_platform_data k3g_pdata = {
	.axis_map_x	= 1,
	.axis_map_y	= 1,
	.axis_map_z	= 1,
	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
};

static struct i2c_board_info gyro_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("k3g", 0x68),
		.platform_data =  &k3g_pdata,
		.irq = -1,
	},
};

//#define GYRO_USE_IRQ
#ifdef GYRO_USE_IRQ
#define GYRO_INT_N 27
static struct msm_gpio gyro_gpio_cfg_data[] = {
	{
		GPIO_CFG(GYRO_INT_N, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
		"gyro_int"
	},
};

static int gyro_gpio_setup(void) {
	int ret = 0;
	ret = msm_gpios_request_enable(gyro_gpio_cfg_data,
				 sizeof(gyro_gpio_cfg_data)/sizeof(struct msm_gpio));
	if( ret<0 )
		printk(KERN_ERR "%s: Failed to obtain gyro int GPIO %d. Code: %d\n",
				__func__, GYRO_INT_N, ret);
	else {
		gyro_i2c_info[0].irq = gpio_to_irq(GYRO_INT_N);
		printk("gyro_i2c_info[0].irq is %d\n",gyro_i2c_info[0].irq);
	}
	return ret;
}
#endif

#endif // CONFIG_INPUT_L3G4200D


#ifdef CONFIG_BOSCH_BMA250
static struct i2c_board_info bma250_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("bma250", 0x18),
	},
};
#elif defined(CONFIG_BOSCH_BMA222)
static struct i2c_board_info bma222_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("bma222", 0x08),
	},
};
#endif

#ifdef  CONFIG_INPUT_LIS3DH

#define GPIO_ACC_INT 28

static struct lis3dh_acc_platform_data lis3dh_acc_pdata = {
	.poll_interval	= 5,
	.min_interval	= 10,

	.g_range	= LIS3DH_ACC_G_2G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,

	.gpio_int1	= -EINVAL,//GPIO_ACC_INT, not use interrupt
	.gpio_int2	= -EINVAL,

};

static struct i2c_board_info lis3dh_acc_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("lis3dh_acc", 0x18),
		.platform_data =  &lis3dh_acc_pdata,
		.irq = -1,
	},
};

static struct msm_gpio lis3dh_acc_gpio_cfg_data[] = {
	{
		GPIO_CFG(GPIO_ACC_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
		"lis3dh_acc_int"
	},
};

static int lis3dh_acc_gpio_setup(void) {
	int ret = 0;
	ret = msm_gpios_request_enable(lis3dh_acc_gpio_cfg_data,
				 sizeof(lis3dh_acc_gpio_cfg_data)/sizeof(struct msm_gpio));
	if( ret<0 )
		printk(KERN_ERR "%s: Failed to obtain acc int GPIO %d. Code: %d\n",
				__func__, GPIO_ACC_INT, ret);
	//lis3dh_acc_i2c_info[0].irq = gpio_to_irq(GPIO_ACC_INT);
	return ret;
}
#endif

#ifdef CONFIG_SENSORS_AK8975
static struct akm8975_platform_data akm_platform_data_8975 = {
		.gpio_DRDY = COMPASS_DRDY,
};

static struct i2c_board_info akm8975_i2c_info[] __initdata = {
	{
#if defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03)
		I2C_BOARD_INFO("akm8975", 0x0c), // CAD0 & CAD1 both connected to VSS on ES02 and ES03, so slave address is 0x0c
#else
		I2C_BOARD_INFO("akm8975", 0x0e),
#endif
		.platform_data =  &akm_platform_data_8975,
		.flags = I2C_CLIENT_WAKE,
		.irq = MSM_GPIO_TO_INT(COMPASS_DRDY),
	},
};
#endif

#ifdef CONFIG_SENSORS_MMC328XMA
static struct i2c_board_info mmc328xma_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("mmc328x", 0x30),
		.flags = I2C_CLIENT_WAKE,
	},
};
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_INPUT_LTR502)

#ifndef LTR502_INT_GPIO
#define LTR502_INT_GPIO 17
#endif

static struct ltr502_platform_data ltr502_pdata = {
	.int_gpio = LTR502_INT_GPIO,
};

static struct i2c_board_info ltr502_light_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("ltr502", 0x1c),
		.platform_data =  &ltr502_pdata,
	},
};

static struct msm_gpio ltr502_light_gpio_cfg_data[] = {
	{GPIO_CFG(LTR502_INT_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "ltr502_light_int"},
};

static int ltr502_light_gpio_setup(void) {
	int ret = 0;

	ret = msm_gpios_request_enable(ltr502_light_gpio_cfg_data, 1);
	if(ret < 0)
		printk(KERN_ERR "%s: Failed to obtain acc int GPIO %d. Code: %d\n",
				__func__, LTR502_INT_GPIO, ret);

	return ret;
}
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_APDS990X)
static struct i2c_board_info apds990x_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("apds990x", 0x39),
	},
};
#endif

#if defined(CONFIG_I2C)
static void __init register_i2c_devices(void)
{
#ifdef CONFIG_INPUT_L3G4200D

#ifdef GYRO_USE_IRQ
	gyro_gpio_setup();
#endif

	printk("i2c_register_board_info GYRO\n");
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				gyro_i2c_info,
				ARRAY_SIZE(gyro_i2c_info));
#endif

#ifdef CONFIG_BOSCH_BMA250
	if (!(machine_is_msm7x27a_sku3() && hw_version_is(1,0))) {
		printk("i2c_register_board_info BMA250 ACC\n");
		i2c_register_board_info(acc_sensor_i2c_bus_id,
					bma250_i2c_info,
					ARRAY_SIZE(bma250_i2c_info));
	}
#elif defined(CONFIG_BOSCH_BMA222)
	pr_info("i2c_register_board_info BMA222 ACC\n");
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,/*acc_sensor_i2c_bus_id,*/
				bma222_i2c_info,
				ARRAY_SIZE(bma222_i2c_info));
#endif

#ifdef CONFIG_INPUT_LIS3DH
	if (!(machine_is_msm7x27a_sku3() && hw_version_ge(2,0))) {
		lis3dh_acc_gpio_setup();
		printk("i2c_register_board_info LIS3DH ACC\n");

		i2c_register_board_info(acc_sensor_i2c_bus_id,
					lis3dh_acc_i2c_info,
					ARRAY_SIZE(lis3dh_acc_i2c_info));
	}
#endif


#ifdef CONFIG_SENSORS_AK8975
	printk("i2c_register_board_info AKM8975\n");
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,/*compass_sensor_i2c_bus_id,*/
				akm8975_i2c_info,
				ARRAY_SIZE(akm8975_i2c_info));
#endif

#ifdef CONFIG_SENSORS_MMC328XMA
	printk("i2c_register_board_info MMC328XMA\n");
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,/*compass_sensor_i2c_bus_id,*/
				mmc328xma_i2c_info,
				ARRAY_SIZE(mmc328xma_i2c_info));
#endif
#ifdef CONFIG_INPUT_LTR502
	printk("i2c_register_board_info LTR502\n");
	ltr502_light_gpio_setup();
	i2c_register_board_info(light_sensor_i2c_bus_id,
				ltr502_light_i2c_info,
				ARRAY_SIZE(ltr502_light_i2c_info));
#elif defined(CONFIG_SENSORS_APDS990X)
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,/*light_sensor_i2c_bus_id,*/
				apds990x_i2c_info,
				ARRAY_SIZE(apds990x_i2c_info));
#endif

#if defined(CONFIG_GPIO_SX150X)
	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
				cam_exp_i2c_info,
				ARRAY_SIZE(cam_exp_i2c_info));

	if (machine_is_msm7x27a_surf())
		sx150x_data[SX150X_CORE].io_open_drain_ena = 0xe0f0;

	core_exp_i2c_info[0].platform_data =
			&sx150x_data[SX150X_CORE];

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				core_exp_i2c_info,
				ARRAY_SIZE(core_exp_i2c_info));
#endif
#if 0 // #9 removed by X	defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE) 
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				bahama_devices,
				ARRAY_SIZE(bahama_devices));
#endif
}
#endif

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc;

	if (adap_id < 0 || adap_id > 1)
		return;

	/* Each adapter gets 2 lines from the table */
	if (config_type)
		rc = msm_gpios_request_enable(&qup_i2c_gpios_hw[adap_id*2], 2);
	else
		rc = msm_gpios_request_enable(&qup_i2c_gpios_io[adap_id*2], 2);
	if (rc < 0)
		pr_err("QUP GPIO request/enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.clk			= "gsbi_qup_clk",
	.pclk			= "gsbi_qup_pclk",
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.clk			= "gsbi_qup_clk",
	.pclk			= "gsbi_qup_pclk",
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

#if defined(CONFIG_TOUCHSCREEN_FT5206) || defined(CONFIG_TOUCHSCREEN_FT5306)
#ifndef FT5X06_IRQ_GPIO
#define FT5X06_IRQ_GPIO (48)
#endif

#ifndef FT5X06_RESET_GPIO
#define FT5X06_RESET_GPIO (26)
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5306)
#define FT5X06_WAKE_GPIO (30)
#endif

static struct msm_gpio ft5x06_cfg_data[] = {
{GPIO_CFG(FT5X06_IRQ_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),"ft5x06_irq"},
{GPIO_CFG(FT5X06_RESET_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA),"ft5x06_reset"},
#ifdef CONFIG_TOUCHSCREEN_FT5306
{GPIO_CFG(FT5X06_WAKE_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA),"ft5x06_wake"},
#endif
};

static ssize_t
ft5x06_virtual_keys_register(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     char *buf)
{
#ifdef CONFIG_TOUCHSCREEN_RESOLUTION_IS_WVGA
	return sprintf(buf,
                 __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":60:830:120:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":180:830:120:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":300:830:120:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":420:830:120:60"
            "\n");
#else
	return sprintf(buf,
                 __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":40:510:80:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":120:510:80:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":200:510:80:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":280:510:80:60"
            "\n");
#endif
}

static struct kobj_attribute ft5x06_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5x0x_ts",
		.mode = S_IRUGO,
	},
	.show = &ft5x06_virtual_keys_register,
};

static struct attribute *ft5x06_virtual_key_properties_attrs[] = {
        &ft5x06_virtual_keys_attr.attr,
        NULL
};

static struct attribute_group ft5x06_virtual_key_properties_attr_group = {
        .attrs = ft5x06_virtual_key_properties_attrs,
};

struct kobject *ft5x06_virtual_key_properties_kobj;

static struct ft5x0x_ts_platform_data ft5x06_platformdata = {
	.irq	= MSM_GPIO_TO_INT(FT5X06_IRQ_GPIO),
	.reset	= FT5X06_RESET_GPIO,
#if defined(CONFIG_TOUCHSCREEN_FT5306)
	.wake	= FT5X06_WAKE_GPIO,
#endif
};

static struct i2c_board_info i2c_info_ft5206 = {
	I2C_BOARD_INFO("ft5x0x_ts", 0x38),
	.platform_data = &ft5x06_platformdata,
};

static int ft5x06_touchpad_setup(void)
{
	int retval = 0;

    ft5x06_virtual_key_properties_kobj = kobject_create_and_add( \
    "board_properties", NULL);
    if (ft5x06_virtual_key_properties_kobj)
            retval = sysfs_create_group(ft5x06_virtual_key_properties_kobj, \
            &ft5x06_virtual_key_properties_attr_group);
    if (!ft5x06_virtual_key_properties_kobj || retval)
            pr_err("failed to create ft5206 board_properties\n");

	retval = msm_gpios_request_enable(ft5x06_cfg_data, sizeof(ft5x06_cfg_data)/sizeof(struct msm_gpio));
	if(retval) {
		printk(KERN_ERR "%s: Failed to obtain touchpad GPIO. Code: %d.", __func__, retval);
	}

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			&i2c_info_ft5206, 1);

	return retval;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_MCS6024_TS)
static struct i2c_client mcs6024_platformdata = {
        .irq    = MSM_GPIO_TO_INT(48),
        //.wake   = 35,//del by Joe 120507
};


static struct i2c_board_info i2c_info_mcs6024 = {
	I2C_BOARD_INFO("mcs6024", 0x20),
	.platform_data = &mcs6024_platformdata,
};

static int mcs6024_touchpad_setup(void)
{
	int retval = 0;

	pr_info("the mcs6024_touchpad_setup is setup\n");

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			&i2c_info_mcs6024, 1);

	return retval;
}
#endif // CONFIG_TOUCHSCREEN_MCS6024_TS


#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C

#ifndef CLEARPAD3000_ATTEN_GPIO
#define CLEARPAD3000_ATTEN_GPIO (48)
#endif

#ifndef CLEARPAD3000_RESET_GPIO
#define CLEARPAD3000_RESET_GPIO (26)
#endif

static int synaptics_touchpad_setup(void);
static void synaptics_touchpad_teardown(void);

static struct msm_gpio clearpad3000_cfg_data[] = {
{GPIO_CFG(CLEARPAD3000_ATTEN_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),"rmi4_attn"},
{GPIO_CFG(CLEARPAD3000_RESET_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),"rmi4_reset"},
};

static struct rmi_XY_pair rmi_offset = {.x = 0, .y = 0};
static struct rmi_range rmi_clipx = {.min = 48, .max = 980};
static struct rmi_range rmi_clipy = {.min = 7, .max = 1647};
static struct rmi_f11_functiondata synaptics_f11_data = {
 	.swap_axes = false,
 	.flipX = false,
	.flipY = false,
	.offset = &rmi_offset,
	.button_height = 113,
	.clipX = &rmi_clipx,
	.clipY = &rmi_clipy,
};

static ssize_t
clearpad3000_virtual_keys_register(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     char *buf)
{
	if (machine_is_msm7x27a_sku1())
	{
		return sprintf(buf,
        	         __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":60:830:120:60"
        	    ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":180:830:120:60"
        	    ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":300:830:120:60"
        	    ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":420:830:120:60"
        	    "\n");
	}
	else if (machine_is_msm7x27a_sku3())
	{
                return sprintf(buf,
                         __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":40:500:80:40"
                    ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":120:500:80:40"
                    ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":200:500:80:40"
                    ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":280:500:80:40"
                    "\n");
	}
	else
	{
		return sprintf(buf, "Not support this machine yet!\n");
	}
}

static struct kobj_attribute clearpad3000_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.sensor00fn11",
		.mode = S_IRUGO,
	},
	.show = &clearpad3000_virtual_keys_register,
};

static struct attribute *virtual_key_properties_attrs[] = {
        &clearpad3000_virtual_keys_attr.attr,
        NULL
};

static struct attribute_group virtual_key_properties_attr_group = {
        .attrs = virtual_key_properties_attrs,
};

struct kobject *virtual_key_properties_kobj;

static struct rmi_functiondata synaptics_functiondata[] = {
	{
		.function_index = RMI_F11_INDEX,
		.data = &synaptics_f11_data,
	},
};

static struct rmi_functiondata_list synaptics_perfunctiondata = {
	.count = ARRAY_SIZE(synaptics_functiondata),
	.functiondata = synaptics_functiondata,
};

static struct rmi_sensordata synaptics_sensordata = {
	.perfunctiondata = &synaptics_perfunctiondata,
	.rmi_sensor_setup	= synaptics_touchpad_setup,
	.rmi_sensor_teardown	= synaptics_touchpad_teardown,
};

static struct rmi_i2c_platformdata synaptics_platformdata = {
	.i2c_address = 0x2c,
	//.irq_type = IORESOURCE_IRQ_LOWLEVEL,
	.sensordata = &synaptics_sensordata,
};

static struct i2c_board_info synaptic_i2c_clearpad3k = {
	I2C_BOARD_INFO("rmi4_ts", 0x2c),
	.platform_data = &synaptics_platformdata,
};

static int synaptics_touchpad_setup(void)
{
	int retval = 0;

    virtual_key_properties_kobj = kobject_create_and_add( \
    "board_properties", NULL);
    if (virtual_key_properties_kobj)
            retval = sysfs_create_group(virtual_key_properties_kobj, \
            &virtual_key_properties_attr_group);
    if (!virtual_key_properties_kobj || retval)
            pr_err("failed to create clearpad3000 board_properties\n");

	retval = msm_gpios_request_enable(clearpad3000_cfg_data, sizeof(clearpad3000_cfg_data)/sizeof(struct msm_gpio));
	if(retval)
	{
		printk(KERN_ERR "%s: Failed to obtain touchpad GPIO %d. Code: %d.", __func__, CLEARPAD3000_ATTEN_GPIO, retval);
		retval = 0;//ignore the err
	}
	//synaptics_platformdata.irq = gpio_to_irq(CLEARPAD3000_ATTEN_GPIO);

	gpio_set_value(CLEARPAD3000_RESET_GPIO, 1);
	gpio_set_value(CLEARPAD3000_RESET_GPIO, 0);
	usleep(2);
	gpio_set_value(CLEARPAD3000_RESET_GPIO, 1);
	usleep(35000);

	return retval;
}

static void synaptics_touchpad_teardown(void)
{
	printk(KERN_WARNING "%s: NO SYNAPTICS GPIO TEARDOWN!", __func__);
}
#endif

#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_PMEM_MDP_SIZE       0x1DD1000
#define MSM_PMEM_ADSP_SIZE      0x1400000

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER	// #9 
#define MSM_FB_SIZE		0x4C0000 //Double for RGBA_8888, Tony modify on 20120211, original 0x260000
#else
#define MSM_FB_SIZE		0x32A000 //Double for RGBA_8888, Tony modify on 20120211, original 0x195000
#endif

#endif

#if defined(CONFIG_BATTERY_BQ27510) || \
                defined(CONFIG_BATTERY_BQ27510_MODULE)
static unsigned fuel_config_gpio_on[] = {
        /*BATT_LOW*/
        GPIO_CFG(9, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
        /*BI/TOUT*/
        GPIO_CFG(10, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static unsigned fuel_config_gpio_off[] = {
        /*BATT_LOW*/
        GPIO_CFG(9, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
        /*BI/TOUT*/
        GPIO_CFG(10, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static int config_fuel_hw(int mode)
{
        int pin, rc = 0;

        if (mode == 0) {
		pr_err("%s fuel gauge is on\n", __func__);
                for (pin = 0; pin < ARRAY_SIZE(fuel_config_gpio_on); pin++) {
                                rc = gpio_tlmm_config(fuel_config_gpio_on[pin],
                                        GPIO_CFG_ENABLE);
                                if (rc < 0)
                                        return rc;
                        }
        } else if (mode == 1) {
                pr_err("%s fuel gauge is off", __func__);
                for (pin = 0; pin < ARRAY_SIZE(fuel_config_gpio_off);
                        pin++) {
                                rc = gpio_tlmm_config(fuel_config_gpio_off[pin],
                                        GPIO_CFG_ENABLE);
                                if (rc < 0)
                                        return rc;
                        }

        }

        return rc;
}

static struct bq27510_platform_data bq27510_pdata = {
        .name           = "fuel-gauge",
        .vreg_name      = "8058_s3",
        .vreg_value     = 1800000,
	.irq		=  MSM_GPIO_TO_INT(9),
	.enable_gpio    = config_fuel_hw,                   /* */
};

static struct i2c_board_info msm_bq27510_board_info[] __initdata= {
        {
                I2C_BOARD_INFO("bq27510", 0xaa>>1),
                .platform_data = &bq27510_pdata,
        },
};
#endif

static char *usb_functions_default[] = {
	"diag",
	"modem",
	"nmea",
 #ifdef CONFIG_PCUI_SUPPORT
	"pcui",
 #endif
 #ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
 #endif
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
 #ifdef CONFIG_PCUI_SUPPORT
	"pcui",
 #endif
 #ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
 #endif
	"usb_mass_storage",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"adb",
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
 #ifdef CONFIG_PCUI_SUPPORT
	"pcui",
 #endif
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
	"usb_mass_storage",
};

static struct android_usb_product usb_products[] = 
#ifdef CONFIG_USB_TCT_VID
{
	{
		.product_id	= 0x012D,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
	},
	{
		.product_id	= 0x012C,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0x012A,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0156,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_CDROM_SUPPORT
	{
		.product_id	= 0x011F,
		.num_functions	= ARRAY_SIZE(usb_functions_cdrom),
		.functions	= usb_functions_cdrom,
	},
#endif
};
#else
{
	{ /* switch 9025 to the first since it's default device for adb enabled */
		.product_id	= 0x9025,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0x9026,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
	},
	{
		.product_id	= 0xf00e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x9024,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};
#endif

static struct usb_mass_storage_platform_data mass_storage_pdata = {
#ifdef CONFIG_USB_TCT_VID
	.nluns		= 2,
	.vendor		= "Android ",
#else
	.nluns		= 3,
	.vendor		= "Qualcomm Incorporated",
#endif
	.product	= "Mass storage",
	.release	= 0x0100,
#ifdef CONFIG_USB_TCT_VID
	.cdrom_num	= 1,
#else
	.cdrom_num	= 2,
#endif
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
#ifdef CONFIG_USB_TCT_VID
	.vendor_id	= 0x1BBB,
	.product_id	=  0x012D,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "Android Terminal",
	.serial_number = "1BBBE03ABCDEF",
#else
	.vendor_id	= 0x05C6,
	.product_id	= 0x9026,
	.version	= 0x0100,
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
#endif
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
#ifdef CONFIG_USB_TCT_VID
	.vendorID	= 0x1BBB,
	.vendorDescr	= "Android Terminal",
#else
	.vendorID	= 0x05C6,
	.vendorDescr	= "Qualcomm Incorporated",
#endif
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	//android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc = 0;
	unsigned gpio;

	gpio = GPIO_HOST_VBUS_EN;
	if (on)
                msm_hsusb_vbus_powerup();
        else
                msm_hsusb_vbus_shutdown();

	rc = gpio_request(gpio, "i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", gpio);
		return;
	}
	gpio_direction_output(gpio, !!on);
	gpio_set_value_cansleep(gpio, !!on);
	gpio_free(gpio);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};

static void __init msm7x2x_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret = 0;

	if (init)
		ret = msm_pm_app_rpc_init(callback);
	else
		msm_pm_app_rpc_deinit(callback);

	return ret;
}
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init	 = msm_hsusb_pmic_notif_init,
#else
	.vbus_power		 = msm_hsusb_vbus_power,
#endif
	.rpc_connect		 = hsusb_rpc_connect,
	.core_clk		 = 1,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.chg_init		 = hsusb_chg_init,
	.chg_connected		 = hsusb_chg_connected,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
};
#endif

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x90000300,
		.end   = 0x900003ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(4),
		.end   = MSM_GPIO_TO_INT(4),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;
static struct vreg *vreg_emmc;

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

/**
 * Due to insufficient drive strengths for SDC GPIO lines some old versioned
 * SD/MMC cards may cause data CRC errors. Hence, set optimal values
 * for SDC slots based on timing closure and marginality. SDC1 slot
 * require higher value since it should handle bad signal quality due
 * to size of T-flash adapters.
 */
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_14MA),
								"sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_0"},
#ifdef CONFIG_BCM_4330_del								
	{GPIO_CFG(39, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
								"wlan_oob"},
#endif								
};

static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_0"},
#ifdef CONFIG_BCM_4330_del								
	{GPIO_CFG(39, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
								"wlan_oob"},
#endif						
};
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_0"},
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_7"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_6"},
	{GPIO_CFG(21, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_5"},
	{GPIO_CFG(108, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_4"},
#endif
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(19, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_3"},
	{GPIO_CFG(20, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_2"},
	{GPIO_CFG(21, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_1"},
	{GPIO_CFG(107, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_cmd"},
	{GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_0"},
	{GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = sdc2_sleep_cfg_data,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
};

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable)||(3 == dev_id))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			pr_err("%s: Failed to turn on GPIOs for slot %d\n",
					__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			rc = msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return rc;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc)
			pr_err("%s: vreg_set_level() = %d\n", __func__, rc);

		rc = vreg_enable(curr->vreg_data);
		if (rc)
			pr_err("%s: vreg_enable() = %d\n", __func__, rc);
	} else {
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc)
			pr_err("%s: vreg_disable() = %d\n", __func__, rc);
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	rc = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	if (rc)
		goto out;

	rc = msm_sdcc_setup_vreg(pdev->id, !!vdd);
out:
	return rc;
}


#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) \
	&& defined(CONFIG_MMC_MSM_CARD_HW_DETECTION)
	
#if defined(CONFIG_MACH_MSM7X27A_SKU1) || defined(CONFIG_MACH_MSM7X27A_SKU3)
#define GPIO_SDC1_HW_DET 42
#else
#define GPIO_SDC1_HW_DET 85
#endif

static unsigned int msm7x2xa_sdcc_slot_status(struct device *dev)
{
	int status;

	status = gpio_tlmm_config(GPIO_CFG(GPIO_SDC1_HW_DET, 2, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (status)
		pr_err("%s:Failed to configure tlmm for GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);

	status = gpio_request(GPIO_SDC1_HW_DET, "SD_HW_Detect");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);
	} else {
		status = gpio_direction_input(GPIO_SDC1_HW_DET);
		if (!status)
			status = gpio_get_value(GPIO_SDC1_HW_DET);
		gpio_free(GPIO_SDC1_HW_DET);
	}

	if((machine_is_msm7x27a_sku3())||(machine_is_msm7x27a_sku1()&& hw_version_ge(2,0))){
		status = !status;
	}

	return status;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data sdc1_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x2xa_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(GPIO_SDC1_HW_DET),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data sdc2_plat_data = {
	/*
	 * SDC2 supports only 1.8V, claim for 2.85V range is just
	 * for allowing buggy cards who advertise 2.8V even though
	 * they can operate at 1.8V supply.
	 */
	.ocr_mask	= MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(66),
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data sdc3_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
static struct mmc_platform_data sdc4_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
};
#endif
#endif

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif
static struct msm_pm_platform_data msm7x27a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 1,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}

early_param("fb_size", fb_size_setup);

/* LK splash flag, 0 - off, 1 - on */
int boot_splash = 0;
static int __init lk_splash_setup(char *str)
{
	boot_splash = simple_strtol(str, NULL, 0);
	printk("boot_splash = %d\n", boot_splash);
	boot_splash = 1;//add by Joe for splash display white screen
	return 0;
}
__setup("splash=", lk_splash_setup);

#if defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03)
#define GPIO_LCD_DSI_SEL 0
#define GPIO_LCDC_BRDG_PD	0
#define GPIO_LCDC_BRDG_RESET_N	85
#else
#define GPIO_LCD_DSI_SEL 125
#define GPIO_LCDC_BRDG_PD	128
#define GPIO_LCDC_BRDG_RESET_N	129
#endif


#if !defined(CONFIG_PROJECT_ES03) // removed by X

#define LCD_CAMERA_LDO_2V8 35 //SKU1&SKU3 2.8V LDO
#define SKU1_LCD_CAMERA_LDO_1V8 40 //SKU1 1.8V LDO
#define SKU3_LCD_CAMERA_LDO_1V8 34 //SKU3 1.8V LDO
#define SKU3_1_LCD_CAMERA_LDO_1V8 58 //SKU3-1 1.8V LDO

static int lcd_camera_ldo_1v8 = GPIO_INVALID;
static atomic_t lcd_camera_power_cnt = ATOMIC_INIT(-1);

#endif

static struct vreg *lcd_camera_vreg = NULL; // ES03: 2850mV	 //SKU3 VREG 1.8V


static int lcd_camera_control_is_needed(void)
{
  int need = 0;
  
#if defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03)
  need = 1;
#else
  if (machine_is_msm7x27a_sku3()) {

    if (!hw_version_ge(2,0)) {
      pr_info("need not in old board\n");
      need = 0;
    } else {
      need = 1;
    }
  } else if (machine_is_msm7x27a_sku1()) {
    /* TODO: check for SKU1 */
    need = 1;
  } else {
    pr_info("need not in surf & ffa board\n");
    need = 0;
  }
#endif

  pr_info("%s: need = %d\n", __func__, need);

  return need;
}

static void lcd_camera_power_init(void)
{
  int rc = 0;

  pr_info("lcd_camera_power_init\n");

  if (lcd_camera_control_is_needed() == 0)
    return;

#if defined(CONFIG_PROJECT_ES03)

	lcd_camera_vreg = vreg_get(NULL, "gp2"); // VREG_L12
	rc = vreg_set_level(lcd_camera_vreg, 2850);
	if (rc < 0) {
	  pr_err("%s: set regulator level failed with :(%d)\n", __func__, rc);
	  goto fail_vreg;
	}

#else // !CONFIG_PROJECT_ES03



#if defined(CONFIG_PROJECT_ES02)
  if (1) {
#else
  if (machine_is_msm7x27a_sku1()) {
#endif
    lcd_camera_ldo_1v8 = SKU1_LCD_CAMERA_LDO_1V8;
  } else {
    if (hw_version_is(3,0)) {
      lcd_camera_ldo_1v8 = SKU3_LCD_CAMERA_LDO_1V8; //SKU3 PVT
    } else if (hw_version_is(10,0)) {
      lcd_camera_ldo_1v8 = SKU3_1_LCD_CAMERA_LDO_1V8; //SKU3-1 PVT
    }
  }

#if defined(CONFIG_PROJECT_ES02) 
  if (0) {
#else
  if (machine_is_msm7x27a_sku3()) {
#endif
    //VREG_L1, SKU3 PVT uses it for CTP 1.8V, DVT uses it for LCD/Camera 1.8V
    lcd_camera_vreg = vreg_get(NULL, "rfrx1");
    rc = vreg_set_level(lcd_camera_vreg, 1800);
    if (rc < 0) {
      pr_err("%s: set regulator level failed "
      "with :(%d)\n", __func__, rc);
      goto fail_vreg;
    }
  }

  if (!boot_splash) {
#if 0  //#9 removed
    //LDO_EXT2V8
    if (gpio_request(LCD_CAMERA_LDO_2V8, "lcd_camera_ldo_2v8")) {
      pr_err("failed to request gpio lcd_camera_ldo_2v8\n");
      return;
    }

    rc = gpio_tlmm_config(GPIO_CFG(LCD_CAMERA_LDO_2V8, 0,
          GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
          GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc < 0) {
      pr_err("%s: unable to enable lcd_camera_ldo_2v8!\n", __func__);
      goto fail_gpio2;
    }
#endif

#if defined(CONFIG_PROJECT_ES02) 
    if (1) {
#else
    if (machine_is_msm7x27a_sku1() || (machine_is_msm7x27a_sku3() && hw_version_ge(3,0))) {
#endif
      //LDO_EVT1V8
      if (gpio_request(lcd_camera_ldo_1v8, "lcd_camera_ldo_1v8")) {
        pr_err("failed to request gpio lcd_camera_ldo_1v8\n");
        goto fail_gpio2;
      }

      rc = gpio_tlmm_config(GPIO_CFG(lcd_camera_ldo_1v8, 0,
              GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
              GPIO_CFG_2MA), GPIO_CFG_ENABLE);
      if (rc < 0) {
        pr_err("%s: unable to enable lcd_camera_ldo_1v8!\n", __func__);
        goto fail_gpio1;
      }
    }
  }

  return;

fail_gpio1:
  gpio_free(lcd_camera_ldo_1v8);
fail_gpio2:
  gpio_free(LCD_CAMERA_LDO_2V8);

#endif // CONFIG_PROJECT_ES03
fail_vreg:
  vreg_put(lcd_camera_vreg);
  return;
}


#if !defined(CONFIG_PROJECT_ES03)	// removed by X

static int lcd_camera_power_on_sku1(void)
{
  pr_info("turn on sku1 lcd_camera_ldo\n");
  gpio_set_value_cansleep(lcd_camera_ldo_1v8, 1);
  gpio_set_value_cansleep(LCD_CAMERA_LDO_2V8, 1);

  return 0;
}

static int lcd_camera_power_off_sku1(void)
{
  pr_info("turn off sku1 lcd_camera_ldo\n");

  gpio_set_value_cansleep(lcd_camera_ldo_1v8, 0);
  gpio_set_value_cansleep(LCD_CAMERA_LDO_2V8, 0);

  return 0;
}

static int lcd_camera_power_on_sku3(void)
{
  int rc = 0;

  if (hw_version_ge(3,0)) {
    pr_info("turn on sku3 lcd_camera_ldo_1v8\n");
    gpio_set_value_cansleep(lcd_camera_ldo_1v8, 1);
  }

  pr_info("turn on sku3 lcd_camera_vreg\n");
  rc = vreg_enable(lcd_camera_vreg);
  if (rc) {
    pr_err("vreg_enable: lcd_camera_vreg vreg failed\n");
  }

  pr_info("turn on sku3 lcd_camera_ldo\n");
  gpio_set_value_cansleep(LCD_CAMERA_LDO_2V8, 1);

  return rc;
}

static int lcd_camera_power_off_sku3(void)
{
  int rc = 0;

  if (hw_version_ge(3,0)) {
    pr_info("turn off sku3 lcd_camera_ldo_1v8\n");
    gpio_set_value_cansleep(lcd_camera_ldo_1v8, 0);
  }

  pr_info("turn off sku3 lcd_camera_vreg\n");
  rc = vreg_disable(lcd_camera_vreg);
  if (rc) {
    pr_err("vreg_disable: lcd_camera_vreg vreg failed\n");
  }

  pr_info("turn off sku3 lcd_camera_ldo\n");
  gpio_set_value_cansleep(LCD_CAMERA_LDO_2V8, 0);

  return rc;
}
#endif // CONFIG_PROJECT_ES03


int lcd_camera_power_onoff(int on)
{
  int rc = 0;

#if !defined(CONFIG_PROJECT_ES03)  // removed by X
  int count = 0;
  /* print the count number */
//  return rc;
  
  count = atomic_read(&lcd_camera_power_cnt);
  pr_info("lcd_camera_power_onoff on = %d, count number is %d \n", on, count);

  if (lcd_camera_control_is_needed() == 0)
    return rc;

  if (on) {
    if (atomic_inc_and_test(&lcd_camera_power_cnt)) {
#if defined(CONFIG_PROJECT_ES02)
        rc = lcd_camera_power_on_sku1();
        return rc;
#endif
      if (machine_is_msm7x27a_sku1()) {
        rc = lcd_camera_power_on_sku1();
      } else {
        rc = lcd_camera_power_on_sku3();
      }
    }
  } else {
    if (atomic_sub_return(1, &lcd_camera_power_cnt) == -1) {
#if defined(CONFIG_PROJECT_ES02)
        rc = lcd_camera_power_off_sku1();
        return rc;
#endif
      if (machine_is_msm7x27a_sku1()) {
        rc = lcd_camera_power_off_sku1();
      } else {
        rc = lcd_camera_power_off_sku3();
      }
    }
  }
#endif // !CONFIG_PROJECT_ES03

  return rc;
}
EXPORT_SYMBOL(lcd_camera_power_onoff);




#ifndef CONFIG_PROJECT_ES03	// removed by X

static const char * const msm_fb_lcdc_vreg[] = {
		"gp2",		
		"msme1",
};

static const int msm_fb_lcdc_vreg_mV[] = {
	2850,
	1800,
};

struct vreg *lcdc_vreg[ARRAY_SIZE(msm_fb_lcdc_vreg)];
static uint32_t lcdc_gpio_initialized;

#ifndef CONFIG_MACH_MSM7X27A_SKU3
static void lcdc_toshiba_gpio_init(void)
{
	int i, rc = 0;
	if (!lcdc_gpio_initialized) {
		if (gpio_request(GPIO_SPI_CLK, "spi_clk")) {
			pr_err("failed to request gpio spi_clk\n");
			return;
		}
		if (gpio_request(GPIO_SPI_CS0_N, "spi_cs")) {
			pr_err("failed to request gpio spi_cs0_N\n");
			goto fail_gpio6;
		}
		if (gpio_request(GPIO_SPI_MOSI, "spi_mosi")) {
			pr_err("failed to request gpio spi_mosi\n");
			goto fail_gpio5;
		}
		if (gpio_request(GPIO_SPI_MISO, "spi_miso")) {
			pr_err("failed to request gpio spi_miso\n");
			goto fail_gpio4;
		}

		if (gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr")) {
			pr_err("failed to request gpio_disp_pwr\n");
			goto fail_gpio3;
		}
		if (gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en")) {
			pr_err("failed to request gpio_bkl_en\n");
			goto fail_gpio2;
		}
		pmapp_disp_backlight_init();

		for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_vreg); i++) {
			lcdc_vreg[i] = vreg_get(0, msm_fb_lcdc_vreg[i]);

			rc = vreg_set_level(lcdc_vreg[i],
						msm_fb_lcdc_vreg_mV[i]);

			if (rc < 0) {
				pr_err("%s: set regulator level failed "
					"with :(%d)\n", __func__, rc);
				goto fail_gpio1;
			}
		}
		lcdc_gpio_initialized = 1;
	}
	return;

fail_gpio1:
	for (; i > 0; i--)
			vreg_put(lcdc_vreg[i - 1]);

	gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio2:
	gpio_free(GPIO_DISPLAY_PWR_EN);
fail_gpio3:
	gpio_free(GPIO_SPI_MISO);
fail_gpio4:
	gpio_free(GPIO_SPI_MOSI);
fail_gpio5:
	gpio_free(GPIO_SPI_CS0_N);
fail_gpio6:
	gpio_free(GPIO_SPI_CLK);
	lcdc_gpio_initialized = 0;
}

static uint32_t lcdc_gpio_table[] = {
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_SPI_MOSI,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_SPI_MISO,
};
#else
static uint32_t lcdc_gpio_table[] = {
	GPIO_SPI_MOSI,  /* spi_sdi */
	GPIO_SPI_CLK,   /* spi_clk */
	GPIO_SPI_CS0_N, /* spi_cs  */
	GPIO_BACKLIGHT_EN,
	GPIO_DISPLAY_RESET,
};
static char *lcdc_gpio_name_table[6] = {
	"spi_mosi",
	"spi_clk",
	"spi_cs",
	"gpio_bkl_en",
	"gpio_disp_reset",
};
static void lcdc_truly_gpio_init(void)
{
	int i;
	int rc = 0;

	pr_info("%s: lcdc_gpio_initialized = %d\n", __func__, lcdc_gpio_initialized);

	if (!lcdc_gpio_initialized) {
		for (i = 0; i < ARRAY_SIZE(lcdc_gpio_table); i++) {
			rc = gpio_request(lcdc_gpio_table[i], lcdc_gpio_name_table[i]);
			if (rc < 0) {
				pr_err("Error request gpio %s\n", lcdc_gpio_name_table[i]);
				break;
			}
			rc = gpio_tlmm_config(GPIO_CFG(lcdc_gpio_table[i], 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			if (rc < 0) {
				pr_err("Error config lcdc gpio:%d\n", lcdc_gpio_table[i]);
				break;
			}
			rc = gpio_direction_output(lcdc_gpio_table[i], 1);
			if (rc < 0) {
				pr_err("Error direct lcdc gpio:%d\n", lcdc_gpio_table[i]);
				break;
			}
		}

		if (rc < 0) {
			for (; i >= 0; i--) {
				gpio_free(lcdc_gpio_table[i]);
			}
			lcdc_gpio_initialized = 0;
		} else {
			lcdc_gpio_initialized = 1;
		}
	}
	return;
}
#endif

#ifndef CONFIG_MACH_MSM7X27A_SKU3
static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n, rc = 0;

	if (lcdc_gpio_initialized) {
		if (enable) {
			for (n = 0; n < (len - 1); n++)
				rc = gpio_direction_output(table[n], 1);
		}
	}
}

static void lcdc_toshiba_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}
#endif

#ifndef CONFIG_MACH_MSM7X27A_SKU3
static int msm_fb_lcdc_power_save(int on)
{
	int i, rc = 0;
	/* Doing the init of the LCDC GPIOs very late as they are from
		an I2C-controlled IO Expander */
	lcdc_toshiba_gpio_init();

	if (lcdc_gpio_initialized) {
		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);

		for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_vreg); i++) {
			if (on) {
				rc = vreg_enable(lcdc_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_enable: %s vreg"
						"operation failed\n",
						msm_fb_lcdc_vreg[i]);
						goto lcdc_vreg_fail;
				}
			} else {
				rc = vreg_disable(lcdc_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_disable: %s vreg "
						"operation failed\n",
						msm_fb_lcdc_vreg[i]);
					goto lcdc_vreg_fail;
				}
			}
		}
	}

	return rc;

lcdc_vreg_fail:
	if (on) {
		for (; i > 0; i--)
			vreg_disable(lcdc_vreg[i - 1]);
	} else {
		for (; i > 0; i--)
			vreg_enable(lcdc_vreg[i - 1]);
	}

return rc;

}
#else
static int msm_fb_lcdc_power_save(int on)
{
	int rc = 0;

  pr_info("%s\n", __func__);

  if (on) {
    lcd_camera_power_onoff(1);

    lcdc_truly_gpio_init();

    if (lcdc_gpio_initialized) {
      //LCD reset
      gpio_set_value(GPIO_DISPLAY_RESET, 1);
      msleep(10);
      gpio_set_value(GPIO_DISPLAY_RESET, 0);
      msleep(10);
      gpio_set_value(GPIO_DISPLAY_RESET, 1);
      msleep(10);
    }
  } else {
     /* pull down LCD IO to avoid current leakage */
     gpio_set_value(GPIO_SPI_MOSI, 0);
     gpio_set_value(GPIO_SPI_CLK, 0);
     gpio_set_value(GPIO_SPI_CS0_N, 0);
     gpio_set_value(GPIO_DISPLAY_RESET, 0);

     lcd_camera_power_onoff(0);
  }

	return rc;
}
#endif

#ifndef CONFIG_MACH_MSM7X27A_SKU3
static int lcdc_toshiba_set_bl(int level)
{
	int ret;

	ret = pmapp_disp_backlight_set_brightness(level);
	if (ret)
		pr_err("%s: can't set lcd backlight!\n", __func__);

	return ret;
}
#endif

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = NULL,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};
#ifndef CONFIG_MACH_MSM7X27A_SKU3
static int lcd_panel_spi_gpio_num[] = {
		GPIO_SPI_MOSI,  /* spi_sdi */
		GPIO_SPI_MISO,  /* spi_sdoi */
		GPIO_SPI_CLK,   /* spi_clk */
		GPIO_SPI_CS0_N, /* spi_cs  */
};

static struct msm_panel_common_pdata lcdc_toshiba_panel_data = {
	.panel_config_gpio = lcdc_toshiba_config_gpios,
	.pmic_backlight = lcdc_toshiba_set_bl,
	.gpio_num	  = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_toshiba_panel_device = {
	.name   = "lcdc_toshiba_fwvga_pt",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_toshiba_panel_data,
	}
};
#else

static struct msm_panel_common_pdata lcdc_truly_panel_data = {
	.panel_config_gpio = NULL,
	.gpio_num	  = lcdc_gpio_table,
};

static struct platform_device lcdc_truly_panel_device = {
	.name   = "lcdc_truly_hvga_ips3p2335_pt",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_truly_panel_data,
	}
};

#endif

#endif // !CONFIG_PROJECT_ES03



static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)	// modified by X
{
	int ret = -EPERM;
	
#ifdef CONFIG_LCDC_HX8357B
	if (!strcmp(name, "lcdc_wvga_HX8357B")) {
		printk("%s name:%s\n", __FUNCTION__, name);
		return 0;
	}
#endif

#ifdef CONFIG_FB_MSM_MIPI_HX8363A_VIDEO_WVGA_PT
	if (!strcmp(name, "mipi_video_hx8363a_wvga"))
		return 0;
#endif

	if (machine_is_msm7x27a_surf()){
		if (!strncmp(name, "lcdc_toshiba_fwvga_pt", 21))
			ret = 0;
	}else if (machine_is_msm7x27a_sku1()) {
		if (!strncmp(name, "mipi_video_truly_wvga", 21))
			ret = 0;
	}else if (machine_is_msm7x27a_sku3()) {
		if (!strncmp(name, "lcdc_truly_hvga_ips3p2335_pt", 28))
			ret = 0;
	}else {
		ret = -ENODEV;
	}

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#ifdef CONFIG_FB_MSM_MIPI_DSI
static int mipi_set_bl(int level)
{
#if defined(CONFIG_FB_MSM_MIPI_DSI_HX8363A)
	gpio_set_value(GPIO_BACKLIGHT_EN, !!level);
	return 0;
#else
	int ret;

	ret = pmapp_disp_backlight_set_brightness(level);
	if (ret)
		pr_err("%s: can't set lcd backlight!\n", __func__);

	return ret;
#endif
}

static struct msm_panel_common_pdata mipi_pdata = {
	.pmic_backlight = mipi_set_bl,
	.gpio = GPIO_BACKLIGHT_EN,
};

static struct platform_device mipi_dsi_panel_device = {
#ifdef CONFIG_FB_MSM_MIPI_DSI_RENESAS
	.name = "mipi_renesas",
#elif defined(CONFIG_FB_MSM_MIPI_DSI_HX8363A)
	.name	= "mipi_truly_hx8363a",
#elif defined(CONFIG_FB_MSM_MIPI_DSI_TRULY)
	.name = "mipi_truly",
#else
	.name = "mipi_unknown",
#endif
	.id = 0,
	.dev    = {
		.platform_data = &mipi_pdata,
	}
};
#endif

static void __init msm7x27a_init_mmc(void)
{
	vreg_emmc = vreg_get(NULL, "emmc");
	if (IS_ERR(vreg_emmc)) {
		pr_err("%s: vreg get failed (%ld)\n",
				__func__, PTR_ERR(vreg_emmc));
		return;
	}

	vreg_mmc = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_mmc)) {
		pr_err("%s: vreg get failed (%ld)\n",
				__func__, PTR_ERR(vreg_mmc));
		return;
	}
#if defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03)
        if(machine_is_msm7x27a_sku1() || machine_is_msm7x27a_sku3()){
#else
        if(machine_is_msm7x27a_sku1()){
#endif
        /* eMMC slot */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
                sdcc_vreg_data[2].vreg_data = vreg_emmc;
                sdcc_vreg_data[2].level = 2850;
                msm_add_sdcc(3, &sdc3_plat_data);
#endif
        /* Micro-SD slot */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
                sdcc_vreg_data[0].vreg_data = vreg_mmc;
                sdcc_vreg_data[0].level = 2850;
                msm_add_sdcc(1, &sdc1_plat_data);
#endif
        /* SDIO WLAN slot */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
                sdcc_vreg_data[1].vreg_data = vreg_mmc;
                sdcc_vreg_data[1].level = 2850;
                msm_add_sdcc(2, &sdc2_plat_data);
#endif
        /* Not Used */
#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
        && !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
                sdcc_vreg_data[3].vreg_data = vreg_mmc;
                sdcc_vreg_data[3].level = 2850;
                msm_add_sdcc(4, &sdc4_plat_data);
#endif
        }else{
	/* Micro-SD slot */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
		sdcc_vreg_data[0].vreg_data = vreg_mmc;
		sdcc_vreg_data[0].level = 2850;
		msm_add_sdcc(1, &sdc1_plat_data);
#endif
	/* SDIO WLAN slot */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
		sdcc_vreg_data[1].vreg_data = vreg_mmc;
		sdcc_vreg_data[1].level = 2850;
		msm_add_sdcc(2, &sdc2_plat_data);
#endif
	/* eMMC slot */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
		sdcc_vreg_data[2].vreg_data = vreg_emmc;
		sdcc_vreg_data[2].level = 2850;
		msm_add_sdcc(3, &sdc3_plat_data);
#endif
	/* Not Used */
#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
	&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
		sdcc_vreg_data[3].vreg_data = vreg_mmc;
		sdcc_vreg_data[3].level = 2850;
		msm_add_sdcc(4, &sdc4_plat_data);
#endif
	}
}
#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(FM_DIGITAL_STEREO_HEADSET, 26),
	SND(FM_DIGITAL_SPEAKER_PHONE, 27),
	SND(FM_DIGITAL_BT_A2DP_HEADSET, 28),
	SND(STEREO_HEADSET_AND_SPEAKER, 31),
	SND(HEADSET_AND_SPEAKER, 32),
	SND(CURRENT, 0x7FFFFFFE),
	SND(FM_ANALOG_STEREO_HEADSET, 35),
	SND(FM_ANALOG_STEREO_HEADSET_CODEC, 36),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
#ifndef CONFIG_PROJECT_QRD
	.voltage_min_design     = 3400,
	.voltage_max_design     = 4200,
#else
	.voltage_min_design     = 3200,
	.voltage_max_design     = 4300,
#endif
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	const static u32 voltage_to_capacity_table[][2] = {
		{4150, 100},
		{4066,  95},
		{4024,  90},
		{3984,  85},
		{3948,  80},
		{3915,  75},
		{3880,  70},
		{3845,  65},
		{3816,  60},
		{3794,  55},
		{3776,  50},
		{3761,  45},
		{3749,  40},
		{3738,  35},
		{3730,  30},
		{3717,  25},
		{3698,  20},
		{3664,  15},
		{3649,  10},
		{3622,   5},
		{3300,   0},
		{   0,   0},
	};
	int i, base, top, percent_diff;

	if (current_voltage >= voltage_to_capacity_table[0][0])
	{
		return 100;
	}

	for (i = 1; i < ARRAY_SIZE(voltage_to_capacity_table); i++) {
		if (current_voltage >= voltage_to_capacity_table[i][0]) {
			break;
		}
	}

	base = voltage_to_capacity_table[i][0];
	top = voltage_to_capacity_table[i - 1][0];
	percent_diff =  voltage_to_capacity_table[i - 1][1] -
		voltage_to_capacity_table[i][1];
	return voltage_to_capacity_table[i][1] +
		(current_voltage - base) * percent_diff / (top - base);
}

#ifdef  CONFIG_BATTERY_MSM
static struct platform_device msm_batt_device = {
       .name               = "msm-battery",
       .id                 = -1,
       .dev.platform_data  = &msm_psy_batt_data,
};
#endif

#ifdef CONFIG_BATTERY_MSM7X27A
static struct msm_charger_platform_data msm_charger_data = {
	.safety_time = 180,
	.update_time = 1,
	.max_voltage = 4200,
	.min_voltage = 3200,
};

static struct platform_device msm_charger_device = {
	.name = "msm-charger",
	.id = -1,
	.dev = {
		.platform_data = &msm_charger_data,
	}
};
#endif

#ifdef CONFIG_SMSC911X
static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= 0x90000000,
		.end	= 0x90007fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(48),
		.end	= MSM_GPIO_TO_INT(48),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "smsc911x_irq"  },
	{ GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "eth_fifo_sel" },
};

#define ETH_FIFO_SEL_GPIO	49
static void msm7x27a_cfg_smsc911x(void)
{
	int res;

	res = msm_gpios_request_enable(smsc911x_gpios,
				 ARRAY_SIZE(smsc911x_gpios));
	if (res) {
		pr_err("%s: unable to enable gpios for SMSC911x\n", __func__);
		return;
	}

	/* ETH_FIFO_SEL */
	res = gpio_direction_output(ETH_FIFO_SEL_GPIO, 0);
	if (res) {
		pr_err("%s: unable to get direction for gpio %d\n", __func__,
							 ETH_FIFO_SEL_GPIO);
		msm_gpios_disable_free(smsc911x_gpios,
						 ARRAY_SIZE(smsc911x_gpios));
		return;
	}
	gpio_set_value(ETH_FIFO_SEL_GPIO, 0);
}
#endif

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

#ifndef CONFIG_MACH_MSM7X27A_SKU1 //SKU1&SKU3 don't use this
#ifdef CONFIG_MSM_CAMERA_FLASH
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.led1 = GPIO_CAM_GP_LED_EN1,
	._fsrc.current_driver_src.led2 = GPIO_CAM_GP_LED_EN2,
};
#endif
#endif

#ifndef CONFIG_PROJECT_ES03	// removed by X
static struct vreg *vreg_gp1;
static struct vreg *vreg_gp2;
static struct vreg *vreg_gp3;
#endif

static void msm_camera_vreg_config(int vreg_en)
{

#ifndef CONFIG_PROJECT_ES03	// removed by X

	int rc;

	if (vreg_gp1 == NULL) {
		vreg_gp1 = vreg_get(NULL, "msme1");
		if (IS_ERR(vreg_gp1)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "msme1", PTR_ERR(vreg_gp1));
			return;
		}

		rc = vreg_set_level(vreg_gp1, 1800);
		if (rc) {
			pr_err("%s: GP1 set_level failed (%d)\n",
				__func__, rc);
			return;
		}
	}

	if (vreg_gp2 == NULL) {
		vreg_gp2 = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_gp2)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp2", PTR_ERR(vreg_gp2));
			return;
		}

		rc = vreg_set_level(vreg_gp2, 2850);
		if (rc) {
			pr_err("%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_gp3 == NULL) {
		vreg_gp3 = vreg_get(NULL, "usb2");
		if (IS_ERR(vreg_gp3)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp3", PTR_ERR(vreg_gp3));
			return;
		}

		rc = vreg_set_level(vreg_gp3, 1800);
		if (rc) {
			pr_err("%s: GP3 set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_gp1);
		if (rc) {
			pr_err("%s: GP1 enable failed (%d)\n",
				__func__, rc);
			return;
		}

		rc = vreg_enable(vreg_gp2);
		if (rc) {
			pr_err("%s: GP2 enable failed (%d)\n",
				__func__, rc);
		}

		rc = vreg_enable(vreg_gp3);
		if (rc) {
			pr_err("%s: GP3 enable failed (%d)\n",
				__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_gp1);
		if (rc)
			pr_err("%s: GP1 disable failed (%d)\n",
				__func__, rc);

		rc = vreg_disable(vreg_gp2);
		if (rc) {
			pr_err("%s: GP2 disable failed (%d)\n",
				__func__, rc);
		}

		rc = vreg_disable(vreg_gp3);
		if (rc) {
			pr_err("%s: GP3 disable failed (%d)\n",
				__func__, rc);
		}
	}
	
#endif 	
}

static int config_gpio_table(uint32_t *table, int len)
{
	int rc = 0, i = 0;

	for (i = 0; i < len; i++) {
		rc = gpio_tlmm_config(table[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s not able to get gpio\n", __func__);
			for (i--; i >= 0; i--)
				gpio_tlmm_config(camera_off_gpio_table[i],
							GPIO_CFG_ENABLE);
			break;
		}
	}
	return rc;
}


static int config_camera_on_gpios_rear(void)
{
	int rc = 0;

	if (machine_is_msm7x27a_ffa())
		msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));

	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
	}

    return rc;
}

static void config_camera_off_gpios_rear(void)
{
	if (machine_is_msm7x27a_ffa())
		msm_camera_vreg_config(0);
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_on_gpios_front(void)
{
	int rc = 0;

	if (machine_is_msm7x27a_ffa())
		msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));

	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_front(void)
{
	if (machine_is_msm7x27a_ffa())
		msm_camera_vreg_config(0);

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

struct msm_camera_device_platform_data msm_camera_device_data_rear = {
	.camera_gpio_on  = config_camera_on_gpios_rear,
	.camera_gpio_off = config_camera_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

struct msm_camera_device_platform_data msm_camera_device_data_front = {
	.camera_gpio_on  = config_camera_on_gpios_front,
	.camera_gpio_off = config_camera_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_OV5647
#define GPIO_SKU3_CAM_5MP_SHDN_N 109  //PWDN
#define GPIO_SKU3_CAM_5MP_CAMIF_RESET 29 //(board_is(EVT))?123:121 //RESET
#define GPIO_SKU3_CAM_5MP_CAM_DRIVER_PWDN 34
#define GPIO_FLASH_LED_EN   96
#define GPIO_FLASH_LED_MODE 107

static struct msm_camera_sensor_platform_info ov5647_sensor_7627a_info = {
        .mount_angle = 90
};

static struct msm_camera_sensor_flash_src msm_flash_src_ov5647 = {
        .flash_sr_type = MSM_CAMERA_FLASH_SRC_LED,
        ._fsrc.led_src.led_name = "flashlight",
        ._fsrc.led_src.led_name_len = 10,
};

static struct msm_camera_sensor_flash_data flash_ov5647 = {
        .flash_type             = MSM_CAMERA_FLASH_LED,
        .flash_src              = &msm_flash_src_ov5647,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov5647_data = {
        .sensor_name    = "ov5647",
        .sensor_reset_enable = 1,
        .sensor_reset   = GPIO_SKU3_CAM_5MP_CAMIF_RESET,
        .sensor_pwd     = GPIO_SKU3_CAM_5MP_SHDN_N,
        .vcm_pwd        = GPIO_SKU3_CAM_5MP_CAM_DRIVER_PWDN,
        .vcm_enable     = 0,
        .pdata          = &msm_camera_device_data_rear,
        .flash_data     = &flash_ov5647,
        .sensor_platform_info   = &ov5647_sensor_7627a_info,
        .csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov5647 = {
        .name      = "msm_camera_ov5647",
        .dev       = {
                .platform_data = &msm_camera_sensor_ov5647_data,
        },
};
#endif
#ifdef CONFIG_OV5640

#if defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03)
#define GPIO_SKU1_CAM_5MP_SHDN 109  //PWR
#define GPIO_SKU1_CAM_5MP_CAMIF_RESET_N 29 //(machine_is_msm7x27a_sku1()&&hw_version_is(2,0))?121:123 //RESET
#define GPIO_SKU1_CAM_5MP_CAM_DRIVER_PWDN 0
#else
#ifdef CONFIG_MACH_MSM7X27A_SKU3
#define GPIO_SKU1_CAM_5MP_SHDN 93  //PWDN
#define GPIO_SKU1_CAM_5MP_CAMIF_RESET_N 23 //(board_is(EVT))?123:121 //RESET
#define GPIO_SKU1_CAM_5MP_CAM_DRIVER_PWDN 30
#else
#define GPIO_SKU1_CAM_5MP_SHDN 120  //PWR
#define GPIO_SKU1_CAM_5MP_CAMIF_RESET_N 121 //(machine_is_msm7x27a_sku1()&&hw_version_is(2,0))?121:123 //RESET
#endif
#endif

static struct msm_camera_sensor_platform_info ov5640_sensor_7627a_info = {
        .mount_angle = 90
};

static struct msm_camera_sensor_flash_src msm_flash_src_ov5640 = {
		.flash_sr_type = MSM_CAMERA_FLASH_SRC_LED,
		._fsrc.led_src.led_name = "flashlight",
		._fsrc.led_src.led_name_len = 10,
};

static struct msm_camera_sensor_flash_data flash_ov5640 = {
        .flash_type             = MSM_CAMERA_FLASH_LED,
		.flash_src              = &msm_flash_src_ov5640,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov5640_data = {
        .sensor_name    		= "ov5640",
        .sensor_reset_enable 	= 1,
        .sensor_reset 			= GPIO_SKU1_CAM_5MP_CAMIF_RESET_N ,
        .sensor_pwd             = GPIO_SKU1_CAM_5MP_SHDN,
#ifdef CONFIG_MACH_MSM7X27A_SKU3
        .vcm_pwd                = GPIO_SKU1_CAM_5MP_CAM_DRIVER_PWDN,
        .vcm_enable             = 1,
#else
		.vcm_pwd                = 0,
        .vcm_enable             = 0,
#endif
        .pdata                  = &msm_camera_device_data_rear,
        .flash_data             = &flash_ov5640,
        .sensor_platform_info   = &ov5640_sensor_7627a_info,
        .csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov5640 = {
        .name   = "msm_camera_ov5640",
        .dev    = {
                .platform_data = &msm_camera_sensor_ov5640_data,
        },
};
#endif

#ifdef CONFIG_WEBCAM_OV7692


#if defined(CONFIG_PROJECT_ES02) 
#define GPIO_SKU1_CAM_VGA_SHDN 108

#elif defined(CONFIG_PROJECT_ES03)
#define GPIO_SKU1_CAM_VGA_SHDN 113

#else

#ifdef CONFIG_MACH_MSM7X27A_SKU3
#define GPIO_SKU1_CAM_VGA_SHDN 91
#define GPIO_SKU1_CAM_VGA_RESET_N 29
#else
#define GPIO_SKU1_CAM_VGA_SHDN 118
#endif

#endif

static struct msm_camera_sensor_platform_info ov7692_sensor_7627a_info = {
        .mount_angle = 270
};


static struct msm_camera_sensor_flash_data flash_ov7692 = {
        .flash_type             = MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7692_data = {
        .sensor_name    		= "ov7692",
#if defined(CONFIG_MACH_MSM7X27A_SKU3)
		.sensor_reset_enable	= 1,
		.sensor_reset			= GPIO_SKU1_CAM_VGA_RESET_N,
#else
		.sensor_reset_enable	= 0,
		.sensor_reset			= 0,
#endif
        .sensor_pwd             = GPIO_SKU1_CAM_VGA_SHDN,
        .vcm_pwd                = 0,
        .vcm_enable             = 0,
        .pdata                  = &msm_camera_device_data_front,
        .flash_data             = &flash_ov7692,
        .sensor_platform_info   = &ov7692_sensor_7627a_info,
        .csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov7692 = {
        .name   = "msm_camera_ov7692",
        .dev    = {
                .platform_data = &msm_camera_sensor_ov7692_data,
        },
};
#endif

#ifdef CONFIG_S5K5CA
#define GPIO_CAM_3MP_SHDN_N 124 //ES02:109, ES03: 124  //PWDN
#define GPIO_CAM_3MP_CAMIF_RESET 84	//ES02: 29, ES03: 84   //RESET

#define GPIO_CAM_3MP_CAM_DRIVER_PWDN	INVALID_GPIO
#define GPIO_FLASH_LED_EN   INVALID_GPIO	// gpio 59 is unused in ES03	
#define GPIO_FLASH_LED_MODE INVALID_GPIO	 

static struct msm_camera_sensor_platform_info s5k5ca_sensor_7627a_info = {
        .mount_angle = 90
};

#if 0 // #9 removed
static struct msm_camera_sensor_flash_src msm_flash_src_s5k5ca = {
        .flash_sr_type = MSM_CAMERA_FLASH_SRC_LED_EXT,
        ._fsrc.led_src.led_name = "flashlight",
        ._fsrc.led_src.led_name_len = 10,
        ._fsrc.led_src.led_mode = GPIO_FLASH_LED_MODE,
        ._fsrc.led_src.led_en = GPIO_FLASH_LED_EN,
};
#endif

static struct msm_camera_sensor_flash_data flash_s5k5ca = {
        .flash_type             = MSM_CAMERA_FLASH_NONE,//MSM_CAMERA_FLASH_LED,
        .flash_src              = NULL,//&msm_flash_src_s5k5ca,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k5ca_data = {
        .sensor_name    = "s5k5ca",
        .sensor_reset_enable = 1,
        .sensor_reset   = GPIO_CAM_3MP_CAMIF_RESET,
        .sensor_pwd     = GPIO_CAM_3MP_SHDN_N,
        //.vcm_pwd        = GPIO_CAM_3MP_CAM_DRIVER_PWDN,
        .vcm_enable     = 0,
        .pdata          = &msm_camera_device_data_rear,
        .flash_data     = &flash_s5k5ca,
        .sensor_platform_info   = &s5k5ca_sensor_7627a_info,
        .csi_if                 = 1
};

static struct platform_device msm_camera_sensor_s5k5ca = {
        .name      = "msm_camera_s5k5ca",
        .dev       = {
                .platform_data = &msm_camera_sensor_s5k5ca_data,
        },
};
#endif //CONFIG_S5K5CA

#ifdef CONFIG_S5K4E1
static struct msm_camera_sensor_platform_info s5k4e1_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_s5k4e1 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data = {
	.sensor_name    = "s5k4e1",
	.sensor_reset_enable = 1,
	.sensor_reset   = GPIO_CAM_GP_CAMIF_RESET_N,
	.sensor_pwd             = 0,
	.vcm_pwd                = GPIO_CAM_GP_CAM_PWDN,
	.vcm_enable             = 1,
	.pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_s5k4e1,
	.sensor_platform_info   = &s5k4e1_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_s5k4e1 = {
	.name   = "msm_camera_s5k4e1",
	.dev    = {
		.platform_data = &msm_camera_sensor_s5k4e1_data,
	},
};
#endif

#ifdef CONFIG_IMX072
static struct msm_camera_sensor_platform_info imx072_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_imx072 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_imx072_data = {
	.sensor_name    = "imx072",
	.sensor_reset_enable = 1,
	.sensor_reset   = GPIO_CAM_GP_CAMIF_RESET_N, /* TODO 106,*/
	.sensor_pwd             = 0,
	.vcm_pwd                = GPIO_CAM_GP_CAM_PWDN,
	.vcm_enable             = 1,
	.pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_imx072,
	.sensor_platform_info = &imx072_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_imx072 = {
	.name   = "msm_camera_imx072",
	.dev    = {
		.platform_data = &msm_camera_sensor_imx072_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_OV9726
static struct msm_camera_sensor_platform_info ov9726_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov9726 = {
	.flash_type             = MSM_CAMERA_FLASH_NONE,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data = {
	.sensor_name    = "ov9726",
	.sensor_reset_enable = 0,
	.sensor_reset   = GPIO_CAM_GP_CAM1MP_XCLR,
	.sensor_pwd             = 0,
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_device_data_front,
	.flash_data             = &flash_ov9726,
	.sensor_platform_info   = &ov9726_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov9726 = {
	.name   = "msm_camera_ov9726",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov9726_data,
	},
};
#endif

#ifdef CONFIG_MT9E013
static struct msm_camera_sensor_platform_info mt9e013_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 0,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 0,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_mt9e013,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif

static struct i2c_board_info i2c_camera_devices[] = {
	#ifdef CONFIG_S5K4E1
	{
		I2C_BOARD_INFO("s5k4e1", 0x36),
	},
	{
		I2C_BOARD_INFO("s5k4e1_af", 0x8c >> 1),
	},
	#endif
	#ifdef CONFIG_WEBCAM_OV9726
	{
		I2C_BOARD_INFO("ov9726", 0x10),
	},
	#endif
	#ifdef CONFIG_IMX072
	{
		I2C_BOARD_INFO("imx072", 0x34),
	},
	#endif
	#ifdef CONFIG_MT9E013
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
	#endif
    #ifdef CONFIG_OV5640
    {
            I2C_BOARD_INFO("ov5640", 0x78 >> 1),
    },
    #endif
    #ifdef CONFIG_OV5647
	{
		I2C_BOARD_INFO("ov5647", 0x36 << 1),
	},
	{
		I2C_BOARD_INFO("ov5647_af", 0x18 >> 1),
	},
	#endif
	#ifdef CONFIG_S5K5CA
	{
		I2C_BOARD_INFO("s5k5ca", 0x5a>>1),
	},
	#endif
	#ifdef CONFIG_WEBCAM_OV7692
    {
            I2C_BOARD_INFO("ov7692", 0x78),
    },
	#endif
 #ifdef CONFIG_SC628A
    {
		I2C_BOARD_INFO("sc628a", 0x37),
	},
 #endif
};
#endif


static void msm7x27a_camera_gpio_cfg(void)
{
    int rc = 0;

#ifdef CONFIG_OV5640
    rc = gpio_request(GPIO_SKU1_CAM_5MP_SHDN, "ov5640");
    if (rc < 0) {
        pr_err("%s: gpio_request---GPIO_SKU1_CAM_5MP_SHDN failed!", __func__);
    }

    rc = gpio_tlmm_config(GPIO_CFG(GPIO_SKU1_CAM_5MP_SHDN, 0,
            GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
            GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc < 0) {
        pr_err("%s: unable to enable Power Down gpio for main camera!\n", __func__);
        gpio_free(GPIO_SKU1_CAM_5MP_SHDN);
    }

    gpio_direction_output(GPIO_SKU1_CAM_5MP_SHDN, 1);

    rc = gpio_request(GPIO_SKU1_CAM_5MP_CAMIF_RESET_N, "ov5640");
    if (rc < 0) {
        pr_err("%s: gpio_request---GPIO_SKU1_CAM_5MP_CAMIF_RESET_N failed!", __func__);
    }

    rc = gpio_tlmm_config(GPIO_CFG(GPIO_SKU1_CAM_5MP_CAMIF_RESET_N, 0,
            GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
            GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc < 0) {
        pr_err("%s: unable to enable reset gpio for main camera!\n", __func__);
        gpio_free(GPIO_SKU1_CAM_5MP_CAMIF_RESET_N);
    }

    gpio_direction_output(GPIO_SKU1_CAM_5MP_CAMIF_RESET_N, 1);

#ifdef CONFIG_MACH_MSM7X27A_SKU3
    rc = gpio_request(GPIO_SKU1_CAM_5MP_CAM_DRIVER_PWDN, "ov5640");
    if (rc < 0) {
        pr_err("%s: gpio_request---GPIO_SKU1_CAM_5MP_CAM_DRIVER_PWDN failed!", __func__);
    }

    rc = gpio_tlmm_config(GPIO_CFG(GPIO_SKU1_CAM_5MP_CAM_DRIVER_PWDN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc < 0) {
        pr_err("%s: unable to set camera driver power down gpio for front camera!\n", __func__);
        gpio_free(GPIO_SKU1_CAM_5MP_CAM_DRIVER_PWDN);
    }
    gpio_direction_output(GPIO_SKU1_CAM_5MP_CAM_DRIVER_PWDN, 1);
#endif
#endif //CONFIG_OV5640

#ifdef CONFIG_OV5647
	rc = gpio_request(GPIO_SKU3_CAM_5MP_SHDN_N, "ov5647");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_SKU3_CAM_5MP_SHDN_N failed!", __func__);
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_SKU3_CAM_5MP_SHDN_N, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable Power Down gpio for main camera!\n", __func__);
		gpio_free(GPIO_SKU3_CAM_5MP_SHDN_N);
	}

	gpio_direction_output(GPIO_SKU3_CAM_5MP_SHDN_N, 1);

	rc = gpio_request(GPIO_SKU3_CAM_5MP_CAMIF_RESET, "ov5647");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_SKU3_CAM_5MP_CAMIF_RESET failed!", __func__);
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_SKU3_CAM_5MP_CAMIF_RESET, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable reset gpio for main camera!\n", __func__);
		gpio_free(GPIO_SKU3_CAM_5MP_CAMIF_RESET);
	}

	gpio_direction_output(GPIO_SKU3_CAM_5MP_CAMIF_RESET, 1);

	rc = gpio_request(GPIO_SKU3_CAM_5MP_CAM_DRIVER_PWDN, "ov5647");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_SKU3_CAM_5MP_CAM_DRIVER_PWDN failed!", __func__);
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_SKU3_CAM_5MP_CAM_DRIVER_PWDN, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to set camera driver power down gpio for front camera!\n", __func__);
		gpio_free(GPIO_SKU3_CAM_5MP_CAM_DRIVER_PWDN);
	}
	gpio_direction_output(GPIO_SKU3_CAM_5MP_CAM_DRIVER_PWDN, 1);
	/*camera flash led gpio init begin*/
	rc = gpio_request(GPIO_FLASH_LED_EN, "flash_led_enable");
        if (rc < 0)
            pr_err("%s: gpio_request---GPIO_FLASH_LED_ENABLE failed!", __func__);
    
        rc = gpio_tlmm_config(GPIO_CFG(GPIO_FLASH_LED_EN, 0,
                    GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                    GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        if (rc < 0) {
            pr_err("%s: unable to config gpio for flash led enable!\n", __func__);
            gpio_free(GPIO_FLASH_LED_EN);
        }

	gpio_direction_output(GPIO_FLASH_LED_EN, 0);

	    rc = gpio_request(GPIO_FLASH_LED_MODE, "flash_led_mode");
       if (rc < 0)
               pr_err("%s: gpio_request---GPIO_FLASH_LED_MODE failed!", __func__);
	    rc = gpio_tlmm_config(GPIO_CFG(GPIO_FLASH_LED_MODE, 0,
                               GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                              GPIO_CFG_2MA), GPIO_CFG_ENABLE);
       if (rc < 0) {
               pr_err("%s: unable to enable config gpio for flash led mode!\n", __func__);
               gpio_free(GPIO_FLASH_LED_MODE);
       }

       gpio_direction_output(GPIO_FLASH_LED_MODE, 0);
	/*camera flash led gpio init end*/

#endif //CONFIG_OV5647

#ifdef CONFIG_WEBCAM_OV7692
	rc = gpio_request(GPIO_SKU1_CAM_VGA_SHDN, "ov7692");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_SKU1_CAM_VGA_SHDN failed!", __func__);
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_SKU1_CAM_VGA_SHDN, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable Power Down gpio for front camera!\n", __func__);
		gpio_free(GPIO_SKU1_CAM_VGA_SHDN);
	}

	gpio_direction_output(GPIO_SKU1_CAM_VGA_SHDN, 1);

#ifdef CONFIG_MACH_MSM7X27A_SKU3
	rc = gpio_request(GPIO_SKU1_CAM_VGA_RESET_N, "ov7692");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_SKU1_CAM_VGA_RESET_N failed!", __func__);
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_SKU1_CAM_VGA_RESET_N, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (rc < 0) {
		pr_err("%s: unable to enable reset gpio for front camera!\n", __func__);
		gpio_free(GPIO_SKU1_CAM_VGA_RESET_N);
	}
	gpio_direction_output(GPIO_SKU1_CAM_VGA_RESET_N, 1);
#endif
#endif //CONFIG_WEBCAM_OV7692

#ifdef CONFIG_S5K5CA
	rc = gpio_request(GPIO_CAM_3MP_SHDN_N, "s5k5ca");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_CAM_3MP_SHDN_N failed!", __func__);
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_CAM_3MP_SHDN_N, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable Power Down gpio for main camera!\n", __func__);
		gpio_free(GPIO_CAM_3MP_SHDN_N);
	}

	gpio_direction_output(GPIO_CAM_3MP_SHDN_N, 1);

	rc = gpio_request(GPIO_CAM_3MP_CAMIF_RESET, "s5k5ca");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_CAM_3MP_CAMIF_RESET failed!", __func__);
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_CAM_3MP_CAMIF_RESET, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable reset gpio for main camera!\n", __func__);
		gpio_free(GPIO_CAM_3MP_CAMIF_RESET);
	}

	gpio_direction_output(GPIO_CAM_3MP_CAMIF_RESET, 1);
#ifdef CONFIG_S5K5CA_AUTO_FOCUS
	rc = gpio_request(GPIO_CAM_3MP_CAM_DRIVER_PWDN, "s5k5ca");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_CAM_3MP_CAM_DRIVER_PWDN failed!", __func__);
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_CAM_3MP_CAM_DRIVER_PWDN, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to set camera driver power down gpio for front camera!\n", __func__);
		gpio_free(GPIO_CAM_3MP_CAM_DRIVER_PWDN);
	}
	gpio_direction_output(GPIO_CAM_3MP_CAM_DRIVER_PWDN, 1);
#endif
/*camera flash led gpio init begin*/

#if 0 // #9 removed
	rc = gpio_request(GPIO_FLASH_LED_EN, "flash_led_enable");
        if (rc < 0)
            pr_err("%s: gpio_request---GPIO_FLASH_LED_ENABLE failed!", __func__);
    
        rc = gpio_tlmm_config(GPIO_CFG(GPIO_FLASH_LED_EN, 0,
                    GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                    GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        if (rc < 0) {
            pr_err("%s: unable to config gpio for flash led enable!\n", __func__);
            gpio_free(GPIO_FLASH_LED_EN);
        }

	gpio_direction_output(GPIO_FLASH_LED_EN, 0);

	    rc = gpio_request(GPIO_FLASH_LED_MODE, "flash_led_mode");
       if (rc < 0)
               pr_err("%s: gpio_request---GPIO_FLASH_LED_MODE failed!", __func__);
	    rc = gpio_tlmm_config(GPIO_CFG(GPIO_FLASH_LED_MODE, 0,
                               GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                              GPIO_CFG_2MA), GPIO_CFG_ENABLE);
       if (rc < 0) {
               pr_err("%s: unable to enable config gpio for flash led mode!\n", __func__);
               gpio_free(GPIO_FLASH_LED_MODE);
       }

       gpio_direction_output(GPIO_FLASH_LED_MODE, 0);
#endif

/*camera flash led gpio init end*/
#endif //CONFIG_S5K5CA
}

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) \
		&& defined(CONFIG_MSM_SHARED_GPIO_FOR_UART2DM)
static struct msm_gpio uart2dm_gpios[] = {
	{GPIO_CFG(19, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rfr_n" },
	{GPIO_CFG(20, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_cts_n" },
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

static void msm7x27a_cfg_uart2dm_serial(void)
{
	int ret;
	ret = msm_gpios_request_enable(uart2dm_gpios,
					ARRAY_SIZE(uart2dm_gpios));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);
}
#else
static void msm7x27a_cfg_uart2dm_serial(void) { }
#endif

static struct platform_device *rumi_sim_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&smc91x_device,
	&msm_device_uart1,
	&msm_device_nand,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
};
#ifdef CONFIG_ATH_WIFI /* [WIFI] Wi-Fi Support ++ */
static struct platform_device msm_wlan_ar6000_pm_device = {
	.name	=	"wlan_ar6000_pm_dev",
	.id	=	1,
	.num_resources	=	0,
	.resource	=	NULL,
};

#define GPIO_WLAN_CHIP_PWD_L 124

static int wlan_gpio_power(unsigned int gpio, unsigned char on)
{
	int rc = 0;

	rc = gpio_tlmm_config(GPIO_CFG(gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if(rc){
		pr_err("%s gpio_tlmm_config %d failed,error = %d\n", __func__,gpio,rc);
		return rc;
	}
	gpio_set_value_vote(gpio, on, GPIO_VOTE_VDD33_WLAN, (on ? GPIO_VOTE_VDD33_WLAN_MASK : GPIO_VOTE_VDD33_ALL_MASK));

	return rc;
}

struct wlan_vreg_info {
       const char *name;
       unsigned int pmapp_id;
       unsigned int level;
       unsigned int is_pin_controlled;
       struct vreg *vregs;
       unsigned int clock_id;
};

static struct wlan_vreg_info wlan_vregs_v1[] = {
       {"wlan4", 0, 1800, 0, NULL, PMAPP_CLOCK_ID_A0},
};

static struct wlan_vreg_info wlan_vregs_v2[] = {
       {"wlan4", 0, 1800, 0, NULL, PMAPP_CLOCK_ID_A0},
       {"bt", 21, 3300, 1, NULL, PMAPP_CLOCK_ID_A0},
};

static int wlan_switch_regulator(int on, struct wlan_vreg_info *t_vreg, const char *id)
{
	int rc = 0;

	rc = on ? vreg_set_level(t_vreg->vregs,
			t_vreg->level) : 0;

	if (rc < 0) {
		pr_err("%s: vreg set level failed (%d)\n",
				__func__, rc);
		goto vreg_set_level_fail;
	}
	rc = on ? vreg_enable(t_vreg->vregs) : 0;

	if (rc < 0) {
		pr_err("%s: vreg %s %s failed(%d)\n",
				__func__, t_vreg->name,
				on ? "enable" : "disable", rc);
		goto vreg_fail;
	}
	if (t_vreg->is_pin_controlled == 1) {
		rc = pmapp_vreg_lpm_pincntrl_vote(id,
				t_vreg->pmapp_id,
				t_vreg->clock_id,
				on ? PMAPP_CLOCK_VOTE_ON :
				PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0) {
			pr_err("%s: vreg %s pin ctrl failed(%d)\n",
					__func__, t_vreg->name,
					rc);
			goto pincntrl_fail;
		}
	}

	rc = on ? 0 : vreg_disable(t_vreg->vregs);
	if (rc < 0) {
		pr_err("%s: vreg %s %s failed(%d)\n",
				__func__, t_vreg->name,
				on ? "enable" : "disable", rc);
		goto vreg_fail;
	}


	return rc;

pincntrl_fail:
	if (on)
		vreg_disable(t_vreg->vregs); 	/* vreg will always be not null in this time*/
vreg_fail:
vreg_set_level_fail:
	return rc;
}

static int wlan_switch_regulators(int on, struct wlan_vreg_info *t_vregs, unsigned int size, const char *id)
{
	int i, rc = 0;
	int index;

	for (i = 0; i < size; i++) {
		//only when on == 1 will vregs be null
		if (!t_vregs[i].vregs) {
			t_vregs[i].vregs = vreg_get(NULL,
					t_vregs[i].name);
			if (IS_ERR(t_vregs[i].vregs)) {
				pr_err("%s: vreg get %s failed (%ld)\n",
					   __func__, t_vregs[i].name,
					   PTR_ERR(t_vregs[i].vregs));
				rc = PTR_ERR(t_vregs[i].vregs);
				goto vreg_get_fail;
			}
		}
	}

	for (i = 0; i < size; i++) {
		/*index purpose: to ensure on and off was a revert sequence. */
		index = on ? i : (size - i - 1);
		rc = wlan_switch_regulator(on, &t_vregs[index], id);
		if (rc < 0) {
			pr_err("%s: vreg %s swith %d failed (%d)\n",
						   __func__, t_vregs[i].name, on, rc);
			goto switch_reg_fail;
		}

	}

	return rc;

switch_reg_fail:
	if(!on) {
		/* if disable vreg unseccessful, only let the up API to do it all again*/
		return rc;
	}
	while (i) {
		wlan_switch_regulator(0, &t_vregs[i--], id);
	}
	i = size - 1;

vreg_get_fail:
	while (i) {
		vreg_put(t_vregs[i--].vregs);
	}
	return rc;
}

static int wlan_power(unsigned char on, const char *id)
{
	static struct wlan_vreg_info *wlan_vreg_cur = NULL;
	int rc = 0;
	unsigned int size = 0;
	unsigned int gpio = GPIO_INVALID;

	if(machine_is_msm7x27a_sku1()&&!hw_version_ge(3,3)){
		wlan_vreg_cur = wlan_vregs_v1;
		size = ARRAY_SIZE(wlan_vregs_v1);
		gpio = GPIO_VDD33_EN;
	} else {
		wlan_vreg_cur = wlan_vregs_v2;
		size = ARRAY_SIZE(wlan_vregs_v2);
	}
	if(!on && gpio != GPIO_INVALID) {
		rc = wlan_gpio_power(gpio, on);
		if (rc < 0) {
			pr_err("%s:wlan_gpio_power Failed, rc = %d", __func__, rc);
			goto gpio_pw_first_fail;
		}
	}

	rc = wlan_switch_regulators(on, wlan_vreg_cur, size, id);
	if (rc < 0) {
		pr_err("%s:wlan_switch_regulators Failed, rc = %d", __func__, rc);
		goto sw_reg_fail;
	}

	if(gpio != GPIO_INVALID && on) {
		rc = wlan_gpio_power(gpio, on);
		if (rc < 0) {
			pr_err("%s:wlan_gpio_power Failed, rc = %d", __func__, rc);
			goto gpio_pw_fail;
		}
	}

	return rc;
gpio_pw_fail:
	wlan_switch_regulators(on ? 0 : 1, wlan_vreg_cur, size, id);
sw_reg_fail:
gpio_pw_first_fail:
	return rc;
}

int wlan_power_ctrl(unsigned char on)
{
	int rc = 0;
	const char *id = "WLPW";

	if(on){
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_CHIP_PWD_L, 
			0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		if(rc){
			pr_err("%s gpio_tlmm_config 124 failed,error = %d\n", __func__, rc);
			goto gpio_config_fail;
		}
		gpio_set_value(GPIO_WLAN_CHIP_PWD_L, 0);

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
			PMAPP_CLOCK_VOTE_ON);
		if (rc < 0) {
			pr_err("Failed to vote for TCXO_A0 ON(WLAN)\n");
			goto clock_vote_fail;
		}

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
			PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			pr_err("%s:Pin Control Failed, rc = %d", __func__, rc);

		rc = wlan_power(on, id);
		if (rc < 0) {
			pr_err("%s:wlan_power Failed, rc = %d", __func__, rc);
			goto wlan_power_on_fail;
		}

		gpio_set_value(GPIO_WLAN_CHIP_PWD_L, 1);
	}else{
		gpio_set_value(GPIO_WLAN_CHIP_PWD_L, 0);

		rc = wlan_power(on, id);
		if (rc < 0) {
			pr_err("%s:wlan_power Failed, rc = %d", __func__, rc);
			goto wlan_power_off_fail;
		}

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
			PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0) {
			pr_err("Failed to vote for TCXO_A0 OFF(WLAN)\n");
			goto clock_vote_fail;
		}
	}

	return rc;
wlan_power_on_fail:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0, PMAPP_CLOCK_VOTE_OFF);
clock_vote_fail:
wlan_power_off_fail:
gpio_config_fail:
	return rc;
}
EXPORT_SYMBOL(wlan_power_ctrl);
#endif /* CONFIG_ATH_WIFI [WIFI] Wi-Fi Support -- */


#ifdef CONFIG_PROJECT_ES03

#define GPIO_WLAN_REG_ON 87

#if 0
static void (*wifi_status_cb)(int card_present, void *dev_id);
static struct msmsdcc_host *wifi_status_cb_devid;

static int
brcm_wifi_status_register(void (*callback)(int card_present, void *dev_id),
                               void *dev_id)
{
       if (wifi_status_cb)
               return -EAGAIN;
       wifi_status_cb = callback;
       wifi_status_cb_devid = (struct msmsdcc_host*)dev_id;
       return 0;
}

static int brcm_wifi_cd;       /* WiFi virtual 'card detect' status */

static unsigned int brcm_wifi_status(struct device *dev)
{
       return brcm_wifi_cd;
}
#endif

void bcm_wlan_power_on(int state)
{
    	msm_sdcc_setup_gpio(2,1); 
	
	gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_REG_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_WLAN_REG_ON, 0);
	msleep(150);
	gpio_set_value(GPIO_WLAN_REG_ON, 1);
	printk(KERN_ERR "%s: power on \n", __func__);
}

EXPORT_SYMBOL(bcm_wlan_power_on);

void bcm_wlan_power_off(int state)
{
	gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_REG_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_WLAN_REG_ON, 0);

    	msm_sdcc_setup_gpio(2,0); 
	
	printk(KERN_INFO "%s: power off \n", __func__);
}

EXPORT_SYMBOL(bcm_wlan_power_off);

/*
int brcm_wifi_set_carddetect(int val)
{
       printk(KERN_INFO "%s: %d\n", __func__, val);
       brcm_wifi_cd = val;
       if (wifi_status_cb)
               wifi_status_cb(val, wifi_status_cb_devid);
       else
               printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
		//bcm_detect_card(val);
       return 0;
}
EXPORT_SYMBOL(brcm_wifi_set_carddetect);
*/
#endif 



#ifdef CONFIG_LEDS_TRICOLOR_FLAHSLIGHT

#if 0 /* es02 evb no tricolor, the flash led gpio config be include in ov5647 camera gpio config */
#ifdef CONFIG_MACH_MSM7X27A_SKU3
#define LED_BLUE 107
#define LED_RED 108
#define LED_GREEN 109
#define LED_FLASH 96
#else
#define LED_BLUE 12
#define LED_RED 13
#define LED_GREEN 14
#define LED_FLASH 96
#endif


static struct msm_gpio tricolor_leds_gpio_cfg_data[] = {
	{
		GPIO_CFG(LED_BLUE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"blue led"
	},{
		GPIO_CFG(LED_RED, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"red led"
	},{
		GPIO_CFG(LED_GREEN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"green led"
	},{
		GPIO_CFG(LED_FLASH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"flashlight"
	},
};

static int tricolor_leds_gpio_setup(void) {
	int ret = 0;
	ret = msm_gpios_request_enable(tricolor_leds_gpio_cfg_data,
				 sizeof(tricolor_leds_gpio_cfg_data)/sizeof(struct msm_gpio));
	if( ret<0 )
		printk(KERN_ERR "%s: Failed to obtain tricolor_leds GPIO . Code: %d\n",
				__func__, ret);
	return ret;
}

#endif /* es02 evb no tricolor */

static struct platform_device msm_device_tricolor_leds = {
	.name	= "tricolor leds and flashlight",
	.id	= -1,
};
#endif

//add by Joe 120419
static struct platform_device msm_device_pmic_leds = {
	.name   = "pmic-leds",
	.id = -1,
};

static struct platform_device msm_dbg_info_device = {
	.name 		    = "msm-dbg-info",
	.id		    = -1,
};


static struct platform_device *surf_ffa_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1, // ES03: debug port
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
#if defined(CONFIG_PROJECT_ES02)
	&msm_device_uart3,
#endif

#ifdef DEBUG_ACC_SENSOR
	&acc_sensor_gpio_i2c_device,
#endif
#ifdef DEBUG_COMPASS_SENSOR
	&compass_sensor_gpio_i2c_device,
#endif
#ifdef DEBUG_LIGHT_SENSOR
	&light_sensor_gpio_i2c_device,
#endif
	&msm_device_nand,
	
#ifndef CONFIG_EMUL_I2C__GPIO_61_60 // modified by X, remove the default HW i2c0
	&msm_gsbi0_qup_i2c_device,
#endif

	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&usb_mass_storage_device,
	&rndis_device,
	&usb_diag_device,
	&usb_gadget_fserial_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_fb_device,

#ifndef CONFIG_PROJECT_ES03 // removed by X	
#ifndef CONFIG_MACH_MSM7X27A_SKU3
	&lcdc_toshiba_panel_device,
#else
	&lcdc_truly_panel_device,
#endif
#endif

#ifdef CONFIG_BATTERY_MSM7X27A
	&msm_charger_device,
#endif
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif
#ifdef CONFIG_SMSC911X
	&smsc911x_device,
#endif
#ifdef CONFIG_S5K4E1
	&msm_camera_sensor_s5k4e1,
#endif
#ifdef CONFIG_IMX072
	&msm_camera_sensor_imx072,
#endif
#ifdef CONFIG_WEBCAM_OV9726
	&msm_camera_sensor_ov9726,
#endif
#ifdef CONFIG_MT9E013
	&msm_camera_sensor_mt9e013,
#endif
#ifdef CONFIG_OV5640
	&msm_camera_sensor_ov5640,
#endif
#ifdef CONFIG_OV5647
        &msm_camera_sensor_ov5647,
#endif
#ifdef CONFIG_WEBCAM_OV7692
	&msm_camera_sensor_ov7692,
#endif
#ifdef CONFIG_S5K5CA
	&msm_camera_sensor_s5k5ca,
#endif
#ifdef CONFIG_FB_MSM_MIPI_DSI
	&mipi_dsi_panel_device,
#endif
	&msm_kgsl_3d0,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
#if defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03)
	&msm_bluesleep_device,
#endif	
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#ifdef CONFIG_LEDS_TRICOLOR_FLAHSLIGHT
	&msm_device_tricolor_leds,
#endif
    &msm_device_pmic_leds,//add by Joe 120419
	&msm_dbg_info_device,	// X
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
}

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27a_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

static void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	msm_reserve();
}

static void __init msm_device_i2c_init(void)
{
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;
}

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
	.mdp_rev = MDP_REV_303,
};

#define LCDC_RESET_PHYS		0x90008014

static	void __iomem *lcdc_reset_ptr;

static unsigned mipi_dsi_gpio[] = {
	GPIO_CFG(GPIO_LCDC_BRDG_RESET_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* LCDC_BRDG_RESET_N */
	GPIO_CFG(GPIO_LCDC_BRDG_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* GPIO_LCDC_BRDG_PD */
};

enum {
	DSI_SINGLE_LANE = 1,
	DSI_TWO_LANES,
};

static int msm_fb_get_lane_config(void)
{
	int rc = DSI_TWO_LANES;

	if (cpu_is_msm7x25a() || cpu_is_msm7x25aa()) {
		rc = DSI_SINGLE_LANE;
		pr_info("DSI Single Lane\n");
	} else {
		pr_info("DSI Two Lanes\n");
	}
	return rc;
}

static int msm_fb_dsi_client_reset_surf_ffa(void)
{
	int rc = 0;

	rc = gpio_request(GPIO_LCDC_BRDG_RESET_N, "lcdc_brdg_reset_n");
	if (rc < 0) {
		pr_err("failed to request lcd brdg reset_n\n");
		return rc;
	}

	rc = gpio_request(GPIO_LCDC_BRDG_PD, "lcdc_brdg_pd");
	if (rc < 0) {
		pr_err("failed to request lcd brdg pd\n");
		return rc;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		goto gpio_error;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[1], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge pd enable\n");
		goto gpio_error2;
	}

	rc = gpio_direction_output(GPIO_LCDC_BRDG_RESET_N, 1);

	rc |= gpio_direction_output(GPIO_LCDC_BRDG_PD, 1);
	gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);
	if (!rc) {
		if (machine_is_msm7x27a_surf()) {
			lcdc_reset_ptr = ioremap_nocache(LCDC_RESET_PHYS,
				sizeof(uint32_t));

			if (!lcdc_reset_ptr)
				return 0;
		}
		return rc;
	} else {
		goto gpio_error;
	}

gpio_error2:
	pr_err("Failed GPIO bridge pd\n");
	gpio_free(GPIO_LCDC_BRDG_PD);

gpio_error:
	pr_err("Failed GPIO bridge reset\n");
	gpio_free(GPIO_LCDC_BRDG_RESET_N);
	return rc;
}

static int msm_fb_dsi_client_reset_sku1(void)
{
	int rc = 0;

	printk("%s\n", __func__);

	if (boot_splash) {
		printk("boot_splash, do not init display gpios\n");
		return rc;
	}

	/* reset */
	rc = gpio_request(GPIO_LCDC_BRDG_RESET_N, "lcdc_brdg_reset_n");
	if (rc < 0) {
		pr_err("failed to request lcd brdg reset_n\n");
		return rc;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		gpio_free(GPIO_LCDC_BRDG_RESET_N);
		goto fail_reset;
	}

	rc = gpio_direction_output(GPIO_LCDC_BRDG_RESET_N, 1);
	if(rc) {
		pr_err("Failed to GPIO_LCDC_BRDG_RESET_N output\n");
		goto fail_reset;
	}

	/* BS */
	rc = gpio_request(GPIO_LCD_DSI_SEL, "lcd_dsi_sel");
	if (rc < 0) {
		pr_err("failed to request lcd_dsi_sel\n");
		goto fail_reset;
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_LCD_DSI_SEL, 0,
	        GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	        GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable lcd_dsi_sel\n");
		goto fail_bs;
	}

	rc = gpio_direction_output(GPIO_LCD_DSI_SEL, 1);
	if(rc) {
		pr_err("Failed to lcd_dsi_sel output\n");
		goto fail_bs;
	}

	/* backlight */
	rc = gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en");
	if (rc < 0) {
		pr_err("failed to request GPIO_BACKLIGHT_EN\n");
		goto fail_bs;
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_BACKLIGHT_EN, 0,
	        GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	        GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("failed to config GPIO_BACKLIGHT_EN\n");
		goto fail_back;
	}

	rc = gpio_direction_output(GPIO_BACKLIGHT_EN, 0);
	if (rc < 0) {
		pr_err("failed to enable GPIO_BACKLIGHT_EN\n");
		goto fail_back;
	}

	return rc;

fail_back:
	gpio_free(GPIO_BACKLIGHT_EN);
fail_bs:
	gpio_free(GPIO_LCD_DSI_SEL);
fail_reset:
	gpio_free(GPIO_LCDC_BRDG_RESET_N);
	return rc;
}

static int msm_fb_dsi_client_reset_es02(void)
{
	int rc = 0;

	printk("%s\n", __func__);

	if (boot_splash) {
		printk("boot_splash, do not init display gpios\n");
		return rc;
	}

	/* reset */
	rc = gpio_request(GPIO_LCDC_BRDG_RESET_N, "lcdc_brdg_reset_n");
	if (rc < 0) {
		pr_err("failed to request lcd brdg reset_n\n");
		return rc;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		gpio_free(GPIO_LCDC_BRDG_RESET_N);
		goto fail_reset;
	}

	rc = gpio_direction_output(GPIO_LCDC_BRDG_RESET_N, 1);
	if(rc) {
		pr_err("Failed to GPIO_LCDC_BRDG_RESET_N output\n");
		goto fail_reset;
	}

	/* BS */
	rc = gpio_request(GPIO_LCD_DSI_SEL, "lcd_dsi_sel");
	if (rc < 0) {
		pr_err("failed to request lcd_dsi_sel\n");
		goto fail_reset;
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_LCD_DSI_SEL, 0,
	        GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	        GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable lcd_dsi_sel\n");
		goto fail_bs;
	}

	rc = gpio_direction_output(GPIO_LCD_DSI_SEL, 1);
	if(rc) {
		pr_err("Failed to lcd_dsi_sel output\n");
		goto fail_bs;
	}

	/* backlight */
	rc = gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en");
	if (rc < 0) {
		pr_err("failed to request GPIO_BACKLIGHT_EN\n");
		goto fail_bs;
	}

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_BACKLIGHT_EN, 0,
	        GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	        GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("failed to config GPIO_BACKLIGHT_EN\n");
		goto fail_back;
	}

	rc = gpio_direction_output(GPIO_BACKLIGHT_EN, 0);
	if (rc < 0) {
		pr_err("failed to enable GPIO_BACKLIGHT_EN\n");
		goto fail_back;
	}

	return rc;

fail_back:
	gpio_free(GPIO_BACKLIGHT_EN);
fail_bs:
	gpio_free(GPIO_LCD_DSI_SEL);
fail_reset:
	gpio_free(GPIO_LCDC_BRDG_RESET_N);
	return rc;
}

static int msm_fb_dsi_client_reset(void)
{
	int rc = 0;

#if  defined(CONFIG_PROJECT_ES02) || defined(CONFIG_PROJECT_ES03)
	rc = msm_fb_dsi_client_reset_es02();
	return rc;	
#endif

	if (machine_is_msm7x27a_surf() || machine_is_msm7x27a_ffa()) {
		rc = msm_fb_dsi_client_reset_surf_ffa();
	} else if (machine_is_msm7x27a_sku1()) {
		rc = msm_fb_dsi_client_reset_sku1();
	}

	return rc;
}

static const char * const msm_fb_dsi_vreg[] = {
	"gp2", // #9
	"msme1",
};

static const int msm_fb_dsi_vreg_mV[] = {
	2850,
	1800,
};

static struct vreg *dsi_vreg[ARRAY_SIZE(msm_fb_dsi_vreg)];
static int dsi_gpio_initialized;

static int mipi_dsi_panel_power_surf_ffa(int on)
{
	int rc = 0;
	int i;
	uint32_t lcdc_reset_cfg;

	/* I2C-controlled GPIO Expander -init of the GPIOs very late */
	if (!dsi_gpio_initialized) {
		pmapp_disp_backlight_init();

		rc = gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr");
		if (rc < 0) {
			pr_err("failed to request gpio_disp_pwr\n");
			return rc;
		}

		rc = gpio_direction_output(GPIO_DISPLAY_PWR_EN, 1);
		if (rc < 0) {
			pr_err("failed to enable display pwr\n");
			goto fail_gpio1;
		}

		if (machine_is_msm7x27a_surf()) {
			rc = gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en");
			if (rc < 0) {
				pr_err("failed to request gpio_bkl_en\n");
				goto fail_gpio1;
			}

			rc = gpio_direction_output(GPIO_BACKLIGHT_EN, 1);
			if (rc < 0) {
				pr_err("failed to enable backlight\n");
				goto fail_gpio2;
			}
		}

		for (i = 0; i < ARRAY_SIZE(msm_fb_dsi_vreg); i++) {
			dsi_vreg[i] = vreg_get(0, msm_fb_dsi_vreg[i]);

			if (IS_ERR(dsi_vreg[i])) {
				pr_err("%s: vreg get failed with : (%ld)\n",
					__func__, PTR_ERR(dsi_vreg[i]));
				goto fail_gpio2;
			}

			rc = vreg_set_level(dsi_vreg[i],
				msm_fb_dsi_vreg_mV[i]);

			if (rc < 0) {
				pr_err("%s: set regulator level failed "
					"with :(%d)\n",	__func__, rc);
				goto vreg_fail1;
			}
		}

		dsi_gpio_initialized = 1;
	}

		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);

		if (machine_is_msm7x27a_surf()) {
			gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
		} else if (machine_is_msm7x27a_ffa()) {
			if (on) {
				/* This line drives an active low pin on FFA */
				rc = gpio_direction_output(GPIO_DISPLAY_PWR_EN,
					!on);
				if (rc < 0)
					pr_err("failed to set direction for "
						"display pwr\n");
			} else {
				gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN,
					!on);
				rc = gpio_direction_input(GPIO_DISPLAY_PWR_EN);
				if (rc < 0)
					pr_err("failed to set direction for "
						"display pwr\n");
			}
		}

		if (on) {
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);

			if (machine_is_msm7x27a_surf()) {
				lcdc_reset_cfg = readl_relaxed(lcdc_reset_ptr);
				rmb();
				lcdc_reset_cfg &= ~1;

				writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
				msleep(20);
				wmb();
				lcdc_reset_cfg |= 1;
				writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
			} else {
				gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N,
					0);
				msleep(20);
				gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N,
					1);
			}

			if (pmapp_disp_backlight_set_brightness(100))
				pr_err("backlight set brightness failed\n");
		} else {
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 1);

			if (pmapp_disp_backlight_set_brightness(0))
				pr_err("backlight set brightness failed\n");
		}

		/*Configure vreg lines */
		for (i = 0; i < ARRAY_SIZE(msm_fb_dsi_vreg); i++) {
			if (on) {
				rc = vreg_enable(dsi_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_enable: %s vreg"
						"operation failed\n",
						msm_fb_dsi_vreg[i]);

					goto vreg_fail2;
				}
			} else {
				rc = vreg_disable(dsi_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_disable: %s vreg "
						"operation failed\n",
						msm_fb_dsi_vreg[i]);
					goto vreg_fail2;
				}
			}
		}


	return rc;

vreg_fail2:
	if (on) {
		for (; i > 0; i--)
			vreg_disable(dsi_vreg[i - 1]);
	} else {
		for (; i > 0; i--)
			vreg_enable(dsi_vreg[i - 1]);
	}

vreg_fail1:
	for (; i > 0; i--)
		vreg_put(dsi_vreg[i - 1]);

fail_gpio2:
	gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio1:
	gpio_free(GPIO_DISPLAY_PWR_EN);
	dsi_gpio_initialized = 0;
	return rc;
}

static int mipi_dsi_panel_power_sku1(int on)
{
	pr_debug("%s\n", __func__);

	/* keep pulldown to avoid current leakage */
	gpio_set_value_cansleep(GPIO_LCD_DSI_SEL, 0);
	gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N, 0);

	lcd_camera_power_onoff(on);

	return 0;
}


#if defined(CONFIG_PROJECT_ES02)
static int mipi_dsi_panel_power_es02(int on)
{
	pr_debug("%s\n", __func__);

	lcd_camera_power_onoff(on);

	return 0;
}
#endif

static int mipi_dsi_panel_power(int on)
{
	int rc = 0;

#if defined(CONFIG_PROJECT_ES02)
	rc = mipi_dsi_panel_power_es02(on);
	return rc;
#endif

	if (machine_is_msm7x27a_surf() || machine_is_msm7x27a_ffa()) {
		rc = mipi_dsi_panel_power_surf_ffa(on);
	} else if (machine_is_msm7x27a_sku1()) {
		rc = mipi_dsi_panel_power_sku1(on);
	}

	return rc;
}

#define MDP_303_VSYNC_GPIO 97

#ifdef CONFIG_FB_MSM_MDP303
static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio = MDP_303_VSYNC_GPIO,
	.dsi_power_save   = mipi_dsi_panel_power,
	.dsi_client_reset = msm_fb_dsi_client_reset,
	.get_lane_config = msm_fb_get_lane_config,
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	// #9 removed by X, set in MP.	msm_fb_register_device("lcdc", &lcdc_pdata);
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
}

#define MSM_EBI2_PHYS			0xa0d00000
#define MSM_EBI2_XMEM_CS2_CFG1		0xa0d10030

static void __init msm7x27a_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void __iomem *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_PHYS, sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_rumi3() || machine_is_msm7x27a_surf())
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_surf())
		ebi2_cfg |= (1 << 31);

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);
}

#define ATMEL_TS_I2C_NAME "maXTouch"
static struct vreg *vreg_l12;
static struct vreg *vreg_s3;

#define ATMEL_TS_GPIO_IRQ 82

static int atmel_ts_power_on(bool on)
{
	int rc;

	rc = on ? vreg_enable(vreg_l12) : vreg_disable(vreg_l12);
	if (rc) {
		pr_err("%s: vreg %sable failed (%d)\n",
		       __func__, on ? "en" : "dis", rc);
		return rc;
	}

	rc = on ? vreg_enable(vreg_s3) : vreg_disable(vreg_s3);
	if (rc) {
		pr_err("%s: vreg %sable failed (%d) for S3\n",
		       __func__, on ? "en" : "dis", rc);
		!on ? vreg_enable(vreg_l12) : vreg_disable(vreg_l12);
		return rc;
	}
	/* vreg stabilization delay */
	msleep(50);
	return 0;
}

static int atmel_ts_platform_init(struct i2c_client *client)
{
	int rc;

	vreg_l12 = vreg_get(NULL, "gp2");
	if (IS_ERR(vreg_l12)) {
		pr_err("%s: vreg_get for L2 failed\n", __func__);
		return PTR_ERR(vreg_l12);
	}

	rc = vreg_set_level(vreg_l12, 2850);
	if (rc) {
		pr_err("%s: vreg set level failed (%d) for l2\n",
		       __func__, rc);
		goto vreg_put_l2;
	}

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg_get for S3 failed\n", __func__);
		rc = PTR_ERR(vreg_s3);
		goto vreg_put_l2;
	}

	rc = vreg_set_level(vreg_s3, 1800);
	if (rc) {
		pr_err("%s: vreg set level failed (%d) for S3\n",
		       __func__, rc);
		goto vreg_put_s3;
	}

	rc = gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto vreg_put_s3;
	}

	/* configure touchscreen interrupt gpio */
	rc = gpio_request(ATMEL_TS_GPIO_IRQ, "atmel_maxtouch_gpio");
	if (rc) {
		pr_err("%s: unable to request gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto ts_gpio_tlmm_unconfig;
	}

	rc = gpio_direction_input(ATMEL_TS_GPIO_IRQ);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto free_ts_gpio;
	}
	return 0;

free_ts_gpio:
	gpio_free(ATMEL_TS_GPIO_IRQ);
ts_gpio_tlmm_unconfig:
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
vreg_put_s3:
	vreg_put(vreg_s3);
vreg_put_l2:
	vreg_put(vreg_l12);
	return rc;
}

static int atmel_ts_platform_exit(struct i2c_client *client)
{
	gpio_free(ATMEL_TS_GPIO_IRQ);
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	vreg_disable(vreg_s3);
	vreg_put(vreg_s3);
	vreg_disable(vreg_l12);
	vreg_put(vreg_l12);
	return 0;
}

static u8 atmel_ts_read_chg(void)
{
	return gpio_get_value(ATMEL_TS_GPIO_IRQ);
}

static u8 atmel_ts_valid_interrupt(void)
{
	return !atmel_ts_read_chg();
}

#define ATMEL_X_OFFSET 13
#define ATMEL_Y_OFFSET 0

static struct mxt_platform_data atmel_ts_pdata = {
	.numtouch = 4,
	.init_platform_hw = atmel_ts_platform_init,
	.exit_platform_hw = atmel_ts_platform_exit,
	.power_on = atmel_ts_power_on,
	.display_res_x = 480,
	.display_res_y = 864,
	.min_x = ATMEL_X_OFFSET,
	.max_x = (505 - ATMEL_X_OFFSET),
	.min_y = ATMEL_Y_OFFSET,
	.max_y = (863 - ATMEL_Y_OFFSET),
	.valid_interrupt = atmel_ts_valid_interrupt,
	.read_chg = atmel_ts_read_chg,
};

static struct i2c_board_info atmel_ts_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO(ATMEL_TS_I2C_NAME, 0x4a),
		.platform_data = &atmel_ts_pdata,
		.irq = MSM_GPIO_TO_INT(ATMEL_TS_GPIO_IRQ),
	},
};

#ifdef CONFIG_MACH_MSM7X27A_SKU1
#define KEYPAD_ROW 91
#define KEYPAD_COL 36, 37, 38
#endif

static unsigned int kp_row_gpios[] = {
#ifndef KEYPAD_ROW
31, 32, 33, 34, 35
#else
KEYPAD_ROW
#endif
};

static unsigned int kp_col_gpios[] = {
#ifndef KEYPAD_COL
36, 37, 38, 39, 40
#else
KEYPAD_COL
#endif
};

#define KP_INDEX(row, col) ((row)*ARRAY_SIZE(kp_col_gpios) + (col))
#ifdef CONFIG_MACH_MSM7X27A_SKU1
#define KEYPAD_LAYOUT				\
        [KP_INDEX(0, 0)] = KEY_VOLUMEUP,	\
        [KP_INDEX(0, 1)] = KEY_VOLUMEDOWN,	\
        [KP_INDEX(0, 2)] = KEY_HOME,	\
        //[KP_INDEX(1, 1)] = KEY_CAMERA,
#endif
static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
#ifndef KEYPAD_LAYOUT
	[KP_INDEX(0, 0)] = KEY_7,
	[KP_INDEX(0, 1)] = KEY_DOWN,
	[KP_INDEX(0, 2)] = KEY_UP,
	[KP_INDEX(0, 3)] = KEY_RIGHT,
	[KP_INDEX(0, 4)] = KEY_ENTER,

	[KP_INDEX(1, 0)] = KEY_LEFT,
	[KP_INDEX(1, 1)] = KEY_SEND,
	[KP_INDEX(1, 2)] = KEY_1,
	[KP_INDEX(1, 3)] = KEY_4,
	[KP_INDEX(1, 4)] = KEY_CLEAR,

	[KP_INDEX(2, 0)] = KEY_6,
	[KP_INDEX(2, 1)] = KEY_5,
	[KP_INDEX(2, 2)] = KEY_8,
	[KP_INDEX(2, 3)] = KEY_3,
	[KP_INDEX(2, 4)] = KEY_NUMERIC_STAR,

	[KP_INDEX(3, 0)] = KEY_9,
	[KP_INDEX(3, 1)] = KEY_NUMERIC_POUND,
	[KP_INDEX(3, 2)] = KEY_0,
	[KP_INDEX(3, 3)] = KEY_2,
	[KP_INDEX(3, 4)] = KEY_SLEEP,

	[KP_INDEX(4, 0)] = KEY_BACK,
	[KP_INDEX(4, 1)] = KEY_HOME,
	[KP_INDEX(4, 2)] = KEY_MENU,
	[KP_INDEX(4, 3)] = KEY_VOLUMEUP,
	[KP_INDEX(4, 4)] = KEY_VOLUMEDOWN,
#else
	KEYPAD_LAYOUT
#endif
};

/* SURF keypad platform device information */
static struct gpio_event_matrix_info kp_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= kp_row_gpios,
	.input_gpios	= kp_col_gpios,
	.noutputs	= ARRAY_SIZE(kp_row_gpios),
	.ninputs	= ARRAY_SIZE(kp_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS|GPIOKPF_PRINT_MAPPED_KEYS,
};

static struct gpio_event_info *kp_info[] = {
	&kp_matrix_info.info
};

static struct gpio_event_platform_data kp_pdata = {
	.name		= "7x27a_kp",
	.info		= kp_info,
	.info_count	= ARRAY_SIZE(kp_info)
};

static struct platform_device kp_pdev = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &kp_pdata,
	},
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

//#define LED_GPIO_PDM		96
#define UART1DM_RX_GPIO		45
#ifdef CONFIG_ATH_WIFI /* [WIFI] Wi-Fi Support ++ */
#define NV_ITEM_WLAN_MAC_ADDR   4678
extern int msm_read_nv(unsigned int nv_item, void *buf);
unsigned char wlan_mac_addr[6];
static int __init msm7x27a_init_ar6000pm(void)
{
	msm_read_nv(NV_ITEM_WLAN_MAC_ADDR,wlan_mac_addr);
	return platform_device_register(&msm_wlan_ar6000_pm_device);
}
#endif

static void __init msm7x2x_init(void)
{
	msm7x2x_misc_init();

	/* Common functions for SURF/FFA/RUMI3 */
	msm_device_i2c_init();
	msm7x27a_init_ebi2();
	msm7x27a_cfg_uart2dm_serial();

	/* LCD and camera power (VREG & LDO) init */
	lcd_camera_power_init();
	/* keep reference count to avoid other device shutdown display power */
	if (boot_splash)
		lcd_camera_power_onoff(1);
	msm7x27a_camera_gpio_cfg();

#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

	if (machine_is_msm7x27a_rumi3()) {
		platform_add_devices(rumi_sim_devices,
				ARRAY_SIZE(rumi_sim_devices));
	}

	if (machine_is_msm7x27a_surf() || machine_is_msm7x27a_ffa() || machine_is_msm7x27a_sku1() || machine_is_msm7x27a_sku3()) {
#ifdef CONFIG_USB_MSM_OTG_72K
		msm_otg_pdata.swfi_latency =
			msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
		msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif
		msm_device_gadget_peripheral.dev.platform_data =
							&msm_gadget_pdata;
#ifdef CONFIG_SMSC911X
		msm7x27a_cfg_smsc911x();
#endif
		platform_add_devices(msm_footswitch_devices,
			     msm_num_footswitch_devices);
		platform_add_devices(surf_ffa_devices,
				ARRAY_SIZE(surf_ffa_devices));
		msm_fb_add_devices();
#ifdef CONFIG_ATH_WIFI
		/* Ensure ar6000pm device is registered before MMC/SDC */
		msm7x27a_init_ar6000pm();
#endif
#ifdef CONFIG_MMC_MSM
		msm7x27a_init_mmc();
#endif
#ifdef CONFIG_USB_EHCI_MSM_72K
		msm7x2x_init_host();
#endif
	}

	msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));

#if defined(CONFIG_I2C)
	register_i2c_devices();
#endif
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)

#ifndef CONFIG_PROJECT_ES03	// removed by X
        if(machine_is_msm7x27a_sku3() && hw_version_is(10, 0)) {
            GPIO_BT_SYS_REST_EN_ = GPIO_BT_SYS_REST_EN_SKU3_1;
        }
#endif

	bt_power_init();
#endif
	if (cpu_is_msm7x25a() || cpu_is_msm7x25aa()) {
		atmel_ts_pdata.min_x = 0;
		atmel_ts_pdata.max_x = 480;
		atmel_ts_pdata.min_y = 0;
		atmel_ts_pdata.max_y = 320;
	}

#if defined(CONFIG_BATTERY_BQ27510) || \
                defined(CONFIG_BATTERY_BQ27510_MODULE)
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
                                msm_bq27510_board_info,
                                ARRAY_SIZE(msm_bq27510_board_info));
#endif
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		atmel_ts_i2c_info,
		ARRAY_SIZE(atmel_ts_i2c_info));
#if defined(CONFIG_TOUCHSCREEN_FT5206) || defined(CONFIG_TOUCHSCREEN_FT5306)
	ft5x06_touchpad_setup();
#endif

//add by joe 120411
#if defined(CONFIG_TOUCHSCREEN_MCS6024_TS)
    mcs6024_touchpad_setup();
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			&synaptic_i2c_clearpad3k, 1);
#endif

#ifdef CONFIG_EMUL_I2C__GPIO_61_60	// #9 modified by X
	platform_device_register(&gpio_i2c_61_60_dev); /* camera & touch panel */
	i2c_register_board_info(10, i2c_camera_devices, ARRAY_SIZE(i2c_camera_devices));
#else
	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID, i2c_camera_devices, ARRAY_SIZE(i2c_camera_devices));
#endif

	platform_device_register(&kp_pdev);
	platform_device_register(&hs_pdev);

	/* configure it as a pdm function*/
/*
	if (gpio_tlmm_config(GPIO_CFG(LED_GPIO_PDM, 3,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, LED_GPIO_PDM);
	else
		platform_device_register(&led_pdev);
*/
#ifdef CONFIG_MSM_RPC_VIBRATOR
	if (machine_is_msm7x27a_ffa() || machine_is_msm7x27a_sku1() || machine_is_msm7x27a_sku3())
		msm_init_pmic_vibrator();
#endif
	/*7x25a kgsl initializations*/
	msm7x25a_kgsl_3d0_init();
#ifdef CONFIG_LEDS_TRICOLOR_FLAHSLIGHT
	/*tricolor leds init*/
//	tricolor_leds_gpio_setup();
#endif

}

static void __init msm7x2x_init_early(void)
{
	msm_msm7x2x_allocate_memory_regions();
}

MACHINE_START(MSM7X27A_RUMI3, "QCT MSM7x27a RUMI3")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_SURF, "QCT MSM7x27a SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_FFA, "QCT MSM7x27a FFA")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_SKU1, "QRD MSM7x27a SKU1")
#ifdef CONFIG_MSM_DEBUG_UART
        .phys_io        = MSM_DEBUG_UART_PHYS,
        .io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
        .boot_params    = PHYS_OFFSET + 0x100,
        .map_io         = msm_common_io_init,
	.reserve	= msm7x27a_reserve,
        .init_irq       = msm_init_irq,
        .init_machine   = msm7x2x_init,
        .timer          = &msm_timer,
        .init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_SKU3, "QRD MSM7x27a SKU3")
#ifdef CONFIG_MSM_DEBUG_UART
        .phys_io        = MSM_DEBUG_UART_PHYS,
        .io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
        .boot_params    = PHYS_OFFSET + 0x100,
        .map_io         = msm_common_io_init,
        .reserve        = msm7x27a_reserve,
        .init_irq       = msm_init_irq,
        .init_machine   = msm7x2x_init,
        .timer          = &msm_timer,
        .init_early     = msm7x2x_init_early,
MACHINE_END

