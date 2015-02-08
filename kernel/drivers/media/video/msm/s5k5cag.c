#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <linux/leds.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <linux/timer.h>
#include "s5k5cag.h"

/* Debug switch */
#ifdef CDBG
#undef CDBG
#endif
#ifdef CDBG_HIGH
#undef CDBG_HIGH
#endif

#define S5K5CAG_VERBOSE_DGB

#ifdef S5K5CAG_VERBOSE_DGB
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#define CDBG_HIGH(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#define CDBG_HIGH(fmt, args...) pr_err(fmt, ##args)
#endif

//Camera init flag for check if it is inited.
static bool camera_init_flag;
struct s5k5cag_work {
   struct work_struct work;
};
static struct s5k5cag_work *s5k5cag_sensorw;
static struct i2c_client    *s5k5cag_client;

static DECLARE_WAIT_QUEUE_HEAD(s5k5cag_wait_queue);
DEFINE_MUTEX(s5k5cag_mutex);
static u8 s5k5cag_i2c_buf[4];
static u8 s5k5cag_counter = 0;
static int S5K5CAG_CSI_CONFIG = 0;
static u8 need_send_preview_regs = 1;	// added by X
static int effect_value = 0;

static struct msm_camera_sensor_flash_data *s5k5ca_fdata;
static int is_autoflash =0;
static int is_autoflash_working = 0;

struct __s5k5cag_ctrl 
{
	const struct msm_camera_sensor_info *sensordata;
	int sensormode;
	uint fps_divider; /* init to 1 * 0x00000400 */
	uint pict_fps_divider; /* init to 1 * 0x00000400 */
	u16 curr_step_pos;
	u16 curr_lens_pos;
	u16 init_curr_lens_pos;
	u16 my_reg_gain;
	u16 my_reg_line_count;
	enum msm_s_resolution prev_res;
	enum msm_s_resolution pict_res;
	enum msm_s_resolution curr_res;
	enum msm_s_test_mode  set_test;
};
static struct __s5k5cag_ctrl *s5k5cag_ctrl;
//static u32 exp_gain_tbl[3][2];

static int s5k5cag_pwdn_gpio;
static int s5k5cag_reset_gpio;
static int s5k5cag_driver_pwdn_gpio;
extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;
// #9 removed extern int lcd_camera_power_onoff(int on);


/* 
NOTE:
1) SADDR=1, 0x5A WRITE & 0x5B READ
2) GPIO_1=GPIO_2=DOVDD
3) PD_REG_DIG=0, INTERAL REGULATOR ON
4) TST=1, STBYN(PWDN) ACTIVE HIGH
5) RESET ACTIVE LOW
*/



#define GPIO_PWDN_LOW		gpio_set_value(s5k5cag_pwdn_gpio, 0)
#define GPIO_PWDN_HIGH		gpio_set_value(s5k5cag_pwdn_gpio, 1)	// ACTIVE 

#define GPIO_RESET_LOW		gpio_set_value(s5k5cag_reset_gpio, 0)
#define GPIO_RESET_HIGH		gpio_set_value(s5k5cag_reset_gpio, 1)


static int s5k5cag_i2c_txdata(u16 saddr,u8 *txdata,int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	if (i2c_transfer(s5k5cag_client->adapter, msg, 1) < 0)	return -EIO;
	else return 0;
}

static int s5k5cag_i2c_write(unsigned short saddr, unsigned int waddr,
	unsigned short bdata,u8 trytimes)
{
   int rc = -EIO;
   s5k5cag_counter = 0;
   s5k5cag_i2c_buf[0] = (waddr & 0xFF00)>>8;
   s5k5cag_i2c_buf[1] = (waddr & 0x00FF);
   s5k5cag_i2c_buf[2] = (bdata & 0xFF00)>>8;
   s5k5cag_i2c_buf[3] = (bdata & 0x00FF);
   
   while ( (s5k5cag_counter<trytimes) &&(rc != 0) )
   {
      rc = s5k5cag_i2c_txdata(saddr, s5k5cag_i2c_buf, 4);
      if (rc < 0){
	      	s5k5cag_counter++;
	      	CDBG_HIGH( "***Tom i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr,waddr, bdata,s5k5cag_counter,rc);
	      	msleep(4);
      }
   }
   return rc;
}

static int s5k5cag_i2c_write_table(PS5K5CAGA_WREG p,int32_t len,u8 trytimes)
{
	int i=0,rc=0;
	for(;i<len;i++) {
		if(p[i].addr == 0xffff) {
			mdelay(p[i].data); // #9 mdelay(100);
		} else {
			rc = s5k5cag_i2c_write(s5k5cag_client->addr,p[i].addr,p[i].data,trytimes);
			if (rc < 0) return rc;
		}
	}
	return rc;
}
static int s5k5cag_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(s5k5cag_client->adapter, msgs, 2) < 0) {
		CDBG_HIGH("s5k5cag_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k5cag_i2c_read(unsigned short   saddr,
	unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k5cag_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)	return rc;
	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)	CDBG_HIGH("s5k5cag_i2c_read failed!\n");

	return rc;
}


static int s5k5cag_probe_readID(const struct msm_camera_sensor_info *data)
{

	u32 device_id=0;

	s5k5cag_i2c_write(s5k5cag_client->addr,0x002c,0x0000,10);
	s5k5cag_i2c_write(s5k5cag_client->addr,0x002e,0x0040,10);
	s5k5cag_i2c_read(s5k5cag_client->addr, 0x0f12, &device_id);
	printk(KERN_ERR "%s:  s5k5ca device id=0x%x\n",__func__,device_id);

	//s5k5cag chip id is 0x05ca
	if(device_id != 0x05ca)
	{
		printk(KERN_ERR"--CAMERA-- %s ERROR , device id=0x%x\n",__func__, device_id);
		return -1;
	}

	return 0;
}

static int s5k5cag_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int rc = -1;
	CDBG( "%s\n",__func__);
	
	s5k5cag_pwdn_gpio = data->sensor_pwd;
	s5k5cag_reset_gpio = data->sensor_reset;
	s5k5cag_driver_pwdn_gpio = data->vcm_pwd ;
	s5k5ca_fdata = data->flash_data;

	CDBG( "%s pwdn: %d, reset:%d\n",__func__, s5k5cag_pwdn_gpio, s5k5cag_reset_gpio);


	//set Camera PMIC and power on	 
	// #9 removed  lcd_camera_power_onoff(1);
	
#if 1 // #3
	GPIO_PWDN_HIGH;
	GPIO_RESET_LOW;
	mdelay(1);
	GPIO_PWDN_LOW;
	mdelay(1);
	
	GPIO_RESET_HIGH;
	mdelay(1);

	msm_camio_clk_rate_set(24000000); 	//Set Clock = 48 MHz
	mdelay(5);
#endif 	

	//Read Device ID
	 rc = s5k5cag_probe_readID(data);

#if 0 // #3
	mdelay(10);
	rc = S5K5CAG_WriteTABLE(preview_s5k5cag_reg); 
	need_send_preview_regs = 0; 
	mdelay(10);
#endif

	GPIO_PWDN_HIGH; // #3
	mdelay(100); // It's NEED!
	
 	// #9 removed lcd_camera_power_onoff(0);

	camera_init_flag=true;

   	return rc;
}

static int s5k5cag_set_flash_light(unsigned led_state)
{
	int rc = 0;
	CDBG("--CAMERA-- s5k5cag_set_flash_light led_state = %d\n", led_state);
	// #9 removed rc = msm_camera_flash_set_led_state(s5k5ca_fdata , led_state);
	
	return rc;
}

static int s5k5cag_setting(enum msm_s_reg_update rupdate,enum msm_s_setting rt)
{
	int rc = -EINVAL;
	struct msm_camera_csi_params s5k5cag_csi_params;
	CDBG( "%s, rupdate=%d, rt:%d\n",__func__, rupdate, rt);
	
	switch (rupdate)
	{
		case S_REG_INIT:
			CDBG( "--CAMERA-- S_REG_INIT (Start)\n");

			//set register setting
			if (need_send_preview_regs) {
				need_send_preview_regs = 0; // #9
				rc = S5K5CAG_WriteTABLE(preview_s5k5cag_reg);
			} else {
				rc = 0;
			}
			
			/* reset fps_divider */
			s5k5cag_ctrl->fps_divider = 1 * 0x0400;
			CDBG( "--CAMERA-- S_REG_INIT (End)\n");
			break; /* case REG_INIT: */
		case S_UPDATE_PERIODIC:

			if(!S5K5CAG_CSI_CONFIG){
				CDBG( "--CAMERA-- S5K5CAG_CSI_CONFIG-- \n");
			    s5k5cag_i2c_write(s5k5cag_client->addr,0x0028,0xD000,10);
				s5k5cag_i2c_write(s5k5cag_client->addr,0x002a,0x107e,10);
				s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12,0x0001,10);//stop stream;
				msleep(20);
				
				s5k5cag_csi_params.lane_cnt = 1;
				s5k5cag_csi_params.data_format = CSI_8BIT;
				s5k5cag_csi_params.lane_assign = 0xe4;
				s5k5cag_csi_params.dpcm_scheme = 0;
				s5k5cag_csi_params.settle_cnt = 0x14;
	
				rc = msm_camio_csi_config(&s5k5cag_csi_params);	
				msleep(20);			  
				s5k5cag_i2c_write(s5k5cag_client->addr,0x107e,0x0000,10);//start stream;
				S5K5CAG_CSI_CONFIG = 1;
			}
			if(rt == S_RES_PREVIEW) {
     			rc = S5K5CAG_WriteTABLE(preview_tbl_1);	
			} else {					
				rc = S5K5CAG_WriteTABLE(capture_tbl_1);
				//msleep(266);/* Delay to get Snapshot Working */
				s5k5cag_set_flash_light(MSM_CAMERA_LED_OFF);
				is_autoflash_working = 0;
			}
			break; /* UPDATE_PERIODIC */
		default:
			break;
	} /* switch (rupdate) */

	CDBG( "--CAMERA-- %s (End), rupdate=%d \n",__func__,rupdate);
	return rc;
}

static int s5k5cag_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = -ENOMEM;
	CDBG( "--CAMERA-- %s\n",__func__);

	s5k5cag_ctrl = kzalloc(sizeof(struct __s5k5cag_ctrl), GFP_KERNEL);
	if (!s5k5cag_ctrl)
	{
		CDBG_HIGH( "--CAMERA-- kzalloc s5k5cag_ctrl error !!\n");
		kfree(s5k5cag_ctrl);
		return rc;
	}
	s5k5cag_ctrl->fps_divider = 1 * 0x00000400;
	s5k5cag_ctrl->pict_fps_divider = 1 * 0x00000400;
	s5k5cag_ctrl->set_test = S_TEST_OFF;
	s5k5cag_ctrl->prev_res = S_QTR_SIZE;
	s5k5cag_ctrl->pict_res = S_FULL_SIZE;
	
	if (data) s5k5cag_ctrl->sensordata = data;

	// #9 removed lcd_camera_power_onoff(1); 

	GPIO_PWDN_LOW;
	msleep(5);
	
	/* enable mclk = 24 MHz first */
	msm_camio_clk_rate_set(24000000);//48000000);//origin: 24000000);
	mdelay(5);

	//#9 we send preview registers once only:  s5k5cag_power_reset();
	//msleep(5);
	
	if (s5k5cag_ctrl->prev_res == S_QTR_SIZE)
		rc = s5k5cag_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = s5k5cag_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0)
	{
		CDBG_HIGH( "--CAMERA-- %s : s5k5cag_setting failed. rc = %d\n",__func__,rc);
		kfree(s5k5cag_ctrl);
		s5k5cag_ctrl = NULL;

		//s5k5cag_standby(1);//#3 
		GPIO_PWDN_HIGH;
		
		// #9 removed lcd_camera_power_onoff(0);
		return rc;
	}
	return rc;
}

static int s5k5cag_sensor_release(void)
{
	CDBG( "--CAMERA--s5k5cag_sensor_release!!\n");

	mutex_lock(&s5k5cag_mutex);
	
	GPIO_PWDN_HIGH;
	mdelay(100); // It's NEED!
	
	// #9 removed lcd_camera_power_onoff(0);

	
	kfree(s5k5cag_ctrl);
	s5k5cag_ctrl = NULL;
	S5K5CAG_CSI_CONFIG = 0;
	mutex_unlock(&s5k5cag_mutex);
	return 0;
}

static int s5k5cag_video_config(int mode)
{
	int rc = -1;
	if ((rc = s5k5cag_setting(S_UPDATE_PERIODIC,S_RES_PREVIEW)) < 0) {
	    CDBG_HIGH("--CAMERA-- s5k5cag_video_config failed\n");
	    return rc;
  	}
	s5k5cag_ctrl->curr_res = s5k5cag_ctrl->prev_res;
	s5k5cag_ctrl->sensormode = mode;
	return rc;
}

static int s5k5cag_snapshot_config(int mode)
{
	int rc = -1;
	if ((rc = s5k5cag_setting(S_UPDATE_PERIODIC,S_RES_CAPTURE)) < 0) {
	    CDBG_HIGH("--CAMERA-- s5k5cag_snapshot_config failed\n");
	    return rc;
  	}
	s5k5cag_ctrl->curr_res = s5k5cag_ctrl->pict_res;
	s5k5cag_ctrl->sensormode = mode;
	return rc;
}

/*
static int s5k5cag_raw_snapshot_config(int mode)
{
	int rc;
	rc = s5k5cag_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)	return rc;

	s5k5cag_ctrl->curr_res = s5k5cag_ctrl->pict_res;
	s5k5cag_ctrl->sensormode = mode;
	return rc;
}
*/
static int s5k5cag_set_sensor_mode(int mode, int res)
{
	int rc = -EINVAL;

	CDBG( "--CAMERA-- s5k5cag_set_sensor_mode mode = %d, res = %d\n", mode, res);

	switch (mode)
	{
	case SENSOR_PREVIEW_MODE:
		CDBG( "--CAMERA-- SENSOR_PREVIEW_MODE\n");
		s5k5cag_ctrl->prev_res =res;
		rc = s5k5cag_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		CDBG( "--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
		s5k5cag_ctrl->pict_res =res;
		rc = s5k5cag_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		CDBG( "--CAMERA-- SENSOR_RAW_SNAPSHOT_MODE\n");
		s5k5cag_ctrl->pict_res = res;
		//#9 todo rc = s5k5cag_raw_snapshot_config(mode);
		break;
	default:
		CDBG_HIGH( "--CAMERA--s5k5cag_set_sensor_mode  DEFAULT\n");
		break;
	}
	return rc;
}

static int s5k5cag_set_wb_oem(uint8_t param)
{
	CDBG( "s5k5cag_set_wb_oem %d\r\n",param);

	s5k5cag_i2c_write(s5k5cag_client->addr,0x0028, 0x7000,10);
	switch (param)
	{
	case CAMERA_WB_AUTO:
		CDBG( "CAMERA_WB_AUTO %d\r\n",param);
		//R Gain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04D2,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x067F,10);
		break;
	case CAMERA_WB_CLOUDY_DAYLIGHT: //4500K
		CDBG( "CAMERA_WB_CLOUDY_DAYLIGHT %d\r\n",param);
		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04D2,10);		
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0677,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A0,10); 	//Rgain 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0190,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A4,10);		//G Gain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0100,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A8,10);		//B Gain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0153,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);			
		break;
	case CAMERA_WB_DAYLIGHT://D65
		CDBG( "CAMERA_WB_DAYLIGHT %d\r\n",param);
		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04D2,10);		//R Gain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0677,10); 

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A0,10); 	//Rgain 1.42
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x017c,10);   
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A4,10);		//G Gain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0100,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A8,10);		//B Gain 1.27
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x012d,10);   
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);
		break;
	case CAMERA_WB_INCANDESCENT: //INCA
		CDBG( "CAMERA_WB_INCANDESCENT %d\r\n",param);
		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04D2,10);		
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0677,10); 

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A0,10); 		//Rgain1.16
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0100,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A4,10);		//G Gain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0100,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A8,10);		//B Gain 1.74
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0255,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);
		break;		
	case CAMERA_WB_FLUORESCENT://CWF
		CDBG( "CAMERA_WB_FLUORESCENT %d\r\n",param);
		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04D2,10);		
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0677,10); 

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A0,10); 		//Rgain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0167,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A4,10);		//G Gain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0100,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);

		s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x04A8,10);		//B Gain
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x01fc,10); 
		s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x0001,10);
		break;		
	default:
		break;			
	}	
	return 0;
}

static long s5k5cag_antibanding(int antibanding)
{
	int32_t rc = 0;
	unsigned int old_afc_val;

	CDBG("[xxm]: **********s5k5cag_set_antibanding antibanding = %d!\n", antibanding);

	if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002c, 0x7000,10) < 0){ goto set_antibanding_failed;}
	if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002e, 0x04d2,10) < 0){ goto set_antibanding_failed;}
	if(s5k5cag_i2c_read(s5k5cag_client->addr,0x0f12, &old_afc_val) < 0){ goto set_antibanding_failed;}
	if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0028, 0x7000,10) < 0){ goto set_antibanding_failed;}
	if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002a, 0x04d2,10) < 0){ goto set_antibanding_failed;}
	CDBG("[xxm]: **********s5k5cag_set_antibanding old_afc_val = 0x%x!\n", old_afc_val);

	switch(antibanding)
	{
	case CAMERA_ANTIBANDING_OFF:
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, old_afc_val & ~(1<<5),10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002a, 0x04ba,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, 0x0001,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002a, 0x04bc,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, 0x0001,10) < 0){ goto set_antibanding_failed;}
		break;

	case CAMERA_ANTIBANDING_60HZ:
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, old_afc_val & ~(1<<5),10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002a, 0x04ba,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, 0x0002,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002a, 0x04bc,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, 0x0001,10) < 0){ goto set_antibanding_failed;}
		break;

	case CAMERA_ANTIBANDING_50HZ:
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, old_afc_val & ~(1<<5),10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002a, 0x04ba,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, 0x0001,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x002a, 0x04bc,10) < 0){ goto set_antibanding_failed;}
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, 0x0001,10) < 0){ goto set_antibanding_failed;}
		break;

	case CAMERA_ANTIBANDING_AUTO:
		if(s5k5cag_i2c_write(s5k5cag_client->addr,0x0f12, old_afc_val | (1<<5),10) < 0){ goto set_antibanding_failed;}
		break;	

	default:
		return -EINVAL;
	}
  	return 0;
	set_antibanding_failed:
		CDBG_HIGH("[xxm]: s5k5cag_set_antibanding write failed. rc = %d\n", rc);
		return -EIO;
}

static long s5k5cag_set_effect(int mode,int para)
{
	int  rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		CDBG( "--CAMERA-- %s ...SENSOR_PREVIEW_MODE\n",__func__);
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		CDBG( "--CAMERA-- %s ...SENSOR_SNAPSHOT_MODE\n",__func__);
		break;

	default:
		break;
	}
	CDBG( "s5k5cag_set_effect %d\r\n",para);
	effect_value = para;
	switch (para)
	{	
	case CAMERA_EFFECT_OFF:
		S5K5CAG_WriteTABLE(s5k5cag_effect_normal_tbl);  				   
		break;		
	case CAMERA_EFFECT_MONO: 
		S5K5CAG_WriteTABLE(s5k5cag_effect_mono_tbl);
		break;		
	case CAMERA_EFFECT_BW:
		S5K5CAG_WriteTABLE(s5k5cag_effect_bw_tbl);
		break;
	case CAMERA_EFFECT_BLUISH:
		S5K5CAG_WriteTABLE(s5k5cag_effect_bluish_tbl);
		break;
	case CAMERA_EFFECT_REDDISH:
		S5K5CAG_WriteTABLE(s5k5cag_effect_reddish_tbl);
	    break;
	case CAMERA_EFFECT_GREENISH:
		S5K5CAG_WriteTABLE(s5k5cag_effect_greenish_tbl);
	    break;
	case CAMERA_EFFECT_SOLARIZE: 
		S5K5CAG_WriteTABLE(s5k5cag_effect_solarize_tbl);
		break;
	case CAMERA_EFFECT_SEPIA:  
		S5K5CAG_WriteTABLE(s5k5cag_effect_sepia_tbl);
		break;		
	case CAMERA_EFFECT_NEGATIVE:
		S5K5CAG_WriteTABLE(s5k5cag_effect_negative_tbl);
		break;		

	default:
		rc = 0;
	}
	return 0;
}

static int s5k5cag_set_brightness(int8_t para)
{
    CDBG( "s5k5cag_set_brightness %d\r\n",para);
	s5k5cag_i2c_write(s5k5cag_client->addr,0x0028, 0x7000,10);
	switch (para)
	{
		case /*CAMERA_BRIGHTNESS_LV4*/CAMERA_EXPOSURE_COMPENSATION_LV4:
			s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x020C,10);	//#TVAR_ae_BrAve //ae target
			s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x003D-(4*12),10);  					
			break;		

		case /*CAMERA_BRIGHTNESS_LV3*/CAMERA_EXPOSURE_COMPENSATION_LV3:
			s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x020C,10);	//#TVAR_ae_BrAve //ae target
			s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x003D-(2*12),10);  				   
			break;				
		case /*CAMERA_BRIGHTNESS_LV2*/CAMERA_EXPOSURE_COMPENSATION_LV2:
			s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x020C,10);	//#TVAR_ae_BrAve //ae target
			s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x003D,10);       		 
			break;				
		case /*CAMERA_BRIGHTNESS_LV1*/CAMERA_EXPOSURE_COMPENSATION_LV1:
			s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x020C,10);	//#TVAR_ae_BrAve //ae target
			s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x003D+(2*14),10);  				   
			break;				
		case /*CAMERA_BRIGHTNESS_LV0*/CAMERA_EXPOSURE_COMPENSATION_LV0:
			s5k5cag_i2c_write(s5k5cag_client->addr,0x002A, 0x020C,10);	//#TVAR_ae_BrAve //ae target
			s5k5cag_i2c_write(s5k5cag_client->addr,0x0F12, 0x003D+(4*14),10);  				   
			break;
		default:
			return 0;
	}	
	
	return 0;	
}

static int s5k5cag_set_contrast(int contrast)
{
    int rc = 0;
    CDBG("--CAMERA-- %s ...contrast = %d\n",__func__ , contrast);

    if (effect_value == CAMERA_EFFECT_OFF)
    {
        switch (contrast)
        {
        case CAMERA_CONTRAST_LV0:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV0\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_lv0_tbl);
            break;
        case CAMERA_CONTRAST_LV1:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV1\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_lv1_tbl);
            break;
        case CAMERA_CONTRAST_LV2:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV2\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_lv2_tbl);
            break;
        case CAMERA_CONTRAST_LV3:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV3\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_lv3_tbl);
            break;
        case CAMERA_CONTRAST_LV4:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV4\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_default_lv4_tbl);
            break;
        case CAMERA_CONTRAST_LV5:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV5\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_lv5_tbl);
            break;
        case CAMERA_CONTRAST_LV6:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV6\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_lv6_tbl);
            break;
        case CAMERA_CONTRAST_LV7:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV7\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_lv7_tbl);
            break;
        case CAMERA_CONTRAST_LV8:
            CDBG("--CAMERA--CAMERA_CONTRAST_LV8\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_contrast_lv8_tbl);
            break;
        default:
            CDBG_HIGH("--CAMERA--CAMERA_CONTRAST_ERROR COMMAND\n");
            break;
        }
    }
    return rc;
}

static int s5k5cag_set_saturation(int saturation)
{
    long rc = 0;
    CDBG("--CAMERA-- %s ...saturation = %d\n",__func__ , saturation);

    if (effect_value == CAMERA_EFFECT_OFF)
    {
        switch (saturation)
        {
        case CAMERA_SATURATION_LV0:
            CDBG("--CAMERA--CAMERA_SATURATION_LV0\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_lv0_tbl);
            break;
        case CAMERA_SATURATION_LV1:
            CDBG("--CAMERA--CAMERA_SATURATION_LV1\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_lv1_tbl);

            break;
        case CAMERA_SATURATION_LV2:
            CDBG("--CAMERA--CAMERA_SATURATION_LV2\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_lv2_tbl);

            break;
        case CAMERA_SATURATION_LV3:
            CDBG("--CAMERA--CAMERA_SATURATION_LV3\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_lv3_tbl);

            break;
        case CAMERA_SATURATION_LV4:
            CDBG("--CAMERA--CAMERA_SATURATION_LV4\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_default_lv4_tbl);

            break;
        case CAMERA_SATURATION_LV5:
            CDBG("--CAMERA--CAMERA_SATURATION_LV5\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_lv5_tbl);

            break;
        case CAMERA_SATURATION_LV6:
            CDBG("--CAMERA--CAMERA_SATURATION_LV6\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_lv6_tbl);

            break;
        case CAMERA_SATURATION_LV7:
            CDBG("--CAMERA--CAMERA_SATURATION_LV7\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_lv7_tbl);

            break;
        case CAMERA_SATURATION_LV8:
            CDBG("--CAMERA--CAMERA_SATURATION_LV8\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_saturation_lv8_tbl);

            break;        
        default:
            CDBG_HIGH("--CAMERA--CAMERA_SATURATION_ERROR COMMAND\n");
            break;
        }
    }
    return rc;
}

static int s5k5cag_set_sharpness(int sharpness)
{
    int rc = 0;
    CDBG("--CAMERA-- %s ...sharpness = %d\n",__func__ , sharpness);

    if (effect_value == CAMERA_EFFECT_OFF)
    {
        switch(sharpness)
        {
        case CAMERA_SHARPNESS_LV0:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV0\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_lv0_tbl);
            break;
        case CAMERA_SHARPNESS_LV1:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV1\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_lv1_tbl);

            break;
        case CAMERA_SHARPNESS_LV2:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV2\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_default_lv2_tbl);

            break;
        case CAMERA_SHARPNESS_LV3:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV3\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_lv3_tbl);

            break;
        case CAMERA_SHARPNESS_LV4:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV4\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_lv4_tbl);

            break;
        case CAMERA_SHARPNESS_LV5:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV5\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_lv5_tbl);

            break;
        case CAMERA_SHARPNESS_LV6:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV6\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_lv6_tbl);

            break;
        case CAMERA_SHARPNESS_LV7:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV7\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_lv7_tbl);

            break;
        case CAMERA_SHARPNESS_LV8:
            CDBG("--CAMERA--CAMERA_SHARPNESS_LV8\n");
            rc = S5K5CAG_WriteTABLE(s5k5cag_sharpness_lv8_tbl);

            break;
        default:
            CDBG_HIGH("--CAMERA--CAMERA_SHARPNESS_ERROR COMMAND\n");
            break;
        }
    }
    return rc;
}

static int s5k5cag_set_iso(int8_t iso_type)
{
    long rc = 0;

    CDBG("--CAMERA-- %s ...iso_type = %d\n",__func__ , iso_type);
    switch(iso_type)
    {
		case CAMERA_ISO_TYPE_AUTO:
		    CDBG("--CAMERA--CAMERA_ISO_TYPE_AUTO\n");
		    rc = S5K5CAG_WriteTABLE(s5k5cag_iso_type_auto);
		    break;
		case CAMEAR_ISO_TYPE_HJR:
		    CDBG("--CAMERA--CAMEAR_ISO_TYPE_HJR\n");
		    rc = S5K5CAG_WriteTABLE(s5k5cag_iso_type_auto);
		    break;
		case CAMEAR_ISO_TYPE_100:
		    CDBG("--CAMERA--CAMEAR_ISO_TYPE_100\n");
		    rc = S5K5CAG_WriteTABLE(s5k5cag_iso_type_100);
		    break;
		case CAMERA_ISO_TYPE_200:
		    CDBG("--CAMERA--CAMERA_ISO_TYPE_200\n");
		    rc = S5K5CAG_WriteTABLE(s5k5cag_iso_type_200);
		    break;
		case CAMERA_ISO_TYPE_400:
		    CDBG("--CAMERA--CAMERA_ISO_TYPE_400\n");
		    rc = S5K5CAG_WriteTABLE(s5k5cag_iso_type_400);
		    break;
		case CAMEAR_ISO_TYPE_800:
		    CDBG("--CAMERA--CAMEAR_ISO_TYPE_800\n");
		    rc = S5K5CAG_WriteTABLE(s5k5cag_iso_type_800);
		    break;
		case CAMERA_ISO_TYPE_1600:
		    CDBG("--CAMERA--CAMERA_ISO_TYPE_1600\n");
		    rc = S5K5CAG_WriteTABLE(s5k5cag_iso_type_1600);
		    break;
		
		default:
		    CDBG_HIGH("--CAMERA--ERROR ISO TYPE\n");
		    break;
    }
    return rc;
}

static int s5k5cag_auto_led_start(void)
{
    int rc = 0;

#if 0 // #9 ES03 don't flash led	
    unsigned int tmp;

    CDBG("--CAMERA-- %s (Start...)\n",__func__);
    if (is_autoflash == 1) {
	   	s5k5cag_i2c_write(s5k5cag_client->addr,0x002c,0x7000,10);
		s5k5cag_i2c_write(s5k5cag_client->addr,0x002e ,0x2448 ,10);
		s5k5cag_i2c_read(s5k5cag_client->addr, 0x0f12, &tmp);
		CDBG("--CAMERA-- NormBr :%d\n",tmp);
		if(tmp > 100) {
	           //s5k5cag_set_flash_light(MSM_CAMERA_LED_OFF);
	      	} else {
	         s5k5cag_set_flash_light(MSM_CAMERA_LED_HIGH);
		  is_autoflash_working = 1;
		}	
    }
#endif

    return rc;
}

static int s5k5cag_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long  rc = 0;
	if (copy_from_user(&cdata,(void *)argp,sizeof(struct sensor_cfg_data))) 
		return -EFAULT;

	CDBG( "--CAMERA-- %s %d\n",__func__,cdata.cfgtype);
		
	mutex_lock(&s5k5cag_mutex);
	switch (cdata.cfgtype)
	{
	case CFG_SET_MODE:   // 0
		rc = s5k5cag_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_SET_EFFECT: // 1
		CDBG( "--CAMERA-- CFG_SET_EFFECT mode=%d, effect = %d !!\n",cdata.mode, cdata.cfg.effect);
		rc = s5k5cag_set_effect(cdata.mode, cdata.cfg.effect);
		break;
	case CFG_START:      // 2
		CDBG( "--CAMERA-- CFG_START (Not Support) !!\n");
		// Not Support
		break;
	case CFG_PWR_UP:     // 3
		CDBG( "--CAMERA-- CFG_PWR_UP (Not Support) !!\n");
		// Not Support
		break;
	case CFG_PWR_DOWN:   // 4
		CDBG( "--CAMERA-- CFG_PWR_DOWN (Not Support) \n");
		break;
	case CFG_SET_DEFAULT_FOCUS:  // 06
		CDBG( "--CAMERA-- CFG_SET_DEFAULT_FOCUS (Not Implement) !!\n");
		break;		
	case CFG_MOVE_FOCUS:     //  07
		CDBG( "--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
		break;
	 case CFG_SET_WB:
		s5k5cag_set_wb_oem(cdata.cfg.wb_val);
		rc = 0 ;
		break;
	 case CFG_SET_ANTIBANDING:     //  17
		rc = s5k5cag_antibanding(cdata.cfg.antibanding);
		break;
	 case CFG_SET_BRIGHTNESS:     //  12
		rc = s5k5cag_set_brightness(cdata.cfg.brightness);
		break;            
	case CFG_SET_AUTOFLASH:
		CDBG( "--CAMERA-- CFG_SET_AUTO_FLASH !\n");
		is_autoflash = cdata.cfg.is_autoflash;
		rc = 0;
		break;
	case CFG_SET_EXPOSURE_COMPENSATION:
		rc = s5k5cag_set_brightness(cdata.cfg.brightness);
		break;
	case CFG_SET_CONTRAST:
		rc = s5k5cag_set_contrast(cdata.cfg.contrast);
		break;
	case CFG_SET_SATURATION:
	    CDBG("--CAMERA-- CFG_SET_SATURATION !!\n");
	    rc = s5k5cag_set_saturation(cdata.cfg.saturation);
	    break;
	 case CFG_SET_SHARPNESS:
	    CDBG("--CAMERA-- CFG_SET_SHARPNESS !!\n");
	    rc = s5k5cag_set_sharpness(cdata.cfg.sharpness);
	    break;
	 case CFG_SET_ISO:
	    CDBG("--CAMERA-- CFG_SET_ISO !\n");
	    rc = s5k5cag_set_iso(cdata.cfg.iso_type);
	    break;
	 case CFG_SET_AUTO_LED_START:
	    rc = s5k5cag_auto_led_start();
	    break;
	default:
		CDBG_HIGH( "--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
		rc = -EINVAL;
		break;	
	}
	mutex_unlock(&s5k5cag_mutex);

	CDBG( "%s, rc: %d\n",__func__, ((int)rc));
	
	return rc;	
}

static int s5k5cag_init_client(struct i2c_client *client)
{
   /* Initialize the MSM_CAMI2C Chip */
   init_waitqueue_head(&s5k5cag_wait_queue);
   return 0;
}

static int s5k5cag_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	CDBG( "--CAMERA-- %s ... (Start...)\n",__func__);
   
   	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
   	{
      	CDBG_HIGH( "--CAMERA--i2c_check_functionality failed\n");
      	return -ENXIO;
   	}
   
   	s5k5cag_sensorw = kzalloc(sizeof(struct s5k5cag_work), GFP_KERNEL);
   	if (!s5k5cag_sensorw)
   	{
      	CDBG_HIGH( "--CAMERA--kzalloc failed\n");
      	return -ENOMEM;
   	}
   
   	i2c_set_clientdata(client, s5k5cag_sensorw);
   
   	s5k5cag_init_client(client);
   	s5k5cag_client = client;
     
   	return 0;
}

static int s5k5cag_i2c_remove(struct i2c_client *client)
{
	if(NULL != s5k5cag_sensorw){
		kfree(s5k5cag_sensorw);
		s5k5cag_sensorw = NULL;
	}
	s5k5cag_client = NULL;
	return 0;
}

static const struct i2c_device_id s5k5cag_i2c_id[] = {
	{"s5k5ca", 0},
	{}
};

static struct i2c_driver s5k5cag_i2c_driver = {
	.id_table = s5k5cag_i2c_id,
	.probe  = s5k5cag_i2c_probe,
	.remove = s5k5cag_i2c_remove,
	.driver = {
		.name = "s5k5ca",
	},
};

static int s5k5cag_sensor_probe(const struct msm_camera_sensor_info *info,struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;
  	CDBG( "--CAMERA-- %s (Start...)\n",__func__);

	rc = i2c_add_driver(&s5k5cag_i2c_driver);
  	if ((rc < 0 ) || (s5k5cag_client == NULL)){
  		printk(KERN_ERR "%s:  i2c_add_driver failed! rc=%d\n",__func__, rc);
   		return rc;
  	}
  
  	rc = s5k5cag_probe_init_sensor(info);
  	if (rc < 0){
  		printk(KERN_ERR "%s:  s5k5cag_probe_init_sensor failed! rc=%d\n",__func__, rc);
  		i2c_del_driver(&s5k5cag_i2c_driver);
  		return rc;
  	}
  	s->s_init = s5k5cag_sensor_open_init;
  	s->s_release = s5k5cag_sensor_release;
  	s->s_config  = s5k5cag_sensor_config;
  	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle  =  info->sensor_platform_info->mount_angle;
  	return rc;
}

static int __s5k5cag_probe(struct platform_device *pdev)
{
   return msm_camera_drv_start(pdev, s5k5cag_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __s5k5cag_probe,
    .driver = {
        .name = "msm_camera_s5k5ca",
        .owner = THIS_MODULE,
    },
};

static int __init s5k5cag_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k5cag_init);

MODULE_DESCRIPTION("S5K5CA 3M YUV MIPI sensor driver");
