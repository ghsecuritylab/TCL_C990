/* drivers/input/keyboard/mcs6024_ts.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>


#define MCS6024_I2C_NAME		"mcs6024"

#define GPIO_TOUCH_PANEL_INT	48
#define GPIO_PANEL_ID			35


static struct workqueue_struct *mcs6024_wq;

struct mcs6024_ts_data {
	/* uint16_t addr; */
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int (*power)(int on);
    struct early_suspend early_suspend;	//add by Joe 120508
};
//add by Joe 120508
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mcs6024_ts_early_suspend(struct early_suspend *h);
static void mcs6024_ts_late_resume(struct early_suspend *h);
#endif
//add end by Joe 120508
/* virtual key map */ //add by Joe 120508
#if 1

static ssize_t es01_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	/*X: home (10~80), search(120~190), back(230~310)
	    Y: 510~570
	*/
	return sprintf(buf,
	   __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":45:540:70:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":155:540:70:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":270:540:80:60"
	   "\n");


}
static struct kobj_attribute es01_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.mcs6024-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &es01_virtual_keys_show,
};
static struct attribute *es01_properties_attrs[] = {
	&es01_virtual_keys_attr.attr,
	NULL
};
static struct attribute_group es01_properties_attr_group = {
	.attrs = es01_properties_attrs,
};
#endif
int32_t mcs6024_i2c_txdata(struct mcs6024_ts_data *ts, unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer(ts->client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "mcs6024_i2c_txdata faild\n");
		return -EIO;
	}

	return 0;
}

static int mcs6024_init_panel(struct mcs6024_ts_data *ts)
{
	return 0;
}

static void mcs6024_ts_work_func(struct work_struct *work_param)
{
	struct mcs6024_ts_data *ts = container_of(work_param, struct mcs6024_ts_data, work);

	int		ret;

	struct	i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[13];
	int		buf_len = 13;

	int		x1, y1;
	int		x2, y2;
	int		keyIndex, screenStatus;
	int		key[3]			= {KEY_MENU, KEY_SEARCH, KEY_BACK};
	int		keyStatusNow	= 0;
//	static int	keyStatusOld	= 0;
	int		screenStatusNow[2]={0, 0};

	int		screenNum		=0;
	uint8_t	finger2_pressed = 0;

	x1 = y1 = 0;
	x2 = y2 = 0;

	// send msg
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x10;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = buf_len;
	msg[1].buf = buf;
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_INFO "ts, rd i2c fd!\n");
		goto __Done;
	}

	//data
	x1 = (buf[1] << 8 | buf[2])  & 0x3ff;
	y1 = (buf[3] << 8 | buf[4])  & 0x3ff;
	x2 = (buf[9] << 8 | buf[10]) & 0x3ff;
	y2 = (buf[11]<< 8 | buf[12]) & 0x3ff;

	screenStatus= (buf[0])>>0 & 0x3;
	keyStatusNow= (buf[8])>>0 & 0x3;
	keyIndex	= (buf[8])>>2 & 0x3;
	switch(screenStatus)
	{
		case 0x0:	screenStatusNow[0]=0;	screenStatusNow[1]=0;	screenNum=0;	break;
		case 0x1:	screenStatusNow[0]=1;	screenStatusNow[1]=0;	screenNum=1;	break;
		case 0x2:	screenStatusNow[0]=0;	screenStatusNow[1]=1;	screenNum=1;	break;
		case 0x3:	screenStatusNow[0]=1;	screenStatusNow[1]=1;	screenNum=2;	break;
	}

//	printk("	x1:%d	y1:%d	x2:%d	y2:%d	screenStatusNow:%d	keyStatusNow:%d	keyIndex:%d	\n",x1, y1, x2, y2, screenStatusNow[0], keyStatusNow, keyIndex);
	// report key
	input_report_key(ts->input_dev, key[keyIndex], keyStatusNow);
	if(screenStatusNow[0]){
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x1);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y1);
	}
	input_report_key(ts->input_dev, BTN_TOUCH, screenNum);
	input_mt_sync(ts->input_dev);

	// report screenStatus
	if (screenStatusNow[1]) {
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x2);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y2);
		input_report_key(ts->input_dev, BTN_TOUCH, finger2_pressed);
		input_mt_sync(ts->input_dev);
	}

	input_sync(ts->input_dev);
__Done:
	//enable_irq(ts->client->irq);//del by Joe 120507
	return;
}


static irqreturn_t mcs6024_ts_irq_handler(int irq, void *dev_id)
{
	struct mcs6024_ts_data *ts = dev_id;

	//disable_irq_nosync(ts->client->irq);//del by Joe 120507

	schedule_work(&ts->work); /* ? */
	return IRQ_HANDLED;
}

static int mcs6024_addr_autoTest(struct mcs6024_ts_data *ts)
{
	struct	i2c_msg msg[2];
	uint8_t startReg;
	uint8_t buf[1];
	int		bufLen	=1;
	int		num		=20;
	int		ret		=0;
	mdelay(25);//add by Joe 120508
//modify by Joe 120508
	msg[0].addr		= ts->client->addr;
	msg[0].flags	= 0;
	msg[0].len		= 1;
	msg[0].buf		= &startReg;
	startReg		= 0x10;
	msg[1].addr		= ts->client->addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].len		= bufLen;
	msg[1].buf		= buf;
	while(num){

		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if(ret>0){
			printk(KERN_ERR "mcs6024_addr_autoTest== Num = %d,ret = %d,reg = %x",num,ret,ts->client->addr);
			break;
		}
		else{
			printk(KERN_INFO "%s:%d rd i2c fd!\n",__func__,  num);
			mdelay(5);
		}		
		num--;
        
	}
	return ret;
}

static int mcs6024_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mcs6024_ts_data *ts;
	int ret = 0;
	/* struct mcs6024_ts_suspend *pdata; */
	unsigned long irqflags;
    printk(KERN_ERR "%s",__func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "mcs6024_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}


	INIT_WORK(&ts->work, mcs6024_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);


	//ID
	ret	=gpio_request(GPIO_PANEL_ID, "touch screen wake");
	if(ret) {
		printk(KERN_ERR "mcs6024_ts_probe: gpio_id request error\n");
		goto err_request_gpio_id;
	}
	ret =gpio_tlmm_config(GPIO_CFG(GPIO_PANEL_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_2MA),	GPIO_CFG_ENABLE);
	if(ret) {
		printk(KERN_ERR "mcs6024_ts_probe: gpio_id config error\n");
		goto err_config_gpio_id;
	}
	ret = gpio_direction_output(GPIO_PANEL_ID, 1);
	if(ret) {
		printk(KERN_ERR "mcs6024_ts_probe: gpio_id output error\n");
		goto err_output_gpio_id;
	}

    msleep(130);//add by Joe to modify loglevel = 1 ,8
	// panel
	ret = mcs6024_init_panel(ts);
	if (ret < 0) {
		printk(KERN_ERR "mcs6024_ts_probe: init_panel error\n");
		goto err_detect_failed;
	}
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "mcs6024_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "mcs6024-touchscreen";


	set_bit(EV_SYN,		ts->input_dev->evbit);
	set_bit(EV_KEY,		ts->input_dev->evbit);
	set_bit(EV_ABS,		ts->input_dev->evbit);
	set_bit(BTN_TOUCH,	ts->input_dev->keybit);
	//set_bit(KEY_MENU,	ts->input_dev->keybit);//del by Joe 120508
	//set_bit(KEY_SEARCH, ts->input_dev->keybit);//del by Joe 120508
	//set_bit(KEY_BACK,	ts->input_dev->keybit);//del by Joe 120508

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 320 /*319*/, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 480 /* 479 */, 0, 0);

	input_set_capability(ts->input_dev, EV_KEY, KEY_MENU);//add by Joe 120508
	input_set_capability(ts->input_dev, EV_KEY, KEY_SEARCH);//add by Joe 120508
	input_set_capability(ts->input_dev, EV_KEY, KEY_BACK);//add by Joe 120508

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "mcs6024_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#if 1 /* for virtual keypad */ //add by Joe 120508
	{
		struct kobject *properties_kobj;
		properties_kobj = kobject_create_and_add("board_properties", NULL);
		if (properties_kobj) {
			ret = sysfs_create_group(properties_kobj, &es01_properties_attr_group);
		}
		if (!properties_kobj || ret) {
			printk(KERN_INFO "failed to create board_properties\n");
		}
	}
#endif




	// panel int
	ret	= gpio_request(GPIO_TOUCH_PANEL_INT, "touch screen irq");
	if(ret) {
		printk(KERN_ERR "mcs6024_ts_probe: gpio_int error\n");
		goto err_request_gpio_panel_int;
	}
	ret = gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_PANEL_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(ret) {
		printk(KERN_ERR "mcs6024_ts_probe: gpio_int config\n");
		goto err_config_gpio_panel_int;
	}
	client->irq = MSM_GPIO_TO_INT(GPIO_TOUCH_PANEL_INT);
	ret	= mcs6024_addr_autoTest(ts);
	if(ret<0)
	{
		printk("proboe mcs6024 ret: autoTest failed. \n" );
		goto err_autoTest_failed;
	}


	irqflags = /* IRQF_TRIGGER_LOW |*/IRQF_TRIGGER_FALLING | IRQF_DISABLED ;
	ret = request_irq(client->irq, mcs6024_ts_irq_handler, irqflags, client->name, ts);
	if (ret == 0) {
		/* if panel IC need to enable by send a command, do it here */
	} else {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq;
	}

	printk(KERN_INFO "mcs6024_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, "interrupt");
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = mcs6024_ts_early_suspend;
	ts->early_suspend.resume = mcs6024_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	return 0;

err_request_irq:
err_autoTest_failed:
	gpio_free(GPIO_TOUCH_PANEL_INT);
err_config_gpio_panel_int:
err_request_gpio_panel_int:

	input_unregister_device(ts->input_dev);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_detect_failed:
err_output_gpio_id:
err_config_gpio_id:
	gpio_free(GPIO_PANEL_ID);
err_request_gpio_id:
	kfree(ts);
err_alloc_data_failed:

err_check_functionality_failed:
	return ret;
}


static int mcs6024_ts_remove(struct i2c_client *client)
{
	struct mcs6024_ts_data *ts = i2c_get_clientdata(client);

//add by Joe 120508
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(client->irq, ts);

	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

//modify by Joe 120508
static void mcs6024_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
    struct mcs6024_ts_data *ts = i2c_get_clientdata(client);
    disable_irq(client->irq);
	ret = cancel_work_sync(&ts->work);
	if (ret) { /* if work was pending disable-count is now 2 */
		//enable_irq(client->irq);
	}

	printk(KERN_INFO"mcs6024_ts_suspend\n");
	ret = gpio_tlmm_config(GPIO_CFG(GPIO_PANEL_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_2MA),	GPIO_CFG_ENABLE);
	if(ret) {
		//printk(KERN_ERR,"gpio_tlmm_config error in suspend\n");
	}
	ret = gpio_direction_output(GPIO_PANEL_ID, 0);
	if(ret) {
		//printk(KERN_ERR,"error gpiio direction output setting \n");
	}

	return;
}

//modify by Joe 120508
static void mcs6024_ts_resume(struct i2c_client *client)
{//waiting for the power supply and go on adding code for power consume.just waiting....
	//uint32_t ret = 0 ;
    printk(KERN_ERR "%s",__func__);
	// wake the driver ic.
	gpio_tlmm_config(GPIO_CFG(GPIO_PANEL_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	

	gpio_set_value(GPIO_PANEL_ID, 1);
	//gpio_set_value(GPIO_PANEL_ID, 0);
	//msleep(10);
	//gpio_set_value(GPIO_PANEL_ID, 1);

	msleep(130);//350
	enable_irq(client->irq);
	return;
}

////add by Joe 120508
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mcs6024_ts_early_suspend(struct early_suspend *h)
{
	struct mcs6024_ts_data *ts;
	ts = container_of(h, struct mcs6024_ts_data, early_suspend);
	mcs6024_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void mcs6024_ts_late_resume(struct early_suspend *h)
{
	struct mcs6024_ts_data *ts;
	ts = container_of(h, struct mcs6024_ts_data, early_suspend);
	mcs6024_ts_resume(ts->client);
}
#endif
static const struct i2c_device_id mcs6024_ts_id[] = {
	{ MCS6024_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver mcs6024_ts_driver = {
	.probe		= mcs6024_ts_probe,
	.remove		= mcs6024_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= mcs6024_ts_suspend,
	.resume		= mcs6024_ts_resume,
#endif	////add by Joe 120508
	.id_table	= mcs6024_ts_id,
	.driver = {
		.name	= MCS6024_I2C_NAME,
	},
};



static int __devinit mcs6024_ts_init(void)
{
	uint32_t ret =0;
    printk(KERN_ERR "%s",__func__);
	mcs6024_wq = create_singlethread_workqueue("mcs6024_wq");
	if (!mcs6024_wq)
		return -ENOMEM;
	ret = i2c_add_driver(&mcs6024_ts_driver);

	return ret;
}


static void __exit mcs6024_ts_exit(void)
{
	i2c_del_driver(&mcs6024_ts_driver);
	if (mcs6024_wq)
		destroy_workqueue(mcs6024_wq);

}

module_init(mcs6024_ts_init);
module_exit(mcs6024_ts_exit);

MODULE_DESCRIPTION("mcs6024 Touchscreen Driver");
MODULE_LICENSE("GPL");
