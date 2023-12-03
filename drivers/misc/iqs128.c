//--------------- Header files ------------------//
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/interrupt.h>
#include <linux/input-polldev.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>

#include <mach/mt_gpio.h>                  
#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>
#include <mach/eint.h>

//-------------- Define ---------------//
#define DEBUG 0
#define IF_I2C_DEVICE 0
#define LOG_TAG "iqs128"
#define SENSOR_NAME "iqs128"
#define CAP_SENSOR_GPIO EINT7_CAP1
#define DOCK_ID 0
#define DOCK_ON 1
#define DOCK_OFF 0
#define delay_time 20
#define SPEAKRE_RESET 0

//-------------- Define Sensor Data Struct ---------------//
struct sensor_data {
    struct switch_dev sdev;
    struct switch_dev sdev2;
    struct delayed_work work;
    struct delayed_work work2;
    struct delayed_work speaker_work;
    struct device dev;
    struct platform_device *pdev;	
    unsigned gpio;
    atomic_t irq1;
    atomic_t irq2;
};

//-------------- Global para. --------------//
static struct sensor_data *this_data = NULL;
static struct platform_device *sensor_pdev = NULL;
static struct platform_driver capsensor_driver;
static struct miscdevice cap_misc_dev;
static struct mutex speaker_mutex;
static int last_state = 1, last_state2 = 1;
static bool phone_call = false;
static bool cap_power = false;
static bool airplane_mode = false;

//-------------- Global functions ---------------//
static int sensor_config_power(bool enable)
{
	int res;
	printk("[cap_sensor][iqs128]>>>cap_power=%d\n", enable);
	if(enable) {
		if(!airplane_mode) 
			res = hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "CAP_SEN");
		else
			return res;
	}
	else
		res = hwPowerDown(MT6323_POWER_LDO_VGP1, "CAP_SEN");
	cap_power = enable;
	return res;
}

static int sensor_config_eint(int eint, bool enable)
{
	int res = 0;
	printk("[cap_config_eint][iqs128]>>>eint=%d, enable=%d\n", eint, enable);
	if(enable) {
		if(!airplane_mode) {
		if(eint == CUST_EINT_EINT7_CAP1_NUM) {
			if(atomic_read(&this_data->irq1)) 
				return res;
			atomic_set(&this_data->irq1, 1);
		}
#ifdef _LVP9_
		else {
			if(atomic_read(&this_data->irq2)) 
				return res;
			atomic_set(&this_data->irq2, 1);
		}
#endif		
		mt_eint_unmask(eint);
		}
	}
	else {
		if(eint == CUST_EINT_EINT7_CAP1_NUM) {
			if(!atomic_read(&this_data->irq1)) 
				return res;
			atomic_set(&this_data->irq1, 0);
		}
#ifdef _LVP9_
		else {
			if(!atomic_read(&this_data->irq2)) 
				return res;
			atomic_set(&this_data->irq2, 0);
		}
#endif		
		mt_eint_mask(eint); 
	}
	printk("[cap_config_eint][iqs128]>>>irq1=%d, irq2=%d\n", atomic_read(&this_data->irq1), atomic_read(&this_data->irq2));
	return res;
}

static int sensor_read_gpio_data(unsigned gpio_number, const char* label)
{
	int gpio_data = 0;
	gpio_data = mt_get_gpio_in(gpio_number);
#if DEBUG
		printk("[cap_sensor][iqs128]>>>gpio_data[%d]=%d\n", gpio_number, gpio_data);
#endif

	return gpio_data;	
}

#if SPEAKRE_RESET
static int read_speaker_gpio_data(unsigned gpio_number, const char* label)
{
	int gpio_data = 0;
	gpio_data = mt_get_gpio_out(gpio_number);
#if DEBUG
		printk("[cap_sensor][iqs128]>>>speaker_gpio_data[%d]=%d\n", gpio_number, gpio_data);
#endif

	return gpio_data;	
}
#endif
//-------------- irq func ---------------//
static void gpio_irq_handler(void)
{
#if DEBUG
	printk("[cap_sensor][iqs128]>>>gpio_irq_handler\n");
#endif
	schedule_delayed_work(&this_data->work, 0);
    //mt_eint_unmask(CUST_EINT_EINT7_CAP1_NUM);  
	sensor_config_eint(CUST_EINT_EINT7_CAP1_NUM, 1);

}

#ifdef _LVP9_
static void switch_work_func2(struct work_struct *data);

static void gpio_irq_handler2(void)
{
#if DEBUG
	printk("[cap_sensor][iqs128]>>>gpio_irq_handler2\n");
#endif
	schedule_delayed_work(&this_data->work2, 0);
    //mt_eint_unmask(CUST_EINT_EINT6_CAP2_NUM);  
	sensor_config_eint(CUST_EINT_EINT6_CAP2_NUM, 1);
}
#endif

//-------------- Work func ---------------//
static void switch_work_func(struct work_struct *data)
{
	int state;
	char *STATE[2]   = { NULL, NULL };
	//struct sensor_data *dev = container_of(data, struct sensor_data, work.work);
	
	state = sensor_read_gpio_data(EINT7_CAP1, SENSOR_NAME);
#if DEBUG
	printk("[cap_sensor][iqs128]>>>switch_work_func state=%d\n", state);
#endif
	if(last_state != state) {
	if(state == 1) {
		switch_set_state(&this_data->sdev, DOCK_OFF);
		STATE[0]   = "CAP_STATE=UNTOUCHED";
		mt_eint_set_polarity(CUST_EINT_EINT7_CAP1_NUM, 0);
	}
	else if(state == 0) {
		switch_set_state(&this_data->sdev, DOCK_ON);
		STATE[0]   = "CAP_STATE=TOUCHED";
		mt_eint_set_polarity(CUST_EINT_EINT7_CAP1_NUM, 1);
	}

#ifdef _LVP9_
	if(!switch_get_state(&this_data->sdev2))
#endif
		kobject_uevent_env(&cap_misc_dev.this_device->kobj, KOBJ_CHANGE, STATE);
#if DEBUG
	printk("[cap_sensor][iqs128]>>>uevent state=%s\n", STATE[0]);
#endif
	}
	
	last_state = state;

	//schedule_delayed_work(&this_data->work, delay_time);
}

#ifdef _LVP9_
static void switch_work_func2(struct work_struct *data)
{
	int state;
	char *STATE[2]   = { NULL, NULL };
	//struct sensor_data *dev = container_of(data, struct sensor_data, work.work);
	
	state = sensor_read_gpio_data(EINT6_CAP2, SENSOR_NAME);
#if DEBUG
	printk("[cap_sensor][iqs128]>>>switch_work_func2 state=%d\n", state);
#endif
	if(last_state2 != state) {
	if(state == 1) {
		switch_set_state(&this_data->sdev2, DOCK_OFF);
		STATE[0]   = "CAP_STATE=UNTOUCHED";
		mt_eint_set_polarity(CUST_EINT_EINT6_CAP2_NUM, 0);
	}
	else if(state == 0) {
		switch_set_state(&this_data->sdev2, DOCK_ON);
		STATE[0]   = "CAP_STATE=TOUCHED";
		mt_eint_set_polarity(CUST_EINT_EINT6_CAP2_NUM, 1);
	}

	if(!switch_get_state(&this_data->sdev))
		kobject_uevent_env(&cap_misc_dev.this_device->kobj, KOBJ_CHANGE, STATE);
#if DEBUG
		printk("[cap_sensor][iqs128]>>>uevent state=%s\n", STATE[0]);
#endif
	}
	
	last_state2 = state;

	//schedule_delayed_work(&this_data->work2, delay_time);
}
#endif

#if SPEAKRE_RESET
static void detect_speaker_work_func(struct work_struct *data)
{
	int state;
	static int counter = 0, had_reset = 0;
	
	state = read_speaker_gpio_data(GPIO_SPEAKER_EN_PIN, NULL);
#if DEBUG
	printk("[cap_sensor][iqs128]>>>detect_speaker_work_func state=%d, counter=%d\n", state, counter);
#endif
	if(state == 1 && had_reset == 0) {
		if(counter == 10) {
			mutex_lock(&speaker_mutex);
			sensor_config_power(false);
			msleep(30);
			sensor_config_power(true);
			msleep(30);
			mutex_unlock(&speaker_mutex);
			counter = 0;
			had_reset = 1;
		}
		else
			counter++;
	}
	else if(state == 0) {
		counter = 0;
		had_reset = 0;
	}
	
	schedule_delayed_work(&this_data->speaker_work, delay_time);
}
#endif

static ssize_t cap_show_active(struct device *dev, struct device_attribute *attr, char *buf)
{
	int state;

	state = switch_get_state(&this_data->sdev);
	return sprintf(buf, "%d\n", state);
}

#ifdef _LVP9_
static ssize_t cap_show_active2(struct device *dev, struct device_attribute *attr, char *buf)
{
	int state;

	state = switch_get_state(&this_data->sdev2);
	return sprintf(buf, "%d\n", state);
}
#endif

static ssize_t cap_store_reset(struct device_driver *ddri, const char *buf, size_t count)
{
        printk("%s:buf=%x %x\n",__func__,buf[0],buf[1]);

        if(buf[0] == '1') {
		mutex_lock(&speaker_mutex);
		printk("%s reset cap sensor\n",__func__);
		sensor_config_power(false);
		msleep(30);
		sensor_config_power(true);
		msleep(30);
		mutex_unlock(&speaker_mutex);
	}
        return 1;
}

static ssize_t cap_store_power(struct device_driver *ddri, const char *buf, size_t count)
{
		int res;
        printk("%s:buf=%x %x\n",__func__,buf[0],buf[1]);

		mutex_lock(&speaker_mutex);
        if(buf[0] == '0') {
			res = sensor_config_power(false);
        }
		else if(buf[0] == '1') {
			res = sensor_config_power(true);
		}
		mutex_unlock(&speaker_mutex);
		printk("%s:res=%d\n",__func__, res);
		msleep(100);
	
        return 1;
}

static ssize_t cap_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", phone_call);
}

static ssize_t cap_store_enable(struct device_driver *ddri, const char *buf, size_t count)
{
		int res, state, state2=1;
		char *STATE[2]	 = { NULL, NULL };
        printk("[iqs128]%s:buf=%x %x\n",__func__,buf[0],buf[1]);

		mutex_lock(&speaker_mutex);
		if(!airplane_mode && !phone_call && buf[0] == '1') {
			phone_call = true;
			msleep(100);
        	state = sensor_read_gpio_data(EINT7_CAP1, SENSOR_NAME);
			last_state = state;
			switch_set_state(&this_data->sdev, (!state)? DOCK_ON : DOCK_OFF);
#ifdef _LVP9_
			state2 = sensor_read_gpio_data(EINT6_CAP2, SENSOR_NAME);
			last_state2 = state2;
			switch_set_state(&this_data->sdev2, (!state2)? DOCK_ON : DOCK_OFF);
			STATE[0]   = (!state || !state2)? "CAP_STATE=TOUCHED" : "CAP_STATE=UNTOUCHED";
			
			printk("[iqs128]STATE[0]=%s, state= %d, state2= %d\n",STATE[0],state,state2);
#else
			STATE[0]   = (!state)? "CAP_STATE=TOUCHED" : "CAP_STATE=UNTOUCHED";

			printk("[iqs128]STATE[0]=%s, state= %d\n",STATE[0],state);
#endif

			if(state)
				mt_eint_set_polarity(CUST_EINT_EINT7_CAP1_NUM, 0);
			else
				mt_eint_set_polarity(CUST_EINT_EINT7_CAP1_NUM, 1);
#ifdef _LVP9_
			if(state2)
				mt_eint_set_polarity(CUST_EINT_EINT6_CAP2_NUM, 0);
			else
				mt_eint_set_polarity(CUST_EINT_EINT6_CAP2_NUM, 1);
			//mt_eint_unmask(CUST_EINT_EINT6_CAP2_NUM);  
			sensor_config_eint(CUST_EINT_EINT6_CAP2_NUM, 1);
#endif
			//mt_eint_unmask(CUST_EINT_EINT7_CAP1_NUM); 
			sensor_config_eint(CUST_EINT_EINT7_CAP1_NUM, 1);
			kobject_uevent_env(&cap_misc_dev.this_device->kobj, KOBJ_CHANGE, STATE);
			//schedule_delayed_work(&this_data->work, 0);
			//schedule_delayed_work(&this_data->work2, 0);

		}
		else if(phone_call && buf[0] == '0'){
			//mt_eint_mask(CUST_EINT_EINT7_CAP1_NUM); 
			sensor_config_eint(CUST_EINT_EINT7_CAP1_NUM, 0);
#ifdef _LVP9_
			//mt_eint_mask(CUST_EINT_EINT6_CAP2_NUM);
			sensor_config_eint(CUST_EINT_EINT6_CAP2_NUM, 0);
#endif
			//cancel_delayed_work_sync(&this_data->work);
			//cancel_delayed_work_sync(&this_data->work2);
			phone_call = false;
		}
		else if(!airplane_mode && buf[0] == '3'){
			airplane_mode = true;
			phone_call = false;
 			//mt_eint_mask(CUST_EINT_EINT7_CAP1_NUM); 
			sensor_config_eint(CUST_EINT_EINT7_CAP1_NUM, 0);
#ifdef _LVP9_
			//mt_eint_mask(CUST_EINT_EINT6_CAP2_NUM);
			sensor_config_eint(CUST_EINT_EINT6_CAP2_NUM, 0);
#endif
			sensor_config_power(false);
		}
		else if(airplane_mode && buf[0] == '4'){
			airplane_mode = false;
			/*if(phone_call) {
 			sensor_config_eint(CUST_EINT_EINT7_CAP1_NUM, 1);
#ifdef _LVP9_
			sensor_config_eint(CUST_EINT_EINT6_CAP2_NUM, 1);
#endif
			}*/
			sensor_config_power(true);
		}
		mutex_unlock(&speaker_mutex);
        return 1;
}

static DRIVER_ATTR(active, 0644, cap_show_active, NULL);
#ifdef _LVP9_
static DRIVER_ATTR(active2, 0644, cap_show_active2, NULL);
#endif
static DRIVER_ATTR(reset, 0644, NULL, cap_store_reset);
static DRIVER_ATTR(power, 0664, NULL, cap_store_power);
static DRIVER_ATTR(enable,0664, cap_show_enable, cap_store_enable);

static struct driver_attribute *sensor_attr_list[] = {
    &driver_attr_active,
#ifdef _LVP9_
	&driver_attr_active2,
#endif
    &driver_attr_reset,
	&driver_attr_power,
	&driver_attr_enable,
};

static struct miscdevice cap_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "iqs128_dev",
	//.fops = &AKECS_fops,
};

//-------------- Probe ---------------//
static int sensor_probe(struct platform_device *pdev)
{
	int rt, idx;
	struct sensor_data *data = NULL;
	printk("****CAP sensor_probe\n");
	sensor_config_power(true);

    mt_set_gpio_mode(EINT7_CAP1, EINT7_CAP1_M_EINT);
    //mt_set_gpio_dir(EINT7_CAP1, GPIO_DIR_IN);
    //mt_set_gpio_pull_enable(EINT7_CAP1, true);
    //mt_set_gpio_pull_select(EINT7_CAP1, GPIO_PULL_UP);
	
    //mt_eint_set_sens(CUST_EINT_HALL_1_NUM, CUST_EINT_HALL_1_SENSITIVE);
	mt_eint_set_polarity(CUST_EINT_EINT7_CAP1_NUM, 0);
	mt_eint_set_hw_debounce(CUST_EINT_EINT7_CAP1_NUM, CUST_EINT_EINT7_CAP1_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_EINT7_CAP1_NUM, CUST_EINT_EINT7_CAP1_TYPE, gpio_irq_handler, 1);
	mt_eint_mask(CUST_EINT_EINT7_CAP1_NUM);  
#ifdef _LVP9_
    mt_set_gpio_mode(EINT6_CAP2, EINT6_CAP2_M_EINT);
    //mt_set_gpio_dir(EINT6_CAP2, GPIO_DIR_IN);
    //mt_set_gpio_pull_enable(EINT6_CAP2, true);
    //mt_set_gpio_pull_select(EINT6_CAP2, GPIO_PULL_UP);
	
    //mt_eint_set_sens(CUST_EINT_HALL_1_NUM, CUST_EINT_HALL_1_SENSITIVE);
	mt_eint_set_polarity(CUST_EINT_EINT6_CAP2_NUM, 0);
	mt_eint_set_hw_debounce(CUST_EINT_EINT6_CAP2_NUM, CUST_EINT_EINT6_CAP2_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_EINT6_CAP2_NUM, CUST_EINT_EINT6_CAP2_TYPE, gpio_irq_handler2, 1);
	mt_eint_mask(CUST_EINT_EINT6_CAP2_NUM);  
#endif
#if SPEAKRE_RESET
    //mt_set_gpio_mode(GPIO_SPEAKER_EN_PIN, GPIO_MODE_00);
    //mt_set_gpio_dir(GPIO_SPEAKER_EN_PIN, GPIO_DIR_OUT);
#endif
	//switch device registing	
	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
        	rt = -ENOMEM;
        	printk("[iqs128]sensor_probe>>>kzalloc fail\n");
		return rt;
    	}

	data->sdev.name = SENSOR_NAME;
#ifdef _LVP9_
	data->sdev2.name = "iqs128_2";
#endif
	data->gpio = CAP_SENSOR_GPIO;	

	rt = switch_dev_register(&data->sdev);
	if (rt != 0)
		printk("[iqs128]sensor_probe>>>switch_dev_register fail\n");
	
	INIT_DELAYED_WORK(&data->work, switch_work_func);
	
#ifdef _LVP9_
	rt = switch_dev_register(&data->sdev2);
	if (rt != 0)
		printk("[iqs128]sensor_probe>>>switch_dev_register sdev2 fail\n");
	
	INIT_DELAYED_WORK(&data->work2, switch_work_func2);
#endif

#if SPEAKRE_RESET
	//INIT_DELAYED_WORK(&data->speaker_work, detect_speaker_work_func);
#endif
	this_data = data;
	atomic_set(&this_data->irq1, 0);
	atomic_set(&this_data->irq2, 0);

	int num = (int)(sizeof(sensor_attr_list)/sizeof(sensor_attr_list[0]));
	for(idx = 0; idx < num; idx++)
	{
		if(rt = driver_create_file(&capsensor_driver.driver, sensor_attr_list[idx]))
		{            
			printk("driver_create_file (%s) = %d\n", sensor_attr_list[idx]->attr.name, rt);
			break;
		}
	}    

	rt = misc_register(&cap_misc_dev);
	if (rt) {
		pr_err("%s: misc_register failed\n", __func__);
	}
	
	mutex_init(&speaker_mutex);
#if SPEAKRE_RESET
	//detect_speaker_work_func(&data->speaker_work);
#endif
	return 0;
}

//-------------- Remove ---------------//
static int sensor_remove(struct platform_device *pdev)
{
	switch_dev_unregister(&this_data->sdev);
#ifdef _LVP9_
	switch_dev_unregister(&this_data->sdev2);
	//mt_eint_mask(CUST_EINT_EINT6_CAP2_NUM); 
	sensor_config_eint(CUST_EINT_EINT6_CAP2_NUM, 0);
#endif
	//mt_eint_mask(CUST_EINT_EINT7_CAP1_NUM);  
	sensor_config_eint(CUST_EINT_EINT7_CAP1_NUM, 0);
	kfree(this_data);
	return 0;
}

//-------------- Suspend ---------------//
static int sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
#if DEBUG
	printk("[iqs128][suspend]!!!!!!\n");
#endif
#if SPEAKRE_RESET
	//cancel_delayed_work_sync(&this_data->speaker_work);
#endif
	
	if(!phone_call) {
	//cancel_delayed_work_sync(&this_data->work);
#ifdef _LVP9_
	//cancel_delayed_work_sync(&this_data->work2);
	//mt_eint_mask(CUST_EINT_EINT6_CAP2_NUM);
	sensor_config_eint(CUST_EINT_EINT6_CAP2_NUM, 0);
#endif
	//mt_eint_mask(CUST_EINT_EINT7_CAP1_NUM);  
	sensor_config_eint(CUST_EINT_EINT7_CAP1_NUM, 0);
	sensor_config_power(false);
	}
	return 0;
}

//-------------- Resume ---------------//
static int sensor_resume(struct platform_device *pdev)
{
	int rt;
#if DEBUG	
	printk("[iqs128][resume]!!!!!!\n");
#endif

#if SPEAKRE_RESET
	//schedule_delayed_work(&this_data->speaker_work, 0);
#endif
	//schedule_delayed_work(&this_data->work, 0);
	//schedule_delayed_work(&this_data->work2, 0);
	if(!phone_call)
		sensor_config_power(true);

	return 0;
}

static struct platform_driver capsensor_driver = {
    .probe      = sensor_probe,
    .remove     = sensor_remove,
    .suspend    = sensor_suspend,
    .resume     = sensor_resume,
    .driver = {
        .name   = SENSOR_NAME,
        .owner  = THIS_MODULE,
    },
};

//[Bug 861] Luke 2011.0908 T1 boot fail due to sensors
//extern int cci_ftm_boot; 
static int __init sensor_init(void)
{
    int rc;
   	
	printk("****CAP sensor_init\n");
    /*sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
    if (IS_ERR(sensor_pdev)) {
	printk(KERN_ERR "[%s]%s - device register FAIL!\n", LOG_TAG, __FUNCTION__);
        return -1;
    }*/
    rc = platform_driver_register(&capsensor_driver);
    if (rc < 0) {
	
	//MODULE_VERSION(DRIVER_VERSION);
	
    }
    return rc;	  
}

static void __exit sensor_exit(void)
{
    platform_driver_unregister(&capsensor_driver);
    platform_device_unregister(sensor_pdev);
}

module_init(sensor_init);
module_exit(sensor_exit);

MODULE_DESCRIPTION("iqs128 cap driver");
MODULE_LICENSE("GPL");
//MODULE_VERSION(DRIVER_VERSION);













