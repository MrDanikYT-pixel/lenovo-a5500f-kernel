//--------------- Header files ------------------//
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
//#include <linux/gpio_keys.h>
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

#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>
#include <mach/eint.h>
//#include <mtk_kpd.h>
//#include <linux/wakelock.h>

//-------------- Define ---------------//
#define DEBUG 0
#define IF_I2C_DEVICE 0
#define LOG_TAG "EM6781"
#define SENSOR_NAME "em6781"
#define HALL_SENSOR_GPIO EINT8_HALL
#define DOCK_ID 0
#define DOCK_ON 1
#define DOCK_OFF 0
#define delay_time 100
//-------------- Define Sensor Data Struct ---------------//
struct sensor_data {
    struct switch_dev sdev;
    struct delayed_work work;
    struct device *dev;
    struct platform_device *pdev;	
    atomic_t state;
    unsigned gpio;
    atomic_t check_state;
};

//-------------- Global para. --------------//
static struct sensor_data *this_data = NULL;
static struct platform_device *sensor_pdev = NULL;
static struct platform_driver sensor_driver;
static struct input_dev 	*input;
static int old_INT_stat = 1, hall_suspend = 0;
static int suspend_em6781 = 1;
static void switch_work_func(struct work_struct *work);

//-------------- Global functions ---------------//
static int sensor_read_gpio_data(unsigned gpio_number, const char* label)
{
	int gpio_data = 0;
	gpio_data = mt_get_gpio_in(gpio_number);
#if DEBUG
		printk("[hall_sensor][em6781]>>>gpio_data[%d]=%d\n", gpio_number, gpio_data);
#endif

	return gpio_data;	
}
//-------------- irq func ---------------//
static void hall_irq_handler(void)
{
#if DEBUG
	printk("[hall_sensor][em6781]>>>hall_irq_handler\n");
#endif
	//cancel_delayed_work_sync(&this_data->work);
	if(!atomic_read(&this_data->check_state)) {
	atomic_set(&this_data->check_state, 1);
	schedule_delayed_work(&this_data->work, 0);
	//mt_eint_unmask(CUST_EINT_EINT8_HALL_NUM); 
	}
}

//-------------- Work func ---------------//
static void switch_work_func(struct work_struct *work)
{
	int state;
	
	state = sensor_read_gpio_data(EINT8_HALL, SENSOR_NAME);
#if DEBUG
	printk("[hall_sensor][em6781]>>>switch_work_func state=%d, old_INT_stat=%d, irq2=%d\n", state, old_INT_stat, atomic_read(&this_data->check_state));
#endif
	
	if(old_INT_stat != state) {
		if(state == 1 && hall_suspend) {
			input_report_key(input, KEY_POWER, 1);
			input_report_key(input, KEY_POWER, 0);
			input_sync(input);
		}
		else if(state == 0 && !hall_suspend) {
			input_report_key(input, KEY_POWER, 1);
			input_report_key(input, KEY_POWER, 0);
			input_sync(input);
		}
		if(state == 0)
			mt_eint_set_polarity(CUST_EINT_EINT8_HALL_NUM, 1);
		else
			mt_eint_set_polarity(CUST_EINT_EINT8_HALL_NUM, 0);
		old_INT_stat = state;
	}
	atomic_set(&this_data->state, !state);
	//cancel_delayed_work_sync(&this_data->work);

	if(atomic_read(&this_data->check_state)) {
#if DEBUG
		printk("[hall_sensor][em6781]>>>irq=1\n");
#endif
		schedule_delayed_work(&this_data->work, delay_time);
		atomic_set(&this_data->check_state, 0);
	}
	
}

static ssize_t magnetic_show_active(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&this_data->state));
}

static DRIVER_ATTR(active, 0644, magnetic_show_active, NULL);
static struct driver_attribute *sensor_attr_list[] = {
    &driver_attr_active,
};

//-------------- Probe ---------------//
static int sensor_probe(struct platform_device *pdev)
{
	int rt, idx;
	int error;
	struct sensor_data *data = NULL;
	printk("****HALL sensor_probe\n");

    mt_set_gpio_mode(EINT8_HALL, EINT8_HALL_M_EINT);
    //mt_set_gpio_dir(EINT8_HALL, GPIO_DIR_IN);
    //mt_set_gpio_pull_enable(EINT8_HALL, true);
    //mt_set_gpio_pull_select(EINT8_HALL, GPIO_PULL_UP);
	
    //mt_eint_set_sens(CUST_EINT_HALL_1_NUM, CUST_EINT_HALL_1_SENSITIVE);
	mt_eint_set_polarity(CUST_EINT_EINT8_HALL_NUM, 0);
	mt_eint_set_hw_debounce(CUST_EINT_EINT8_HALL_NUM, CUST_EINT_EINT8_HALL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_EINT8_HALL_NUM, CUST_EINT_EINT8_HALL_TYPE, hall_irq_handler, 1);
	//mt_eint_unmask(CUST_EINT_EINT8_HALL_NUM);  

	//switch device registing	
	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
        	rt = -ENOMEM;
        	printk("[em6781]sensor_probe>>>kzalloc fail\n");
		return rt;
    	}
	data->sdev.name = SENSOR_NAME;
	data->gpio = EINT8_HALL;	

	rt = switch_dev_register(&data->sdev);
	if (rt != 0)
		printk("[em6781]sensor_probe>>>switch_dev_register fail\n");

	INIT_DELAYED_WORK(&data->work, switch_work_func);
	this_data = data;

	rt = driver_create_file(&sensor_driver.driver, sensor_attr_list[0]);
	if (rt) {
		pr_err("sensor_probe: driver_create_file fail!!! \n");
	}

	input=input_allocate_device();
	if (!input)
		return -ENOMEM;
	else
		printk("input device allocate Success !!\n");
	
	__set_bit(EV_KEY, input->evbit);
	__set_bit(KEY_POWER, input->keybit);
		input->name = "em6781";
		if(input_register_device(input))
		{
			printk("[em6781]input register : fail!\n");
		}else
		{
			printk("[em6781]input register : success!!\n");
		} 
		
	return 0;
}

//-------------- Remove ---------------//
static int sensor_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&this_data->work);
	//switch_dev_unregister(&this_data->sdev);
	//device_remove_file(&pdev->dev, &dev_attr_active);
	kfree(this_data);
	return 0;
}

//-------------- Suspend ---------------//
static int sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
#if DEBUG
	printk("[em6781][suspend]!!!!!!\n");
#endif

	return 0;
}

//-------------- Resume ---------------//
static int sensor_resume(struct platform_device *pdev)
{
	int rt;
#if DEBUG	
	printk("[em6781][resume]!!!!!!\n");
#endif

	return 0;
}

static void em6781_early_suspend(struct early_suspend *h)
{
	printk("[em6781_early_suspend]!!!!!!\n");
    hall_suspend = 1;
}

static void em6781_late_resume(struct early_suspend *h)
{
	printk("[em6781_late_resume]!!!!!!\n");
    hall_suspend = 0;
}

static struct platform_driver sensor_driver = {
    .probe      = sensor_probe,
    .remove     = sensor_remove,
    .suspend    = sensor_suspend,
    .resume     = sensor_resume,
    .driver = {
        .name   = SENSOR_NAME,
        .owner  = THIS_MODULE,
    }
};

static struct early_suspend em6781_early_suspend_desc = {
    .level      = 50,
    .suspend    = em6781_early_suspend,
    .resume     = em6781_late_resume,
};

//[Bug 861] Luke 2011.0908 T1 boot fail due to sensors
//extern int cci_ftm_boot; 
static int __init sensor_init(void)
{
    int rc;
   	
	printk("****HALL sensor_init\n");

    rc = platform_driver_register(&sensor_driver);
    if (rc < 0) {
	
	//MODULE_VERSION(DRIVER_VERSION);
	
    }
	
	register_early_suspend(&em6781_early_suspend_desc);
    return rc;	  
}

static void __exit sensor_exit(void)
{
    platform_driver_unregister(&sensor_driver);
    platform_device_unregister(sensor_pdev);
}

module_init(sensor_init);
module_exit(sensor_exit);

MODULE_DESCRIPTION("EM6781 hall driver");
MODULE_LICENSE("GPL");
//MODULE_VERSION(DRIVER_VERSION);









