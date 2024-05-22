#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define DW_GPIO_DEV_NAME "duowei-gpio"

typedef struct _dw_gpio{
	int mifi_5g_gled_gpio;
	int mifi_4g_gled_gpio;
	int mifi_pw_rled_gpio;
	int mifi_pw_gled_gpio;
}dw_gpio_st;

#define LED_TOGGLE_INTERVAL_MS (600)  //600MS

static int s_gledBlinkFlag = 0;
//static struct timer_list gled_timer;

static dw_gpio_st *p_mifi_gpio = NULL;

static int dw_gpio_dts_init(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *node = NULL;
	node = of_find_compatible_node(NULL, NULL, "mediatek,dw-gpio");
	if (node) {
		p_mifi_gpio->mifi_5g_gled_gpio = of_get_named_gpio(node, "mifi,5g-gled-en", 0);
		ret = gpio_request(p_mifi_gpio->mifi_5g_gled_gpio, "mifi,5g-gled-en control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_mifi_gpio->mifi_5g_gled_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_mifi_gpio->mifi_5g_gled_gpio) )
		{ 
			return -ENODEV;
		}

		p_mifi_gpio->mifi_4g_gled_gpio = of_get_named_gpio(node, "mifi,4g-gled-en", 0);
		ret = gpio_request(p_mifi_gpio->mifi_4g_gled_gpio, "mifi,4g-gled-en control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_mifi_gpio->mifi_4g_gled_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_mifi_gpio->mifi_4g_gled_gpio) )
		{ 
			return -ENODEV;
		}

		p_mifi_gpio->mifi_pw_rled_gpio = of_get_named_gpio(node, "mifi,pw-rled-en", 0);
		ret = gpio_request(p_mifi_gpio->mifi_pw_rled_gpio, "mifi,pw-rled-en control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_mifi_gpio->mifi_pw_rled_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_mifi_gpio->mifi_pw_rled_gpio) )
		{ 
			return -ENODEV;
		}

		p_mifi_gpio->mifi_pw_gled_gpio = of_get_named_gpio(node, "mifi,pw-gled-en", 0);
		ret = gpio_request(p_mifi_gpio->mifi_pw_gled_gpio, "mifi,pw-gled-en control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_mifi_gpio->mifi_pw_gled_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_mifi_gpio->mifi_pw_gled_gpio) )
		{ 
			return -ENODEV;
		}

	} else {
		pr_err("dw_gpio_dts_init device node error!\n");
		ret = -1;
	}

	return ret;
}


static ssize_t show_mifi_5g_gled(struct device *dev, struct device_attribute *attr,char *buf)
{
    int gpio_status = gpio_get_value(p_mifi_gpio->mifi_5g_gled_gpio);	
	return sprintf(buf, "%d\n", gpio_status);
}
static ssize_t store_mifi_5g_gled(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int temp = 0;
	if(1 == sscanf(buf, "%d", &temp))
	{
		if(temp==0){
			gpio_direction_output(p_mifi_gpio->mifi_5g_gled_gpio, 0);
		}
		else if(temp==1){
			gpio_direction_output(p_mifi_gpio->mifi_5g_gled_gpio, 1);
		}else {
			pr_err("invalid value = '%d'\n", temp);
		}	
	}
	else
	{
		pr_err("invalid format = '%s'\n", buf);
	}
	return size;
}
static DEVICE_ATTR(led5G, 0664, show_mifi_5g_gled, store_mifi_5g_gled);

static ssize_t show_mifi_4g_gled(struct device *dev, struct device_attribute *attr,char *buf)
{
    int gpio_status = gpio_get_value(p_mifi_gpio->mifi_4g_gled_gpio);	
	return sprintf(buf, "%d\n", gpio_status);
}
static ssize_t store_mifi_4g_gled(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int temp = 0;
	if(1 == sscanf(buf, "%d", &temp))
	{
		if(temp==0){
			gpio_direction_output(p_mifi_gpio->mifi_4g_gled_gpio, 0);
		}
		else if(temp==1){
			gpio_direction_output(p_mifi_gpio->mifi_4g_gled_gpio, 1);
		}else {
			pr_err("invalid value = '%d'\n", temp);
		}	
	}
	else
	{
		pr_err("invalid format = '%s'\n", buf);
	}
	return size;
}
static DEVICE_ATTR(led4G, 0664, show_mifi_4g_gled, store_mifi_4g_gled);

static ssize_t show_mifi_pw_rled(struct device *dev, struct device_attribute *attr,char *buf)
{
    int gpio_status = gpio_get_value(p_mifi_gpio->mifi_pw_rled_gpio);	
	return sprintf(buf, "%d\n", gpio_status);
}
static ssize_t store_mifi_pw_rled(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int temp = 0;
	if(1 == sscanf(buf, "%d", &temp))
	{
		if(temp==0){
			gpio_direction_output(p_mifi_gpio->mifi_pw_rled_gpio, 0);
		}
		else if(temp==1){
			gpio_direction_output(p_mifi_gpio->mifi_pw_rled_gpio, 1);
		}else {
			pr_err("invalid value = '%d'\n", temp);
		}	
	}
	else
	{
		pr_err("invalid format = '%s'\n", buf);
	}
	return size;
}
static DEVICE_ATTR(rledPW, 0664, show_mifi_pw_rled, store_mifi_pw_rled);


static ssize_t show_mifi_pw_gled(struct device *dev, struct device_attribute *attr,char *buf)
{
    int gpio_status = gpio_get_value(p_mifi_gpio->mifi_pw_gled_gpio);	
	return sprintf(buf, "%d\n", gpio_status);
}
static ssize_t store_mifi_pw_gled(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int temp = 0;
	pr_info("%s,%d",__func__,__LINE__);
	if(1 == sscanf(buf, "%d", &temp))
	{
		if(temp==0){
			gpio_direction_output(p_mifi_gpio->mifi_pw_gled_gpio, 0);
		}
		else if(temp==1){
			gpio_direction_output(p_mifi_gpio->mifi_pw_gled_gpio, 1);
		}else {
			pr_err("invalid value = '%d'\n", temp);
		}	
	}
	else
	{
		pr_err("invalid format = '%s'\n", buf);
	}
	return size;
}
static DEVICE_ATTR(gledPW, 0664, show_mifi_pw_gled, store_mifi_pw_gled);

extern void set_is_blink_flag(int isblink);
static ssize_t show_mifi_pw_gled_blink(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", s_gledBlinkFlag);
}
static ssize_t store_mifi_pw_gled_blink(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//int temp = 0;
	pr_info("%s,%d",__func__,__LINE__);
	if(1 == sscanf(buf, "%d", &s_gledBlinkFlag))
	{
		if(s_gledBlinkFlag==0){
			//gpio_direction_output(p_mifi_gpio->mifi_pw_gled_gpio, 0);
			/* Stop timer */
			//del_timer(&gled_timer);
			set_is_blink_flag(0);
		}
		else if(s_gledBlinkFlag==1){
			//gpio_direction_output(p_mifi_gpio->mifi_pw_gled_gpio, 1);
			/* Start timer */
			//mod_timer(&gled_timer, jiffies + msecs_to_jiffies(LED_TOGGLE_INTERVAL_MS));
			set_is_blink_flag(1);
		}else {
			pr_err("invalid value = '%d'\n", s_gledBlinkFlag);
		}
	}
	else
	{
		pr_err("invalid format = '%s'\n", buf);
	}
	return size;
}
static DEVICE_ATTR(gledBlink, 0664, show_mifi_pw_gled_blink, store_mifi_pw_gled_blink);

void set_mifi_gled_gpio(int state)
{
	gpio_direction_output(p_mifi_gpio->mifi_pw_gled_gpio, state);
}
EXPORT_SYMBOL(set_mifi_gled_gpio);

// static void gled_blink_timer_callback(struct timer_list *timer) {
//     static int led_state = 0; // 0表示LED关闭，1表示LED打开

//     // 切换LED状态
//     led_state = !led_state;
//     //gpio_direction_output(p_mifi_gpio->mifi_pw_gled_gpio, led_state);

//     // 重新启动定时器
//     mod_timer(timer, jiffies + msecs_to_jiffies(LED_TOGGLE_INTERVAL_MS));
// }

static int dw_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	p_mifi_gpio = devm_kzalloc(dev, sizeof(*p_mifi_gpio), GFP_KERNEL);
	if (!p_mifi_gpio)
		return -ENOMEM;


	ret = dw_gpio_dts_init(pdev);
	if(0 != ret)
	{
		pr_err("dw_gpio_dts_init failed!\n");
		return ret;
	}

	ret = device_create_file(&(pdev->dev), &dev_attr_led5G);
	if (ret) {
		pr_err("dev_attr_led5G fail!\n");		
		goto err_led5G;
	}

	ret = device_create_file(&(pdev->dev), &dev_attr_led4G);
	if (ret) {
		pr_err("dev_attr_led4G fail!\n");		
		goto err_led4G;
	}

	ret = device_create_file(&(pdev->dev), &dev_attr_rledPW);
	if (ret) {
		pr_err("dev_attr_rledPW fail!\n");		
		goto err_rledPW;
	}

	ret = device_create_file(&(pdev->dev), &dev_attr_gledPW);
	if (ret) {
		pr_err("dev_attr_gledPW fail!\n");		
		goto err_gledPW;
	}

	ret = device_create_file(&(pdev->dev), &dev_attr_gledBlink);
	if (ret) {
		pr_err("dev_attr_gledBlink fail!\n");
		goto err_gledBlink;
	}

	// 初始化定时器
	//timer_setup(&gled_timer, gled_blink_timer_callback, 0);

	pr_info("%s() probe successfully!\n", __func__);
	return ret;

err_gledBlink:
	device_remove_file(&(pdev->dev), &dev_attr_gledBlink);
	return -1;

err_gledPW:
	device_remove_file(&(pdev->dev), &dev_attr_gledPW);
	return -1;

err_rledPW:
	device_remove_file(&(pdev->dev), &dev_attr_rledPW);
	return -1;

err_led4G:
	device_remove_file(&(pdev->dev), &dev_attr_led4G);
	return -1;

err_led5G:
	device_remove_file(&(pdev->dev), &dev_attr_led5G);
	return -1;
}

static int dw_gpio_remove(struct platform_device *pdev)
{
	if(p_mifi_gpio) {
		kfree(p_mifi_gpio);
		p_mifi_gpio = NULL;
	}
	//del_timer(&gled_timer);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dw_gpio_of_match[] = {
	{.compatible = "mediatek,dw-gpio"},
	{},
};
#endif/* CONFIG_OF */

static struct platform_driver dw_gpio_driver = {
	.driver = {
		   .name = DW_GPIO_DEV_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = dw_gpio_of_match,
#endif/* CONFIG_OF */

		   },
	.probe = dw_gpio_probe,
	.remove = dw_gpio_remove,
};

static int __init dw_gpio_init(void)
{
	int ret;
	ret = platform_driver_register(&dw_gpio_driver);
	return ret;
}
static void __exit dw_gpio_exit(void)
{
	platform_driver_unregister(&dw_gpio_driver);
}
MODULE_AUTHOR("dingjun");
MODULE_DESCRIPTION("gpio operations driver");
MODULE_LICENSE("GPL");
module_init(dw_gpio_init);
module_exit(dw_gpio_exit);
