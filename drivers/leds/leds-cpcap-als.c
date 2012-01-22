/*
 * Copyright (C) 2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include <linux/leds-cpcap-als.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>

struct cpcap_als_led_data {
	struct input_dev *idev;
	struct platform_device *pdev;
	struct led_classdev cpcap_als_led_class_dev;
	struct cpcap_device *cpcap;
	struct workqueue_struct *working_queue;
	struct delayed_work dwork;
	struct work_struct work;
	struct cpcap_leds *leds;
	struct regulator *als_regulator;
	u8 mode;
	unsigned int als2lux_rate;
};

static uint32_t debug;
module_param_named(debug, debug, uint, 0664);

static struct cpcap_als_led_config_data als_led_config_data;

#define ADC_READ_LOOPS 5

static void cpcap_als_read_a2d(struct cpcap_als_led_data *info)
{
	int err;
	struct cpcap_adc_request request;
	struct cpcap_device *cpcap = info->cpcap;
	unsigned int  light_value = 0, lux_value = 0;
	int i;

	request.format = CPCAP_ADC_FORMAT_RAW;
	request.timing = CPCAP_ADC_TIMING_IMM;
	request.type = CPCAP_ADC_TYPE_BANK_1;

	for (i = 0; i < ADC_READ_LOOPS; i++) {
		err = cpcap_adc_sync_read(cpcap, &request);
		if (err < 0) {
			pr_err("%s: A2D read error %d\n", __func__, err);
			return;
		}
		light_value += request.result[CPCAP_ADC_TSY1_AD14];
		udelay(250);
	}
	light_value /= ADC_READ_LOOPS;

	if (light_value < info->leds->als_data.als_min)
		lux_value = info->leds->als_data.lux_min;
	else
		lux_value = (light_value * info->als2lux_rate) >> 8;

	if (debug) {
		pr_err("%s: ALS Data: %x\n",
		       __func__, light_value);
		pr_err("%s: Lux value: %x\n", __func__,
			lux_value);
	}
	input_event(info->idev, EV_MSC, MSC_RAW, light_value);
	input_event(info->idev, EV_LED, LED_MISC, lux_value);
	input_sync(info->idev);
}

static void cpcap_display_led_work(struct work_struct *work)
{
	struct cpcap_als_led_data *info =
	  container_of(work, struct cpcap_als_led_data, work);

	cpcap_als_read_a2d(info);
	if (debug)
		pr_err("%s: Workqueue\n", __func__);

}

static void cpcap_display_led_dwork(struct work_struct *work)
{
	struct cpcap_als_led_data *info =
	  container_of((struct delayed_work *)work, struct cpcap_als_led_data,
	  dwork);

	cpcap_als_read_a2d(info);
	queue_delayed_work(info->working_queue, &info->dwork,
			      msecs_to_jiffies(info->leds->display_led.
					       poll_intvl));
	if (debug)
		pr_err("%s: Delayed Workqueue\n", __func__);

}

static ssize_t cpcap_display_led_als_store(struct device *dev,
					   struct device_attribute
					   *attr, const char *buf, size_t size)
{
	unsigned long mode;
	struct cpcap_als_led_data *info;
	struct platform_device *pdev =
	  container_of(dev->parent, struct platform_device, dev);

	info = platform_get_drvdata(pdev);

	if ((strict_strtoul(buf, 10, &mode)) < 0)
		return -1;

	if (mode == AUTOMATIC) {
		info->mode = AUTOMATIC;
		if (!regulator_is_enabled(info->als_regulator))
			regulator_enable(info->als_regulator);
		queue_delayed_work(info->working_queue, &info->dwork,
			      msecs_to_jiffies(info->leds->display_led.
			      poll_intvl));
	} else {
		info->mode = MANUAL;
		cancel_delayed_work(&info->dwork);
		cancel_delayed_work_sync(&info->dwork);
		if (regulator_is_enabled(info->als_regulator))
			regulator_disable(info->als_regulator);
	}

	return info->mode;
}

static ssize_t cpcap_display_led_als_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct cpcap_als_led_data *info;
	struct platform_device *pdev =
	  container_of(dev->parent, struct platform_device, dev);

	info = platform_get_drvdata(pdev);

	scnprintf(buf , PAGE_SIZE, "0x%X", info->mode);
	if (debug)
		pr_err("%s : als node - Show value %d\n",
			 __func__, info->mode);

	return 0;
}

static DEVICE_ATTR(als, 0666, cpcap_display_led_als_show,
	cpcap_display_led_als_store);

static int cpcap_als_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct cpcap_als_led_data *info;
	struct spi_device *spi;
	struct cpcap_platform_data *data;
	unsigned int num, den;

	if (pdev == NULL) {
		pr_err("%s: platform data required\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	info = kzalloc(sizeof(struct cpcap_als_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		goto err_pd_or_kzalloc_failed;
	}

	if (pdev->dev.platform_data == NULL) {
		pr_err("%s: Platform device missing\n", __func__);
		ret = -ENODEV;
		goto err_info_missing;
	}
	memcpy(&als_led_config_data, pdev->dev.platform_data,
		sizeof(als_led_config_data));

	info->cpcap = platform_get_drvdata(pdev);
	if (info->cpcap == NULL) {
		pr_err("%s: Driver data mising\n", __func__);
		goto err_info_missing;
	}
	platform_set_drvdata(pdev, info);
	spi = info->cpcap->spi;
	data = (struct cpcap_platform_data *)(spi->controller_data);
	info->leds = data->leds;
	info->mode = MANUAL;
	info->idev = input_allocate_device();
	if (!info->idev) {
		ret = -ENOMEM;
		printk(KERN_CRIT "%s: input device allocate failed: %d\n",
		__func__, ret);
		goto err_input_allocate_failed;
	}
	info->idev->name = BACKLIGHT_ALS;
	num = (unsigned int)((info->leds->als_data.lux_max -
			info->leds->als_data.lux_min) << 16);
	den = (unsigned int)((info->leds->als_data.als_max -
			info->leds->als_data.als_min) << 8);
	if (den != 0)
		info->als2lux_rate = (unsigned int)(num / den);
	else
		info->als2lux_rate = 0xFFFF00;

	input_set_capability(info->idev, EV_MSC, MSC_RAW);
	input_set_capability(info->idev, EV_LED, LED_MISC);
	if (input_register_device(info->idev)) {
		pr_err("%s: input device register failed\n", __func__);
		goto err_input_register_failed;
	}

	info->als_regulator = regulator_get(NULL, CPCAP_ALS_REG);
	if (IS_ERR(info->als_regulator)) {
		printk(KERN_CRIT"%s: Cannot get %s ALS regulator\n", __func__,
				CPCAP_ALS_REG);
		ret = PTR_ERR(info->als_regulator);
		goto err_als_reg_request_failed;
	}

	info->cpcap_als_led_class_dev.name = als_led_config_data.class_name;
	ret = led_classdev_register(&pdev->dev, &info->cpcap_als_led_class_dev);
	if (ret < 0) {
		pr_err("%s:Register display backlight class failed\n",
			__func__);
		goto err_reg_led_class_failed;
		}
	if ((device_create_file(info->cpcap_als_led_class_dev.dev,
				&dev_attr_als)) < 0) {
		pr_err("%s:File device creation failed: %d\n",
		       __func__, -ENODEV);
		goto err_dev_als_create_failed;
	}
	info->working_queue = create_singlethread_workqueue("cpcap_als_wq");
	if (!info->working_queue) {
		pr_err("%s: Cannot create work queue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_wq_failed;
	}
	INIT_WORK(&info->work, cpcap_display_led_work);
	queue_work(info->working_queue, &info->work);
	INIT_DELAYED_WORK(&info->dwork, cpcap_display_led_dwork);

	if (debug)
		pr_info("%s successful\n", __func__);

	return ret;

err_create_singlethread_wq_failed:
	device_remove_file(info->cpcap_als_led_class_dev.dev, &dev_attr_als);
err_dev_als_create_failed:
	led_classdev_unregister(&info->cpcap_als_led_class_dev);
err_reg_led_class_failed:
err_als_reg_request_failed:
	if (info->als_regulator)
		regulator_put(info->als_regulator);
err_input_register_failed:
	input_free_device(info->idev);
err_input_allocate_failed:
err_info_missing:
	kfree(info);

err_pd_or_kzalloc_failed:
	if (debug)
		pr_info("%s failed\n", __func__);

	return ret;
}

static int cpcap_als_remove(struct platform_device *pdev)
{
	struct cpcap_als_led_data *info = platform_get_drvdata(pdev);

	if (info->als_regulator)
		regulator_put(info->als_regulator);

	cancel_delayed_work(&info->dwork);
	cancel_delayed_work_sync(&info->dwork);
	if (info->working_queue)
		destroy_workqueue(info->working_queue);
	device_remove_file(info->cpcap_als_led_class_dev.dev, &dev_attr_als);
	led_classdev_unregister(&info->cpcap_als_led_class_dev);
	input_unregister_device(info->idev);
	kfree(info);
	return 0;
}

static struct platform_driver cpcap_als_driver = {
	.probe = cpcap_als_probe,
	.remove = cpcap_als_remove,
	.driver = {
		   .name = CPCAP_ALS_LED_DRV_NAME,
		   },
};

static int __init cpcap_als_init(void)
{
	return cpcap_driver_register(&cpcap_als_driver);
}

static void __exit cpcap_als_exit(void)
{
	platform_driver_unregister(&cpcap_als_driver);
}

module_init(cpcap_als_init);
module_exit(cpcap_als_exit);

MODULE_DESCRIPTION("CPCAP ALS driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
