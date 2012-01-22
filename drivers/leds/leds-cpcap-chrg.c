/*
 * Copyright (C) 2011 Motorola Mobility, Inc.
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
#include <linux/spi/cpcap-regbits.h>
#include <linux/leds-cpcap-chrg.h>


struct cpcap_chrg_led_data {
	struct led_classdev cpcap_chrg_led_class_dev;
	struct cpcap_device *cpcap;
	struct regulator *regulator;
	int regulator_state;
};

static uint32_t debug;
module_param_named(debug, debug, uint, 0664);

static struct cpcap_chrg_led_config_data chrg_led_config_data;

static void cpcap_chrg_led_set(struct led_classdev *led_cdev,
						enum led_brightness value)
{
	int cpcap_status = 0;
	int cpcap_register = 0;

	struct cpcap_chrg_led_data *led_data =
		container_of(led_cdev, struct cpcap_chrg_led_data,
					cpcap_chrg_led_class_dev);

	/* Register 641, bit 13 is used to enable/disable the charging LED */
    /* Take the corresponding register from cpcap-regacc.c */
	cpcap_register = CPCAP_REG_CRM;

	if (debug)
		pr_info("%s %d\n", __func__, value);

    if (value > LED_OFF) {
		chrg_led_config_data.on = CPCAP_CHRG_LED_EN;
		cpcap_status = cpcap_regacc_write(led_data->cpcap,
							cpcap_register,
							chrg_led_config_data.on,
							CPCAP_CHRG_ON_OFF_MASK);
	} else {
		cpcap_status = cpcap_regacc_write(led_data->cpcap,
							cpcap_register,
							CPCAP_CHRG_LED_DIS,
							CPCAP_CHRG_ON_OFF_MASK);
	}
	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
			   __func__, cpcap_status);

	return;
}

static int cpcap_chrg_led_chrg_probe(struct platform_device *pdev)
{
	int ret = 0;
	int cpcap_status = 0;
	struct cpcap_chrg_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: Platform device missing\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	info = kzalloc(sizeof(struct cpcap_chrg_led_data), GFP_KERNEL);
	if (info == NULL) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_pd_or_kzalloc_failed;
	}

	if (pdev->dev.platform_data == NULL) {
		pr_err("%s: Platform data missing\n", __func__);
		ret = -ENODEV;
		goto err_info_missing;
	}

	memcpy(&chrg_led_config_data,
			pdev->dev.platform_data, sizeof(chrg_led_config_data));

	info->cpcap = platform_get_drvdata(pdev);
	if (info->cpcap == NULL) {
		pr_err("%s: Driver data mising\n", __func__);
		goto err_info_missing;
	}

	platform_set_drvdata(pdev, info);

	info->cpcap_chrg_led_class_dev.name = chrg_led_config_data.class_name;
	info->cpcap_chrg_led_class_dev.brightness_set = cpcap_chrg_led_set;
	ret = led_classdev_register(&pdev->dev,
				&info->cpcap_chrg_led_class_dev);
	if (ret) {
		printk(KERN_ERR "Register CHRG led class failed %d\n", ret);
		goto err_reg_chrg_failed;
	}

	if (debug)
		pr_info("%s successful\n", __func__);

	return ret;

err_reg_chrg_failed:
	if (debug)
		pr_info("%s failed\n", __func__);

err_info_missing:
	kfree(info);

err_pd_or_kzalloc_failed:
	if (debug)
		pr_info("%s failed\n", __func__);

	return ret;
}

static int cpcap_chrg_led_chrg_remove(struct platform_device *pdev)
{
	struct cpcap_chrg_led_data *info = platform_get_drvdata(pdev);;

	if (debug)
		pr_info("%s\n", __func__);

	led_classdev_unregister(&info->cpcap_chrg_led_class_dev);
	kfree(info);
	return 0;
}

static struct platform_driver cpcap_chrg_led_chrg_driver = {
	.probe   = cpcap_chrg_led_chrg_probe,
	.remove  = cpcap_chrg_led_chrg_remove,
	.driver  = {
		.name  = CPCAP_CHRG_LED_DRV_NAME,
		.owner = THIS_MODULE,
	},
};


static int cpcap_chrg_led_chrg_init(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	return cpcap_driver_register(&cpcap_chrg_led_chrg_driver);
}

static void cpcap_chrg_led_chrg_shutdown(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	platform_driver_unregister(&cpcap_chrg_led_chrg_driver);
}

module_init(cpcap_chrg_led_chrg_init);
module_exit(cpcap_chrg_led_chrg_shutdown);

MODULE_DESCRIPTION("CPCAP CHARGE-ONLY LED driver");
MODULE_AUTHOR("Motorola Mobility");
MODULE_LICENSE("GNU");
