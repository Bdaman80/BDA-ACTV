/*
 * Copyright (C) 2009-2010 Motorola, Inc.
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <mach/gpio.h>

#include <linux/regulator/consumer.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>

#include <sound/cs42l73.h>
#include <sound/soc-dapm.h>

#define NUM_KEYS 4
#define VOLT_CUSHION 20
#define HS_DET_B_GPIO	29

struct volt_key {
	short voltage;
	int key;
};

static struct volt_key key_lut[NUM_KEYS] = {
	{273, KEY_PLAYCD},
	{381, KEY_VOLUMEUP},
	{485, KEY_VOLUMEDOWN},
	{586, KEY_POWER_SONG}
};

enum {
	NO_DEVICE,
	HEADSET_WITH_MIC,
	HEADSET_WITHOUT_MIC,
};

struct cpcap_3mm5_data {
	struct cpcap_device *cpcap;
	struct switch_dev sdev;
	unsigned int key_state;
	struct regulator *regulator;
	unsigned char audio_low_pwr_det;
	unsigned char audio_low_pwr_mac13;
	struct delayed_work work;
	struct delayed_work key_work;
	struct workqueue_struct *hs_workqueue;
	struct work_struct hs_work;
	int last_key;
	struct wake_lock hs_work_wakelock;
};

#if 0
/*Due to HW issues,send/end key may be detected while HS is inserted */
/*Soln:After HS detection wait for some time before sending send/end key */
static unsigned long hs_det_time; /* Time(tick) when headset was detected */
#define SEND_KEY_TMO   4 /* Timeout in seconds before sending send/end key */
#endif

int headset_connected;
int has_vol_controls; /* Indicates if connected headset has volume controls */
struct cpcap_device *cpcap;

static ssize_t print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case HEADSET_WITH_MIC:
		return sprintf(buf, "Headset with mic\n");
	case HEADSET_WITHOUT_MIC:
		return sprintf(buf, "Headset without mic\n");
	}

	return -EINVAL;
}

static void audio_low_power_set(struct cpcap_3mm5_data *data,
				unsigned char *flag)
{
	if (!(*flag)) {
		regulator_set_mode(data->regulator, REGULATOR_MODE_STANDBY);
		*flag = 1;
	}
}

static void audio_low_power_clear(struct cpcap_3mm5_data *data,
				  unsigned char *flag)
{
	if (*flag) {
		regulator_set_mode(data->regulator, REGULATOR_MODE_NORMAL);
		*flag = 0;
	}
}

static void send_key_event(struct cpcap_3mm5_data *data, unsigned int state)
{
	dev_info(&data->cpcap->spi->dev, "Headset key event: old=%d, new=%d\n",
		 data->key_state, state);

	if (data->key_state != state) {
		data->key_state = state;
		cpcap_broadcast_key_event(data->cpcap, KEY_MEDIA, state);
	}
}

static irqreturn_t hs_handler(int irq, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;

	queue_work(data_3mm5->hs_workqueue, &data_3mm5->hs_work);

	return IRQ_HANDLED;
}

unsigned short read_mic_adc(struct cpcap_device *cpcap)
{
	unsigned short adc_value = 0;
	unsigned short value = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(50);

	cpcap_regacc_write(cpcap, CPCAP_REG_ADCC1,
		(CPCAP_BIT_AD_SEL1 | CPCAP_BIT_ADEN),
		(CPCAP_BIT_AD_SEL1 | CPCAP_BIT_RAND1 | CPCAP_BIT_RAND0 |
			CPCAP_BIT_ADEN));
	cpcap_regacc_write(cpcap, CPCAP_REG_ADCC2, CPCAP_BIT_ASC,
		CPCAP_BIT_ASC);

	do {
		schedule_timeout_uninterruptible(1);
		cpcap_regacc_read(cpcap, CPCAP_REG_ADCC2, &value);
	} while ((value & CPCAP_BIT_ASC) && time_before(jiffies, timeout));

	cpcap_regacc_read(cpcap, CPCAP_REG_ADCD7, (unsigned short *)&adc_value);

	return adc_value;
}

static void hs_gpio_work(struct work_struct *work)
{
	struct cpcap_3mm5_data *data_3mm5 =
		container_of(work, struct cpcap_3mm5_data, hs_work);
	int i;
	unsigned short key_adc_value1, key_adc_value2;
	u8 reg = 0;

	wake_lock(&data_3mm5->hs_work_wakelock);

	/* Reset headset state while detecting headset */
	headset_connected = NO_DEVICE;
	has_vol_controls = 0;

	/* Check HS Detect GPIO five times to ensure no false removal signal */
	for (i = 0; i < 5; i++)	{
		msleep(10);
		/* HS_DET_B_GPIO of 1 means no headset present,
		   0 means headset attached. */
		if (!gpio_get_value(HS_DET_B_GPIO)) {
			/* Wait up to ten seconds for codec to be initialized */
			int count = 100;
			while (codec_suspended && count--)
				msleep(100);

			if (cs42l73_codec) {
				/* Turn on CODEC */
				cs42l73_codec->set_bias_level(cs42l73_codec,
					SND_SOC_BIAS_PREPARE);
				cs42l73_codec->set_bias_level(cs42l73_codec,
					SND_SOC_BIAS_ON);

				/* Turn on MIC2_BIAS */
				reg = snd_soc_read(cs42l73_codec,
					CS42L73_PWRCTL2);
				snd_soc_write(cs42l73_codec, CS42L73_PWRCTL2,
					reg & ~PDN_MIC2_BIAS);
			}

			msleep(100);
			key_adc_value1 = read_mic_adc(data_3mm5->cpcap);
			msleep(100);
			key_adc_value2 = read_mic_adc(data_3mm5->cpcap);
			/* Try for up to five seconds */
			count = 50;
			while ((abs(key_adc_value1 - key_adc_value2) > 1)
					&& count--) {
				key_adc_value1 = key_adc_value2;
				msleep(100);
				key_adc_value2 = read_mic_adc(data_3mm5->cpcap);
			}

			if (key_adc_value2 < 180) {
				/* ADC value less than ~500 mV */
				headset_connected = HEADSET_WITHOUT_MIC;
				has_vol_controls = 0;
			} else if (key_adc_value2 < 575) {
				/* ADC value less than ~1.6 V */
				headset_connected = HEADSET_WITH_MIC;
				has_vol_controls = 0;
			} else {
				/* ADC value greater than ~1.6 V */
				/* Assume has volume controls */
				headset_connected = HEADSET_WITH_MIC;
				has_vol_controls = 1;
			}

			if (cs42l73_codec) {
				if (headset_connected != HEADSET_WITH_MIC) {
					/* Power down MIC2 Bias */
					reg = snd_soc_read(cs42l73_codec,
						CS42L73_PWRCTL2);
					snd_soc_write(cs42l73_codec,
						CS42L73_PWRCTL2,
						reg | PDN_MIC2_BIAS);
				}

				/* Put CODEC in STANDBY */
				cs42l73_codec->set_bias_level(cs42l73_codec,
					SND_SOC_BIAS_PREPARE);
				cs42l73_codec->set_bias_level(cs42l73_codec,
					SND_SOC_BIAS_STANDBY);
			}
			break;
		}
	}

	if (switch_get_state(&data_3mm5->sdev) != headset_connected) {
		switch_set_state(&data_3mm5->sdev, headset_connected);
		if (data_3mm5->cpcap->h2w_new_state)
			data_3mm5->cpcap->h2w_new_state(headset_connected);

		dev_info(&data_3mm5->cpcap->spi->dev, "New headset state: %d\n",
			headset_connected);
		if (NO_DEVICE == headset_connected) {
			send_key_event(data_3mm5, 0);
			cs42l73_clear_key_pressed();

			if (cs42l73_codec) {
				/* Turn on CODEC */
				cs42l73_codec->set_bias_level(cs42l73_codec,
					SND_SOC_BIAS_PREPARE);
				cs42l73_codec->set_bias_level(cs42l73_codec,
					SND_SOC_BIAS_ON);
				/* Turn off MIC2_SDET interrupt */
				reg = snd_soc_read(cs42l73_codec, CS42L73_IM1);
				snd_soc_write(cs42l73_codec, CS42L73_IM1,
					reg & ~MIC2_SDET);
				/* Power down MIC2 Bias */
				reg = snd_soc_read(cs42l73_codec,
					CS42L73_PWRCTL2);
				snd_soc_write(cs42l73_codec, CS42L73_PWRCTL2,
					reg | PDN_MIC2_BIAS);
				/* Put CODEC in STANDBY */
				cs42l73_codec->set_bias_level(cs42l73_codec,
					SND_SOC_BIAS_PREPARE);
				cs42l73_codec->set_bias_level(cs42l73_codec,
					SND_SOC_BIAS_STANDBY);
			}

#if 0
		} else {
			hs_det_time = jiffies;
#endif
		}
	}

	wake_unlock(&data_3mm5->hs_work_wakelock);
}

static int read_cpcap_key(struct cpcap_device *cpcap)
{
	unsigned short adc_value = 0;
	unsigned short value = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(50);
	int i;

	cpcap_regacc_write(cpcap, CPCAP_REG_ADCC1,
		(CPCAP_BIT_AD_SEL1 | CPCAP_BIT_ADEN),
		(CPCAP_BIT_AD_SEL1 | CPCAP_BIT_RAND1 | CPCAP_BIT_RAND0 |
			CPCAP_BIT_ADEN));
	cpcap_regacc_write(cpcap, CPCAP_REG_ADCC2, CPCAP_BIT_ASC,
		CPCAP_BIT_ASC);

	do {
		schedule_timeout_uninterruptible(1);
		cpcap_regacc_read(cpcap, CPCAP_REG_ADCC2, &value);
	} while ((value & CPCAP_BIT_ASC) && time_before(jiffies, timeout));

	cpcap_regacc_read(cpcap, CPCAP_REG_ADCD7, (unsigned short *)&adc_value);

	for (i = 0; i < NUM_KEYS; i++) {
		if (((key_lut[i].voltage - VOLT_CUSHION) < adc_value) &&
			((key_lut[i].voltage + VOLT_CUSHION) > adc_value))
			break;
	}

	if (i != NUM_KEYS)
		return key_lut[i].key;
	else
		return KEY_RESERVED;
}

static void key_handler(enum cpcap_irqs irq, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;
	int key1, key2;

	if ((irq != CPCAP_IRQ_MB2) && (irq != CPCAP_IRQ_UC_PRIMACRO_5))
		return;

	if ((cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_MB2, 0) == 0) ||
	    (cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_PTT, 0) == 0)) {

#if 0 /* Disable Media key for Gault */
		/* once HS is detected, wait for some time before sending send
		   key */
		if (time_before((hs_det_time+SEND_KEY_TMO*HZ), jiffies))
			send_key_event(data_3mm5, 1);
		/* If macro not available, only short presses are supported */
		if (!cpcap_uc_status(data_3mm5->cpcap, CPCAP_MACRO_5)) {
			send_key_event(data_3mm5, 0);

			/* Attempt to restart the macro for next time. */
			cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_5);
		}
#endif

		if (data_3mm5->last_key != KEY_RESERVED) {
			cancel_delayed_work_sync(&data_3mm5->key_work);
			cpcap_broadcast_key_event(data_3mm5->cpcap,
				data_3mm5->last_key, 0);
			data_3mm5->last_key = KEY_RESERVED;
		}
	} else {
#if 0 /* Disable Media key for Gault */
		send_key_event(data_3mm5, 0);
#endif

		mdelay(10);
		key1 = read_cpcap_key(data_3mm5->cpcap);
		mdelay(5);
		key2 = read_cpcap_key(data_3mm5->cpcap);

		if ((key1 != KEY_RESERVED) && (key1 == key2)) {
			cancel_delayed_work_sync(&data_3mm5->key_work);
			data_3mm5->last_key = key1;
			cpcap_broadcast_key_event(data_3mm5->cpcap,
				data_3mm5->last_key, 1);
			schedule_delayed_work(&data_3mm5->key_work,
				msecs_to_jiffies(100));
		} else if (data_3mm5->last_key != KEY_RESERVED) {
			cancel_delayed_work_sync(&data_3mm5->key_work);
			cpcap_broadcast_key_event(
				data_3mm5->cpcap, data_3mm5->last_key, 0);
			data_3mm5->last_key = KEY_RESERVED;
		}
	}

	cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
	cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);
}

static void key_release_work(struct work_struct *work)
{
	struct cpcap_3mm5_data *data_3mm5 =
		container_of(work, struct cpcap_3mm5_data, key_work.work);

	if ((data_3mm5->last_key != KEY_RESERVED) &&
		(read_cpcap_key(data_3mm5->cpcap) != data_3mm5->last_key)) {
		cpcap_broadcast_key_event(data_3mm5->cpcap,
			data_3mm5->last_key, 0);
		data_3mm5->last_key = KEY_RESERVED;
	}
}

static void mac13_work(struct work_struct *work)
{
	struct cpcap_3mm5_data *data_3mm5 =
		container_of(work, struct cpcap_3mm5_data, work.work);

	audio_low_power_set(data_3mm5, &data_3mm5->audio_low_pwr_mac13);
	cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_13);
}

static void mac13_handler(enum cpcap_irqs irq, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;

	if (irq != CPCAP_IRQ_UC_PRIMACRO_13)
		return;

	audio_low_power_clear(data_3mm5, &data_3mm5->audio_low_pwr_mac13);
	schedule_delayed_work(&data_3mm5->work, msecs_to_jiffies(200));
}

#ifdef CONFIG_PM
static int cpcap_3mm5_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int cpcap_3mm5_resume(struct platform_device *dev)
{
	return 0;
}
#endif

static int __init cpcap_3mm5_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct cpcap_3mm5_data *data;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	cpcap = data->cpcap;
	data->audio_low_pwr_det = 1;
	data->audio_low_pwr_mac13 = 1;
	data->sdev.name = "h2w";
	data->sdev.print_name = print_name;
	switch_dev_register(&data->sdev);
	INIT_DELAYED_WORK(&data->work, mac13_work);
	platform_set_drvdata(pdev, data);
	data->last_key = KEY_RESERVED;
	INIT_DELAYED_WORK(&data->key_work, key_release_work);

	data->regulator = regulator_get(NULL, "vaudio");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Could not get regulator for cpcap_3mm5\n");
		retval = PTR_ERR(data->regulator);
		goto free_mem;
	}

	regulator_set_voltage(data->regulator, 2775000, 2775000);

	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_MB2);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_13);
	if (retval)
		goto reg_put;

	if (gpio_request(HS_DET_B_GPIO, "HS_DET")) {
		dev_err(&pdev->dev, "HS_DET_B_GPIO: gpio %d request failed\n",
			HS_DET_B_GPIO);
		goto reg_put;
	}
	retval = request_irq(gpio_to_irq(HS_DET_B_GPIO), hs_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "hs_det", data);
	if (retval) {
		dev_err(&pdev->dev,
			"request_irq HS_DET_B_GPIO gpio %d -> irq %d failed\n",
			HS_DET_B_GPIO, gpio_to_irq(HS_DET_B_GPIO));
		goto free_hs;
	}
	enable_irq_wake(gpio_to_irq(HS_DET_B_GPIO));
	INIT_WORK(&data->hs_work, hs_gpio_work);
	wake_lock_init(&data->hs_work_wakelock, WAKE_LOCK_SUSPEND,
		"hs_work_wakelock");
	data->hs_workqueue = create_workqueue("hs_work");
	if (data->hs_workqueue == NULL) {
		retval = -ENOMEM;
		dev_err(&pdev->dev, "unable to initialize workqueue\n");
		goto free_hs;
	}

	retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_MB2, key_handler,
				    data);
	if (retval)
		goto free_hs;

	retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5,
				    key_handler, data);
	if (retval)
		goto free_mb2;

	if (data->cpcap->vendor == CPCAP_VENDOR_ST) {
		retval = cpcap_irq_register(data->cpcap,
					    CPCAP_IRQ_UC_PRIMACRO_13,
					    mac13_handler, data);
		if (retval)
			goto free_mac5;

		cpcap_uc_start(data->cpcap, CPCAP_MACRO_13);
	}

	queue_work(data->hs_workqueue, &data->hs_work);

	return 0;

free_mac5:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);
free_mb2:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_MB2);
free_hs:
	disable_irq_wake(gpio_to_irq(HS_DET_B_GPIO));
	gpio_free(HS_DET_B_GPIO);
	destroy_workqueue(data->hs_workqueue);
	wake_lock_destroy(&data->hs_work_wakelock);
reg_put:
	regulator_put(data->regulator);
free_mem:
	kfree(data);

	return retval;
}

static int __exit cpcap_3mm5_remove(struct platform_device *pdev)
{
	struct cpcap_3mm5_data *data = platform_get_drvdata(pdev);

	disable_irq_wake(gpio_to_irq(HS_DET_B_GPIO));
	gpio_free(HS_DET_B_GPIO);

	cancel_work_sync(&data->hs_work);
	destroy_workqueue(data->hs_workqueue);
	wake_lock_destroy(&data->hs_work_wakelock);

	cancel_delayed_work_sync(&data->work);
	cancel_delayed_work_sync(&data->key_work);

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_MB2);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_13);

	switch_dev_unregister(&data->sdev);
	regulator_put(data->regulator);

	kfree(data);
	return 0;
}

static struct platform_driver cpcap_3mm5_driver = {
	.probe		= cpcap_3mm5_probe,
	.remove		= __exit_p(cpcap_3mm5_remove),
#ifdef CONFIG_PM
	.suspend		= cpcap_3mm5_suspend,
	.resume		= cpcap_3mm5_resume,
#endif
	.driver		= {
		.name	= "cpcap_3mm5",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_3mm5_init(void)
{
	return cpcap_driver_register(&cpcap_3mm5_driver);
}
module_init(cpcap_3mm5_init);

static void __exit cpcap_3mm5_exit(void)
{
	platform_driver_unregister(&cpcap_3mm5_driver);
}
module_exit(cpcap_3mm5_exit);

MODULE_ALIAS("platform:cpcap_3mm5");
MODULE_DESCRIPTION("CPCAP USB detection driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
