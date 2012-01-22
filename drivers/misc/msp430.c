/*
 * Copyright (C) 2010 Motorola, Inc.
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
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/msp430.h>
#include <linux/msp_memory_map.h>
#include <linux/wakeup_timer_kernel.h>

#include <linux/wakelock.h>
#include <linux/spi/cpcap.h>


#define NAME			     "msp430"

#define I2C_RETRY_DELAY              5
#define I2C_RETRIES                  5
#define I2C_RESPONSE_LENGTH          8
#define MSP430_MAXDATA_LENGTH        250
#define MSP430_DELAY_USEC            10
#define MSP430_RESPONSE_MSG          0x3b
#define MSP430_RESPONSE_MSG_SUCCESS  0x00
#define MSP430_CRC_SEED              0xffff
#define MSP430_COMMAND_HEADER        0x80
#define MSP430_CRC_LENGTH            2
#define MSP430_OPCODE_LENGTH         1
#define MSP430_ADDRESS_LENGTH        3
#define MSP430_CORECOMMAND_LENGTH    (MSP430_OPCODE_LENGTH +\
					MSP430_MAXDATA_LENGTH +\
					MSP430_ADDRESS_LENGTH)
#define MSP430_HEADER_LENGTH         1
#define MSP430_CMDLENGTH_BYTES	     2
#define G_MAX                        0x7FFF
#define MSP430_INITIAL_INACTIVITY_TIMEOUT   600000 /* 10 mins in miliseconds */
#define MSP430_MAX_INACTIVITY_TIMEOUT      3600000 /*  1 hour in miliseconds */
#define KDEBUG(format, s...) 	if (g_debug)\
		printk(format, ##s);

#define MAX_PASSIVE_BUFFER_SIZE 12
#define NEW_EPOCH_DIFFERENCE  1262304000
/*29 manual calibration bytes, 2 stray, 22 gps calibration bytes */
#define CALIB_TABLE_SIZE 53
#define WAITING_FOR_MOTION           0xaa   /* Barker representing no motion */
char g_debug;
unsigned char firm_cmdbuff[MSP430_HEADER_LENGTH + MSP430_CMDLENGTH_BYTES +
			MSP430_CORECOMMAND_LENGTH + MSP430_CRC_LENGTH];


enum msp_mode {
	UNINITIALIZED,
	BOOTMODE,
	NORMALMODE,
	FACTORYMODE
};

struct msp430_pedometer_data {
	unsigned int  activity;
	unsigned int   distance;
	unsigned short stepcount;
	unsigned short speed;
};

struct msp430_passive_data {
	unsigned int  activity_level;
	unsigned short stepcount;
	unsigned int   timestamp;
};


struct msp430_data {
	struct i2c_client *client;
	struct msp430_platform_data *pdata;
	/* to avoid two i2c communications at the same time */
	struct mutex lock;
#ifdef G1
	struct delayed_work acc_work;
	struct delayed_work env_work;
	struct delayed_work mag_work;
	struct delayed_work orin_work;
#endif
	struct delayed_work monitor_work;
	struct input_dev *input_dev;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
	struct work_struct passive_work;
	struct work_struct inactivity_work;
	struct timer_cascade_root *waketimer;
	struct timer_cascade_root *inactivity_timer;
	struct wake_lock wake_lock;
	struct wake_lock offmode_lock;
	struct wake_lock timed_lock;
	int hw_initialized;
#ifdef G1
	int acc_poll_interval;
	int env_poll_interval;
	int mag_poll_interval;
	int orin_poll_interval;
#endif
	int motion_poll_interval;
	int monitor_poll_interval;
	int orin_poll_interval;
	atomic_t enabled;
	int irq;
	unsigned int current_addr;
	enum msp_mode mode;
	int in_activity ;
	struct msp430_pedometer_data prev_data;
	struct msp430_passive_data prev_passive_data;
	unsigned char intp_mask;
	unsigned char equipment;
	unsigned int inactivity_timeout;
	bool psuedo_isr;
	bool offmode_wakeups_disabled;
	unsigned char calib_table[CALIB_TABLE_SIZE];
};

enum msp_commands {
	PASSWORD_RESET,
	MASS_ERASE,
	PROGRAM_CODE,
	END_FIRMWARE,
	PASSWORD_RESET_DEFAULT
};

enum msp_opcode {
	PASSWORD_OPCODE = 0x11,
	MASSERASE_OPCODE = 0x15,
	RXDATA_OPCODE = 0x10,
};

enum msp_powermodes {
	POWER_NORMAL_MODE = 0x01,
	POWER_ANY_MOTION_MODE = 0x02,
	POWER_SLEEP_MODE = 0x03
};

/* Different activities/gestures detected */
enum msp_activities {
	STILL = 0x00,
	WALK = 0x01,
	RUN = 0x03,
	TAP = 0x0A,
	ANY_MOTION = 0x0B,
	CALIBRATION_COMPLETE = 0x0E
};

enum equipment_type {
	NONE,
	TREADMILL,
	ELLIPTICAL,
	STAIRMASTER
};

struct msp_response {

	/* 0x0080 */
	unsigned short header;
	unsigned char len_lsb;
	unsigned char len_msb;
	unsigned char cmd;
	unsigned char data;
	unsigned char crc_lsb;
	unsigned char crc_msb;
};

static unsigned short crc_table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
  0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
  0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
  0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
  0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
  0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
  0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
  0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
  0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
  0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969,
  0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
  0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
  0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,
  0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
  0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6,
  0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
  0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
  0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1,
  0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
  0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
  0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
  0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447,
  0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
  0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2,
  0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
  0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
  0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
  0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0,
  0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
  0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
  0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba,
  0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
  0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

struct msp430_data *msp430_misc_data;

struct msp430_passive_data pdata_buffer[12] = {};


static int msp430_i2c_write_read(struct msp430_data *ps_msp430, u8 *buf,
			int writelen, int readlen)
{
	int tries, err = 0;
	struct msp_response *response;
	struct i2c_msg msgs[] = {
		{
		.addr = ps_msp430->client->addr,
		.flags = ps_msp430->client->flags,
		.len = writelen,
		.buf = buf,
		},
		{
		.addr = ps_msp430->client->addr,
		.flags = ps_msp430->client->flags | I2C_M_RD,
		.len = readlen,
		.buf = buf,
		},
	};
	if (ps_msp430->mode == FACTORYMODE)
		return err;
	if (buf == NULL || writelen == 0 || readlen == 0)
		return -EFAULT;

	if (ps_msp430->mode == BOOTMODE) {
		KDEBUG(KERN_INFO "In msp430_i2c_write_read\n");
		KDEBUG(KERN_INFO "sending: ");
		for (tries = 0; tries < writelen; tries++)
			KDEBUG(KERN_INFO "%02x", buf[tries]);
		KDEBUG(KERN_INFO "\n");
	}
	tries = 0;
	do {
		err = i2c_transfer(ps_msp430->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));
	if (err != 2) {
		dev_err(&ps_msp430->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
		KDEBUG(KERN_INFO "Read from MSP: ");
		for (tries = 0; tries < readlen; tries++)
			KDEBUG(KERN_INFO "%02x", buf[tries]);
		KDEBUG(KERN_INFO "\n");
		if (ps_msp430->mode == BOOTMODE) {
			response = (struct msp_response *) buf;
			if ((response->cmd == MSP430_RESPONSE_MSG &&
			response->data != MSP430_RESPONSE_MSG_SUCCESS) ||
			(response->cmd != MSP430_RESPONSE_MSG))	 {
				printk(KERN_ERR "i2c command returned failure\n");
				err = -EIO;
			}
		}
	}
	return err;
}


static int msp430_i2c_read(struct msp430_data *ps_msp430, u8 *buf, int len)
{
	int tries, err = 0;

	if (ps_msp430->mode == FACTORYMODE)
		return err;
	if (buf == NULL || len == 0)
		return -EFAULT;
	tries = 0;
	do {
		err = i2c_master_recv(ps_msp430->client, buf, len);
		if (err < 0)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err < 0) && (++tries < I2C_RETRIES));
	if (err < 0) {
		dev_err(&ps_msp430->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		KDEBUG("Read was successsful: \n");
		for (tries = 0; tries < err ; tries++)
			KDEBUG(KERN_INFO "%02x", buf[tries]);
		KDEBUG(KERN_INFO "\n");
	}
	return err;
}

static int msp430_i2c_write(struct msp430_data *ps_msp430, u8 * buf, int len)
{
	int err = 0;
	int tries = 0;

	if (ps_msp430->mode == FACTORYMODE)
		return err;
	KDEBUG(" Writing: \n");
	for (tries = 0; tries < len ; tries++)
		KDEBUG(KERN_INFO "%02x", buf[tries]);
	KDEBUG(KERN_INFO "\n");
	tries = 0;
	do {
		err = i2c_master_send(ps_msp430->client, buf, len);
		if (err < 0)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err < 0) && (++tries < I2C_RETRIES));

	if (err < 0) {
		dev_err(&ps_msp430->client->dev, "msp430: write error\n");
		err = -EIO;
	} else {
		KDEBUG(KERN_INFO "msp430 i2c write successful \n");
		err = 0;
	}
	return err;
}

static int msp430_hw_init(struct msp430_data *ps_msp430)
{
	int err = 0;
	KDEBUG(KERN_INFO "in  msp430_hw_init\n");
	ps_msp430->hw_initialized = 1;
	return err;
}

static void msp430_device_power_off(struct msp430_data *ps_msp430)
{
	KDEBUG(KERN_INFO " in msp430_device_power_off\n");
	if (ps_msp430->pdata->power_off) {
		ps_msp430->pdata->power_off();
		ps_msp430->hw_initialized = 0;
	}
}

static int msp430_device_power_on(struct msp430_data *ps_msp430)
{
	int err = 0;
	KDEBUG(KERN_INFO  "In msp430_device_power_on\n");
	if (ps_msp430->pdata->power_on) {
		err = ps_msp430->pdata->power_on();
		if (err < 0) {
			dev_err(&ps_msp430->client->dev,
				"power_on failed: %d\n", err);
			return err;
		}
	}
	if (!ps_msp430->hw_initialized) {
		err = msp430_hw_init(ps_msp430);
		if (err < 0) {
			msp430_device_power_off(ps_msp430);
			return err;
		}
	}
	return err;
}



static void msp430_report_pedometer_values(struct msp430_data *ps_msp430,
				struct msp430_pedometer_data *curr_data)
{
	unsigned short delta_stepcount;
	unsigned int delta_distance;
	unsigned int delta_activity;
	int latitude, longitude;
	char accuracy;
	short heading;
	int err;
	unsigned char buff[10];

	/*printk("msp430_report_pedometer_values\n");*/

	delta_stepcount = curr_data->stepcount - ps_msp430->prev_data.stepcount;
	delta_distance = curr_data->distance - ps_msp430->prev_data.distance;
	delta_activity = curr_data->activity - ps_msp430->prev_data.activity;

	ps_msp430->prev_data.stepcount = curr_data->stepcount;
	ps_msp430->prev_data.distance = curr_data->distance;
	ps_msp430->prev_data.activity = curr_data->activity;
	input_report_rel(ps_msp430->input_dev, REL_STEPCOUNT,
			delta_stepcount);
	input_report_rel(ps_msp430->input_dev, REL_SPEED,
			curr_data->speed);
	input_report_rel(ps_msp430->input_dev, REL_DISTANCE,
			delta_distance);
	input_report_rel(ps_msp430->input_dev, REL_ACTIVITY_TYPE,
			delta_activity);
	input_sync(ps_msp430->input_dev);
	KDEBUG(KERN_INFO "Sending motion data : stepcount = %d,\
	speed = %d, distance = %d , activity = %d\n", delta_stepcount,\
	curr_data->speed, delta_distance, delta_activity);

	/* Read location data */
	buff[0] = MSP_LATITUDE;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 4);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	latitude = (buff[0] << 24) | (buff[1] << 16)\
		| (buff[2] << 8) | buff[3];
	buff[0] = MSP_LONGITUDE;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 4);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	longitude = (buff[0] << 24) | (buff[1] << 16)\
		| (buff[2] << 8) | buff[3];
	buff[0] = WALK_DIRN_HEAD;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 2);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	heading = (buff[0] << 8) | buff[1];
	buff[0] = WALK_DIRN_QUALITY;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 1);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	accuracy =  buff[0];
	input_report_abs(ps_msp430->input_dev, ABS_MSP_LATITUDE, latitude);
	input_report_abs(ps_msp430->input_dev, ABS_MSP_LONGITUDE, longitude);
	input_report_abs(ps_msp430->input_dev, ABS_MSP_HEADING, (int)heading);
	input_report_abs(ps_msp430->input_dev, ABS_MSP_ACCURACY, (int)accuracy);
	input_sync(ps_msp430->input_dev);
	KDEBUG(" Sending locdata : lat = %d, lon = %d, heading = %d, acc = %d",
		latitude, longitude, (int)heading, (int)accuracy);
}

static void msp430_read_passive_data(struct msp430_data *ps_msp430,
		struct msp430_passive_data *passive_data)
{

	int err;
	int buffer_count;
	unsigned char buff[10];
	passive_data->activity_level = 0;
	passive_data->stepcount = 0;

	for (buffer_count = 0; buffer_count < MAX_PASSIVE_BUFFER_SIZE; \
	buffer_count++) {
		if (pdata_buffer[0].timestamp == (-1)*NEW_EPOCH_DIFFERENCE) {
			KDEBUG("TIMESTAMP FOR BUFFER 0 IS 0");
			buffer_count = 0;
		}
		buff[0] = MONITORME_60SEC_METS;
		err = msp430_i2c_write_read(ps_msp430, buff, 1, 7);
		if (err < 0) {
			printk(KERN_INFO "Reading from msp failed\n");
			return;
		}
		pdata_buffer[buffer_count].activity_level = buff[0];
		pdata_buffer[buffer_count].stepcount = (buff[1] << 8) \
		| (buff[2]);
		pdata_buffer[buffer_count].timestamp = (buff[3] << 24) \
		| (buff[4] << 16) | (buff[5] << 8) | \
		(buff[6]);
		pdata_buffer[buffer_count].timestamp = \
		pdata_buffer[buffer_count].timestamp - NEW_EPOCH_DIFFERENCE;
		KDEBUG("Buffer - %d :Activity level = %d, \
		Step Count = %d, Timestamp = %d", buffer_count, \
		pdata_buffer[buffer_count].activity_level,  \
		pdata_buffer[buffer_count].stepcount, \
		pdata_buffer[buffer_count].timestamp);
	}
	/*msp_cmdbuff[0] = MONITORME_60SEC_METS;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 1);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	passive_data->activity_level = msp_cmdbuff[0];

	msp_cmdbuff[0] = TOTAL_STEP_COUNT;

	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 2);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	passive_data->stepcount = (msp_cmdbuff[0] << 8) | (msp_cmdbuff[1]);*/

}

static void msp430_report_passive_values(struct msp430_data *ps_msp430,
		struct msp430_passive_data *passive_data)
{
	int i;
	static int passive_timestamp_start = 1308070005;
	static int buffer_count;
	unsigned int delta_stepcount;
	unsigned int delta_activity_level;

	/*printk("VIDI: Inside send_buffer_to_app\n");
	printk("VIDI : curr_pass_steps = %d prev_pass_steps=%d\n",
	passive_data->stepcount,  ps_msp430->prev_passive_data.stepcount);
	*/
	/*delta_stepcount = passive_data->stepcount - \
	ps_msp430->prev_passive_data.stepcount;
	delta_activity_level = passive_data->activity_level - \
	ps_msp430->prev_passive_data.activity_level;*/

	/*printk("VIDI: buffer_count=%d delta_steps=%d pass_time=%d\n",
	buffer_count,delta_stepcount,passive_timestamp_start);*/

	/*ps_msp430->prev_passive_data.stepcount =  passive_data->stepcount;
	ps_msp430->prev_passive_data.activity_level = \
	passive_data->activity_level;

	pdata_buffer[buffer_count].activity_level = delta_activity_level;
	pdata_buffer[buffer_count].stepcount = delta_stepcount;
	pdata_buffer[buffer_count].timestamp = passive_timestamp_start;
	buffer_count++;
	passive_timestamp_start += 5;

	if (buffer_count == MAX_PASSIVE_BUFFER_SIZE) {
		buffer_count = 0;

		for (i = 0; i < MAX_PASSIVE_BUFFER_SIZE; i++) {
			input_report_rel(ps_msp430->input_dev, REL_WHEEL, \
			pdata_buffer[i].stepcount);
			input_report_rel(ps_msp430->input_dev, \
			REL_ACTIVITY_LEVEL, pdata_buffer[i].activity_level);
			input_report_rel(ps_msp430->input_dev, REL_HWHEEL, \
			pdata_buffer[i].timestamp);
			input_sync(ps_msp430->input_dev);
		}
	}*/
	for (i = 0; i < MAX_PASSIVE_BUFFER_SIZE; i++) {
		input_report_rel(ps_msp430->input_dev, REL_WHEEL, \
		pdata_buffer[i].stepcount);
		input_report_rel(ps_msp430->input_dev, \
		REL_ACTIVITY_LEVEL, pdata_buffer[i].activity_level);
		input_report_rel(ps_msp430->input_dev, REL_HWHEEL, \
		pdata_buffer[i].timestamp);
		input_sync(ps_msp430->input_dev);
	}
}

static void msp430_read_motion_data(struct msp430_data *ps_msp430,
		struct msp430_pedometer_data *pedo)
{

	int err;
	unsigned char buff[10];

	/* For G2 use activity_level */

	buff[0] = MONITORME_10SEC_METS;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 4);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	pedo->activity =  (buff[0] << 24)\
		| (buff[1] << 16) | (buff[2] << 8)\
		| buff[3];

	if ((ps_msp430->equipment == NONE) ||
		(ps_msp430->equipment == TREADMILL)) {
		buff[0] = TOTAL_STEP_COUNT;
	} else if (ps_msp430->equipment == ELLIPTICAL) {
		buff[0] = TOTAL_STEP_COUNT;
	} else if (ps_msp430->equipment == STAIRMASTER) {
		buff[0] = TOTAL_STEP_COUNT;
	}

	err = msp430_i2c_write_read(ps_msp430, buff, 1, 2);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	pedo->stepcount = (buff[0] << 8) | (buff[1]);

	buff[0] = SPEED;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 2);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	pedo->speed = (buff[0] << 8) | (buff[1]);

	buff[0] = TOTAL_DISTANCE;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 4);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		return;
	}
	pedo->distance =  (buff[0] << 24)\
		| (buff[1] << 16) | (buff[2] << 8)\
		| buff[3];

}

static void msp430_kick_idletimer(struct msp430_data *ps_msp430){

	int err;
	unsigned char buff[10];

	if (!wake_lock_active(&(ps_msp430->offmode_lock)))
		wake_lock(&(ps_msp430->offmode_lock));

	buff[0] = OMAP_OFF_MODE;
	buff[1] = WAITING_FOR_MOTION;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < 0)
		printk(KERN_ERR "Unable to set MSP to waiting for motion\n");

	KDEBUG(KERN_INFO "%s() set regsiter 0x%x to 0x%x\n", __func__,
			  buff[0], buff[1]);

	wakeup_stop_status_timer(ps_msp430->inactivity_timer);
	wakeup_start_status_timer(ps_msp430->inactivity_timer,
				  ps_msp430->inactivity_timeout);
}

static int msp430_handle_inactivity_timeout(void){

	struct msp430_data *ps_msp430 = msp430_misc_data;

	queue_work(ps_msp430->irq_work_queue, &ps_msp430->inactivity_work);

	return 0;
}

void msp430_psuedo_isr(void){
	msp430_misc_data->psuedo_isr = true;
}

static void msp430_out_of_offmode(struct msp430_data *ps_msp430)
{
	int err;
	unsigned char buff[10];

	/* Send event up to user space */
	input_report_rel(ps_msp430->input_dev, REL_DIAL, ANY_MOTION+1);
	input_sync(ps_msp430->input_dev);

	/* Tell MSP we're out of offmode by clearning ANY_MOTION int*/
	ps_msp430->intp_mask = ps_msp430->intp_mask & ~M_ANY_MOTION;
	buff[0] = INTERRUPT_MASK;
	buff[1] = ps_msp430->intp_mask;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < 0)
		printk(KERN_ERR "Unable to disable any motion irq\n");

	ps_msp430->inactivity_timeout = MSP430_INITIAL_INACTIVITY_TIMEOUT;
	printk(KERN_INFO "%s() disabled offmode & start %d second timer\n",
			 __func__, ps_msp430->inactivity_timeout/1000);

	msp430_kick_idletimer(ps_msp430);
}

/* Interface to disable offmode if it is enabled */
void msp430_disable_offmode(void){

	if (msp430_misc_data) {
		/* Check if we are currently in off mode */
		if (msp430_misc_data->intp_mask & M_ANY_MOTION) {
			msp430_out_of_offmode(msp430_misc_data);
		} else
			msp430_kick_idletimer(msp430_misc_data);
	}
}

static irqreturn_t msp430_isr(int irq, void *dev)
{
	struct msp430_data *ps_msp430 = dev;

	if (ps_msp430->psuedo_isr)
		KDEBUG(KERN_INFO "%s() There is a psuedo ISR scheduled, but"
				 " queing work anyway\n", __func__);

	if (!wake_lock_active(&(ps_msp430->wake_lock)))
		wake_lock(&(ps_msp430->wake_lock));

	queue_work(ps_msp430->irq_work_queue, &ps_msp430->irq_work);
	return IRQ_HANDLED;
}

static void msp430_irq_work_func(struct work_struct *work)
{
	int err;
	unsigned char irq_status;
	struct msp430_data *ps_msp430 = container_of(work,
			struct msp430_data, irq_work);
	struct msp430_pedometer_data pedo;
	struct msp430_passive_data passive_data;
	unsigned char buff[10];

	if (ps_msp430->mode == BOOTMODE)
		goto RETURN;

	KDEBUG(KERN_INFO "In msp430_irq_work_func\n");
	mutex_lock(&ps_msp430->lock);


	/* read interrupt mask register */
	buff[0] = INTERRUPT_STATUS;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 1);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	irq_status = buff[0];
	KDEBUG(KERN_INFO "%s() got irq = 0x%x; wakelock = %d\n", __func__,
	       irq_status, wake_lock_active(&(ps_msp430->wake_lock)));
	/*For debugging purpose reading interrupt mask*/
	buff[0] = INTERRUPT_MASK;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 1);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
#ifdef G1
	if (irq_status & M_ACTIVITY_CHANGE) {
		msp430_read_motion_data(ps_msp430, &pedo);
		msp430_report_pedometer_values(ps_msp430, &pedo);
		if (ps_msp430->prev_data.activity == STILL) {
			KDEBUG(KERN_INFO "detected STILL stop wakeup timer\n");
			wakeup_stop_status_timer(ps_msp430->waketimer);
		} else {
			KDEBUG(KERN_INFO "Restart wakeup timer\n");
			wakeup_start_status_timer(ps_msp430->waketimer,
			ps_msp430->motion_poll_interval);
		}
	}
#endif

	if (irq_status & M_ACTIVITY_CHANGE) {
		buff[0] = ACTIVITY_DETECTION;
		err = msp430_i2c_write_read(ps_msp430, buff, 1, 1);
		if (err < 0) {
			printk(KERN_INFO "Reading from msp failed\n");
			goto EXIT;
		}
		/* Add 1 to allow still(0) to pass up a REL event */
		KDEBUG(KERN_INFO "Got act_change act=%d\n", buff[0]+1);
		input_report_rel(ps_msp430->input_dev, REL_DIAL,
				buff[0]+1);
		input_sync(ps_msp430->input_dev);

		/* Hold a 500ms wakelock for starting a RUN */
		if (buff[0] == 3)
			wake_lock_timeout(&ps_msp430->timed_lock, 0.5 * HZ);
	}


	if (irq_status & M_MME_60) {
		KDEBUG(KERN_INFO "detected PASSIVE_MODE\n");
		msp430_read_passive_data(ps_msp430, &passive_data);
		msp430_report_passive_values(ps_msp430, &passive_data);
		/* Hold a 500ms wakelock to let data get to LiveDataHandler */
		wake_lock_timeout(&ps_msp430->timed_lock, 0.5 * HZ);
#ifdef G1
		/*send activity level */
		printk(KERN_INFO "Sending 60 sec mme = %d", buff[0]);
		input_report_rel(ps_msp430->input_dev, REL_ACTIVITY_LEVEL,
			buff[0]);
		input_sync(ps_msp430->input_dev);
#endif
	}
	if (irq_status & M_ACTIVE_MODE) {
		KDEBUG(KERN_INFO "detected ACTIVE_MODE\n");
		msp430_read_motion_data(ps_msp430, &pedo);
		msp430_report_pedometer_values(ps_msp430, &pedo);

	}
	if (irq_status & M_TAP_TAP) {
		KDEBUG(KERN_INFO "Got tap interrupt\n");
		/* Add 1 to allow still(0) to pass up a REL event */
		input_report_rel(ps_msp430->input_dev, REL_DIAL, TAP+1);
		input_sync(ps_msp430->input_dev);

		/* If ANY_MOTION was enabled, then we were in offmode */
		if (ps_msp430->intp_mask & M_ANY_MOTION) {
			printk(KERN_INFO "%s() Got TAP interrupt; disabling "
					 "offmode & kicking timer\n", __func__);
			msp430_out_of_offmode(ps_msp430);
		}
	}
	if (irq_status & M_ANY_MOTION) {
		printk(KERN_INFO "%s() Got any motion interrupt; disabling "
				 "offmode & kicking timer\n", __func__);
		msp430_out_of_offmode(ps_msp430);
	}
	if (irq_status & M_CALIBRATION_COMPLETE_WALK) {
		KDEBUG(KERN_INFO "Walk Calibration complete !!! \n");
		input_report_rel(ps_msp430->input_dev, REL_DIAL,
			CALIBRATION_COMPLETE+1);
		input_sync(ps_msp430->input_dev);
	}
	if (irq_status & M_CALIBRATION_COMPLETE_RUN) {
		KDEBUG(KERN_INFO "Run calibration complete !!!\n");
		input_report_rel(ps_msp430->input_dev, REL_DIAL,
			CALIBRATION_COMPLETE+1);
		input_sync(ps_msp430->input_dev);
	}
	if (irq_status & M_CALIBRATION_COMPLETE_JOG) {
		KDEBUG(KERN_INFO "Jog calibration complete !!!\n");
		input_report_rel(ps_msp430->input_dev, REL_DIAL,
			CALIBRATION_COMPLETE+1);
		input_sync(ps_msp430->input_dev);
	}
EXIT:
	mutex_unlock(&ps_msp430->lock);
RETURN:
	wake_unlock(&ps_msp430->wake_lock);
}

static int msp430_check_for_motion(struct msp430_data *ps_msp430)
{

	int err;
	unsigned char buff[10];

	/* Check MSP; see if motion occured since we cleared status register */
	buff[0] = OMAP_OFF_MODE;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 1);
	if (err < 0) {
		printk(KERN_ERR "Reading motion state failed\n");
		return err;
	}

	if (buff[0] != WAITING_FOR_MOTION) {
		KDEBUG("%s() Motion detected (0x%x)\n", __func__,
			buff[0]);
		msp430_kick_idletimer(ps_msp430);
		return 1;
	}

	KDEBUG("%s() No motion detected (0x%x)\n", __func__, buff[0]);

	return 0;
}

static void msp430_inactivity_work_func(struct work_struct *work)
{

	struct msp430_data *ps_msp430 = container_of(work,
		struct msp430_data, inactivity_work);

	if (msp430_check_for_motion(ps_msp430)) {
		/*
		 * Scale up the inactivity timer up when there is constant
		 * motion to reduce the wakeup frequency
		 */
		ps_msp430->inactivity_timeout *= 2;
		if (ps_msp430->inactivity_timeout >
		    MSP430_MAX_INACTIVITY_TIMEOUT)
			ps_msp430->inactivity_timeout =
				MSP430_MAX_INACTIVITY_TIMEOUT;
		printk(KERN_INFO "%s() Motion detected; Incrementing "
				 "inactivity timeout to %d seconds\n",
				 __func__, ps_msp430->inactivity_timeout/1000);
		/* Kick the timer again so the new timeout takes effect */
		msp430_kick_idletimer(ps_msp430);
	} else {
		printk(KERN_INFO "%s() No motion detected; removing offlock\n",
				 __func__);
		wake_unlock(&(ps_msp430->offmode_lock));
	}
}

static void msp430_passive_work_func(struct work_struct *work)
{
	struct msp430_data *ps_msp430 = container_of(work,
		struct msp430_data, passive_work);
	struct msp430_pedometer_data curr;
	if (ps_msp430->mode == BOOTMODE)
		return;
	mutex_lock(&ps_msp430->lock);
	/* read current motion data */
	msp430_read_motion_data(ps_msp430, &curr);
	msp430_report_pedometer_values(ps_msp430, &curr);
	mutex_unlock(&ps_msp430->lock);


}

static void msp430_stop_timers(struct msp430_data *ps_msp430)
{
	KDEBUG("Stop all timers\n");
#ifdef G1
	cancel_delayed_work(&ps_msp430->acc_work);
	cancel_delayed_work(&ps_msp430->env_work);
	cancel_delayed_work(&ps_msp430->mag_work);
	cancel_delayed_work(&ps_msp430->orin_work);
	cancel_delayed_work(&ps_msp430->monitor_work);
#endif
}

static void msp430_restart_timers(struct msp430_data *ps_msp430)
{

	KDEBUG("Restart active timers\n");
#ifdef G1
	if (ps_msp430->acc_poll_interval > 0) {
		schedule_delayed_work(&ps_msp430->acc_work,
		msecs_to_jiffies(ps_msp430->acc_poll_interval));
	}
	if (ps_msp430->env_poll_interval > 0) {
		schedule_delayed_work(&ps_msp430->env_work,
		msecs_to_jiffies(ps_msp430->env_poll_interval));
	}
	if (ps_msp430->mag_poll_interval > 0) {
		schedule_delayed_work(&ps_msp430->mag_work,
			msecs_to_jiffies(ps_msp430->mag_poll_interval));
	}
	if (ps_msp430->orin_poll_interval > 0) {
		schedule_delayed_work(&ps_msp430->orin_work,
			msecs_to_jiffies(ps_msp430->orin_poll_interval));
	}
	if ((ps_msp430->monitor_poll_interval != 60000) &&
		(ps_msp430->monitor_poll_interval > 0)) {
		schedule_delayed_work(&ps_msp430->monitor_work,
			msecs_to_jiffies(ps_msp430->monitor_poll_interval));
	}
#endif
}

static int msp430_set_powermode(enum msp_powermodes mode,
			struct msp430_data *ps_msp430)
{
	int err = 0;
	unsigned char buff[10];

	buff[0] = POWER_MODE;
	buff[1] = mode;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < 0)
		printk(KERN_ERR "Unable to put msp to sleep\n");
	return err;
}

static int msp430_enable(struct msp430_data *ps_msp430)
{
	int err = 0;


	KDEBUG(KERN_INFO "msp430_enable\n");
	if (!atomic_cmpxchg(&ps_msp430->enabled, 0, 1)) {
		err = msp430_device_power_on(ps_msp430);
		if (err < 0) {
			atomic_set(&ps_msp430->enabled, 0);
			return err;
		}

	}

	return err;
}

static int msp430_disable(struct msp430_data *ps_msp430)
{
	int err = 0;

	KDEBUG(KERN_INFO "msp430_disable\n");
	if (atomic_cmpxchg(&ps_msp430->enabled, 1, 0))
		msp430_device_power_off(ps_msp430);
	return err;
}

static int msp430_misc_open(struct inode *inode, struct file *file)
{
	int err = 0;
	KDEBUG(KERN_INFO "msp430_misc_open\n");

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;
	file->private_data = msp430_misc_data;

	err = msp430_enable(msp430_misc_data);

	return err;
}

unsigned short msp430_calculate_crc(unsigned char *data, size_t length)
{
	size_t count;
	unsigned int crc = MSP430_CRC_SEED;
	unsigned int temp;
	unsigned short final = 0;
	for (count = 0; count < length; ++count) {
		temp = (*data++ ^ (crc >> 8)) & 0xff;
		crc = crc_table[temp] ^ (crc << 8);
	}
	return (unsigned short)(crc ^ final);
}

/* the caller function is resposible to free mem allocated in this function. */
void msp430_build_command(enum msp_commands cmd,
		const char *inbuff, unsigned int *length)
{
	unsigned int index = 0, i, len = *length;
	unsigned short corecmdlen;
	unsigned short crc;
	struct msp430_data *ps_msp = msp430_misc_data;

	/*allocate for the msp command */
	firm_cmdbuff[index++] = MSP430_COMMAND_HEADER; /* header */
	switch (cmd) {
	case PASSWORD_RESET:
		firm_cmdbuff[index++] = 0x21; /* len LSB */
		firm_cmdbuff[index++] = 0x00; /* len MSB */
		firm_cmdbuff[index++] = PASSWORD_OPCODE; /* opcode */
		for (i = 4; i < 36; i++) /* followed by 30 FFs */
			firm_cmdbuff[index++] = 0xff;
		firm_cmdbuff[index++] = 0x9e; /* CRC LSB */
		firm_cmdbuff[index++] = 0xe6; /* CRC MSB */
		break;
	case MASS_ERASE:
		firm_cmdbuff[index++] = 0x01; /* len LSB */
		firm_cmdbuff[index++] = 0x00; /* len MSB */
		firm_cmdbuff[index++] = MASSERASE_OPCODE; /* opcode */
		firm_cmdbuff[index++] = 0x64; /* crc LSB */
		firm_cmdbuff[index++] = 0xa3; /* crc MSB */
		break;
	case PROGRAM_CODE:
		/*code length */
		KDEBUG(KERN_INFO "No of bytes got from user = %d", len);
		corecmdlen = len + MSP430_OPCODE_LENGTH + MSP430_ADDRESS_LENGTH;
		firm_cmdbuff[index++] =
			(unsigned char)(corecmdlen & 0xff); /* LSB len */
		firm_cmdbuff[index++] =
			(unsigned char)((corecmdlen >> 8) & 0xff);/*MSB len*/
		firm_cmdbuff[index++] = RXDATA_OPCODE; /* opcode */
		/* LSB of write addr on MSP */
		firm_cmdbuff[index++] =
			(unsigned char)(ps_msp->current_addr & 0xff);
		/* middle byte of write addr */
		firm_cmdbuff[index++] =
			(unsigned char)((ps_msp->current_addr >> 8) & 0xff);
		/* MSB of write addr on MSP */
		firm_cmdbuff[index++] =
			(unsigned char)((ps_msp->current_addr >> 16) & 0xff);
		/* copy data from user to kernel space */
		if (copy_from_user(firm_cmdbuff+index, inbuff, len)) {
			printk(KERN_INFO "copy from user returned error\n");
			index = 0;
		} else {
			index += len; /*increment index with data len*/
			crc = msp430_calculate_crc(firm_cmdbuff+3, len+1+3);
			/* crc LSB */
			firm_cmdbuff[index++] = (unsigned char)(crc & 0xff);
			/* crc MSB */
			firm_cmdbuff[index++] =
				(unsigned char)((crc >> 8)&0xff);
		}
		break;
	case END_FIRMWARE:
		firm_cmdbuff[index++] = 0x06; /* len LSB */
		firm_cmdbuff[index++] = 0x00; /* len MSB */
		firm_cmdbuff[index++] = RXDATA_OPCODE; /* opcode */
		firm_cmdbuff[index++] = 0xfe;
		firm_cmdbuff[index++] = 0xff;
		firm_cmdbuff[index++] = 0x00;
		firm_cmdbuff[index++] = 0x00;
		firm_cmdbuff[index++] = 0x44;
		firm_cmdbuff[index++] = 0x89; /* crc LSB */
		firm_cmdbuff[index++] = 0xa7; /* crc MSB */
		break;
	case PASSWORD_RESET_DEFAULT:
		firm_cmdbuff[index++] = 0x21; /* len LSB */
		firm_cmdbuff[index++] = 0x00; /* len MSB */
		firm_cmdbuff[index++] = PASSWORD_OPCODE; /* opcode */
		for (i = 0; i < 32; i++) /* followed by 30 FFs */
			firm_cmdbuff[index++] = 0xff;
		firm_cmdbuff[index++] = 0x9e; /* CRC LSB */
		firm_cmdbuff[index++] = 0xe6; /* CRC MSB */
		break;
	default:
		printk(KERN_INFO  "Invalid msp430 cmd \n");
		index = 0;
		break;
	}
	/*command length */
	*length = index;
}

static ssize_t msp430_misc_write(struct file *file, const char __user *buff,
				 size_t count, loff_t *ppos)
{
	int i, err = 0;
	struct msp430_data *ps_msp430;
	unsigned int len = (unsigned int)count;

	KDEBUG(KERN_INFO "msp430_misc_write\n");
	ps_msp430 = msp430_misc_data;
	if (len > MSP430_MAXDATA_LENGTH || len == 0) {
		printk(KERN_ERR "Error packet size is more \
			than MSP430_MAXDATA_LENGTH or 0\n");
		err = -EINVAL;
		return err;
	}
	KDEBUG(KERN_INFO "Got from user: ");
	for (i = 0; i < len; i++)
		KDEBUG(KERN_INFO "%02x", buff[i]); /*debug */
	KDEBUG(KERN_INFO "\n Leng = %d", len); /* debug */

	if (ps_msp430->mode == BOOTMODE) {
		KDEBUG(KERN_INFO " msp430_misc_write: boot mode\n");
		/* build the msp430 command to program code */
		msp430_build_command(PROGRAM_CODE, buff, &len);
		err = msp430_i2c_write_read(ps_msp430, firm_cmdbuff,
				 len, I2C_RESPONSE_LENGTH);
		/* increment the current MSP write addr by count */
		if (err == 0) {
			msp430_misc_data->current_addr += count;
			/* return the number of bytes successfully written */
			err = len;
		}
	} else {
		KDEBUG(KERN_INFO "msp430_misc_write: normal mode\n");
		if (copy_from_user(firm_cmdbuff, buff, count)) {
			printk(KERN_INFO "copy from user returned error\n");
			err = -EINVAL;
		}
		err = msp430_i2c_write(ps_msp430, firm_cmdbuff, count);
		if (err == 0)
			err = len;
	}
	return err;
}

/* gpio toggling to switch modes(boot mode,normal mode)on MSP430 */
void switch_msp430_mode(enum msp_mode mode)
{
	unsigned int test_pin_active_value =
		msp430_misc_data->pdata->test_pin_active_value;

	/* bootloader mode */
	if (mode == BOOTMODE) {
		KDEBUG("MSP430 toggling to switch to boot mode\n");
		gpio_set_value(msp430_misc_data->pdata->gpio_test,
				(test_pin_active_value));
		msleep_interruptible(I2C_RETRY_DELAY);
		gpio_set_value(msp430_misc_data->pdata->gpio_reset, 0);
		msleep_interruptible(I2C_RETRY_DELAY);
		gpio_set_value(msp430_misc_data->pdata->gpio_reset, 1);
	} else{
	/*normal mode */
		KDEBUG("MSP430 toggling to normal or factory mode\n");
		gpio_set_value(msp430_misc_data->pdata->gpio_test,
			!(test_pin_active_value));
		msleep_interruptible(I2C_RETRY_DELAY);
		gpio_set_value(msp430_misc_data->pdata->gpio_reset, 0);
		msleep_interruptible(I2C_RETRY_DELAY);
		gpio_set_value(msp430_misc_data->pdata->gpio_reset, 1);
		/* Reset pedometer previous values */
		msp430_misc_data->prev_data.activity = 0;
		msp430_misc_data->prev_data.distance = 0;
		msp430_misc_data->prev_data.stepcount = 0;
	}
	msp430_misc_data->mode = mode;
}


static int msp430_misc_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int err = 0;
	unsigned int addr;
	struct msp430_data *ps_msp430 = file->private_data;
	unsigned int cmdlen = 0;
	unsigned char byte;
	unsigned short readwritebyte;
	unsigned char reg;
	unsigned int bytecount;
	unsigned short delay;
	struct msp430_gps_data gps;
	int i;
	struct msp430_user_profile user;
	int64_t seaLevelPressure = 0;
	short ref_altitude;
	unsigned long current_posix_time;
	struct msp430_pedometer_data pedo;
	struct msp430_workout_data dist_data;
	unsigned char buff[50];

	mutex_lock(&ps_msp430->lock);

	KDEBUG(KERN_INFO "msp430_misc_ioctl = %d\n", cmd);
	switch (cmd) {
	case MSP430_IOCTL_GET_VERSION:
		KDEBUG(KERN_INFO "Switch to normal to get version\n");
		switch_msp430_mode(NORMALMODE);
		mdelay(50);
		KDEBUG(KERN_INFO "MSP software version: ");
		buff[0] = REV_ID;
		err = msp430_i2c_write_read(ps_msp430, buff, 1, 1);
		if (err >= 0) {
			err = (int)buff[0];
			printk(KERN_INFO "%02x", buff[0]);
		}
		break;
	case MSP430_IOCTL_BOOTLOADERMODE:
		/* switch MSP to bootloader mode */
		KDEBUG(KERN_INFO "Switching to bootloader mode\n");
		msp430_stop_timers(ps_msp430);
		switch_msp430_mode(BOOTMODE);
		/* send password reset command to unlock MSP	 */
		KDEBUG(KERN_INFO "Password reset for reset vector 0x4400\n");
		msp430_build_command(PASSWORD_RESET, NULL, &cmdlen);
		err = msp430_i2c_write_read(ps_msp430, firm_cmdbuff,
					cmdlen, I2C_RESPONSE_LENGTH);
		/* password reset for reset vector 0x4400 failed */
		if (err < 0) {
			/* password reset for reset vector 0xffff */
			KDEBUG(KERN_INFO "Default password reset\n");
			msp430_build_command(PASSWORD_RESET_DEFAULT,
				NULL, &cmdlen);
			err = msp430_i2c_write_read(ps_msp430, firm_cmdbuff,
				cmdlen, I2C_RESPONSE_LENGTH);
		}

		if (err >= 0) {
			KDEBUG(KERN_INFO "Erasing the chip\n");
			msp430_build_command(MASS_ERASE, NULL, &cmdlen);
			err = msp430_i2c_write_read(ps_msp430, firm_cmdbuff,
				cmdlen, I2C_RESPONSE_LENGTH);
		}
		break;
	case MSP430_IOCTL_NORMALMODE:
		KDEBUG(KERN_INFO "Switching to normal node\n");
		/* switch MSP to normal mode */
		switch_msp430_mode(NORMALMODE);
		msp430_restart_timers(ps_msp430);
		break;
	case MSP430_IOCTL_MASSERASE:
		KDEBUG(KERN_INFO " In MSP430_IOCTL_MASSERASE\n");
		msp430_build_command(MASS_ERASE, NULL, &cmdlen);
		err = msp430_i2c_write_read(ps_msp430, firm_cmdbuff,
					cmdlen, I2C_RESPONSE_LENGTH);
		break;
	case MSP430_IOCTL_SETSTARTADDR:
		KDEBUG(KERN_INFO " In MSP430_IOCTL_SETSTARTADDR:\n");
		if (copy_from_user(&addr, argp, sizeof(addr))) {
			printk(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		/*store the start addr */
		msp430_misc_data->current_addr = addr;
		/* debug */
		KDEBUG(KERN_INFO "startaddr= %06x", ps_msp430->current_addr);
		break;
/* debug options for bosch */
	case MSP430_IOCTL_TEST_READ:
		KDEBUG(KERN_INFO "In msp430 test read");
		err = msp430_i2c_read(ps_msp430, &byte, 1);
		/*msp430 will return number of bytes read or an error code*/
		if (err > 0)
			err = byte;
		break;
	case MSP430_IOCTL_TEST_WRITE:
		KDEBUG(KERN_INFO "In msp430 test write");
		if (copy_from_user(&byte, argp, sizeof(unsigned char))) {
			printk(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		err = msp430_i2c_write(ps_msp430, &byte, 1);
		break;
	case MSP430_IOCTL_TEST_WRITE_READ:
		if (copy_from_user(&readwritebyte, argp,
			sizeof(unsigned short))) {
			printk(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		bytecount = (readwritebyte >> 8) & 0xff;
		reg = readwritebyte & 0xff;
		buff[0] = reg;
		/*printk(KERN_INFO "Read from address %02x, %02x bytes:",
				reg, bytecount);*/
		err = msp430_i2c_write_read(ps_msp430, buff, 1,
				bytecount);
		for (i = 0; i < bytecount; i++)
			printk("%02x ", buff[i]);
		break;


	case MSP430_IOCTL_SET_ACC_DELAY:
#ifdef G1
		delay = 0;
		if (copy_from_user(&delay, argp, sizeof(delay))) {
			KDEBUG(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		ps_msp430->acc_poll_interval = (int)delay;
		KDEBUG(KERN_INFO "set acc delay = %d\n",
			ps_msp430->acc_poll_interval);
		if ((delay >= 0) && (ps_msp430->mode == NORMALMODE))
			schedule_delayed_work(&ps_msp430->acc_work,
				msecs_to_jiffies(ps_msp430->acc_poll_interval));
#endif
		break;

#ifdef G1
	case MSP430_IOCTL_SET_MAG_DELAY:
		delay = 0;
		if (copy_from_user(&delay, argp, sizeof(delay))) {
			KDEBUG(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		ps_msp430->mag_poll_interval = (int)delay;
		KDEBUG(KERN_INFO "set magnetometer delay = %d\n",
			ps_msp430->mag_poll_interval);
		if ((delay >= 0) && (ps_msp430->mode == NORMALMODE))
			schedule_delayed_work(&ps_msp430->mag_work,
				msecs_to_jiffies(ps_msp430->mag_poll_interval));
		break;
	case MSP430_IOCTL_SET_ORIENTATION_DELAY:
		delay = 0;
		if (copy_from_user(&delay, argp, sizeof(delay))) {
			KDEBUG(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		ps_msp430->orin_poll_interval = (int)delay;
		KDEBUG(KERN_INFO "set orientation delay =  %d\n",
			ps_msp430->orin_poll_interval);
		if ((delay >= 0) && (ps_msp430->mode == NORMALMODE))
			schedule_delayed_work(&ps_msp430->orin_work,
			msecs_to_jiffies(ps_msp430->orin_poll_interval));
		input_report_abs(ps_msp430->input_dev, ABS_RUDDER, 0);
		break;
#endif
	case MSP430_IOCTL_SET_MONITOR_DELAY:
		delay = 0;
		if (copy_from_user(&delay, argp, sizeof(delay))) {
			KDEBUG(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		ps_msp430->monitor_poll_interval = (int)delay;
		/* if not normal mode exit out of here */
		if (ps_msp430->mode != NORMALMODE)
			break;
		KDEBUG(KERN_INFO "set activity monitor delay = %d\n", delay);
		if ((ps_msp430->monitor_poll_interval ==  60000)) {
			cancel_delayed_work(&ps_msp430->monitor_work);
			printk(KERN_INFO "Enable monitor me interrupt\n");
			ps_msp430->intp_mask = ps_msp430->intp_mask | M_MME_60;
			buff[0] = INTERRUPT_MASK;
			buff[1] = ps_msp430->intp_mask;
			err = msp430_i2c_write(ps_msp430, buff, 2);
			if (err < 0)
				printk(KERN_ERR "Unable to enable mme irq\n");
		} else {
#ifdef G1
			schedule_delayed_work(&ps_msp430->monitor_work,
			msecs_to_jiffies(ps_msp430->monitor_poll_interval));
			printk(KERN_INFO "Disable monitor me interrupt\n");
			ps_msp430->intp_mask = ps_msp430->intp_mask&~M_MME_60;
			buff[0] = INTERRUPT_MASK;
			buff[1] = ps_msp430->intp_mask;
			err = msp430_i2c_write(ps_msp430, buff, 2);
			if (err < 0)
				printk(KERN_ERR "Unable to disable mme irq\n");
#endif

			cancel_delayed_work(&ps_msp430->monitor_work);
			printk(KERN_INFO "Disable monitor me interrupt\n");
			ps_msp430->intp_mask = ps_msp430->intp_mask & ~M_MME_60;
			buff[0] = INTERRUPT_MASK;
			buff[1] = ps_msp430->intp_mask;
			err = msp430_i2c_write(ps_msp430, buff, 2);
			if (err < 0)
				printk(KERN_ERR "Unable to disable mme irq\n");

		}
		break;
	case MSP430_IOCTL_SET_MOTION_DELAY:
		delay = 0;
		if (copy_from_user(&delay, argp, sizeof(delay))) {
			KDEBUG(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		ps_msp430->motion_poll_interval = (int)delay;
		/* if not normal mode exit out of here */
		if (ps_msp430->mode != NORMALMODE)
			break;
		/*printk(KERN_INFO "set motion delay = %d\n",
			ps_msp430->motion_poll_interval);*/
		if (ps_msp430->motion_poll_interval == 1000) {

#ifdef G1
			wakeup_stop_status_timer(ps_msp430->waketimer);
			wakeup_start_status_timer(ps_msp430->waketimer,
				ps_msp430->motion_poll_interval);
#endif
			KDEBUG(KERN_INFO "Enable actMode irq\n");

			ps_msp430->intp_mask =
				ps_msp430->intp_mask | M_ACTIVITY_CHANGE;
			ps_msp430->intp_mask =
				ps_msp430->intp_mask | M_ACTIVE_MODE;
			ps_msp430->intp_mask =
				ps_msp430->intp_mask | M_TAP_TAP;
			buff[0] = INTERRUPT_MASK;
			buff[1] = ps_msp430->intp_mask;
			err = msp430_i2c_write(ps_msp430, buff, 2);
			if (err < 0)
				printk(KERN_INFO "Unable to enable actirq\n");

			msp430_read_motion_data(ps_msp430, &pedo);
			msp430_report_pedometer_values(ps_msp430, &pedo);

		} else {

#ifdef G1
			wakeup_stop_status_timer(ps_msp430->waketimer);
			if (ps_msp430->prev_data.activity != STILL) {
					wakeup_start_status_timer(
					ps_msp430->waketimer,
					ps_msp430->motion_poll_interval);
			}
#endif

			KDEBUG(KERN_INFO "Disable actMode irq\n");

			ps_msp430->intp_mask =
				ps_msp430->intp_mask | M_ACTIVITY_CHANGE;
			ps_msp430->intp_mask =
				ps_msp430->intp_mask & ~M_ACTIVE_MODE;
			ps_msp430->intp_mask =
				ps_msp430->intp_mask & ~M_TAP_TAP;
			buff[0] = INTERRUPT_MASK;
			buff[1] = ps_msp430->intp_mask;
			err = msp430_i2c_write(ps_msp430, buff, 2);
			if (err < 0)
				printk(KERN_INFO "Unable to disable act irq\n");
		}
		break;
	case MSP430_IOCTL_SET_POSIX_TIME:
		if (copy_from_user(&current_posix_time, argp, \
		sizeof(current_posix_time))) {
			KDEBUG(KERN_ERR "copy from user returned errorn\n");
			err = -EFAULT;
			break;
		}
		buff[0] = AP_POSIX_TIME;
		buff[1] = (unsigned char)((current_posix_time >> 24) \
		& 0xff);
		buff[2] = (unsigned char)((current_posix_time >> 16) \
		& 0xff);
		buff[3] = (unsigned char)((current_posix_time >> 8) \
		& 0xff);
		buff[4] = (unsigned char)((current_posix_time) & 0xff);
		err = msp430_i2c_write(ps_msp430, buff, 5);
		if (err < -1)
			printk(KERN_INFO "Failed to write current posix time\n");
		break;
	case MSP430_IOCTL_SET_FACTORY_MODE:
		KDEBUG(KERN_INFO "set factory mode\n");
		switch_msp430_mode(FACTORYMODE);
		break;
	case MSP430_IOCTL_SET_PASSIVE_MODE:
		KDEBUG(KERN_INFO "set passive mode\n");
		ps_msp430->in_activity = 0;
		break;
	case MSP430_IOCTL_SET_ACTIVE_MODE:
		KDEBUG(KERN_INFO "set active mode\n");
		ps_msp430->in_activity = 1;
		break;
#ifdef G1
	case MSP430_IOCTL_SET_ENV_DELAY:
		delay = 0;
		if (copy_from_user(&delay, argp, sizeof(delay))) {
			KDEBUG(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
			break;
		}
		/* to facilitate values to be populated on app relaunch */
		input_report_abs(ps_msp430->input_dev, ABS_ALTITUDE, 0);
		input_report_abs(ps_msp430->input_dev, ABS_PRESSURE_PASCAL, 0);
		input_report_abs(ps_msp430->input_dev, ABS_TEMPERATURE, 0);
		input_sync(ps_msp430->input_dev);


		/*ps_msp430->env_poll_interval = (int)delay;*/
		/* hardcoded to 10 seconds */
		ps_msp430->env_poll_interval = 1000;
		KDEBUG(KERN_INFO "set env delay = %d\n",
			ps_msp430->env_poll_interval);
		if ((delay >= 0) && (ps_msp430->mode == NORMALMODE))
			schedule_delayed_work(&ps_msp430->env_work,
			msecs_to_jiffies(ps_msp430->env_poll_interval));
		break;
#endif
	case MSP430_IOCTL_TEST_BOOTMODE:
	/* switch MSP to bootloader mode */
	KDEBUG(KERN_INFO "Switching to TEST bootloader mode\n");
	msp430_stop_timers(ps_msp430);
	switch_msp430_mode(BOOTMODE);
	break;

	case MSP430_IOCTL_SET_DEBUG:
	/* enable or disble msp driver debug messages */
	if (copy_from_user(&g_debug, argp, sizeof(g_debug))) {
		KDEBUG(KERN_ERR "copy from user returned error\n");
			err = -EFAULT;
	}
	break;

	case MSP430_IOCTL_SET_USER_PROFILE:
	KDEBUG(KERN_INFO "Set profile with \n");
	if (copy_from_user(&user, argp, sizeof(user))) {
		printk(KERN_ERR "copy from user returned error\n");
		err = -EFAULT;
		break;
	}
	KDEBUG(KERN_INFO "age = %02x, sex = %02x, weight = %02x, \
	height = %02x \n", user.age, user.sex , user.weight, user.height);
	buff[0] = AGE;
	buff[1] = user.age;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write AGE\n");
	buff[0] = SEX;
	buff[1] = user.sex;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write SEX\n");
	buff[0] = HEIGHT;
	buff[1] = user.height;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write height\n");
	buff[0] = WEIGHT;
	buff[1] = user.weight;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write weight\n");
	break;

	case MSP430_IOCTL_SET_GPS_DATA:
	KDEBUG(KERN_INFO "Got gps data: ");
	if (copy_from_user(&gps, argp, sizeof(gps))) {
		printk(KERN_ERR "copy from user returned error\n");
		err = -EFAULT;
		break;
	}
#ifdef G1
	KDEBUG(KERN_INFO "latitude = %d, longitude = %d,\
		horizontal_accuracy = 0x%02x,vertical_accuracy = 0x%02x,\
		altitude = %d, heading = %d speed= %d\n", gps.latitude,
		gps.longitude, gps.horizontal_accuracy,	gps.vertical_accuracy,
		gps.altitude, gps.heading, gps.speed);

	msp_cmdbuff[0] = GPS_LATITUDE;
	/* gps latitude msb first */
	msp_cmdbuff[1] = (unsigned char)((gps.latitude >> 24) & 0xff);
	msp_cmdbuff[2] = (unsigned char)((gps.latitude >> 16) & 0xff);
	msp_cmdbuff[3] = (unsigned char)((gps.latitude >> 8) & 0xff);
	msp_cmdbuff[4] = (unsigned char)((gps.latitude) & 0xff);
	/* gps longitude */
	msp_cmdbuff[5] = (unsigned char)((gps.longitude >> 24) & 0xff);
	msp_cmdbuff[6] = (unsigned char)((gps.longitude >> 16) & 0xff);
	msp_cmdbuff[7] = (unsigned char)((gps.longitude >> 8) & 0xff);
	msp_cmdbuff[8] = (unsigned char)((gps.longitude) & 0xff);
	/* gps heading */
	msp_cmdbuff[9] = (unsigned char)((gps.heading >> 8) & 0xff);
	msp_cmdbuff[10] = (unsigned char)((gps.heading) & 0xff);
	/*gps fix type (horizontal accuracy) */
	msp_cmdbuff[11] = (unsigned char)gps.horizontal_accuracy;
	/* gps altitude */
	msp_cmdbuff[12] = (unsigned char)((gps.altitude >> 24) & 0xff);
	msp_cmdbuff[13] = (unsigned char)((gps.altitude >> 16) & 0xff);
	msp_cmdbuff[14] = (unsigned char)((gps.altitude >> 8) & 0xff);
	msp_cmdbuff[15] = (unsigned char)((gps.altitude) & 0xff);
	/* gps accuracy  - veritcal accuracy*/
	msp_cmdbuff[16] = (unsigned char)gps.vertical_accuracy;
	/* reserved byte */
	msp_cmdbuff[17] = 0x00;
	/* gps speed */
	msp_cmdbuff[18] = (unsigned char)gps.speed;
	err = msp430_i2c_write(ps_msp430, msp_cmdbuff, 19);
	if (err < -1)
		printk(KERN_INFO "Failed to write gps data\n");
#else
	KDEBUG(" sending gps speed to msp = %d\n", gps.speed);
	buff[0] = GPS_SPEED;
	buff[1] = (unsigned char)gps.speed;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write gps data\n");
#endif
	break;

#ifdef G1
	case MSP430_IOCTL_SET_SEA_LEVEL_PRESSURE:
	KDEBUG(KERN_INFO "Setting sea level pressure: ");
	if (copy_from_user(&seaLevelPressure, argp, sizeof(seaLevelPressure))) {
		printk(KERN_ERR "copy from user returned error\n");
		err = -EFAULT;
		break;
	}
	KDEBUG(KERN_INFO "%lld\n", seaLevelPressure);
	msp_cmdbuff[0] = PRESSURE_SEA_LEVEL;
	msp_cmdbuff[1] = (unsigned char)((seaLevelPressure >> 32) & 0xff);
	msp_cmdbuff[2] = (unsigned char)((seaLevelPressure >> 24) & 0xff);
	msp_cmdbuff[3] = (unsigned char)((seaLevelPressure >> 16) & 0xff);
	msp_cmdbuff[4] = (unsigned char)((seaLevelPressure >> 8) & 0xff);
	msp_cmdbuff[5] = (unsigned char)((seaLevelPressure) & 0xff);
	err = msp430_i2c_write(ps_msp430, msp_cmdbuff, 6);
	if (err < -1)
		printk(KERN_INFO "Failed to write sea level pressure\n");
	break;

	case MSP430_IOCTL_SET_REF_ALTITUDE:
	KDEBUG(KERN_INFO "Setting reference altitude: ");
	if (copy_from_user(&ref_altitude, argp, sizeof(ref_altitude))) {
		printk(KERN_ERR "copy from user returned error\n");
		err = -EFAULT;
		break;
	}
	KDEBUG(KERN_INFO "%d\n", ref_altitude);
	msp_cmdbuff[0] = CURRENT_ALTITUDE;
	msp_cmdbuff[1] = (unsigned char)((ref_altitude >> 8) & 0xff);
	msp_cmdbuff[2] = (unsigned char)((ref_altitude) & 0xff);
	err = msp430_i2c_write(ps_msp430, msp_cmdbuff, 3);
	if (err < -1)
		printk(KERN_INFO "Failed to write reference altitude\n");
	break;

	case MSP430_IOCTL_SET_DOCK_STATUS:
	if (copy_from_user(&byte, argp, sizeof(byte))) {
		printk(KERN_ERR "copy from user returned error doc status\n");
		err = -EFAULT;
		break;
	}
	KDEBUG(KERN_INFO "Dock status = %c", byte);
	break;
#endif
	case MSP430_IOCTL_SET_EQUIPMENT_TYPE:
	if (copy_from_user(&byte, argp, sizeof(byte))) {
		printk(KERN_ERR "copy from user returned error eq type\n");
		err = -EFAULT;
		break;
	}
	KDEBUG(KERN_INFO "Equipment type = %d", byte);
	ps_msp430->equipment = (unsigned char)byte;
	buff[0] = EQUIPMENT_TYPE;
	buff[1] = byte;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write equipment type\n");
	break;

	case MSP430_IOCTL_SET_MANUAL_CALIB_STATUS:
	if (copy_from_user(&byte, argp, sizeof(byte))) {
		printk(KERN_ERR "copy from user returned err: calib status\n");
		err = -EFAULT;
		break;
	}
	printk(KERN_INFO "Manual calibration status = %d", byte);
	buff[0] = CALIBRATION_ON;
	buff[1] = byte;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write manual calib status\n");
	break;

	case MSP430_IOCTL_GET_MANUAL_CALIB_STATUS:
	KDEBUG(KERN_INFO "Get manual calibration status = %d", byte);
	buff[0] = CALIBRATION_TABLE;
	err = msp430_i2c_write_read(ps_msp430, buff, 1,
		CALIB_TABLE_SIZE);
	if (err < -1) {
		printk(KERN_INFO "Failed to read manual calibration table\n");
		err = -EFAULT;
		break;
	}
	err = 0x00;
	/* check if walk calibrated */
	readwritebyte = 0x00;
	readwritebyte = (buff[0] << 8) | buff[1];
	if (readwritebyte != 0x00)
		err |= 0x01;
	/* check if jog calibrated */
	readwritebyte = 0x00;
	readwritebyte = (buff[4] << 8) | buff[5];
	if (readwritebyte != 0x00)
		err |= 0x02;
	/* check if run calibrated */
	readwritebyte = 0x00;
	readwritebyte = (buff[16] << 8) | buff[17];
	if (readwritebyte != 0x00)
		err |= 0x04;
	break;

	case MSP430_IOCTL_SET_MANUAL_CALIB_WALK_SPEED:
	if (copy_from_user(&byte, argp, sizeof(byte))) {
		printk(KERN_ERR "copy from user returned error: walk speed\n");
		err = -EFAULT;
		break;
	}
	printk(KERN_INFO "Manual calibration: walk speed = %d", byte);
	buff[0] = CALIBRATION_SPEED_WALK;
	buff[1] = byte;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write manual calib walk speed\n");
	break;

	case MSP430_IOCTL_SET_MANUAL_CALIB_RUN_SPEED:
	if (copy_from_user(&byte, argp, sizeof(byte))) {
		printk(KERN_ERR "copy from user returned error: run speed\n");
		err = -EFAULT;
		break;
	}
	printk(KERN_INFO "Manual calibration: run speed = %d", byte);
	buff[0] = CALIBRATION_SPEED_RUN;
	buff[1] = byte;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write manual calib walk speed\n");
	break;

	case MSP430_IOCTL_SET_MANUAL_CALIB_JOG_SPEED:
	if (copy_from_user(&byte, argp, sizeof(byte))) {
		printk(KERN_ERR "copy from user returned error: jog speed\n");
		err = -EFAULT;
		break;
	}
	KDEBUG(KERN_INFO "Manual calibration: jog speed = %d", byte);
	buff[0] = CALIBRATION_SPEED_JOG;
	buff[1] = byte;
	err = msp430_i2c_write(ps_msp430, buff, 2);
	if (err < -1)
		printk(KERN_INFO "Failed to write manual calib walk speed\n");
	break;

	case MSP430_IOCTL_GET_MANUAL_CALIB_TABLE:
	printk(KERN_INFO "Read manual calibration table from msp\n");
	buff[0] = CALIBRATION_TABLE;
	err = msp430_i2c_write_read(ps_msp430, buff, 1,
			CALIB_TABLE_SIZE);
	if (err < -1) {
		printk(KERN_INFO "Failed to read manual calibration table\n");
		err = -EFAULT;
		break;
	}
	/*save the calibration table */
	for (i = 0; i < CALIB_TABLE_SIZE; i++)
		ps_msp430->calib_table[i] = buff[i];
	break;

	case MSP430_IOCTL_SET_MANUAL_CALIB_TABLE:
	printk(KERN_INFO "Write manual calibration table into msp\n");
	buff[0] = CALIBRATION_TABLE;
	for (i = 0; i < CALIB_TABLE_SIZE; i++)
		buff[i+1]  = ps_msp430->calib_table[i];
	err = msp430_i2c_write(ps_msp430, buff, CALIB_TABLE_SIZE+1);
	if (err < -1) {
		printk(KERN_INFO "Failed to write to calibration table\n");
		err = -EFAULT;
	}
	break;

	case MSP430_IOCTL_SET_USER_DISTANCE:
	KDEBUG(KERN_INFO "Set user distance for the workout");
	if (copy_from_user(&dist_data, argp, (sizeof(dist_data)))) {
		printk(KERN_ERR "copy from user failed: user distance \n");
		err = -EFAULT;
		break;
	}
	KDEBUG(KERN_INFO "msp distance = %d, user_distance = %d\n", dist_data.msp_distance, dist_data.user_distance);
	buff[0] = WORKOUT_MSP_DIST;
	/* msp distance msb first */
	buff[1] = (unsigned char)((dist_data.msp_distance >> 24) & 0xff);
	buff[2] = (unsigned char)((dist_data.msp_distance >> 16) & 0xff);
	buff[3] = (unsigned char)((dist_data.msp_distance >> 8) & 0xff);
	buff[4] = (unsigned char)((dist_data.msp_distance) & 0xff);
	/* user distance msb first */
	buff[5] = (unsigned char)((dist_data.user_distance >> 24) & 0xff);
	buff[6] = (unsigned char)((dist_data.user_distance >> 16) & 0xff);
	buff[7] = (unsigned char)((dist_data.user_distance >> 8) & 0xff);
	buff[8] = (unsigned char)((dist_data.user_distance) & 0xff);
	err = msp430_i2c_write(ps_msp430, buff, 9);
	if (err < -1) {
		printk(KERN_INFO "Failed to write to user corrected distance\n");
		err = -EFAULT;
	}
	break;

	case MSP430_IOCTL_SET_USER_CALIB_TABLE:
	KDEBUG(KERN_INFO "Set user calibration table\n");
	if (copy_from_user(buff+1, argp, 22)) {
		printk(KERN_ERR "copy from user failed: user calibration table\n");
		err = -EFAULT;
		break;
	}
	for( i=0; i< 22; i++)
		KDEBUG(" 0x%x ", buff[i]);
	KDEBUG("\n");
	buff[0] = USER_CAL_TABLE;
	err = msp430_i2c_write(ps_msp430, buff, 23);
	if (err < -1) {
		printk(KERN_INFO "Failed to write to user calibration table\n");
		err = -EFAULT;
	}
	break;

	default:
		printk(KERN_INFO "MSP430:Invalid ioctl command %d\n", cmd);
		err = -EINVAL;
	}
	mutex_unlock(&ps_msp430->lock);
	return err;
}

static const struct file_operations msp430_misc_fops = {
	.owner = THIS_MODULE,
	.open = msp430_misc_open,
	.ioctl = msp430_misc_ioctl,
	.write = msp430_misc_write,
};

static struct miscdevice msp430_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &msp430_misc_fops,
};

#ifdef G1
static void msp430_env_work_func(struct work_struct *work)
{
	int err = 0;
	struct msp430_data *ps_msp430;
	int pressure;
	short temperature;
	short temp_ti;
	short temp_acc;

	short altitude;

	ps_msp430 = container_of((struct delayed_work *)work,
		struct msp430_data, env_work);

	if (ps_msp430->mode == BOOTMODE)
		return;
	mutex_lock(&ps_msp430->lock);
	KDEBUG(KERN_INFO "Inside  ENV timer work function\n");

	/* read current pressure from MSP */
	msp_cmdbuff[0] = CURRENT_PRESSURE;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 4);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	pressure = (msp_cmdbuff[0] << 24) | (msp_cmdbuff[1] << 16)\
		| (msp_cmdbuff[2] << 8) | msp_cmdbuff[3];

	/* read current temperature from TI */
	msp_cmdbuff[0] = TEMPERATURE_TMP112;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 2);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	temp_ti = (msp_cmdbuff[0] << 8) | msp_cmdbuff[1];

	/* read accelerometer temperature */
	msp_cmdbuff[0] = TEMPERATURE_MSP430;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 2);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	temp_acc = (msp_cmdbuff[0] << 8) | msp_cmdbuff[1];
	KDEBUG(KERN_INFO "ti temp = %d\n, acc temp = %d",
		temp_ti, temp_acc);

	if (temp_ti < 10 && temp_ti > -10)
		temp_ti = temp_ti * 10;

	if (temp_acc < 10 && temp_acc > -10)
		temp_acc = temp_acc * 10;

	temperature = (-151600-400000-326*temp_acc+10296*temp_ti)/10000;

	input_report_abs(ps_msp430->input_dev, ABS_TEMPERATURE, temperature);
	input_sync(ps_msp430->input_dev);
	KDEBUG(KERN_INFO "Sending temperature = %d\n", temperature);

	/* read current altitude from MSP */
	msp_cmdbuff[0] = CURRENT_ALTITUDE;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 2);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	altitude = (msp_cmdbuff[0] << 8) | msp_cmdbuff[1];
	input_report_abs(ps_msp430->input_dev, ABS_PRESSURE_PASCAL, pressure);
	input_report_abs(ps_msp430->input_dev, ABS_ALTITUDE, altitude);
	input_sync(ps_msp430->input_dev);
	KDEBUG(KERN_INFO "Sending pressure = %d altitude = %d\n",
		pressure, altitude);

EXIT:
	schedule_delayed_work(&ps_msp430->env_work,
		msecs_to_jiffies(ps_msp430->env_poll_interval));
	mutex_unlock(&ps_msp430->lock);
}
#endif

static void msp430_monitor_work_func(struct work_struct *work)
{
	int err = 0;
	struct msp430_data *ps_msp430;
	unsigned char buff[10];

	ps_msp430 = container_of((struct delayed_work *)work,
		struct msp430_data, monitor_work);
	if (ps_msp430->mode == BOOTMODE)
		return;
	mutex_lock(&ps_msp430->lock);
	KDEBUG(KERN_INFO "Inside monitor me  timer work function\n");

	buff[0] = MONITORME_10SEC_METS;
	err = msp430_i2c_write_read(ps_msp430, buff, 1, 2);
	if (err < 0) {
		printk(KERN_INFO "Reading activity level from msp failed\n");
		goto EXIT;
	}
	KDEBUG(KERN_INFO "Sending monitor = %d\n", buff[0]);
	/*send activity level */
	input_report_rel(ps_msp430->input_dev, REL_ACTIVITY_LEVEL,
			buff[0]);
	input_sync(ps_msp430->input_dev);

EXIT:
	if ((ps_msp430->monitor_poll_interval != 60000)) {
		schedule_delayed_work(&ps_msp430->monitor_work,
		msecs_to_jiffies(ps_msp430->monitor_poll_interval));
	}
	mutex_unlock(&ps_msp430->lock);
}

#ifdef G1
static void msp430_orin_work_func(struct work_struct *work)
{
	int err = 0;
	struct msp430_data *ps_msp430;
	signed short roll, pitch, yaw, status;

	ps_msp430 = container_of((struct delayed_work *)work,
			struct msp430_data, orin_work);

	if (ps_msp430->mode == BOOTMODE)
		return;

	mutex_lock(&ps_msp430->lock);
	KDEBUG(KERN_INFO "Inside ORIENTATION timer work function\n");

	/*Read orientation values */
	msp_cmdbuff[0] = DEVICE_ORIEN_HEAD;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 6);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	yaw = (msp_cmdbuff[0] << 8) | msp_cmdbuff[1];
	pitch = (msp_cmdbuff[2] << 8) | msp_cmdbuff[3];
	roll = (msp_cmdbuff[4] << 8) | msp_cmdbuff[5];

	msp_cmdbuff[0] = CAL_STATUS;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 1);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	status = msp_cmdbuff[0];
	input_report_abs(ps_msp430->input_dev, ABS_RX, yaw);
	input_report_abs(ps_msp430->input_dev, ABS_RY, pitch);
	input_report_abs(ps_msp430->input_dev, ABS_RZ, roll);
	input_report_abs(ps_msp430->input_dev, ABS_RUDDER, status);
	input_sync(ps_msp430->input_dev);
	KDEBUG("Sending orientation values: yaw = %d, pitch =%d, roll=%d,\
		status=%d\n", yaw, pitch, roll, status);
EXIT:

	schedule_delayed_work(&ps_msp430->orin_work,
		msecs_to_jiffies(ps_msp430->orin_poll_interval));
	mutex_unlock(&ps_msp430->lock);
}

static void msp430_mag_work_func(struct work_struct *work)
{

	int err = 0;
	struct msp430_data *ps_msp430;
	signed short xyz[3];

	ps_msp430 = container_of((struct delayed_work *)work,
		struct msp430_data, mag_work);

	if (ps_msp430->mode == BOOTMODE)
		return;

	mutex_lock(&ps_msp430->lock);
	KDEBUG(KERN_INFO "Inside MAG timer work function\n");

	/*Read magnetometer values TODO */
	msp_cmdbuff[0] = MAG_HX;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 6);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	xyz[0] =  (msp_cmdbuff[0] << 8) | msp_cmdbuff[1];
	xyz[1] =  (msp_cmdbuff[2] << 8) | msp_cmdbuff[3];
	xyz[2] =  (msp_cmdbuff[4] << 8) | msp_cmdbuff[5];
	input_report_abs(ps_msp430->input_dev, ABS_HAT0X, xyz[0]);
	input_report_abs(ps_msp430->input_dev, ABS_HAT0Y, xyz[1]);
	input_report_abs(ps_msp430->input_dev, ABS_BRAKE, xyz[2]);
	input_sync(ps_msp430->input_dev);
	KDEBUG("Sending mag(x,y,z) values: mag_x = %d, mag_y =%d, mag_z=%d\n",
			xyz[0], xyz[1], xyz[2]);
EXIT:
	schedule_delayed_work(&ps_msp430->mag_work,
			msecs_to_jiffies(ps_msp430->mag_poll_interval));
	mutex_unlock(&ps_msp430->lock);
}

static void msp430_acc_work_func(struct work_struct *work)
{
	int err = 0;
	struct msp430_data *ps_msp430;
	signed short xyz[3];

	ps_msp430 = container_of((struct delayed_work *)work,
		struct msp430_data, acc_work);

	if (ps_msp430->mode == BOOTMODE)
		return;

	mutex_lock(&ps_msp430->lock);
	KDEBUG(KERN_INFO "Inside ACC timer work function\n");

	/* read accelerometer values from MSP */
	msp_cmdbuff[0] = ACCEL_X;
	err = msp430_i2c_write_read(ps_msp430, msp_cmdbuff, 1, 6);
	if (err < 0) {
		printk(KERN_INFO "Reading from msp failed\n");
		goto EXIT;
	}
	xyz[0] =  (msp_cmdbuff[0] << 8) | msp_cmdbuff[1];
	xyz[1] =  (msp_cmdbuff[2] << 8) | msp_cmdbuff[3];
	xyz[2] =  (msp_cmdbuff[4] << 8) | msp_cmdbuff[5];
	input_report_abs(ps_msp430->input_dev, ABS_X, xyz[1]);
	input_report_abs(ps_msp430->input_dev, ABS_Y, xyz[0]);
	input_report_abs(ps_msp430->input_dev, ABS_Z, xyz[2]);
	input_sync(ps_msp430->input_dev);
	KDEBUG("Sending acc(x,y,z) values: acc_X = %d, acc_y =%d, acc_z=%d\n",
			xyz[1], xyz[0], xyz[2]);

EXIT:
	schedule_delayed_work(&ps_msp430->acc_work,
		msecs_to_jiffies(ps_msp430->acc_poll_interval));
	mutex_unlock(&ps_msp430->lock);
}
#endif

#ifdef MSP430_OPEN_ENABLE
int msp430_input_open(struct input_dev *input)
{
	struct msp430_data *ps_msp430 = input_get_drvdata(input);
	printk(KERN_INFO "msp430_input_open\n");
	return msp430_enable(ps_msp430);
}

void msp430_input_close(struct input_dev *dev)
{
	struct msp430_data *ps_msp430 = input_get_drvdata(dev);
	printk(KERN_INFO "msp430_input_close\n");
	msp430_disable(ps_msp430);
}
#endif


static int msp430_input_init(struct msp430_data *ps_msp430)
{
	int err;
	printk(KERN_INFO "msp430_input_init\n");
#ifdef G1
	INIT_DELAYED_WORK(&ps_msp430->acc_work, msp430_acc_work_func);
	INIT_DELAYED_WORK(&ps_msp430->env_work, msp430_env_work_func);
	INIT_DELAYED_WORK(&ps_msp430->mag_work, msp430_mag_work_func);
	INIT_DELAYED_WORK(&ps_msp430->orin_work, msp430_orin_work_func);
#endif
	INIT_DELAYED_WORK(&ps_msp430->monitor_work, msp430_monitor_work_func);
	ps_msp430->input_dev = input_allocate_device();
	if (!ps_msp430->input_dev) {
		err = -ENOMEM;
		dev_err(&ps_msp430->client->dev,
			"input device allocate failed: %d\n", err);
		goto err0;
	}
#ifdef msp430_OPEN_ENABLE
	ps_msp430->input_dev->open = msp430_input_open;
	ps_msp430->input_dev->close = msp430_input_close;
#endif
	input_set_drvdata(ps_msp430->input_dev, ps_msp430);
	set_bit(EV_ABS, ps_msp430->input_dev->evbit);
	set_bit(EV_REL, ps_msp430->input_dev->evbit);
#ifdef G1
	set_bit(ABS_TEMPERATURE, ps_msp430->input_dev->absbit);
	set_bit(ABS_ALTITUDE, ps_msp430->input_dev->absbit);
	set_bit(ABS_PRESSURE_PASCAL, ps_msp430->input_dev->absbit);
#endif
	/* check if these 3 lines r needed - TODO */
	set_bit(ABS_X, ps_msp430->input_dev->absbit);
	set_bit(ABS_Y, ps_msp430->input_dev->absbit);
	set_bit(ABS_Z, ps_msp430->input_dev->absbit);
#ifdef G1
	set_bit(ABS_HAT0X, ps_msp430->input_dev->absbit);
	set_bit(ABS_HAT0Y, ps_msp430->input_dev->absbit);
	set_bit(ABS_BRAKE, ps_msp430->input_dev->absbit);
	set_bit(ABS_RX, ps_msp430->input_dev->absbit);
	set_bit(ABS_RY, ps_msp430->input_dev->absbit);
	set_bit(ABS_RZ, ps_msp430->input_dev->absbit);
	set_bit(ABS_RUDDER, ps_msp430->input_dev->absbit);
#endif
	set_bit(ABS_MSP_LATITUDE, ps_msp430->input_dev->absbit);
	set_bit(ABS_MSP_LONGITUDE, ps_msp430->input_dev->absbit);
	set_bit(ABS_MSP_HEADING, ps_msp430->input_dev->absbit);
	set_bit(ABS_MSP_ACCURACY, ps_msp430->input_dev->absbit);
	set_bit(REL_STEPCOUNT, ps_msp430->input_dev->relbit);
	set_bit(REL_DISTANCE, ps_msp430->input_dev->relbit);
	set_bit(REL_SPEED, ps_msp430->input_dev->relbit);
	set_bit(REL_ACTIVITY_TYPE, ps_msp430->input_dev->relbit);
	set_bit(REL_ACTIVITY_LEVEL, ps_msp430->input_dev->relbit);
	/* this is for passive steps*/
	set_bit(REL_WHEEL,  ps_msp430->input_dev->relbit);
	/* this is for timestamp*/
	set_bit(REL_HWHEEL,  ps_msp430->input_dev->relbit);
	/* this is for tap tap */
	set_bit(REL_DIAL,  ps_msp430->input_dev->relbit);

#ifdef G1
	/* T1 temp sensor can measure btw -40 degree C to +125 degree C */
	input_set_abs_params(ps_msp430->input_dev,
			ABS_TEMPERATURE, -400, 1250, 0, 0);
	input_set_abs_params(ps_msp430->input_dev,
			ABS_PRESSURE_PASCAL, 0, 1000000, 0, 0);
	input_set_abs_params(ps_msp430->input_dev,
			ABS_ALTITUDE, 0, 1000000, 0, 0);
#endif
	/*TODO fuzz and flat from kxtf9 */
	input_set_abs_params(ps_msp430->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(ps_msp430->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(ps_msp430->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(ps_msp430->input_dev,
			ABS_MSP_LATITUDE, 0, 1000000, 0, 0);
	input_set_abs_params(ps_msp430->input_dev,
			ABS_MSP_LONGITUDE, 0, 1000000, 0, 0);
	input_set_abs_params(ps_msp430->input_dev,
			ABS_MSP_HEADING, 0, 1000000, 0, 0);
	input_set_abs_params(ps_msp430->input_dev,
			ABS_MSP_ACCURACY, 0, 1000000, 0, 0);
#ifdef G1
	input_set_abs_params(ps_msp430->input_dev, ABS_HAT0X,
			-4096 , 4096, 0, 0);
	input_set_abs_params(ps_msp430->input_dev, ABS_HAT0Y,
			-4096, 4096, 0, 0);
	input_set_abs_params(ps_msp430->input_dev, ABS_BRAKE,
			-4096, 4096, 0, 0);
	/* Max range = 360 * 64 */
	input_set_abs_params(ps_msp430->input_dev, ABS_RX, 0, 23040, 0, 0);
	input_set_abs_params(ps_msp430->input_dev, ABS_RY, 0, 23040, 0, 0);
	input_set_abs_params(ps_msp430->input_dev, ABS_RZ, 0, 23040, 0, 0);
	input_set_abs_params(ps_msp430->input_dev, ABS_RUDDER, 0, 4, 0, 0);
#endif

	ps_msp430->input_dev->name = "msp430sensorprocessor";
	err = input_register_device(ps_msp430->input_dev);
	if (err) {
		dev_err(&ps_msp430->client->dev,
			"unable to register input polled device %s: %d\n",
			ps_msp430->input_dev->name, err);
		goto err1;
	}

	return 0;
err1:
	input_free_device(ps_msp430->input_dev);
err0:

	return err;
}

static void msp430_input_cleanup(struct msp430_data *ps_msp430)
{
	printk(KERN_INFO "msp430_input_cleanup\n");
	input_unregister_device(ps_msp430->input_dev);
	input_free_device(ps_msp430->input_dev);
}

int waketimercallback(void)
{
	struct msp430_data *ps_msp430 = msp430_misc_data;
	KDEBUG("wake up msp430\n");
#ifdef G1
	queue_work(ps_msp430->irq_work_queue, &ps_msp430->passive_work);
	if (ps_msp430->motion_poll_interval != 60000  ||
			ps_msp430->prev_data.activity != STILL) {
		wakeup_start_status_timer(ps_msp430->waketimer,
			ps_msp430->motion_poll_interval);
	}
#endif

	return 0;
}

static int msp430_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct msp430_data *ps_msp430;
	int err = -1;


	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL, exiting\n");
		err = -ENODEV;
		goto err0;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}
	ps_msp430 = kzalloc(sizeof(*ps_msp430), GFP_KERNEL);
	if (ps_msp430 == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: %d\n", err);
		goto err0;
	}

	mutex_init(&ps_msp430->lock);
	mutex_lock(&ps_msp430->lock);
	ps_msp430->waketimer = wakeup_create_status_timer(waketimercallback);
	ps_msp430->inactivity_timer = wakeup_create_status_timer(msp430_handle_inactivity_timeout);
	ps_msp430->client = client;
	ps_msp430->mode = UNINITIALIZED;
#ifdef G1
	ps_msp430->acc_poll_interval = 0;
	ps_msp430->mag_poll_interval = 0;
	ps_msp430->orin_poll_interval = 0;
	ps_msp430->env_poll_interval = 0;
#endif
	ps_msp430->motion_poll_interval = 0;
	ps_msp430->monitor_poll_interval = 0;

	/* initialize pedometer data */
	ps_msp430->prev_data.activity = -1;
	ps_msp430->prev_data.distance = 0;
	ps_msp430->prev_data.stepcount = 0;
	ps_msp430->prev_data.speed = 0;

	/* Set to passive mode by default */
	ps_msp430->in_activity = 0;
	g_debug = 0;
	/* clear the interrupt mask */
	ps_msp430->intp_mask = 0x00;

	ps_msp430->inactivity_timeout = MSP430_INITIAL_INACTIVITY_TIMEOUT;
	ps_msp430->psuedo_isr = false;
	ps_msp430->offmode_wakeups_disabled = false;

	INIT_WORK(&ps_msp430->irq_work, msp430_irq_work_func);
	INIT_WORK(&ps_msp430->passive_work, msp430_passive_work_func);
	INIT_WORK(&ps_msp430->inactivity_work, msp430_inactivity_work_func);
	ps_msp430->irq_work_queue = create_singlethread_workqueue("msp430_wq");
	if (!ps_msp430->irq_work_queue) {
		err = -ENOMEM;
		dev_err(&client->dev, "cannot create work queue: %d\n", err);
		goto err1;
	}
	ps_msp430->pdata = kmalloc(sizeof(*ps_msp430->pdata), GFP_KERNEL);
	if (ps_msp430->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err2;
	}
	memcpy(ps_msp430->pdata, client->dev.platform_data,
		sizeof(*ps_msp430->pdata));
	i2c_set_clientdata(client, ps_msp430);
	ps_msp430->client->flags &= 0x00;

	if (ps_msp430->pdata->init) {
		err = ps_msp430->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err3;
		}
	}

	/*configure interrupt*/
	ps_msp430->irq = gpio_to_irq(ps_msp430->pdata->gpio_int);

	err = msp430_device_power_on(ps_msp430);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err4;
	}
	enable_irq_wake(ps_msp430->irq);
	atomic_set(&ps_msp430->enabled, 1);

	err = msp430_input_init(ps_msp430);
	if (err < 0)
		goto err5;

	msp430_misc_data = ps_msp430;
	err = misc_register(&msp430_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "misc register failed: %d\n", err);
		goto err6;
	}

	msp430_device_power_off(ps_msp430);

	atomic_set(&ps_msp430->enabled, 0);

	err = request_irq(ps_msp430->irq, msp430_isr, IRQF_TRIGGER_RISING,
		"msp430_irq", ps_msp430);
	if (err < 0) {
		dev_err(&client->dev, "request irq failed: %d\n", err);
		goto err7;
	}

	/* Setup wakelocks and inactivity timer */
	wake_lock_init(&ps_msp430->wake_lock, WAKE_LOCK_SUSPEND,
			NAME "_suspend");
	wake_lock_init(&ps_msp430->offmode_lock, WAKE_LOCK_OFFMODE,
			NAME "_offmode");
	wake_lock_init(&ps_msp430->timed_lock, WAKE_LOCK_SUSPEND,
			NAME "_timed");

	msp430_kick_idletimer(ps_msp430);

	mutex_unlock(&ps_msp430->lock);

	dev_info(&client->dev, "msp430 probed\n");

	return 0;

err7:
	misc_deregister(&msp430_misc_device);
err6:
	msp430_input_cleanup(ps_msp430);
err5:
	msp430_device_power_off(ps_msp430);
err4:
	if (ps_msp430->pdata->exit)
		ps_msp430->pdata->exit();
err3:
	kfree(ps_msp430->pdata);
err2:
	destroy_workqueue(ps_msp430->irq_work_queue);
err1:
	mutex_unlock(&ps_msp430->lock);
	kfree(ps_msp430);
err0:
	return err;
}

static int __devexit msp430_remove(struct i2c_client *client)
{
	struct msp430_data *ps_msp430 = i2c_get_clientdata(client);
	printk(KERN_INFO "msp430_remove\n");
	wake_lock_destroy(&ps_msp430->wake_lock);
	wake_lock_destroy(&ps_msp430->offmode_lock);
	wake_lock_destroy(&ps_msp430->timed_lock);
	free_irq(ps_msp430->irq, ps_msp430);
	gpio_free(ps_msp430->pdata->gpio_int);
	gpio_free(ps_msp430->pdata->gpio_reset);
	gpio_free(ps_msp430->pdata->gpio_test);
	misc_deregister(&msp430_misc_device);
	msp430_input_cleanup(ps_msp430);
	msp430_device_power_off(ps_msp430);
	if (ps_msp430->pdata->exit)
		ps_msp430->pdata->exit();
	kfree(ps_msp430->pdata);
	destroy_workqueue(ps_msp430->irq_work_queue);
	wakeup_del_status_timer(ps_msp430->inactivity_timer);
	wakeup_del_status_timer(ps_msp430->waketimer);
	disable_irq_wake(ps_msp430->irq);
	kfree(ps_msp430);

	return 0;
}

static int msp430_resume(struct i2c_client *client)
{

	struct msp430_data *ps_msp430 = i2c_get_clientdata(client);
	int err = 0;
	KDEBUG(KERN_INFO "msp430_resume\n");
	mutex_lock(&ps_msp430->lock);

	if (msp430_enable(ps_msp430) < 0) {
		printk(KERN_ERR "msp430_resume failed\n");
		return err;
	}
	if ((ps_msp430->mode != BOOTMODE) && (ps_msp430->mode != FACTORYMODE))
		ps_msp430->mode = NORMALMODE;

	if (ps_msp430->psuedo_isr) {
		KDEBUG(KERN_INFO "%s() queuing psuedo_isr with wakelock\n",
		       __func__);
		if (!wake_lock_active(&(ps_msp430->wake_lock)))
			wake_lock(&(ps_msp430->wake_lock));

		queue_work(ps_msp430->irq_work_queue, &ps_msp430->irq_work);
		ps_msp430->psuedo_isr = false;
	}

	if (ps_msp430->offmode_wakeups_disabled) {
		cpcap_disable_offmode_wakeups(false);
		ps_msp430->offmode_wakeups_disabled = false;
	}
	mutex_unlock(&ps_msp430->lock);
	return err;
}

static int msp430_suspend(struct i2c_client *client, pm_message_t mesg)
{

	struct msp430_data *ps_msp430 = i2c_get_clientdata(client);
	int err = 0;
	unsigned char buff[10];

	KDEBUG(KERN_INFO "msp430_suspend\n");
	mutex_lock(&ps_msp430->lock);
	if ((ps_msp430->mode == NORMALMODE) && (ps_msp430->mode != FACTORYMODE))
		ps_msp430->mode = UNINITIALIZED;
	msp430_check_for_motion(ps_msp430);
	if (!has_wake_lock(WAKE_LOCK_OFFMODE)) {
		printk(KERN_INFO "%s Going into offmode\n", __func__);

		/* Tell MSP we're going to try to go into offmode */
		ps_msp430->intp_mask = ps_msp430->intp_mask | M_ANY_MOTION;
		buff[0] = INTERRUPT_MASK;
		buff[1] = ps_msp430->intp_mask;
		err = msp430_i2c_write(ps_msp430, buff, 2);
		if (err < 0)
			printk(KERN_ERR "Unable to enable any motion irq\n");
		else {
			cpcap_disable_offmode_wakeups(true);
			ps_msp430->offmode_wakeups_disabled = true;
		}
	} else {
		KDEBUG(KERN_INFO "%s Going into retention\n", __func__);
	}
	if (msp430_disable(ps_msp430) < 0) {
		printk(KERN_ERR "msp430_suspend failed\n");
		return err;
	}
	mutex_unlock(&ps_msp430->lock);
	return err;
}

static const struct i2c_device_id msp430_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, msp430_id);

static struct i2c_driver msp430_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = msp430_probe,
	.remove = __devexit_p(msp430_remove),
	.resume = msp430_resume,
	.suspend = msp430_suspend,
	.id_table = msp430_id,
};

static int __init msp430_init(void)
{
	pr_info(KERN_INFO "msp430 sensor processor\n");
	printk(KERN_INFO "msp430_init\n");
	return i2c_add_driver(&msp430_driver);
}

static void __exit msp430_exit(void)
{
	printk(KERN_INFO "msp430_exit\n");
	i2c_del_driver(&msp430_driver);
	return;
}

module_init(msp430_init);
module_exit(msp430_exit);

MODULE_DESCRIPTION("MSP430 sensor processor");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
