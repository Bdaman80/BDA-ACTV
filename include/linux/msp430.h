/*
 * Copyright (c) 2010, Motorola, Inc. All Rights Reserved.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __MSP430_H__
#define __MSP430_H__

/** The following define the IOCTL command values via the ioctl macros */
#define MSP430_IOCTL_BOOTLOADERMODE	0
#define MSP430_IOCTL_NORMALMODE         1
#define MSP430_IOCTL_MASSERASE          2
#define MSP430_IOCTL_SETSTARTADDR       3
#define MSP430_IOCTL_TEST_READ		4
#define MSP430_IOCTL_TEST_WRITE		5
#define MSP430_IOCTL_TEST_WRITE_READ    6
#define MSP430_IOCTL_SET_MAG_DELAY      7
#define MSP430_IOCTL_TEST_BOOTMODE      8
#define MSP430_IOCTL_SET_ACC_DELAY      9
#define MSP430_IOCTL_SET_MOTION_DELAY   10
#define MSP430_IOCTL_SET_ENV_DELAY      11
#define MSP430_IOCTL_SET_DEBUG          12
#define MSP430_IOCTL_SET_USER_PROFILE   13
#define MSP430_IOCTL_SET_GPS_DATA       14
#define MSP430_IOCTL_SET_SEA_LEVEL_PRESSURE 15
#define MSP430_IOCTL_SET_REF_ALTITUDE       16
#define MSP430_IOCTL_SET_ACTIVE_MODE        17
#define MSP430_IOCTL_SET_PASSIVE_MODE       18
#define MSP430_IOCTL_SET_FACTORY_MODE       19
#define MSP430_IOCTL_TEST_ENABLE_INTERRUPTS 20
#define MSP430_IOCTL_TEST_DISABLE_INTERRUPTS 21
#define MSP430_IOCTL_GET_VERSION            22
#define MSP430_IOCTL_SET_MONITOR_DELAY  23
#define MSP430_IOCTL_SET_DOCK_STATUS     24
#define MSP430_IOCTL_SET_ORIENTATION_DELAY   25
#define MSP430_IOCTL_SET_EQUIPMENT_TYPE      26
#define MSP430_IOCTL_SET_POSIX_TIME     27
#define MSP430_IOCTL_SET_MANUAL_CALIB_STATUS 28
#define MSP430_IOCTL_SET_MANUAL_CALIB_WALK_SPEED 29
#define MSP430_IOCTL_SET_MANUAL_CALIB_RUN_SPEED  30
#define MSP430_IOCTL_SET_MANUAL_CALIB_JOG_SPEED  31
#define MSP430_IOCTL_GET_MANUAL_CALIB_TABLE     32
#define MSP430_IOCTL_SET_MANUAL_CALIB_TABLE     33
#define MSP430_IOCTL_GET_MANUAL_CALIB_STATUS    34
#define MSP430_IOCTL_SET_USER_CALIB_TABLE       35
#define MSP430_IOCTL_SET_USER_DISTANCE          36
#define MSP430_MAX_PACKET_LENGTH           250

#ifdef __KERNEL__
struct msp430_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	int gpio_reset;
	int gpio_test;
	int gpio_int;
	unsigned int test_pin_active_value;
};

void msp430_psuedo_isr(void);
void msp430_disable_offmode(void);

#endif /* __KERNEL__ */

struct msp430_user_profile {

	unsigned char sex;
	unsigned char age;
	unsigned char height;
	unsigned char weight;
};

struct msp430_gps_data {

#ifdef G1
	int latitude;
	int longitude;
	int altitude;
	short heading;
	unsigned char horizontal_accuracy;
	unsigned char vertical_accuracy;
#endif
	unsigned char speed;
};

struct msp430_workout_data {
	int msp_distance;
	int user_distance;
};

#endif  /* __MSP430_H__ */

