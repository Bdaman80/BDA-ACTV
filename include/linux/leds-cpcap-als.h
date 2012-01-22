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

#ifndef __LED_CPCAP_ALS_H__
#define __LED_CPCAP_ALS_H__

#define CPCAP_ALS_LED_CLASS_NAME_SIZE 64
#define CPCAP_ALS_LED_CLASS_NAME "cpcap-als-led"
#define CPCAP_ALS_LED_DRV_NAME "leds-cpcap-als"

#define CPCAP_ALS_REG "vsdio"

#define	MANUAL			0
#define	AUTOMATIC		1

#define BACKLIGHT_ALS "als"

struct cpcap_als_led_config_data {
	char class_name[CPCAP_ALS_LED_CLASS_NAME_SIZE];
};

#endif  /* __LED_CPCAP_ALS_H__ */
