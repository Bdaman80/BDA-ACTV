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

#ifndef __LED_CPCAP_CHRG_H__
#define __LED_CPCAP_CHRG_H__

#define CPCAP_CHRG_LED_CLASS_NAME_SIZE   64
#define CPCAP_CHRG_LED_CHRG_CLASS_NAME   "cpcap-chrg-led"
#define CPCAP_CHRG_LED_DRV_NAME          "leds-cpcap-chrg"
#define CPCAP_CHRG_ON_OFF_MASK           0x2000
#define CPCAP_CHRG_LED_EN                0x2000 /* Enable the charging LED */
#define CPCAP_CHRG_LED_DIS               0x0000 /* Disable the charging LED */
#define CPCAP_CHRG_ON                    0x0001

struct cpcap_chrg_led_config_data {
    u16 init;
    u16 on;
	char class_name[CPCAP_CHRG_LED_CLASS_NAME_SIZE];
};

#define CPCAP_CHRG_LOW_LIMIT      51
#define CPCAP_CHRG_LOW_MED_LIMIT  104
#define CPCAP_CHRG_MEDIUM_LIMIT   155
#define CPCAP_CHRG_MED_HIGH_LIMIT 201

#define CPCAP_CHRG_LOW_VALUE      0x23
#define CPCAP_CHRG_LOW_MED_VALUE  0x23
#define CPCAP_CHRG_MEDIUM_VALUE   0x33
#define CPCAP_CHRG_MED_HIGH_VALUE 0x43
#define CPCAP_CHRG_HIGH_VALUE     0x53

#endif  /* __LED_CPCAP_CHRG_H__ */
