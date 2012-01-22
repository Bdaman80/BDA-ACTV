/*
 * cs42l73.c  --  CS42L73 ALSA Soc Audio driver
 *
 * Copyright 2010 Cirrus Logic, Inc.
 *
 * Author: Georgi Vlaev, Nucleus Systems Ltd, <office@nucleusys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 2010.10 - Initial release
 *
 */

/*
    CS42L73 Codec

    The CS42L73 has three independent, highly configurable, serial ports
    to communicate audio and voice data to and from other devices in the
    system such as application processors, bluetooth transceivers and
    cellphone modems. They are the auXilary, Audio, and Voice Serial
    Ports (XSP, ASP, and VSP).
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/cpcap.h>
#include <mach/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/cs42l73.h>

#define CS42L73_DEVID		0x00042A73
#define CS42L73_MCLKX_MIN	5644800
#define CS42L73_MCLKX_MAX	38400000
#define CS42L73_INT_B_GPIO	27
#define CS42L73_RESET_GPIO	15
#define AUDIO_CLOCK_REQ_GPIO	25
#define HS_KEY_DETECT_GPIO     80
#define HS_KEY_ENABLE_GPIO     81

int hs_key_detect_irq_enabled;
int codec_key_detect_irq_enabled;

/* Copied from cpcap-3mm5.c */
enum {
	NO_DEVICE,
	HEADSET_WITH_MIC,
	HEADSET_WITHOUT_MIC,
};

/**
 * snd_soc_get_volsw - single mixer get callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to get the value of a single mixer control.
 *
 * Returns 0 for success.
 */
int cs42l73_get_volsw(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;

	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	int min = mc->min;
	int mmax = (max > min) ? max : min;
	unsigned int mask = (1 << fls(mmax)) - 1;

	ucontrol->value.integer.value[0] =
	    ((snd_soc_read(codec, reg) >> shift) - min) & mask;
	if (shift != rshift)
		ucontrol->value.integer.value[1] =
		    ((snd_soc_read(codec, reg) >> rshift) - min) & mask;

	return 0;
}

/**
 * snd_soc_put_volsw - single mixer put callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to set the value of a single mixer control.
 *
 * Returns 0 for success.
 */
int cs42l73_put_volsw(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;

	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	int min = mc->min;
	int mmax = (max > min) ? max : min;
	unsigned int mask = (1 << fls(mmax)) - 1;
	unsigned short val, val2, val_mask;

	val = ((ucontrol->value.integer.value[0] + min) & mask);

	val_mask = mask << shift;
	val = val << shift;
	if (shift != rshift) {
		val2 = ((ucontrol->value.integer.value[1] + min) & mask);
		val_mask |= mask << rshift;
		val |= val2 << rshift;
	}
	return snd_soc_update_bits(codec, reg, val_mask, val);
}

/**
 * snd_soc_info_volsw_2r - double mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a double mixer control that
 * spans 2 codec registers.
 *
 * Returns 0 for success.
 */
int cs42l73_info_volsw_2r(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;

	int max = mc->max;

	if (max == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}

/**
 * snd_soc_get_volsw_2r - double mixer get callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to get the value of a double mixer control that spans 2 registers.
 *
 * Returns 0 for success.
 */
int cs42l73_get_volsw_2r(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;

	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	int max = mc->max;
	int min = mc->min;
	int mmax = (max > min) ? max : min;
	unsigned int mask = (1 << fls(mmax)) - 1;
	int val, val2;

	val = snd_soc_read(codec, reg);
	val2 = snd_soc_read(codec, reg2);
	ucontrol->value.integer.value[0] = (val - min) & mask;
	ucontrol->value.integer.value[1] = (val2 - min) & mask;
	return 0;
}

/**
 * snd_soc_put_volsw_2r - double mixer set callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to set the value of a double mixer control that spans 2 registers.
 *
 * Returns 0 for success.
 */
int cs42l73_put_volsw_2r(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;

	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	int max = mc->max;
	int min = mc->min;
	int mmax = (max > min) ? max : min;
	unsigned int mask = (1 << fls(mmax)) - 1;
	int err;
	unsigned short val, val2;

	val = (ucontrol->value.integer.value[0] + min) & mask;
	val2 = (ucontrol->value.integer.value[1] + min) & mask;

	if ((err = snd_soc_update_bits(codec, reg, mask, val)) < 0)
		return err;

	return snd_soc_update_bits(codec, reg2, mask, val2);
}

#define SOC_SINGLE_S8_C_TLV(xname, xreg, xshift, xmax, xmin, tlv_array) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
        .access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
                  SNDRV_CTL_ELEM_ACCESS_READWRITE, \
        .info = snd_soc_info_volsw, .get = cs42l73_get_volsw,\
        .put = cs42l73_put_volsw, .tlv.p  = (tlv_array),\
        .private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .shift = xshift, .rshift = xshift, \
		.max = xmax, .min = xmin} }

#define SOC_DOUBLE_R_S8_C_TLV(xname, xreg, xrreg, xmax, xmin, tlv_array) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
        .access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
                  SNDRV_CTL_ELEM_ACCESS_READWRITE, \
        .info = cs42l73_info_volsw_2r, \
        .get = cs42l73_get_volsw_2r, .put = cs42l73_put_volsw_2r, \
        .tlv.p  = (tlv_array), \
        .private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .rreg = xrreg, .max = xmax, .min = xmin} }

static const u8 cs42l73_reg[CS42L73_CACHEREGNUM] = {
/* 0*/ 0x00, 0x42, 0xA7, 0x30,
/* 4*/ 0x00, 0x00, 0xF1, 0xDF,
/* 8*/ 0x3F, 0x57, 0x43, 0x00,
/* C*/ 0x00, 0x15, 0x00, 0x15,
/*10*/ 0x00, 0x15, 0x00, 0x06,
/*14*/ 0x00, 0x00, 0x00, 0x00,
/*18*/ 0x00, 0x00, 0x00, 0x00,
/*1C*/ 0x00, 0x00, 0x00, 0x00,
/*20*/ 0x00, 0x00, 0x00, 0x00,
/*24*/ 0x00, 0x00, 0x00, 0x7F,
/*28*/ 0x00, 0x00, 0x3F, 0x00,
/*2C*/ 0x00, 0x3F, 0x00, 0x00,
/*30*/ 0x3F, 0x00, 0x00, 0x00,
/*34*/ 0x18, 0x3F, 0x3F, 0x3F,
/*38*/ 0x3F, 0x3F, 0x3F, 0x3F,
/*3C*/ 0x3F, 0x3F, 0x3F, 0x3F,
/*40*/ 0x3F, 0x3F, 0x3F, 0x3F,
/*44*/ 0x3F, 0x3F, 0x3F, 0x3F,
/*48*/ 0x3F, 0x3F, 0x3F, 0x3F,
/*4C*/ 0x3F, 0x3F, 0x3F, 0x3F,
/*50*/ 0x3F, 0x3F, 0x3F, 0x3F,
/*54*/ 0x3F, 0xAA, 0x3F, 0x3F,
/*58*/ 0x3F, 0x3F, 0x3F, 0x3F,
/*5C*/ 0x3F, 0x3F, 0x00, 0x00,
/*60*/ 0x00, 0x00
};

#define CS42L73_SPC(id) (CS42L73_XSPC + (id << 1))
#define CS42L73_MMCC(id) (CS42L73_XSPMMCC + (id << 1))
#define CS42L73_SPFS(id) ((id == CS42L73_ASP) ? CS42L73_ASPC : CS42L73_VXSPFS)

struct sp_config {
	u8 spc, mmcc, spfs;
	u32 srate;
};

struct cs42l73_priv {
	struct snd_soc_codec codec;
	u8 reg_cache[CS42L73_CACHEREGNUM];
	u32 sysclk;		/* external MCLK */
	u8 mclksel;		/* MCLKx */
	u32 mclk;		/* internal MCLK */
	struct sp_config config[3];
	struct regulator *vwlan1;
	struct work_struct work;
	struct delayed_work mic1_bias_work;
	struct input_dev *input_dev;
	struct wake_lock offmode_lock;
};

struct snd_soc_codec *cs42l73_codec;
/* Initialize to 1 so HS detection waits until CODEC is initialized */
int codec_suspended = 1;
int dsp_requested_clock;

void audio_clock_request(int request_clock)
{
	dsp_requested_clock = request_clock;
	if (request_clock)
		/* Set AUDIO_CLOCK_REQ_GPIO to high */
		gpio_set_value(AUDIO_CLOCK_REQ_GPIO, 1);
	else if (cs42l73_codec->bias_level >= SND_SOC_BIAS_STANDBY)
		/* Set AUDIO_CLOCK_REQ_GPIO to low */
		gpio_set_value(AUDIO_CLOCK_REQ_GPIO, 0);
}
EXPORT_SYMBOL_GPL(audio_clock_request);

/*
    CS42L73 I2C read/write.
    7 bit address, 8 bit data
*/

static inline int cs42l73_read_reg_cache(struct snd_soc_codec *codec, u_int reg)
{
	u8 *cache = codec->reg_cache;

	return reg > CS42L73_CACHEREGNUM ? -EINVAL : cache[reg];
}

static inline void cs42l73_write_reg_cache(struct snd_soc_codec *codec,
					   u_int reg, u_int val)
{
	u8 *cache = codec->reg_cache;

	if (reg > CS42L73_CACHEREGNUM)
		return;

	cache[reg] = val & 0xff;
}

int cs42l73_write(struct snd_soc_codec *codec, unsigned reg, u_int val)
{
	u8 data[2];

	if (reg > CS42L73_CACHEREGNUM)
		return -EINVAL;

	data[0] = reg & 0x7f;	/* reg address */
	data[1] = val & 0xff;	/* reg value */

	dev_dbg(codec->dev, "%s: reg 0x%x = %02x (%d)\n",
		 __FUNCTION__, reg, val, val);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		dev_dbg(codec->dev, "%s: hw_write error\n", __func__);
		return -EIO;
	}

	cs42l73_write_reg_cache(codec, reg, val);
	return 0;

}
EXPORT_SYMBOL_GPL(cs42l73_write);

unsigned int cs42l73_read(struct snd_soc_codec *codec, u_int reg)
{
	u8 *cache = codec->reg_cache;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	u8 data = 0;
	u8 addr;

	if (reg <= CS42L73_REVID || reg >= CS42L73_IS1) {
		addr = reg & 0x7f;
		if (codec->hw_write(codec->control_data, &addr, 1) != 1) {
			dev_dbg(codec->dev, "%s: hw_write error\n", __func__);
			return -EIO;
		}

		if (i2c_master_recv(codec->control_data, &data, 1) != 1) {
			dev_dbg(codec->dev, "%s: i2c_master_recv error\n",
				__func__);
			return -EIO;
		}

		dev_dbg(codec->dev, "%s: reg 0x%x = %02x (%d)\n",
			 __FUNCTION__, addr, data, data);

		return data;
	}

	if (reg > CS42L73_CACHEREGNUM)
		return -EINVAL;
#endif
	dev_dbg(codec->dev, "%s: reg 0x%x = %02x (%d)\n",
		__FUNCTION__, reg, cache[reg], cache[reg]);

	return cache[reg];
}
EXPORT_SYMBOL_GPL(cs42l73_read);

/*
    HP,LO Analog Volume TLV
    -76dB ... -50 dB in 2dB steps
    -50dB ... 12dB in 1dB steps
*/
static const unsigned int hpaloa_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 13, TLV_DB_SCALE_ITEM(-7600, 200, 0),
	14,75, TLV_DB_SCALE_ITEM(-4900, 100, 0),
};

/* -102dB ... 12 dB in 0.5 dB steps */
static DECLARE_TLV_DB_SCALE(hl_tlv, -10200, 50, 0);

/* -96dB ... 12 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(ipd_tlv, -9600, 100, 0);

/* -6dB ... 12 dB in 0.5 dB steps */
static DECLARE_TLV_DB_SCALE(micpga_tlv, -600, 50, 0);

/*
    HL, ESL, SPK, Limiter Threshold/Cushion TLV
    0dB -12 dB in -3dB steps
    -12dB -30dB in -6dB steps
*/
static const unsigned int limiter_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 2, TLV_DB_SCALE_ITEM(-3000, 600, 0),
	3, 7, TLV_DB_SCALE_ITEM(-1200, 300, 0),
};

/*
    * Stereo Mixer Input Attenuation (regs 35h-54h) TLV
    * Mono Mixer Input Attenuation (regs 56h-5Dh)

    -62dB ... 0dB in 1dB steps, < -62dB = mute
*/
static const DECLARE_TLV_DB_SCALE(attn_tlv, -6300, 100, 1);

/* Stereo Attenuation Group */
#define SOC_DOUBLE_R_CS42L73_ATTN_GRP(xdest,  xregl_start) \
    SOC_DOUBLE_R_TLV(xdest"-IP Attenuation Volume",\
	 xregl_start + 0, xregl_start + 1, 0, 0x3F, 1, attn_tlv), \
    SOC_DOUBLE_R_TLV(xdest"-XSP Attenuation Volume",\
	 xregl_start + 2, xregl_start + 3, 0, 0x3F, 1, attn_tlv), \
    SOC_DOUBLE_R_TLV(xdest"-ASP Attenuation Volume",\
	 xregl_start + 4, xregl_start + 5, 0, 0x3F, 1, attn_tlv), \
    SOC_DOUBLE_R_TLV(xdest"-VSP Attenuation Volume",\
	 xregl_start + 6, xregl_start + 7, 0, 0x3F, 1, attn_tlv)

/* Mono Attenuation Group */
#define SOC_SINGLE_CS42L73_ATTN_GRP(xdest,  xreg_start) \
    SOC_SINGLE_TLV(xdest"-IP Mono Attenuation Volume",\
	 xreg_start + 0, 0, 0x3F, 1, attn_tlv), \
    SOC_SINGLE_TLV(xdest"-XSP Mono Attenuation Volume",\
	 xreg_start + 1, 0, 0x3F, 1, attn_tlv), \
    SOC_SINGLE_TLV(xdest"-ASP Mono Attenuation Volume",\
	 xreg_start + 2, 0, 0x3F, 1, attn_tlv), \
    SOC_SINGLE_TLV(xdest"-VSP Mono Attenuation Volume",\
	 xreg_start + 3, 0, 0x3F, 1, attn_tlv)

/* Analog Input PGA Mux */
static const char *cs42l73_pgaa_text[] = { "LINEINA", "MIC1" };
static const char *cs42l73_pgab_text[] = { "LINEINB", "MIC2" };


static const struct soc_enum pgaa_enum =
	SOC_ENUM_SINGLE(CS42L73_ADCIPC, 3,
		ARRAY_SIZE(cs42l73_pgaa_text), cs42l73_pgaa_text);

static const struct soc_enum pgab_enum =
	SOC_ENUM_SINGLE(CS42L73_ADCIPC, 7,
		ARRAY_SIZE (cs42l73_pgab_text), cs42l73_pgab_text);

static const struct snd_kcontrol_new pgaa_mux =
SOC_DAPM_ENUM("Left Analog Input Capture Mux", pgaa_enum);

static const struct snd_kcontrol_new pgab_mux =
SOC_DAPM_ENUM("Right Analog Input Capture Mux", pgab_enum);

/* NG */
static const char *cs42l73_ng_delay_text[] =
	{ "50ms", "100ms", "150ms", "200ms" };

static const struct soc_enum ng_delay_enum =
	SOC_ENUM_SINGLE(CS42L73_NGCAB, 0,
		ARRAY_SIZE (cs42l73_ng_delay_text), cs42l73_ng_delay_text);

/* Mono Mixer Select*/
static const char *cs42l73_mono_mixer_text[] =
	{ "Left", "Right", "Mono Mix"};

/* ESL-ASP, ESL-XSP, SPK-ASP, SPK-XSP Mono Mixer Selects */
static const struct soc_enum mono_mixer_enum[] =
{
	SOC_ENUM_SINGLE(CS42L73_MMIXCTL, 6,
		ARRAY_SIZE (cs42l73_mono_mixer_text), cs42l73_mono_mixer_text),
	SOC_ENUM_SINGLE(CS42L73_MMIXCTL, 4,
		ARRAY_SIZE (cs42l73_mono_mixer_text), cs42l73_mono_mixer_text),
	SOC_ENUM_SINGLE(CS42L73_MMIXCTL, 2,
		ARRAY_SIZE (cs42l73_mono_mixer_text), cs42l73_mono_mixer_text),
	SOC_ENUM_SINGLE(CS42L73_MMIXCTL, 0,
		ARRAY_SIZE (cs42l73_mono_mixer_text), cs42l73_mono_mixer_text),
};

static const char *cs42l73_ip_swap_text[] = {
	"Stereo", "Mono A", "Mono B", "Swap A-B"};

static const struct soc_enum ip_swap_enum =
	SOC_ENUM_SINGLE(CS42L73_MIOPC, 6,
		ARRAY_SIZE(cs42l73_ip_swap_text), cs42l73_ip_swap_text);

/* XSPOUT, VSPOUT Mixer output */
static const char *cs42l73_spo_mixer_text[] =
	{ "Mono", "Stereo"};

static const struct soc_enum spo_mixer_enum[] =
{
	SOC_ENUM_SINGLE(CS42L73_MIXERCTL, 5,
		ARRAY_SIZE (cs42l73_spo_mixer_text), cs42l73_spo_mixer_text),
	SOC_ENUM_SINGLE(CS42L73_MIXERCTL, 4,
		ARRAY_SIZE (cs42l73_spo_mixer_text), cs42l73_spo_mixer_text),
};

/* DMICs are muxed with PGA. Input Path is selected wiht the XXX_PDN bits */

static const struct snd_kcontrol_new cs42l73_snd_controls[] = {
/*
    SOC_DOUBLE_R_S8_C (..., max, min)
    min - min from CS datasheet
    max - fls(max) - min + max from CS datasheet
*/
/* Volume */
	SOC_DOUBLE_R_S8_C_TLV("Headphone Analog Playback Volume", CS42L73_HPAAVOL,
			  CS42L73_HPBAVOL, 0x4B, 0x41, hpaloa_tlv),
	SOC_DOUBLE_R_S8_C_TLV("LineOut Analog Playback Volume", CS42L73_LOAAVOL,
			  CS42L73_LOBAVOL, 0x4B, 0x41, hpaloa_tlv),
	SOC_DOUBLE_R_S8_C_TLV("Input PGA Analog Volume", CS42L73_MICAPREPGAAVOL,
			  CS42L73_MICBPREPGABVOL, 0x24, 0x34, micpga_tlv),
	SOC_DOUBLE_R("MIC Preamp Switch", CS42L73_MICAPREPGAAVOL,
		     CS42L73_MICBPREPGABVOL, 6, 1, 0),

	SOC_DOUBLE_R_S8_C_TLV("Input Path Digital Volume", CS42L73_IPADVOL,
			  CS42L73_IPBDVOL, 0x6C, 0xA0, ipd_tlv),
	SOC_DOUBLE_R_S8_C_TLV("Headphone LineOut Digital Playback Volume",
			  CS42L73_HLADVOL, CS42L73_HLBDVOL, 0xE4, 0x34, hl_tlv),

/* Single select Volume */
	SOC_SINGLE_S8_C_TLV("Headphone Analog A Playback Volume",
			  CS42L73_HPAAVOL, 0, 0x4B, 0x41, hpaloa_tlv),
	SOC_SINGLE_S8_C_TLV("Headphone Analog B Playback Volume",
			  CS42L73_HPBAVOL, 0, 0x4B, 0x41, hpaloa_tlv),

	SOC_SINGLE_S8_C_TLV("HL-A Digital Playback Volume",
			  0, CS42L73_HLADVOL, 0xE4, 0x34, hl_tlv),
	SOC_SINGLE_S8_C_TLV("HL-B Digital Playback Volume",
			  0, CS42L73_HLBDVOL, 0xE4, 0x34, hl_tlv),

	SOC_SINGLE_S8_C_TLV("LineOut Analog A Playback Volume", CS42L73_LOAAVOL,
			  0, 0x4B, 0x41, hpaloa_tlv),
	SOC_SINGLE_S8_C_TLV("LineOut Analog B Playback Volume", CS42L73_LOBAVOL,
			  0, 0x4B, 0x41, hpaloa_tlv),

	SOC_SINGLE_S8_C_TLV("MIC 1 PGA Analog Volume", CS42L73_MICAPREPGAAVOL,
			  0, 0x24, 0x34, micpga_tlv),
	SOC_SINGLE_S8_C_TLV("MIC 2 PGA Analog Volume", CS42L73_MICBPREPGABVOL,
			  0, 0x24, 0x34, micpga_tlv),

	SOC_SINGLE_S8_C_TLV("Input Path A Digital Volume", CS42L73_IPADVOL,
			  0, 0x6C, 0xA0, ipd_tlv),
	SOC_SINGLE_S8_C_TLV("Input Path B Digital Volume", CS42L73_IPBDVOL,
			  0, 0x6C, 0xA0, ipd_tlv),

	SOC_SINGLE_S8_C_TLV("Speakerphone Digital Playback Volume", CS42L73_SPKDVOL,
			0, 0xE4, 0x34, hl_tlv),
	SOC_SINGLE_S8_C_TLV("Ear Speaker Digital Playback Volume", CS42L73_ESLDVOL,
			0, 0xE4, 0x34, hl_tlv),

/* Digital/Analog Mute */
	SOC_DOUBLE_R("Headphone Analog Playback Switch", CS42L73_HPAAVOL,
		CS42L73_HPBAVOL, 7, 1, 1),
	SOC_SINGLE("Headphone A Analog Playback Switch", CS42L73_HPAAVOL,
		7, 1, 1),
	SOC_SINGLE("Headphone B Analog Playback Switch", CS42L73_HPBAVOL,
		7, 1, 1),

	SOC_DOUBLE_R("LineOut Analog Playback Switch", CS42L73_LOAAVOL,
		     CS42L73_LOBAVOL, 7, 1, 1),
	SOC_SINGLE("LineOut A Analog Playback Switch", CS42L73_LOAAVOL,
		7, 1, 1),
	SOC_SINGLE("LineOut B Analog Playback Switch", CS42L73_LOBAVOL,
		7, 1, 1),

	SOC_DOUBLE("Input Path Digital Switch", CS42L73_ADCIPC, 0, 4, 1, 1),
	SOC_DOUBLE("Headphone LineOut Digital Playback Switch", CS42L73_PBDC, 0,
		   1, 1, 1),
	SOC_SINGLE("Speakerphone Digital Playback Switch", CS42L73_PBDC, 2, 1,
		   1),
	SOC_SINGLE("Ear Speaker Digital Playback Switch", CS42L73_PBDC, 3, 1,
		   1),

	SOC_SINGLE("PGA Soft-Ramp Switch", CS42L73_MIOPC, 3, 1, 0),
	SOC_SINGLE("Analog Zero Cross Switch", CS42L73_MIOPC, 2, 1, 0),
	SOC_SINGLE("Digital Soft-Ramp Switch", CS42L73_MIOPC, 1, 1, 0),
	SOC_SINGLE("Analog Output Soft-Ramp Switch", CS42L73_MIOPC, 0, 1, 0),

/* ADC */
	SOC_DOUBLE("Invert ADC Signal Polarity Switch", CS42L73_ADCIPC, 1, 5, 1,
		   0),
	SOC_DOUBLE("ADC Boost Switch", CS42L73_ADCIPC, 2, 6, 1, 0),

	SOC_SINGLE("Charge Pump Frequency Volume", CS42L73_CPFCHC, 4, 15, 0),

/* Headphone/LineOut (HL) Limiter */
	SOC_SINGLE("HL Limiter Attack Rate Volume", CS42L73_LIMARATEHL, 0, 0x3F,
		   0),
	SOC_SINGLE("HL Limiter Release Rate Volume", CS42L73_LIMRRATEHL, 0,
		   0x3F, 0),
	SOC_SINGLE("HL Limiter Switch", CS42L73_LIMRRATEHL, 7, 1, 0),
	SOC_SINGLE("HL Limiter All Channels Switch", CS42L73_LIMRRATEHL, 6, 1,
		   0),

	SOC_SINGLE_TLV("HL Limiter Max Threshold Volume", CS42L73_LMAXHL, 5, 7,
		       1,
		       limiter_tlv),
	SOC_SINGLE_TLV("HL Limiter Cushion Volume", CS42L73_LMAXHL, 2, 7, 1,
		       limiter_tlv),

/* Speakerphone Limiter */
	SOC_SINGLE("SPK Limiter Attack Rate Volume", CS42L73_LIMARATESPK, 0,
		   0x3F, 0),
	SOC_SINGLE("SPK Limiter Release Rate Volume", CS42L73_LIMRRATESPK, 0,
		   0x3F, 0),
	SOC_SINGLE("SPK Limiter Switch", CS42L73_LIMRRATESPK, 7, 1, 0),
	SOC_SINGLE("SPK Limiter All Channels Switch", CS42L73_LIMRRATESPK, 6, 1,
		   0),
	SOC_SINGLE_TLV("SPK Limiter Max Threshold Volume", CS42L73_LMAXSPK, 5,
		       7, 1,
		       limiter_tlv),
	SOC_SINGLE_TLV("SPK Limiter Cushion Volume", CS42L73_LMAXSPK, 2, 7, 1,
		       limiter_tlv),

/* Earphone/Speakerphone LO Limiter */
	SOC_SINGLE("ESL Limiter Attack Rate Volume", CS42L73_LIMARATEESL, 0,
		   0x3F, 0),
	SOC_SINGLE("ESL Limiter Release Rate Volume", CS42L73_LIMRRATEESL, 0,
		   0x3F, 0),
	SOC_SINGLE("ESL Limiter Switch", CS42L73_LIMRRATEESL, 7, 1, 0),
	SOC_SINGLE_TLV("ESL Limiter Max Threshold Volume", CS42L73_LMAXESL, 5,
		       7, 1,
		       limiter_tlv),
	SOC_SINGLE_TLV("ESL Limiter Cushion Volume", CS42L73_LMAXESL, 2, 7, 1,
		       limiter_tlv),

/* ALC */

	SOC_SINGLE("ALC Attack Rate Volume", CS42L73_ALCARATE, 0, 0x3F, 0),
	SOC_SINGLE("ALC Release Rate Volume", CS42L73_ALCRRATE, 0, 0x3F, 0),
	SOC_DOUBLE("ALC Switch", CS42L73_ALCARATE, 6, 7, 1, 0),
	SOC_SINGLE_TLV("ALC Max Threshold Volume", CS42L73_ALCMINMAX, 5, 7, 1,
		       limiter_tlv),
	SOC_SINGLE_TLV("ALC Min Threshold Volume", CS42L73_ALCMINMAX, 2, 7, 1,
		       limiter_tlv),

/* Noise Gate */
	SOC_DOUBLE("NG Enable Switch", CS42L73_NGCAB, 6, 7, 1, 0),
	SOC_SINGLE("NG Boost Switch", CS42L73_NGCAB, 5, 1, 0),
	/*
	    NG Threshold depends on NG_BOOTSAB, which selects
	    between two threshold scales in decibels.
	    Set linear values for now ..
	*/
	SOC_SINGLE("NG Threshold", CS42L73_NGCAB, 2, 7, 0),
	SOC_ENUM("NG Delay", ng_delay_enum),

/* Digital IO Attenuation */
	SOC_DOUBLE_R_CS42L73_ATTN_GRP("XSP", CS42L73_XSPAIPAA),
	SOC_DOUBLE_R_CS42L73_ATTN_GRP("ASP", CS42L73_ASPAIPAA),
	SOC_DOUBLE_R_CS42L73_ATTN_GRP("VSP", CS42L73_VSPAIPAA),

/* Output Attenuation */
	SOC_DOUBLE_R_CS42L73_ATTN_GRP("HL", CS42L73_HLAIPAA),
	SOC_SINGLE_CS42L73_ATTN_GRP("SPK", CS42L73_SPKMIPMA),
	SOC_SINGLE_CS42L73_ATTN_GRP("ESL", CS42L73_ESLMIPMA),

/* Channel Swap Record Enum */
	SOC_ENUM("IP Digital Swap/Mono Select", ip_swap_enum),

/* Mono Mixer Select*/
	SOC_ENUM("ESL-ASP Mono Mixer Select", mono_mixer_enum[0]),
	SOC_ENUM("ESL-XSP Mono Mixer Select", mono_mixer_enum[1]),
	SOC_ENUM("SPK-ASP Mono Mixer Select", mono_mixer_enum[2]),
	SOC_ENUM("SPK-XSP Mono Mixer Select", mono_mixer_enum[3]),

/* XSPOUT, VSPOUT Output Mixer Path Select */
	SOC_ENUM("VSP Output Mixer Select", spo_mixer_enum[0]),
	SOC_ENUM("XSP Output Mixer Select", spo_mixer_enum[1]),

};

static const struct snd_soc_dapm_widget cs42l73_dapm_widgets[] = {
/* Platform / Analog Inputs */
	SND_SOC_DAPM_INPUT("LINEINA"),
	SND_SOC_DAPM_INPUT("LINEINB"),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("DMICA"),
	SND_SOC_DAPM_INPUT("DMICB"),

/* Stream */
	/* Digital Outputs*/
	SND_SOC_DAPM_AIF_OUT("XSPOUT", "XSP Capture",  0, CS42L73_PWRCTL2, 1, 1),
	SND_SOC_DAPM_AIF_OUT("ASPOUT", "ASP Capture",  0, CS42L73_PWRCTL2, 3, 1),
	SND_SOC_DAPM_AIF_OUT("VSPOUT", "VSP Capture",  0, CS42L73_PWRCTL2, 4, 1),

	/* Digital Inputs*/
	SND_SOC_DAPM_AIF_IN("XSPIN", "XSP Playback",  0, CS42L73_PWRCTL2, 0, 1),
	SND_SOC_DAPM_AIF_IN("ASPIN", "ASP Playback",  0, CS42L73_PWRCTL2, 2, 1),
	SND_SOC_DAPM_AIF_IN("VSPIN", "VSP Playback",  0, CS42L73_PWRCTL2, 4, 1),

	SND_SOC_DAPM_ADC("ADC Left", NULL, CS42L73_PWRCTL1, 5, 1),
	SND_SOC_DAPM_ADC("ADC Right", NULL, CS42L73_PWRCTL1, 7, 1),
	SND_SOC_DAPM_DAC("DAC Left", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Right", NULL, SND_SOC_NOPM, 0, 0),

/* Path */
	SND_SOC_DAPM_PGA("PGA Left", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA Right", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("PGA Mux Left", SND_SOC_NOPM, 0, 0, &pgaa_mux),
	SND_SOC_DAPM_MUX("PGA Mux Right", SND_SOC_NOPM, 0, 0, &pgab_mux),
/* HP Output PGA */
	SND_SOC_DAPM_PGA("HP Amp Left", CS42L73_PWRCTL3, 0, 1, NULL, 0),
	SND_SOC_DAPM_PGA("HP Amp Right", CS42L73_PWRCTL3, 0, 1, NULL, 0),
/* Line Output PGA */
	SND_SOC_DAPM_PGA("LO Amp Left", CS42L73_PWRCTL3, 1, 1, NULL, 0),
	SND_SOC_DAPM_PGA("LO Amp Right", CS42L73_PWRCTL3, 1, 1, NULL, 0),
/* SPK Output PGA */
	SND_SOC_DAPM_PGA("SPK Amp", CS42L73_PWRCTL3, 2, 1, NULL, 0),
/* ESL Output PGA */
	SND_SOC_DAPM_PGA("EAR Amp", CS42L73_PWRCTL3, 3, 1, NULL, 0),
/* SPK LO PGA */
	SND_SOC_DAPM_PGA("SPKLO Amp", CS42L73_PWRCTL3, 4, 1, NULL, 0),
/* Outputs */
	SND_SOC_DAPM_OUTPUT("HPOUTA"),
	SND_SOC_DAPM_OUTPUT("HPOUTB"),
	SND_SOC_DAPM_OUTPUT("LINEOUTA"),
	SND_SOC_DAPM_OUTPUT("LINEOUTB"),
	SND_SOC_DAPM_OUTPUT("EAROUT"),
	SND_SOC_DAPM_OUTPUT("SPKOUT"),
	SND_SOC_DAPM_OUTPUT("SPKLINEOUT"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* outputs */
	{"HPOUTA", NULL, "HP Amp Left"},
	{"HPOUTB", NULL, "HP Amp Right"},
	{"LINEOUTA", NULL, "LO Amp Left"},
	{"LINEOUTB", NULL, "LO Amp Right"},
	{"SPKOUT", NULL, "SPK Amp"},
	{"EAROUT", NULL, "EAR Amp"},
	{"SPKLINEOUT", NULL, "SPKLO Amp"},

	{"HP Amp Left", "DAC", "DAC Left"},
	{"HP Amp Right", "DAC", "DAC Right"},
	{"LO Amp Left", "DAC", "DAC Left"},
	{"LO Amp Right", "DAC", "DAC Right"},
	{"SPK Amp", "DAC", "DAC Left"},
	{"SPKLO Amp", "DAC", "DAC Right"},
	{"EAR Amp", "DAC", "DAC Right"},

	/* inputs */
	{"PGA Mux Left", "LINEINA", "LINEINA"},
	{"PGA Mux Right", "LINEINB", "LINEINB"},
	{"PGA Mux Left", "MIC1", "MIC1"},
	{"PGA Mux Right", "MIC2", "MIC2"},

	{"PGA Left", NULL, "PGA Mux Left"},
	{"PGA Right", NULL, "PGA Mux Right"},
	{"ADC Left", "ADC", "PGA Left"},
	{"ADC Right", "ADC", "PGA Right"},

	/* AIFx = [XSP,ASP,VSP] */
	{"XSPOUT", NULL, "ADC Left"},
	{"XSPOUT", NULL, "ADC Right"},
	{"DAC Left", NULL, "XSPIN"},
	{"DAC Right", NULL, "XSPIN"},

	{"ASPOUT", NULL, "ADC Left"},
	{"ASPOUT", NULL, "ADC Right"},
	{"DAC Left", NULL, "ASPIN"},
	{"DAC Right", NULL, "ASPIN"},

	{"VSPOUT", NULL, "ADC Left"},
	{"VSPOUT", NULL, "ADC Right"},
	{"DAC Left", NULL, "VSPIN"},
	{"DAC Right", NULL, "VSPIN"},

	/* Digital loopback */
	{"DAC Left", NULL, "ADC Left"},
	{"DAC Right", NULL, "ADC Right"},
};

static int cs42l73_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, cs42l73_dapm_widgets,
				  ARRAY_SIZE(cs42l73_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

struct cs42l73_mclk_div {
	u32 mclk;		/* MCLK (MHz) */
	u32 srate;		/* Sample rate (KHz) */
	u8 mmcc;		/* x_MMCC[5:0] divider */
};

struct cs42l73_mclk_div cs42l73_mclk_coeffs[] = {
	/* MCLK, Sample Rate, xMMCC[5:0] */
	{5644800, 11025, 0x30},
	{5644800, 22050, 0x20},
	{5644800, 44100, 0x10},

	{6000000,  8000, 0x39},
	{6000000, 11025, 0x33},
	{6000000, 12000, 0x31},
	{6000000, 16000, 0x29},
	{6000000, 22050, 0x23},
	{6000000, 24000, 0x21},
	{6000000, 32000, 0x19},
	{6000000, 44100, 0x13},
	{6000000, 48000, 0x11},

	{6144000,  8000, 0x38},
	{6144000, 12000, 0x30},
	{6144000, 16000, 0x28},
	{6144000, 24000, 0x20},
	{6144000, 32000, 0x18},
	{6144000, 48000, 0x10},

	{6500000,  8000, 0x3C},
	{6500000, 11025, 0x35},
	{6500000, 12000, 0x34},
	{6500000, 16000, 0x2C},
	{6500000, 22050, 0x25},
	{6500000, 24000, 0x24},
	{6500000, 32000, 0x1C},
	{6500000, 44100, 0x15},
	{6500000, 48000, 0x14},

	{6400000,  8000, 0x3E},
	{6400000, 11025, 0x37},
	{6400000, 12000, 0x36},
	{6400000, 16000, 0x2E},
	{6400000, 22050, 0x27},
	{6400000, 24000, 0x26},
	{6400000, 32000, 0x1E},
	{6400000, 44100, 0x17},
	{6400000, 48000, 0x16},
};

struct cs42l73_mcklx_div {
	u32 mclkx;		/* MCLK1/2 (MHz) */
	u8 ratio;		/* Required Divide Ratio */
	u8 mclkdiv;		/* MCLKDIV[2:0] */
};

struct cs42l73_mcklx_div cs42l73_mclkx_coeffs[] = {
	{5644800,  1, 0},	/* 5644800 */
	{6000000,  1, 0},	/* 6000000 */
	{6144000,  1, 0},	/* 6144000 */
	{11289600, 2, 2},	/* 5644800 */
	{12288000, 2, 2},	/* 6144000 */
	{12000000, 2, 2},	/* 6000000 */
	{13000000, 2, 2},	/* 6500000 */
	{19200000, 3, 3},	/* 6400000 */
	{24000000, 4, 4},	/* 6000000 */
	{26000000, 4, 4},	/* 6500000 */
	{38400000, 6, 5}	/* 6400000 */
};

int cs42l73_get_mclkx_coeff(int mclkx)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs42l73_mclkx_coeffs); i++) {
		if (cs42l73_mclkx_coeffs[i].mclkx == mclkx)
			return i;
	}
	return -EINVAL;
}

int cs42l73_get_mclk_coeff(int mclk, int srate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs42l73_mclk_coeffs); i++) {
		if (cs42l73_mclk_coeffs[i].mclk == mclk &&
		    cs42l73_mclk_coeffs[i].srate == srate)
			return i;
	}
	return -EINVAL;

}

/*
    cs42l73_set_mclk()
	Set MCLK
*/
static int cs42l73_set_mclk(struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs42l73_priv *priv = (struct cs42l73_priv *)codec->private_data;

	int mclkx_coeff;
	u32 mclk = 0;
	u8 dmmcc = 0;

	/* MCLKX -> MCLK */
	mclkx_coeff = cs42l73_get_mclkx_coeff(priv->sysclk);

	if (mclkx_coeff < 0)
		return -EINVAL;

	mclk = cs42l73_mclkx_coeffs[mclkx_coeff].mclkx /
	    cs42l73_mclkx_coeffs[mclkx_coeff].ratio;

	dev_dbg(codec->dev, "MCLK%u %u  <-> internal MCLK %u\n",
		 priv->mclksel + 1, cs42l73_mclkx_coeffs[mclkx_coeff].mclkx,
		 mclk);

	dmmcc =
	    (priv->mclksel << 4) | (cs42l73_mclkx_coeffs[mclkx_coeff].mclkdiv << 1);

	snd_soc_write(codec, CS42L73_DMMCC, dmmcc);

	priv->mclk = mclk;

	return 0;
}


/*
    cs42l73_set_sysclk()
	Setup MCLK from external MCLKX
*/
static int cs42l73_set_sysclk(struct snd_soc_dai *dai,
			      int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs42l73_priv *priv = (struct cs42l73_priv *)codec->private_data;

	if (clk_id != CS42L73_CLKID_MCLK1 && clk_id != CS42L73_CLKID_MCLK2) {
		dev_err(codec->dev, "Invalid clk_id %u\n", clk_id);
		return -EINVAL;
	}

	if ((cs42l73_get_mclkx_coeff(freq) < 0)) {
		dev_err(codec->dev, "Invalid sysclk %u\n", freq);
		return -EINVAL;
	}

	priv->sysclk = freq;
	priv->mclksel = clk_id;

	return cs42l73_set_mclk(dai);
}

/*
	cs42l73_set_dai_clkdiv()
		Setup MCLKx, MMCC dividers.
		The dividers are selected from the MCLKX and the
		sample rate.
*/
static int cs42l73_set_clkdiv(struct snd_soc_dai *codec_dai,
                int div_id, int div)
{
        struct snd_soc_codec *codec = codec_dai->codec;
	int id = codec_dai->id;
        u8 reg;

        switch (div_id) {
        case CS42L73_MCLKXDIV:
		/* MCLKDIV */
                reg = snd_soc_read(codec, CS42L73_DMMCC) & 0xf1;
		snd_soc_write(codec, CS42L73_DMMCC,
					reg | ((div & 0x07) << 1));
                break;
        case CS42L73_MMCCDIV:
		/* xSP MMCC */
                reg = snd_soc_read(codec,  CS42L73_MMCC(id)) & 0xc0;
		snd_soc_write(codec, CS42L73_MMCC(id),
					reg | (div & 0x3f));
                break;
        default:
                return -EINVAL;
        }
        return 0;
}


/*
    cs42l73_set_dai_fmt()
	Setup interface formats

*/
static int cs42l73_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs42l73_priv *priv = (struct cs42l73_priv *)codec->private_data;
	int id = codec_dai->id;
	int inv, format;
	u8 spc, mmcc;

	spc = snd_soc_read(codec, CS42L73_SPC(id));
	mmcc = snd_soc_read(codec, CS42L73_MMCC(id));

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		mmcc |= MS_MASTER;
		break;

	case SND_SOC_DAIFMT_CBS_CFS:
		mmcc &= ~MS_MASTER;
		break;

	default:
		return -EINVAL;
	}

	format = (fmt & SND_SOC_DAIFMT_FORMAT_MASK);
	inv = (fmt & SND_SOC_DAIFMT_INV_MASK);

	/* interface format */
	switch (format) {
	case SND_SOC_DAIFMT_I2S:
		spc &= ~xSPDIF_PCM;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		if (mmcc & MS_MASTER) {
			dev_err(codec->dev,
				"PCM format is supported only in slave mode\n");
			return -EINVAL;
		}
		if (id == CS42L73_ASP) {
			dev_err(codec->dev,
				"PCM format is not supported on ASP port\n");
			return -EINVAL;
		}
		spc |= xSPDIF_PCM;
		break;
	default:
		return -EINVAL;
	}

	/* TODO:
	   Check the PCM format setup. These below try to
	   match Cirrus PCM formats to OMAP3 'DSP' formats
	   MMP2 ?
	 */
	if (spc & xSPDIF_PCM) {
		spc &= (31 << 3);	/* Clear PCM mode, set MSB->LSB */
		if (format == SND_SOC_DAIFMT_DSP_B
		    && inv == SND_SOC_DAIFMT_IB_IF)
			spc |= (xPCM_MODE0 << 4);
		else

		    if (format == SND_SOC_DAIFMT_DSP_B
				&& inv == SND_SOC_DAIFMT_IB_NF)
			spc |= (xPCM_MODE1 << 4);
		else

		    if (format == SND_SOC_DAIFMT_DSP_A
				&& inv == SND_SOC_DAIFMT_IB_IF)
			spc |= (xPCM_MODE1 << 4);
		else
			return -EINVAL;
	}

	priv->config[id].spc = spc;
	priv->config[id].mmcc = mmcc;

	return 0;
}

/* Sample rate converters */
static u32 cs42l73_asrc_rates[] = {
	8000, 11025, 12000, 16000, 22050,
	24000, 32000, 44100, 48000
};

static unsigned int cs42l73_get_xspfs_coeff(u32 rate)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(cs42l73_asrc_rates); i++) {
		if (cs42l73_asrc_rates[i] == rate)
			return (i + 1);
	}
	return 0;		/* 0 = Don't know */
}

static void cs42l73_update_asrc(struct snd_soc_codec *codec, int id, int srate)
{
	u8 spfs = 0;
	u8 reg;

	if (srate > 0) {
		spfs = cs42l73_get_xspfs_coeff(srate);

		switch (id) {
		case CS42L73_XSP:
			reg = cs42l73_read(codec, CS42L73_VXSPFS);
			reg &= ~0x0f;
			cs42l73_write(codec, CS42L73_VXSPFS,
				reg | spfs);
			break;
		case CS42L73_ASP:
			reg = cs42l73_read(codec, CS42L73_ASPC);
			reg &= ~0x3c;
			cs42l73_write(codec, CS42L73_ASPC,
				reg | (spfs << 2));
			break;
		case CS42L73_VSP:
			reg = cs42l73_read(codec, CS42L73_VXSPFS);
			reg &= ~0xf0;
			cs42l73_write(codec, CS42L73_VXSPFS,
				reg | (spfs << 4));
			break;
		default:
			break;
		}
	}

}

static int cs42l73_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs42l73_priv *priv = (struct cs42l73_priv *)codec->private_data;
	int id = dai->id;
	int mclk_coeff;
	int srate = params_rate(params);

	if (priv->config[id].mmcc & MS_MASTER) {
		/* CS42L73 Master */
		/* MCLK -> srate */
		mclk_coeff =
		    cs42l73_get_mclk_coeff(priv->mclk, srate);

		if (mclk_coeff < 0)
			return -EINVAL;

		dev_dbg(codec->dev,
			 "DAI[%d]: MCLK %u, srate %u, MMCC[5:0] = %x\n",
			 id, priv->mclk, srate,
			 cs42l73_mclk_coeffs[mclk_coeff].mmcc);

		priv->config[id].mmcc &= 0xC0;
		priv->config[id].mmcc |= cs42l73_mclk_coeffs[mclk_coeff].mmcc;
		priv->config[id].spc &= 0xFC;
/*    		priv->config[id].spc |= xMCK_SCLK_MCLK; */
		priv->config[id].spc |= xMCK_SCLK_64FS;

	} else {
		/* CS42L73 Slave */

		dev_dbg(codec->dev, "DAI[%d]: Slave\n", id);
		priv->config[id].spc &= 0xFC;
		priv->config[id].spc |= xMCK_SCLK_64FS;
	}
	/* Update ASRCs */
	priv->config[id].srate = srate;
	cs42l73_update_asrc(codec, id, srate);
	snd_soc_write(codec, CS42L73_SPC(id), priv->config[id].spc);
	snd_soc_write(codec, CS42L73_MMCC(id), priv->config[id].mmcc);

	return 0;
}

static int cs42l73_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	u8 pwrctl1 = snd_soc_read(codec, CS42L73_PWRCTL1);
	u8 dmmcc = snd_soc_read(codec, CS42L73_DMMCC);
	u8 miopc = snd_soc_read(codec, CS42L73_MIOPC);
	u8 hpavol = snd_soc_read(codec, CS42L73_HPAAVOL);
	u8 hpbvol = snd_soc_read(codec, CS42L73_HPBAVOL);
	u8 loavol = snd_soc_read(codec, CS42L73_LOAAVOL);
	u8 lobvol = snd_soc_read(codec, CS42L73_LOBAVOL);
	u8 pwrctl2 = snd_soc_read(codec, CS42L73_PWRCTL2);

	u8 xsp_disabled = pwrctl2 & PDN_XSP_SDIN;
	u8 asp_disabled = (pwrctl2 & PDN_ASP_SDIN) >> 2;
	u8 vsp_disabled = (pwrctl2 & PDN_VSP) >> 4;
	u8 hpamute_enabled = hpavol >> 7;
	u8 hpbmute_enabled = hpbvol >> 7;
	u8 loamute_enabled = loavol >> 7;
	u8 lobmute_enabled = lobvol >> 7;
	u8 mclk_disabled = dmmcc;
	u8 reg;
	struct cs42l73_priv *priv =
		(struct cs42l73_priv *) codec->private_data;
	struct irq_desc *desc;
	u8 analog_input_active = codec->aai_active;

	dev_dbg(codec->dev,
		"%s: Level %d,  PWRCTL1 0x%02x, DMMCC 0x%02x\n",
		__FUNCTION__, level, pwrctl1, dmmcc);

	switch (level) {
	case SND_SOC_BIAS_ON:

		/* When playing audio, prevent off mode */
		if (!wake_lock_active(&priv->offmode_lock))
			wake_lock(&priv->offmode_lock);

		/* Set AUDIO_CLOCK_REQ_GPIO to high */
		gpio_set_value(AUDIO_CLOCK_REQ_GPIO, 1);

		if (hs_key_detect_irq_enabled) {
			hs_key_detect_irq_enabled = 0;
			disable_irq_wake(gpio_to_irq(HS_KEY_DETECT_GPIO));
			disable_irq(gpio_to_irq(HS_KEY_DETECT_GPIO));
			/* Clear HS KEY */
			input_report_key(priv->input_dev, KEY_PLAYCD, 0);
			gpio_set_value(HS_KEY_ENABLE_GPIO, 0);
			if (has_vol_controls)
				msleep(450);
			else
				msleep(200);
		}

		/* If MCLK is disabled, we turn it on */
		if (mclk_disabled & 0x1)
			snd_soc_write(codec, CS42L73_DMMCC, dmmcc & ~MCLKDIS);

		snd_soc_write(codec, CS42L73_PWRCTL3, 0x1E);
		snd_soc_write(codec, CS42L73_PWRCTL1, pwrctl1 & ~PDN);

		if ((headset_connected == HEADSET_WITH_MIC) &&
				(!codec_key_detect_irq_enabled)) {
			codec_key_detect_irq_enabled = 1;
			/* Clear any pending IRQs */
			desc = irq_to_desc(gpio_to_irq(CS42L73_INT_B_GPIO));
			desc->status = desc->status & ~IRQ_PENDING;
			enable_irq(gpio_to_irq(CS42L73_INT_B_GPIO));
			enable_irq_wake(gpio_to_irq(CS42L73_INT_B_GPIO));

			/* Turn on MIC2_SDET interrupt */
			reg = snd_soc_read(codec, CS42L73_IM1);
			snd_soc_write(codec, CS42L73_IM1,
				reg | MIC2_SDET);
		}

		if ((headset_connected == HEADSET_WITH_MIC)
				&& (has_vol_controls)) {
			/* Turn MIC1 BIAS ON after delay */
			msleep(100);
			reg = snd_soc_read(codec, CS42L73_PWRCTL2);
			snd_soc_write(codec, CS42L73_PWRCTL2,
				reg & ~PDN_MIC1_BIAS);
		}

		codec->bias_level = level;
		break;

	case SND_SOC_BIAS_PREPARE:

		codec->bias_level = level;
		break;

	case SND_SOC_BIAS_STANDBY:

		if (!xsp_disabled || !asp_disabled || !vsp_disabled ||
				analog_input_active)
			break;

		/* Set clock request to high required for some registers */
		gpio_set_value(AUDIO_CLOCK_REQ_GPIO, 1);

		if (codec_key_detect_irq_enabled) {
			/* Turn off MIC2_SDET interrupt */
			reg = snd_soc_read(codec, CS42L73_IM1);
			snd_soc_write(codec, CS42L73_IM1,
				reg & ~MIC2_SDET);
			codec_key_detect_irq_enabled = 0;
			disable_irq_wake(gpio_to_irq(CS42L73_INT_B_GPIO));
			disable_irq(gpio_to_irq(CS42L73_INT_B_GPIO));
		}

		if (headset_connected == HEADSET_WITH_MIC)
			gpio_set_value(HS_KEY_ENABLE_GPIO, 1);

		/* If MCLK is disabled, we turn it on */
		if (mclk_disabled & 0x1)
			snd_soc_write(codec, CS42L73_DMMCC, dmmcc & ~MCLKDIS);

		snd_soc_write(codec, CS42L73_MIOPC, miopc | ANLGOSFT);
		snd_soc_write(codec, CS42L73_HPAAVOL, 0x80);
		snd_soc_write(codec, CS42L73_HPBAVOL, 0x80);
		snd_soc_write(codec, CS42L73_LOAAVOL, 0x80);
		snd_soc_write(codec, CS42L73_LOBAVOL, 0x80);

		/* Powerdown all ports, inputs and outputs */
		snd_soc_write(codec, CS42L73_PWRCTL3,
			PDN_THMS | PDN_SPKLO | PDN_EAR |
			PDN_SPK | PDN_LO | PDN_HP);

		if (headset_connected == HEADSET_WITH_MIC)
			snd_soc_write(codec, CS42L73_PWRCTL2,
				PDN_MIC1_BIAS | PDN_VSP | PDN_ASP_SDOUT |
				PDN_ASP_SDIN | PDN_XSP_SDOUT | PDN_XSP_SDIN);
		else
			snd_soc_write(codec, CS42L73_PWRCTL2,
				PDN_MIC1_BIAS | PDN_MIC2_BIAS |
				PDN_VSP | PDN_ASP_SDOUT | PDN_ASP_SDIN |
				PDN_XSP_SDOUT | PDN_XSP_SDIN);

		snd_soc_write(codec, CS42L73_PWRCTL1,
			PDN_ADCB | PDN_ADCA | PDN_DMICB | PDN_DMICA |
			PDN);

		snd_soc_write(codec, CS42L73_MIOPC, miopc & ~ANLGOSFT);

		if (!hpamute_enabled)
			snd_soc_write(codec, CS42L73_HPAAVOL, hpavol & ~HPMUTE);

		if (!hpbmute_enabled)
			snd_soc_write(codec, CS42L73_HPBAVOL, hpbvol & ~HPMUTE);

		if (!loamute_enabled)
			snd_soc_write(codec, CS42L73_LOAAVOL, loavol & ~LOMUTE);

		if (!lobmute_enabled)
			snd_soc_write(codec, CS42L73_LOBAVOL, lobvol & ~LOMUTE);

		msleep(20);
		snd_soc_write(codec, CS42L73_DMMCC, dmmcc | MCLKDIS);

		if ((headset_connected == HEADSET_WITH_MIC) &&
				(!hs_key_detect_irq_enabled)) {
			msleep(200);
			/* Clear any pending IRQs */
			desc = irq_to_desc(gpio_to_irq(HS_KEY_DETECT_GPIO));
			desc->status = desc->status & ~IRQ_PENDING;
			enable_irq(gpio_to_irq(HS_KEY_DETECT_GPIO));
			enable_irq_wake(gpio_to_irq(HS_KEY_DETECT_GPIO));
			hs_key_detect_irq_enabled = 1;
		}

		if (!dsp_requested_clock)
			/* Set AUDIO_CLOCK_REQ_GPIO to low */
			gpio_set_value(AUDIO_CLOCK_REQ_GPIO, 0);

		/* Audio playback done, allow off mode */
		if (wake_lock_active(&priv->offmode_lock))
			wake_unlock(&priv->offmode_lock);

		codec->bias_level = level;
		break;

	case SND_SOC_BIAS_OFF:

		if (!xsp_disabled || !asp_disabled || !vsp_disabled ||
				analog_input_active)
			break;

		/* Set clock request to high required for some registers */
		gpio_set_value(AUDIO_CLOCK_REQ_GPIO, 1);

		/* If MCLK is disabled, we turn it on */
		if (mclk_disabled & 0x1)
			snd_soc_write(codec, CS42L73_DMMCC, dmmcc & ~MCLKDIS);

		snd_soc_write(codec, CS42L73_MIOPC, miopc | ANLGOSFT);
		snd_soc_write(codec, CS42L73_HPAAVOL, hpavol | 0x80);
		snd_soc_write(codec, CS42L73_HPBAVOL, hpbvol | 0x80);
		snd_soc_write(codec, CS42L73_LOAAVOL, loavol | 0x80);
		snd_soc_write(codec, CS42L73_LOBAVOL, lobvol | 0x80);

		/* Powerdown all ports, inputs and outputs */
		snd_soc_write(codec, CS42L73_PWRCTL3,
			PDN_THMS | PDN_SPKLO | PDN_EAR |
			PDN_SPK | PDN_LO | PDN_HP);

		if (headset_connected == HEADSET_WITH_MIC)
			snd_soc_write(codec, CS42L73_PWRCTL2,
				PDN_MIC1_BIAS | PDN_VSP | PDN_ASP_SDOUT |
				PDN_ASP_SDIN | PDN_XSP_SDOUT | PDN_XSP_SDIN);
		else
			snd_soc_write(codec, CS42L73_PWRCTL2,
				PDN_MIC1_BIAS | PDN_MIC2_BIAS |
				PDN_VSP | PDN_ASP_SDOUT | PDN_ASP_SDIN |
				PDN_XSP_SDOUT | PDN_XSP_SDIN);

		snd_soc_write(codec, CS42L73_PWRCTL1,
			PDN_ADCB | PDN_ADCA | PDN_DMICB | PDN_DMICA | PDN);

		snd_soc_write(codec, CS42L73_MIOPC, miopc & ~ANLGOSFT);

		if (!hpamute_enabled)
			snd_soc_write(codec, CS42L73_HPAAVOL, hpavol & ~HPMUTE);

		if (!hpbmute_enabled)
			snd_soc_write(codec, CS42L73_HPBAVOL, hpbvol & ~HPMUTE);

		if (!loamute_enabled)
			snd_soc_write(codec, CS42L73_LOAAVOL, loavol & ~LOMUTE);

		if (!lobmute_enabled)
			snd_soc_write(codec, CS42L73_LOBAVOL, lobvol & ~LOMUTE);

		msleep(20);
		snd_soc_write(codec, CS42L73_DMMCC, dmmcc | MCLKDIS);

		if (!dsp_requested_clock)
			/* Set AUDIO_CLOCK_REQ_GPIO to low */
			gpio_set_value(AUDIO_CLOCK_REQ_GPIO, 0);

		codec->bias_level = level;
		break;
	}

	return 0;
}

/*
	cs42l73_set_tristate()
		Tristate xSP SDOUT
*/
static int cs42l73_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_codec *codec = dai->codec;
	int id = dai->id;

	u8 sp =  snd_soc_read(codec,  CS42L73_SPC(id)) & 0x7F;

	return snd_soc_write (codec,  CS42L73_SPC(id), sp | (tristate << 7));
}

/*
	cs42l73_shutdown()
*/
static void cs42l73_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int id = dai->id;
	struct cs42l73_priv *priv =
		(struct cs42l73_priv *)codec->private_data;
	priv->config[id].srate = 0;
	cs42l73_update_asrc(codec, id, 0);
}

static struct snd_pcm_hw_constraint_list constraints_12_24 = {
	.count  = ARRAY_SIZE(cs42l73_asrc_rates),
	.list   = cs42l73_asrc_rates,
};

/**
 * cs42l73_pcm_startup()
 * @substream:
 * @dai:
 *
 * Setup constraint list for the rates supported by
 * the codec. These depend on the MCLK when the codec
 * is master. Add 12 and 24 KHz for now.
 *
 * Returns 0 for success.
 */
static int cs42l73_pcm_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE,
		&constraints_12_24);
	return 0;
}

/* SNDRV_PCM_RATE_KNOT -> 12000, 24000 Hz, limit with constraint list */
#define CS42L73_RATES (SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT)

#define CS42L73_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops cs42l73_ops = {
	.startup = cs42l73_pcm_startup,
	.hw_params = cs42l73_pcm_hw_params,
	.set_fmt = cs42l73_set_dai_fmt,
	.set_sysclk = cs42l73_set_sysclk,
	.set_clkdiv = cs42l73_set_clkdiv,
	.set_tristate = cs42l73_set_tristate,
	.shutdown   = cs42l73_shutdown,
};

struct snd_soc_dai cs42l73_dai[] = {
	{
	 .name = "CS42L73 XSP",
	 .id = CS42L73_XSP,
	 .playback = {
		      .stream_name = "XSP Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = CS42L73_RATES,
		      .formats = CS42L73_FORMATS,},
	 .capture = {
		     .stream_name = "XSP Capture",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = CS42L73_RATES,
		     .formats = CS42L73_FORMATS,},
	 .ops = &cs42l73_ops,
	 },
	{
	 .name = "CS42L73 ASP",
	 .id = CS42L73_ASP,
	 .playback = {
		      .stream_name = "ASP Playback",
		      .channels_min = 2,
		      .channels_max = 2,
		      .rates = CS42L73_RATES,
		      .formats = CS42L73_FORMATS,},
	 .capture = {
		     .stream_name = "ASP Capture",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = CS42L73_RATES,
		     .formats = CS42L73_FORMATS,},
	 .ops = &cs42l73_ops,
	 .symmetric_rates = 1,
	 },
	{
	 .name = "CS42L73 VSP",
	 .id = CS42L73_VSP,
	 .playback = {
		      .stream_name = "VSP Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = CS42L73_RATES,
		      .formats = CS42L73_FORMATS,},
	 .capture = {
		     .stream_name = "VSP Capture",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = CS42L73_RATES,
		     .formats = CS42L73_FORMATS,},
	 .ops = &cs42l73_ops,
	 .symmetric_rates = 1,
	 }
};

EXPORT_SYMBOL_GPL(cs42l73_dai);

#ifdef CONFIG_PM
static int cs42l73_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct cs42l73_priv *priv = (struct cs42l73_priv *)codec->private_data;
	u8 pwrctl2 = snd_soc_read(codec, CS42L73_PWRCTL2);
	u8 asp_disabled = (pwrctl2 & PDN_ASP_SDIN) >> 2;
	u8 vsp_disabled = (pwrctl2 & PDN_VSP) >> 4;
	u8 analog_input_active = codec->aai_active;

	if (!asp_disabled || !vsp_disabled || analog_input_active) {
		dev_dbg(codec->dev, " asp_disabled = %d, vsp_disabled = %d"
				"codec analog input active =%d\n",
				asp_disabled, vsp_disabled,
				analog_input_active);
		return 0;
	}

	cs42l73_set_bias_level(codec, SND_SOC_BIAS_OFF);

	/* Set CS42L73_RESETn to low */
	gpio_set_value(CS42L73_RESET_GPIO, 1);
	mdelay(3);
	gpio_set_value(CS42L73_RESET_GPIO, 0);
	mdelay(3);

	regulator_disable(priv->vwlan1);
	msleep(100);

	codec_suspended = 1;

	return 0;
}

static int cs42l73_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int i;
	u8 *cache = codec->reg_cache;
	struct cs42l73_priv *priv = (struct cs42l73_priv *)codec->private_data;
	u8 pwrctl2 = snd_soc_read(codec, CS42L73_PWRCTL2);
	u8 asp_disabled = (pwrctl2 & PDN_ASP_SDIN) >> 2;
	u8 vsp_disabled = (pwrctl2 & PDN_VSP) >> 4;
	u8 analog_input_active = codec->aai_active;

	if (!asp_disabled || !vsp_disabled || analog_input_active) {
		dev_dbg(codec->dev, " asp_disabled = %d, vsp_disabled = %d"
				"codec analog input active =%d\n",
				asp_disabled, vsp_disabled,
				analog_input_active);
		return 0;
	}

	regulator_enable(priv->vwlan1);
	msleep(100);

	/* Set CS42L73_RESETn to high */
	gpio_set_value(CS42L73_RESET_GPIO, 0);
	mdelay(3);
	gpio_set_value(CS42L73_RESET_GPIO, 1);
	mdelay(3);

	/* Sync reg_cache with the hardware */
	for (i = CS42L73_PWRCTL1; i < ARRAY_SIZE(cs42l73_reg); i++)
		snd_soc_write(codec, i, cache[i]);

	cs42l73_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	cs42l73_set_bias_level(codec, codec->suspend_bias_level);

	codec_suspended = 0;

	return 0;
}
#else
#define cs42l73_suspend NULL
#define cs42l73_resume NULL
#endif

static int cs42l73_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	if (cs42l73_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = cs42l73_codec;
	codec = cs42l73_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		return ret;
	}

	snd_soc_add_controls(codec, cs42l73_snd_controls,
			     ARRAY_SIZE(cs42l73_snd_controls));

	cs42l73_add_widgets(codec);

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "omap3cdb42l73: failed to register card\n");
	}

	return ret;
}

/* power down chip */
static int cs42l73_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_cs42l73 = {
	.probe = cs42l73_probe,
	.remove = cs42l73_remove,
	.suspend = cs42l73_suspend,
	.resume = cs42l73_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_cs42l73);

/* CS42L73 INT irq handler */
static irqreturn_t cs42l73_int_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct cs42l73_priv *priv = (struct cs42l73_priv *)codec->private_data;

	schedule_work(&priv->work);

	return IRQ_HANDLED;
}

static int key_pressed;
void cs42l73_clear_key_pressed(void)
{
	struct cs42l73_priv *priv =
		(struct cs42l73_priv *) cs42l73_codec->private_data;
	if (key_pressed) {
		input_report_key(priv->input_dev, key_pressed, 0);
		key_pressed = 0;
	}
}

static void mic1_bias_work(struct work_struct *work)
{
	u8 pwrctl2;

	/* Turn MIC1 BIAS ON */
	pwrctl2 = snd_soc_read(cs42l73_codec, CS42L73_PWRCTL2);
	snd_soc_write(cs42l73_codec, CS42L73_PWRCTL2,
		pwrctl2 & ~PDN_MIC1_BIAS);
}

/* gpio work */
static void cs42l73_gpio_work(struct work_struct *work)
{
	struct cs42l73_priv *priv;
	struct snd_soc_codec *codec;
	int ret;
	unsigned short prev_adc_value = 1000, adc_value = 900;
	u8 pwrctl2;
	int count = 20; /* The key should be recognized within 20*5=100 ms */

	priv = container_of(work, struct cs42l73_priv, work);

	if (headset_connected == HEADSET_WITH_MIC) {
		codec = &priv->codec;
		ret = snd_soc_read(codec, CS42L73_IS1);
		if (ret < 0)
			dev_err(codec->dev, "Reading Device IS1 failed\n");
		else if (has_vol_controls) {

			if (!(ret&MIC2_SDET)) {
				if (key_pressed == KEY_PLAYCD) {
					/* Schedule delayed work to turn
					 * MIC1 BIAS back ON */
					cancel_delayed_work_sync(
						&priv->mic1_bias_work);
					schedule_delayed_work(
						&priv->mic1_bias_work,
						msecs_to_jiffies(200));
				}

				cs42l73_clear_key_pressed();

			} else {
				while ((prev_adc_value >= 450)
					&& ((prev_adc_value - adc_value) > 3)
					&& count--) {
					/* If ADC read gives KEY_PLAYCD (< 450),
					 * exit immediately */
					mdelay(5);
					prev_adc_value = adc_value;
					adc_value = read_mic_adc(cpcap);
				}
				if (prev_adc_value < 450) {
					input_report_key(priv->input_dev,
						KEY_PLAYCD, 1);
					key_pressed = KEY_PLAYCD;

					/* Turn MIC1 BIAS OFF so that we
					 * don't get inadvertent
					 * interrupts */
					cancel_delayed_work_sync(
						&priv->mic1_bias_work);
					pwrctl2 = snd_soc_read(codec,
						CS42L73_PWRCTL2);
					snd_soc_write(codec, CS42L73_PWRCTL2,
						pwrctl2 | PDN_MIC1_BIAS);
				}

				if ((count >= 0)
					&& (prev_adc_value > adc_value)) {
					if (prev_adc_value < 530) {
						input_report_key(
							priv->input_dev,
							KEY_VOLUMEDOWN, 1);
						key_pressed = KEY_VOLUMEDOWN;
					} else if (prev_adc_value < 600) {
						input_report_key(
							priv->input_dev,
							KEY_VOLUMEUP, 1);
						key_pressed = KEY_VOLUMEUP;
					}
				}
			}
		} else {
			cs42l73_clear_key_pressed();
			input_report_key(priv->input_dev, KEY_PLAYCD,
				ret&MIC2_SDET);
		}
	} else {
		printk(KERN_INFO "Headset not yet detected\n");
		cs42l73_clear_key_pressed();
		input_report_key(priv->input_dev, KEY_PLAYCD, 0);
	}
}

static irqreturn_t hs_key_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct cs42l73_priv *priv = (struct cs42l73_priv *)codec->private_data;

	input_report_key(priv->input_dev, KEY_PLAYCD, 1);
	input_report_key(priv->input_dev, KEY_PLAYCD, 0);

	return IRQ_HANDLED;
}

static __devinit int cs42l73_register(struct cs42l73_priv *priv)
{
	int ret, i;
	unsigned int devid = 0;

	struct snd_soc_codec *codec = &priv->codec;

	if (cs42l73_codec) {
		dev_err(codec->dev, "Another CS42L73 is registered\n");
		return -EINVAL;
	}

	/* Set AUDIO_CLOCK_REQ_GPIO to high */
	if (gpio_request(AUDIO_CLOCK_REQ_GPIO, "Audio Clock Req")) {
		printk(KERN_INFO "CS42L73: Audio Clock Req gpio %d request failed\n",
			AUDIO_CLOCK_REQ_GPIO);
		return -1;
	}
	gpio_direction_output(AUDIO_CLOCK_REQ_GPIO, 1);

	if (gpio_request(CS42L73_RESET_GPIO, "CS42L73_RESETn")) {
		printk(KERN_INFO "CS42L73: gpio %d request failed\n",
			CS42L73_RESET_GPIO);
		return -1;
	}

	priv->vwlan1 = regulator_get(NULL, "vwlan1");
	if (IS_ERR(priv->vwlan1)) {
		if (priv->vwlan1)
			regulator_put(priv->vwlan1);
		goto err_regulator_vwlan1;
	}
	regulator_enable(priv->vwlan1);
	msleep(100);

	/* Set CS42L73_RESETn to high */
	gpio_direction_output(CS42L73_RESET_GPIO, 0);
	mdelay(3);
	gpio_set_value(CS42L73_RESET_GPIO, 1);
	mdelay(3);
/*	gpio_free(CS42L73_RESET_GPIO);*/

	/* The headset key can be pressed either through the CS42L73 CODEC or
	 * through a GPIO. The two will not be left enabled at the same time
	 * and should be mutually exclusive. But they will trigger the same
	 * interrupt handler function and will be treated the same. However,
	 * only the key press detected through the CODEC will also be capable
	 * of detecting volume keys as well.
	 */
	if (gpio_request(CS42L73_INT_B_GPIO, "CS42L73_INT_B")) {
		printk(KERN_INFO "CS42L73: gpio %d request failed\n",
			CS42L73_INT_B_GPIO);
		goto err_regulator_vwlan1;
	}

	ret = request_irq(gpio_to_irq(CS42L73_INT_B_GPIO), cs42l73_int_handler,
		IRQF_TRIGGER_FALLING, "cs42l73_int_b", codec);
	if (ret) {
		dev_err(codec->dev,
			"request_irq CS42L73_INT_B gpio %d -> irq %d failed\n",
			CS42L73_INT_B_GPIO, gpio_to_irq(CS42L73_INT_B_GPIO));
		gpio_free(CS42L73_INT_B_GPIO);
		goto err_regulator_vwlan1;
	}
	disable_irq(gpio_to_irq(CS42L73_INT_B_GPIO));
	codec_key_detect_irq_enabled = 0;
	INIT_WORK(&priv->work, cs42l73_gpio_work);

	/* Set HS_KEY_ENABLE_GPIO to low */
	if (gpio_request(HS_KEY_ENABLE_GPIO, "HS_KEY_ENABLE")) {
		printk(KERN_INFO "CS42L73: HS_KEY_ENABLE gpio %d request failed\n",
			HS_KEY_ENABLE_GPIO);
		goto err_regulator_vwlan1;
	}
	gpio_direction_output(HS_KEY_ENABLE_GPIO, 0);

	if (gpio_request(HS_KEY_DETECT_GPIO, "HS_KEY_DETECT")) {
		printk(KERN_INFO "CS42L73: HS_KEY_DETECT gpio %d request failed\n",
			HS_KEY_DETECT_GPIO);
		goto err_regulator_vwlan1;
	}

	ret = request_irq(gpio_to_irq(HS_KEY_DETECT_GPIO), hs_key_handler,
		IRQF_TRIGGER_RISING, "HS_KEY_DETECT", codec);
	if (ret) {
		dev_err(codec->dev,
			"request_irq HS_KEY_DETECT gpio %d -> irq %d failed\n",
			CS42L73_INT_B_GPIO, gpio_to_irq(HS_KEY_DETECT_GPIO));
		gpio_free(HS_KEY_DETECT_GPIO);
		goto err_regulator_vwlan1;
	}
	disable_irq(gpio_to_irq(HS_KEY_DETECT_GPIO));
	hs_key_detect_irq_enabled = 0;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->private_data = priv;
	codec->name = "CS42L73";
	codec->owner = THIS_MODULE;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = cs42l73_set_bias_level;
	codec->dai = cs42l73_dai;
	codec->num_dai = ARRAY_SIZE(cs42l73_dai);
	codec->reg_cache_size = CS42L73_CACHEREGNUM;
	codec->reg_cache = &priv->reg_cache;
	codec->read = cs42l73_read;
	codec->write = cs42l73_write;
	codec->aai_active = 0; /* Analog audio interface active? */

	memcpy(codec->reg_cache, cs42l73_reg, sizeof(cs42l73_reg));

	cs42l73_dai[0].dev = codec->dev;
	cs42l73_dai[1].dev = codec->dev;
	cs42l73_dai[2].dev = codec->dev;

	cs42l73_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	cs42l73_codec = codec;

	/* initialize codec */
	ret = snd_soc_read(codec, CS42L73_DEVID_AB);

	if (ret < 0) {
		dev_err(codec->dev, "Reading Device ID AB failed\n");
		goto err;
	}
	devid = (ret & 0xFF) << 12;

	ret = snd_soc_read(codec, CS42L73_DEVID_CD);

	if (ret < 0) {
		dev_err(codec->dev, "Reading Device ID CD failed\n");
		goto err;
	}

	devid |= (ret & 0xFF) << 4;

	ret = snd_soc_read(codec, CS42L73_DEVID_E);

	if (ret < 0) {
		dev_err(codec->dev, "Reading Device ID E failed\n");
		goto err;
	}

	devid |= (ret & 0xF0) >> 4;

	if (devid != CS42L73_DEVID) {
		dev_err(codec->dev,
			"CS42L73 Device ID (%X). Expected %X\n",
			devid, CS42L73_DEVID);
		goto err;
	}

	ret = snd_soc_read(codec, CS42L73_REVID);
	if (ret < 0) {
		dev_err(codec->dev, "Get Revision ID failed\n");
		goto err;
	}

	dev_info(codec->dev,
		 "Cirrus Logic CS42L73, Revision: %02X\n", ret & 0xFF);

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_dais(cs42l73_dai, ARRAY_SIZE(cs42l73_dai));
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		goto err_codec;
	}

	priv->mclksel = CS42L73_CLKID_MCLK1;	/* MCLK1 as master clk */
	priv->mclk = 0;


	/*
	   Set defaults.
	   RESET needed.
	   TODO:
	   Add platfrom data for a RESET gpio
	   in the i2c board setup for a platform
	   (OMAP, MMP2)

	 */

	for (i = CS42L73_PWRCTL1; i < CS42L73_IS1; i++)
	    snd_soc_write(codec, i,  cs42l73_reg[i]);

	/* Powerdown all ports, inputs and outputs */
	snd_soc_write(codec, CS42L73_PWRCTL3,
		      PDN_THMS | PDN_SPKLO | PDN_EAR |
		      PDN_SPK | PDN_LO | PDN_HP);

	if (headset_connected == HEADSET_WITH_MIC)
		snd_soc_write(codec, CS42L73_PWRCTL2,
			PDN_MIC1_BIAS | PDN_VSP | PDN_ASP_SDOUT |
			PDN_ASP_SDIN | PDN_XSP_SDOUT | PDN_XSP_SDIN);
	else
		snd_soc_write(codec, CS42L73_PWRCTL2,
			PDN_MIC1_BIAS | PDN_MIC2_BIAS |
			PDN_VSP | PDN_ASP_SDOUT | PDN_ASP_SDIN |
			PDN_XSP_SDOUT | PDN_XSP_SDIN);

	snd_soc_write(codec, CS42L73_PWRCTL1,
		      PDN_ADCB | PDN_ADCA | PDN_DMICB | PDN_DMICA |
		      PDN);

	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL) {
		dev_err(codec->dev, "can't allocate input device\n");
		ret = -ENOMEM;
		goto err_codec;
	}

	set_bit(EV_KEY, priv->input_dev->evbit);
	set_bit(KEY_PLAYCD, priv->input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, priv->input_dev->keybit);
	set_bit(KEY_VOLUMEUP, priv->input_dev->keybit);

	priv->input_dev->name = "hs-key";

	ret = input_register_device(priv->input_dev);
	if (ret < 0) {
		dev_err(codec->dev, "could not register input device.\n");
		goto err_input;
	}

	wake_lock_init(&priv->offmode_lock, WAKE_LOCK_OFFMODE,
		"CS42L73 OFFMODE LOCK");
	INIT_DELAYED_WORK(&priv->mic1_bias_work, mic1_bias_work);

	return 0;

err_input:
	input_unregister_device(priv->input_dev);
	input_free_device(priv->input_dev);
err_codec:
	snd_soc_unregister_codec(codec);
err:
	if (priv->vwlan1)
		regulator_put(priv->vwlan1);
	kfree(priv);
err_regulator_vwlan1:
	return ret;
}

static __devexit void cs42l73_unregister(struct cs42l73_priv *priv)
{
	wake_lock_destroy(&priv->offmode_lock);
	input_unregister_device(priv->input_dev);
	input_free_device(priv->input_dev);
	cs42l73_set_bias_level(&priv->codec, SND_SOC_BIAS_OFF);
	snd_soc_unregister_dais(cs42l73_dai, ARRAY_SIZE(cs42l73_dai));
	snd_soc_unregister_codec(&priv->codec);
	kfree(priv);
	cs42l73_codec = NULL;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static __devinit int cs42l73_i2c_probe(struct i2c_client *i2c,
				       const struct i2c_device_id *id)
{
	struct cs42l73_priv *priv;
	struct snd_soc_codec *codec;

	priv = kzalloc(sizeof(struct cs42l73_priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	codec = &priv->codec;
	codec->hw_write = (hw_write_t) i2c_master_send;

	i2c_set_clientdata(i2c, priv);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;

	return cs42l73_register(priv);
}

static __devexit int cs42l73_i2c_remove(struct i2c_client *client)
{
	struct cs42l73_priv *priv = i2c_get_clientdata(client);
	cs42l73_unregister(priv);
	return 0;
}

static const struct i2c_device_id cs42l73_i2c_id[] = {
	{"cs42l73", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs42l73_i2c_id);

static struct i2c_driver cs42l73_i2c_driver = {
	.driver = {
		   .name = "CS42L73",
		   .owner = THIS_MODULE,
		   },
	.probe = cs42l73_i2c_probe,
	.remove = __devexit_p(cs42l73_i2c_remove),
	.id_table = cs42l73_i2c_id,
};
#endif

static int __init cs42l73_modinit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	return i2c_add_driver(&cs42l73_i2c_driver);
#else
	return 0;
#endif

}

module_init(cs42l73_modinit);

static void __exit cs42l73_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&cs42l73_i2c_driver);
#endif
}

module_exit(cs42l73_exit);

MODULE_DESCRIPTION("ASoC CS42L73 driver");
MODULE_AUTHOR("Georgi Vlaev, Nucleus Systems Ltd, <office@nucleusys.com>");
MODULE_LICENSE("GPL");
