/*
 * omap3cirrus.c  --  SoC audio for OMAP3 / Cirrus platform
 *
 * Author: Georgi Vlaev, Nucleus Systems <office@nucleusys.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>
#include <plat/control.h>
#include <plat/mux.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include <sound/cs42l73.h>

/* CDB42L73 Y1 (6.144 MHz) )oscillator =  MCLK1 */
#define CDB42L73_DEFAULT_MCLK		6144000
#define OMAP3_CIRRUS_DEFAULT_MCLK	26000000
#define CS42L73_FMT_I2S

struct omap3_sl_clk
{
	int rate;
	int clk_id;
	int clk_freq;
	int div;
};

/* OMAP35x Master -> CS42L73 slave clk table / 2ch, S16 */
struct omap3_sl_clk omap35xcdb42l73_sl_clk_16[] = {
	/* 96 Mhz */
	{ 44100, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 96000000, 68  },
	{ 22050, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 96000000, 136 },
	/* 83 Mhz PER_L4_ICLK -> McBSP_ICLK */
	{ 48000, OMAP_MCBSP_SYSCLK_CLK, 83000000, 54  },
	{ 32000, OMAP_MCBSP_SYSCLK_CLK, 83000000, 81  },
	{ 24000, OMAP_MCBSP_SYSCLK_CLK, 83000000, 108 },
	{ 16000, OMAP_MCBSP_SYSCLK_CLK, 83000000, 162 },
	{ 12000, OMAP_MCBSP_SYSCLK_CLK, 83000000, 216 }
};

/* AM37x Master -> CS42L73 slave clk table / 2ch, S16 */
struct omap3_sl_clk omap37xcdb42l73_sl_clk_16[] =
{
	/* 96 Mhz */
	{ 44100, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 96000000, 68  },
	{ 22050, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 96000000, 136 },
	{ 48000, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 96000000, 62  },
	{ 24000, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 96000000, 134 },
};

/* BeagleBoard/CDB42L73 capture/render paths */
/* Capture Sources */
enum {
        CS42L73_CS_OFF,
        CS42L73_CS_MIC1,        /* Mic 1 */
        CS42L73_CS_MIC2,        /* Mic 2 */
        CS42L73_CS_LINEIN,      /* Line In */
};

/* Playback Targets */
enum {
	CS42L73_PB_OFF,
	CS42L73_PB_HP,           /* Stereo Headphones */
	CS42L73_PB_LINEOUT,      /* Stereo Line Out */
	CS42L73_PB_SPEAKERPHONE, /* Stereo Speakerphone */
	CS42L73_PB_EAR,          /* Mono Ear Speaker */
	CS42L73_PB_LOOPBACK,     /* Analog Loopback Test */
	CS42L73_TCMD_MIC_HS_LOOPBACK, /*Mic-HS TCMD loopback fot ALT test*/
};

static int cs42l73_capture_source = CS42L73_CS_MIC1;
static int cs42l73_playback_target = CS42L73_PB_HP;
/*
    Capture source requires additional input selection of
    the input PGA mux...
*/
static void cs42l73_ext_control(struct snd_soc_codec *codec)
{
	u8 im1, pwrctl2, olmbmsdc;
/* Capture */
        switch (cs42l73_capture_source) {
        case CS42L73_CS_OFF:
			    codec->aai_active = 0;
                snd_soc_dapm_disable_pin(codec, "Ext Mic 1");
                snd_soc_dapm_disable_pin(codec, "Ext Mic 2");
                snd_soc_dapm_disable_pin(codec, "Line In");
                break;

        case CS42L73_CS_MIC1:
                snd_soc_dapm_enable_pin(codec, "Ext Mic 1");
                snd_soc_dapm_disable_pin(codec, "Ext Mic 2");
                snd_soc_dapm_disable_pin(codec, "Line In");
                break;

        case CS42L73_CS_MIC2:
			    codec->aai_active = 1;
                snd_soc_dapm_disable_pin(codec, "Ext Mic 1");
                snd_soc_dapm_enable_pin(codec, "Ext Mic 2");
                snd_soc_dapm_disable_pin(codec, "Line In");
                break;

        case CS42L73_CS_LINEIN:
			    codec->aai_active = 1;/*Analog audio IF active ?*/
                snd_soc_dapm_disable_pin(codec, "Ext Mic 1");
                snd_soc_dapm_disable_pin(codec, "Ext Mic 2");
				snd_soc_dapm_enable_pin(codec, "Line In");

                break;
        }

/* Playback */
        switch (cs42l73_playback_target) {
        case CS42L73_PB_OFF:
                snd_soc_dapm_disable_pin(codec, "Line Out");
                snd_soc_dapm_disable_pin(codec, "Headphone");
                snd_soc_dapm_disable_pin(codec, "Speakerphone Left");
                snd_soc_dapm_disable_pin(codec, "Speakerphone Right");
                snd_soc_dapm_disable_pin(codec, "Ear Speaker");
                break;

        case CS42L73_PB_HP:
                snd_soc_dapm_disable_pin(codec, "Line Out");
                snd_soc_dapm_enable_pin(codec, "Headphone");
                snd_soc_dapm_disable_pin(codec, "Speakerphone Left");
                snd_soc_dapm_disable_pin(codec, "Speakerphone Right");
                snd_soc_dapm_disable_pin(codec, "Ear Speaker");
                break;

        case CS42L73_PB_LINEOUT:
                snd_soc_dapm_enable_pin(codec, "Line Out");
                snd_soc_dapm_disable_pin(codec, "Headphone");
                snd_soc_dapm_disable_pin(codec, "Speakerphone Left");
                snd_soc_dapm_disable_pin(codec, "Speakerphone Right");
                snd_soc_dapm_disable_pin(codec, "Ear Speaker");
                break;

        case CS42L73_PB_SPEAKERPHONE:
                snd_soc_dapm_disable_pin(codec, "Line Out");
                snd_soc_dapm_disable_pin(codec, "Headphone");
                snd_soc_dapm_enable_pin(codec, "Speakerphone Left");
                snd_soc_dapm_enable_pin(codec, "Speakerphone Right");
                snd_soc_dapm_disable_pin(codec, "Ear Speaker");
                break;

        case CS42L73_PB_EAR:
                snd_soc_dapm_disable_pin(codec, "Line Out");
                snd_soc_dapm_disable_pin(codec, "Headphone");
                snd_soc_dapm_disable_pin(codec, "Speakerphone Left");
                snd_soc_dapm_disable_pin(codec, "Speakerphone Right");
                snd_soc_dapm_enable_pin(codec, "Ear Speaker");
                break;

	case CS42L73_PB_LOOPBACK:
		snd_soc_dapm_disable_pin(codec, "Line Out");
		snd_soc_dapm_enable_pin(codec, "Headphone");
		snd_soc_dapm_disable_pin(codec, "Speakerphone Left");
		snd_soc_dapm_disable_pin(codec, "Speakerphone Right");
		snd_soc_dapm_disable_pin(codec, "Ear Speaker");

		snd_soc_dapm_enable_pin(codec, "Line In");
		break;

	case CS42L73_TCMD_MIC_HS_LOOPBACK:
		snd_soc_dapm_disable_pin(codec, "Line Out");
		snd_soc_dapm_enable_pin(codec, "Headphone");
		snd_soc_dapm_disable_pin(codec, "Speakerphone Left");
		snd_soc_dapm_disable_pin(codec, "Speakerphone Right");
		snd_soc_dapm_disable_pin(codec, "Ear Speaker");
		snd_soc_dapm_enable_pin(codec, "Ext Mic 1");
		snd_soc_dapm_enable_pin(codec, "Ext Mic 2");
		break;
	}

	if (has_vol_controls) {
		/* Turn off MIC2_SDET interrupt */
		im1 = snd_soc_read(cs42l73_codec, CS42L73_IM1);
		snd_soc_write(cs42l73_codec, CS42L73_IM1, im1 & ~MIC2_SDET);

		if ((cs42l73_capture_source == CS42L73_CS_MIC2) &&
				(cs42l73_playback_target == CS42L73_PB_HP)) {
			/* If MIC2 and HP are ON, then in a call. Need to use
			   MIC2_BIAS voltages to put headset in mic mode */
			pwrctl2 = snd_soc_read(cs42l73_codec, CS42L73_PWRCTL2);
			snd_soc_write(cs42l73_codec, CS42L73_PWRCTL2,
				pwrctl2 | PDN_MIC1_BIAS | PDN_MIC2_BIAS);
			olmbmsdc = snd_soc_read(cs42l73_codec,
				CS42L73_OLMBMSDC);
			snd_soc_write(cs42l73_codec, CS42L73_OLMBMSDC,
				olmbmsdc | MIC_BIAS_CTRL);
			msleep(200);
			/* Leave MIC1 BIAS OFF */
			snd_soc_write(cs42l73_codec, CS42L73_PWRCTL2,
				(pwrctl2 | PDN_MIC1_BIAS) & ~PDN_MIC2_BIAS);
			msleep(100);
			snd_soc_write(cs42l73_codec, CS42L73_OLMBMSDC,
				olmbmsdc & ~MIC_BIAS_CTRL);
		} else {
			/* else, reset MIC2_BIAS voltages to put headset in
			   button mode */
			pwrctl2 = snd_soc_read(cs42l73_codec, CS42L73_PWRCTL2);
			snd_soc_write(cs42l73_codec, CS42L73_PWRCTL2,
				pwrctl2 | PDN_MIC2_BIAS);
			msleep(450);
			/* Turn MIC2 BIAS ON */
			snd_soc_write(cs42l73_codec, CS42L73_PWRCTL2,
				pwrctl2 & ~PDN_MIC2_BIAS);
			msleep(100);
			/* Turn MIC1 BIAS ON */
			snd_soc_write(cs42l73_codec, CS42L73_PWRCTL2,
				pwrctl2 & ~(PDN_MIC1_BIAS | PDN_MIC2_BIAS));
		}

		/* Turn on MIC2_SDET interrupt */
		snd_soc_write(cs42l73_codec, CS42L73_IM1, im1 | MIC2_SDET);
	}

        snd_soc_dapm_sync(codec);
}

static int cs42l73_get_capture_source(struct snd_kcontrol *kcontrol,
                        struct snd_ctl_elem_value *ucontrol)
{
        ucontrol->value.integer.value[0] = cs42l73_capture_source;
        return 0;
}

static int cs42l73_set_capture_source(struct snd_kcontrol *kcontrol,
                        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

        if (cs42l73_capture_source == ucontrol->value.integer.value[0])
                return 0;

        cs42l73_capture_source = ucontrol->value.integer.value[0];
        cs42l73_ext_control(codec);

        return 1;
}

static int cs42l73_get_playback_target(struct snd_kcontrol *kcontrol,
                        struct snd_ctl_elem_value *ucontrol)
{
        ucontrol->value.integer.value[0] = cs42l73_playback_target;
        return 0;
}

static int cs42l73_set_playback_target(struct snd_kcontrol *kcontrol,
                        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

        if (cs42l73_playback_target == ucontrol->value.integer.value[0])
                return 0;

        cs42l73_playback_target = ucontrol->value.integer.value[0];
        cs42l73_ext_control(codec);

        return 1;
}


static const char *capture_source[] =
        {"Off", "Mic1", "Mic2", "LineIn"};

static const char *playback_target[] =
        {"Off", "Headphone", "LineOut",
	"Speakerphone", "Ear Speaker", "Loopback", "TCMD_Mic_HS_Loopback" };

static const struct soc_enum bb_cs42l73_enum[] = {
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(capture_source), capture_source),
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(playback_target), playback_target),
};

static const struct snd_kcontrol_new bb_cs42l73_controls[] = {
        SOC_ENUM_EXT("Audio Source", bb_cs42l73_enum[0],
                     cs42l73_get_capture_source, cs42l73_set_capture_source),
        SOC_ENUM_EXT("Audio Target", bb_cs42l73_enum[1],
                     cs42l73_get_playback_target, cs42l73_set_playback_target),
};

static int omap3cdb42l73_set_sysclk_div(struct snd_soc_dai* cpu_dai,
	    struct snd_pcm_hw_params *params)
{
/*
    Select OMAP_MCBSP_SYSCLK_CLK for McBSP clock source,
    McBSPi_ICLK for the SRG divider source
*/
	int i, rate = 0, div = 0, ret = 0;
	rate = params_rate(params);

	for (i = 0; i < ARRAY_SIZE(omap37xcdb42l73_sl_clk_16); i++) {
		if (omap37xcdb42l73_sl_clk_16[i].rate == rate) {
			div = omap37xcdb42l73_sl_clk_16[i].clk_freq / rate / 2 /
				16;

			ret = snd_soc_dai_set_sysclk(cpu_dai,
				omap37xcdb42l73_sl_clk_16[i].clk_id,
				omap37xcdb42l73_sl_clk_16[i].clk_freq,
				SND_SOC_CLOCK_IN);

			if (ret < 0) {
				pr_err("can't set cpu system clock\n");
				return ret;
			}

			ret = snd_soc_dai_set_clkdiv(cpu_dai,
				    OMAP_MCBSP_CLKGDV, div);

			if (ret < 0) {
				pr_err("Cannot't set SRG clock divider\n");
				return ret;
			}

			return 0;
		}
	}

	pr_err("Unsupported rate %d\n", rate);
	return -EINVAL;
}


static int omap3cdb42l73_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int fmt;
	int ret;

	/* OMAP3 McBSP Master <=> CS42L73 Slave */
	fmt =	SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
	    | SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_err("can't set cpu DAI configuration\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, CS42L73_CLKID_MCLK1,
				OMAP3_CIRRUS_DEFAULT_MCLK, SND_SOC_CLOCK_IN);
        if (ret < 0) {
                pr_err("can't set codec clock\n");
                return ret;
        }

	/* Slave cpu_dai */
	if (fmt & SND_SOC_DAIFMT_CBS_CFS)
	    return omap3cdb42l73_set_sysclk_div(cpu_dai, params);

	return 0;
}

/* CDB42L73 widgets */
static const struct snd_soc_dapm_widget bb_cs42l73_dapm_widgets[] = {
        SND_SOC_DAPM_MIC("Ext Mic 1", NULL),
        SND_SOC_DAPM_MIC("Ext Mic 2", NULL),
        SND_SOC_DAPM_LINE("Line In", NULL),
        SND_SOC_DAPM_HP("Headphone", NULL),
        SND_SOC_DAPM_LINE("Line Out", NULL),
        SND_SOC_DAPM_SPK("Speakerphone Left", NULL),
        SND_SOC_DAPM_SPK("Speakerphone Right", NULL),
        SND_SOC_DAPM_SPK("Ear Speaker", NULL),
};

/* CDB42L73 Audio Map */
static const struct snd_soc_dapm_route bb_cs42l73_audio_map[] = {
        /* External Mics: MIC1, MIC2 with bias*/
        {"MIC1", NULL, "Ext Mic 1"},
        {"MIC2", NULL, "Ext Mic 2"},

        /* LINEINA, LINEINB <- Line In (L+R) */
        {"LINEINA", NULL, "Line In"},
        {"LINEINB", NULL, "Line In"},

        /* Line Out (L+R) -> LINEOUTA, LINEOUTB */
        {"Line Out", NULL, "LINEOUTA"},
        {"Line Out", NULL, "LINEOUTB"},

        /* Speakerphone -> SPKOUT (L), SPKLINEOUT (R)
	(R) - external amp CS35L0x  */
        {"Speakerphone Left", NULL, "SPKOUT"},
        {"Speakerphone Right", NULL, "SPKLINEOUT"},

        /* Headphone (L+R)->  HPOUTA, HPOUTB */
        {"Headphone", NULL, "HPOUTA"},
        {"Headphone", NULL, "HPOUTB"},

	/* Ear Speaker -> EAROUT */
        {"Ear Speaker", NULL, "EAROUT"},
};

static int beagle_cs42l73_init(struct snd_soc_codec *codec)
{
	int ret;

	/* Add BB/CDB42L73 specific controls */
	ret = snd_soc_add_controls(codec, bb_cs42l73_controls,
		ARRAY_SIZE(bb_cs42l73_controls));
	if (ret < 0)
		return ret;

	/* Add BB specific widgets */
	ret = snd_soc_dapm_new_controls(codec, bb_cs42l73_dapm_widgets,
		ARRAY_SIZE(bb_cs42l73_dapm_widgets));
	if (ret)
		return ret;

	/* Set up BB specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, bb_cs42l73_audio_map,
		ARRAY_SIZE(bb_cs42l73_audio_map));

	/* BB connected pins */
	snd_soc_dapm_enable_pin(codec, "Headphone");
	/* Disable Ext Mic 2 pin by default and enable when headset with
	   mic is connected */
	snd_soc_dapm_disable_pin(codec, "Ext Mic 2");

	/* CS42L73 not connected pins */
	snd_soc_dapm_nc_pin(codec, "DMICA");
	snd_soc_dapm_nc_pin(codec, "DMICB");
	snd_soc_dapm_nc_pin(codec, "Line Out");
	snd_soc_dapm_nc_pin(codec, "Ear Speaker");
	snd_soc_dapm_nc_pin(codec, "Speakerphone Left");
	snd_soc_dapm_nc_pin(codec, "Speakerphone Right");
	snd_soc_dapm_nc_pin(codec, "Line In");
	snd_soc_dapm_nc_pin(codec, "Ext Mic 1");

	return  snd_soc_dapm_sync(codec);
}

/* OMAP3 I2S */
static struct snd_soc_ops omap3cdb42l73_ops = {
	.hw_params = omap3cdb42l73_hw_params,
};

/*
 * DSP Codec DAI.
 * TODO:
 *	Register this DAI from the CS48L10 driver (make it ASoC compatible)
 */

/* Supported codec rates, The DSP->ASP link can handle 12 and 24KHz */
static u32 l10_l73_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000 };

static struct snd_pcm_hw_constraint_list constraints_12_24 = {
	.count  = ARRAY_SIZE(l10_l73_rates),
	.list   = l10_l73_rates,
};

/*
    l10_l73_dsp_startup()
	    Add rate constraint list.
 */
static int l10_l73_dsp_startup(struct snd_pcm_substream *substream)
{
	snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &constraints_12_24);
	return 0;
}

static int l10_l73_dsp_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int fmt;
	int ret;

	/* CS48L10 Master <=> CS42L73 Slave, I2S  */
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration\n");
		return ret;
	}

	/*
	    (!)Impportat: MCLK on our test board is MCLK1.
	    The end device is designed to use MCLK2, so this needs to
	    be changed.
	*/
	ret = snd_soc_dai_set_sysclk(codec_dai, CS42L73_CLKID_MCLK1,
				OMAP3_CIRRUS_DEFAULT_MCLK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops l10_l73_ops = {
	.startup = l10_l73_dsp_startup,
	.hw_params = l10_l73_dsp_hw_params,
};

/* SNDRV_PCM_RATE_KNOT -> 12000, 24000 Hz, add constraint on startup */
#define CS48L10_CS42L73_RATES  \
	(SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT)

static struct snd_soc_dai cs48l10_dai = {
	.name = "CS48L10",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = CS48L10_CS42L73_RATES,
		.formats = SNDRV_PCM_FMTBIT_MPEG | SNDRV_PCM_FMTBIT_S16_LE,
	}
};


/*
 * FM Digital DAI
 */
static int btfm_l73_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int fmt;
	int ret;

	/* BT, FM Digital Master <=> CS42L73 Slave, I2S  */
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, CS42L73_CLKID_MCLK1,
				OMAP3_CIRRUS_DEFAULT_MCLK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec clock\n");
		return ret;
	}

	/* Setup the FM DAI ?*/

	return 0;
}

/* Bluetooth, FM Digital ops{} */
static struct snd_soc_ops btfm_l73_ops = {
	.hw_params = btfm_l73_hw_params,
};


static struct snd_soc_dai fm_dai = {
	.name = "FM Digital",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000, /* <- check these values */
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
};

/*
 * Bluetooth Codec DAI.
 */
/* BT Master <=> codec Slave */
static int omap3_bt_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int fmt;
	int ret;

	/* BT, Master <=> CS42L73 Slave, PCM   */
	fmt =   SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_IF
				| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, CS42L73_CLKID_MCLK1,
				OMAP3_CIRRUS_DEFAULT_MCLK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec clock\n");
		return ret;
	}


	return 0;
}

static struct snd_soc_ops omap3_bt_ops = {
	.hw_params = omap3_bt_hw_params,
};

static struct snd_soc_dai bt_dai = {
	.name = "Bluetooth",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000, /* <- check these values */
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
};



/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link omap3cdb42l73_dai[] =
{
	{
/* PCM Audio Omap3 <=> CS42L73 XSP */
		.name = "CS42L73",
		.stream_name = "PCM Audio",
		.cpu_dai = &omap_mcbsp_dai[0],
		.codec_dai = &cs42l73_dai[CS42L73_XSP],
		.ops = &omap3cdb42l73_ops,
		.init = beagle_cs42l73_init,
	}, {
/* PCM Audio Omap3 <=> CS42L73 VSP */
		.name = "CS42L73",
		.stream_name = "PCM Audio",
		.cpu_dai = &omap_mcbsp_dai[1],
		.codec_dai = &cs42l73_dai[CS42L73_VSP],
		.ops = &omap3cdb42l73_ops,
	}, {
/* Compressed audio - CS48L10 DSP <=> CS42L73 ASP */
/* The DSP dai is a dummy interface here */
		.name = "CS48L10",
		.stream_name = "Compressed Audio",
		.cpu_dai = &cs48l10_dai,
		.codec_dai = &cs42l73_dai[CS42L73_ASP],
		.ops = &l10_l73_ops,
	}, {
/* PCM Audio Omap3 <=> Bluetooth Audio */
		.name = "Bluetooth",
		.stream_name = "PCM Audio",
		.cpu_dai =  &omap_mcbsp_dai[1],
		.codec_dai = &bt_dai,
		.ops = &omap3_bt_ops,
	}, {
/* Bluetooth Audio <=> CS42L73 VSP */
		.name = "Bluetooth",
		.stream_name = "Bluetooth",
		.cpu_dai = &bt_dai,
		.codec_dai = &cs42l73_dai[CS42L73_VSP],
		.ops = &omap3_bt_ops,
	}, {
/* FM Audio <=> CS42L73 ASP */
		.name = "FM Digital",
		.stream_name = "FM Digital",
		.cpu_dai =  &fm_dai,
		.codec_dai = &cs42l73_dai[CS42L73_ASP],
		.ops = &btfm_l73_ops,
	}
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_omap3cdb42l73 = {
	.name = "omap3cirrus",
	.platform = &omap_soc_platform,
	.dai_link = omap3cdb42l73_dai,
	.num_links = ARRAY_SIZE(omap3cdb42l73_dai),
};

/* Audio subsystem */
static struct snd_soc_device omap3cdb42l73_snd_devdata = {
	.card = &snd_soc_omap3cdb42l73,
	.codec_dev = &soc_codec_dev_cs42l73,
};

static struct platform_device *omap3cdb42l73_snd_device;

static int __init omap3cdb42l73_soc_init(void)
{
	int ret;

	pr_info("OMAP3 CDB42L73 SoC init\n");

	/* Register the CS48L10 DAI */
	ret = snd_soc_register_dai(&cs48l10_dai);
	if (ret)
		return ret;

	/* Register the Bluetooth DAI */
	ret = snd_soc_register_dai(&bt_dai);
	if (ret)
		return ret;

	/* Register the FM Digital DAI */
	ret = snd_soc_register_dai(&fm_dai);
	if (ret)
		return ret;

	omap3cdb42l73_snd_device = platform_device_alloc("soc-audio", -1);
	if (!omap3cdb42l73_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(omap3cdb42l73_snd_device, &omap3cdb42l73_snd_devdata);
	omap3cdb42l73_snd_devdata.dev = &omap3cdb42l73_snd_device->dev;
	*(unsigned int *)omap3cdb42l73_dai[0].cpu_dai->private_data =
		1; /* McBSP2 */
	*(unsigned int *)omap3cdb42l73_dai[1].cpu_dai->private_data =
		2; /* McBSP3 */

	ret = platform_device_add(omap3cdb42l73_snd_device);
	if (ret)
		goto err1;

	/* Allow CPCAP to detect HS after CODEC code fully initialized */
	codec_suspended = 0;

	return 0;

err1:
	pr_err("Unable to add platform device\n");
	platform_device_put(omap3cdb42l73_snd_device);

	return ret;
}

static void __exit omap3cdb42l73_soc_exit(void)
{
        snd_soc_unregister_dai(&cs48l10_dai);
	platform_device_unregister(omap3cdb42l73_snd_device);
}

module_init(omap3cdb42l73_soc_init);
module_exit(omap3cdb42l73_soc_exit);

MODULE_AUTHOR("Georgi Vlaev, Nucleus Systems <office@nucleusys.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3 / Cirrus");
MODULE_LICENSE("GPL");
