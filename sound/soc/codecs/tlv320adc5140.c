// SPDX-License-Identifier: GPL-2.0
// TLV320ADC5140 Sound driver
// Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tlv320adc5140.h"

#define TLV320ADC5140_REG_DEBUG 1

#define ADC5140_RESET	BIT(0)

#define ADC5140_BCLKINV_BIT	BIT(2)
#define ADC5140_BCLK_FSYNC_MASTER	BIT(7)
#define ADC5140_I2S_MODE_BIT	BIT(6)
#define ADC5140_LEFT_JUST_BIT	BIT(7)

#define ADC5140_20_BIT_WORD	BIT(4)
#define ADC5140_24_BIT_WORD	BIT(5)
#define ADC5140_32_BIT_WORD	(BIT(4) | BIT(5))
#define ADC5140_WORD_LEN_MSK	0x30

static const struct reg_default adc5140_reg_defaults[] = {
	{ADC5140_PAGE_SELECT, 0x00},
};

static const struct regmap_range_cfg adc5140_ranges[] = {
	{
		.range_min = 0,
		.range_max = 12 * 128,
		.selector_reg = ADC5140_PAGE_SELECT,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

static const struct regmap_config adc5140_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_defaults = adc5140_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(adc5140_reg_defaults),
	.cache_type = REGCACHE_FLAT,
	.ranges = adc5140_ranges,
	.num_ranges = ARRAY_SIZE(adc5140_ranges),
	.max_register = 12 * 128,
};


static const char * const mic_select_text[] = {
	"2.5 kOhm", "10 kOhm", "20 kOhm"
};

static const DECLARE_TLV_DB_SCALE(adc_fgain_tlv, 0, 50, 0);

static SOC_ENUM_SINGLE_DECL(mic1p_p_enum, ADC5140_CH1_CFG0, 2,
	mic_select_text);
static SOC_ENUM_SINGLE_DECL(mic2p_p_enum, ADC5140_CH2_CFG0, 2,
	mic_select_text);
static SOC_ENUM_SINGLE_DECL(mic3p_p_enum, ADC5140_CH3_CFG0, 2,
	mic_select_text);
static SOC_ENUM_SINGLE_DECL(mic4p_p_enum, ADC5140_CH4_CFG0, 2,
	mic_select_text);
static SOC_ENUM_SINGLE_DECL(mic1m_p_enum, ADC5140_CH1_CFG0, 2,
	mic_select_text);
static SOC_ENUM_SINGLE_DECL(mic2m_p_enum, ADC5140_CH2_CFG0, 2,
	mic_select_text);
static SOC_ENUM_SINGLE_DECL(mic3m_p_enum, ADC5140_CH3_CFG0, 2,
	mic_select_text);
static SOC_ENUM_SINGLE_DECL(mic4m_p_enum, ADC5140_CH4_CFG0, 2,
	mic_select_text);

static const struct snd_kcontrol_new p_term_mic1p =
	SOC_DAPM_ENUM("MIC1P P-Terminal", mic1p_p_enum);
static const struct snd_kcontrol_new p_term_mic2p =
	SOC_DAPM_ENUM("MIC2P P-Terminal", mic2p_p_enum);
static const struct snd_kcontrol_new p_term_mic3p =
	SOC_DAPM_ENUM("MIC3P P-Terminal", mic3p_p_enum);
static const struct snd_kcontrol_new p_term_mic4p =
	SOC_DAPM_ENUM("MIC4P P-Terminal", mic4p_p_enum);
static const struct snd_kcontrol_new p_term_mic1m =
	SOC_DAPM_ENUM("MIC1M M-Terminal", mic1m_p_enum);
static const struct snd_kcontrol_new p_term_mic2m =
	SOC_DAPM_ENUM("MIC2M M-Terminal", mic2m_p_enum);
static const struct snd_kcontrol_new p_term_mic3m =
	SOC_DAPM_ENUM("MIC3M M-Terminal", mic3m_p_enum);
static const struct snd_kcontrol_new p_term_mic4m =
	SOC_DAPM_ENUM("MIC4M M-Terminal", mic4m_p_enum);

static const struct snd_soc_dapm_widget adc5140_dapm_widgets[] = {
	/* Inputs */
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC1P"),
	SND_SOC_DAPM_INPUT("MIC1M"),
	SND_SOC_DAPM_INPUT("MIC2P"),
	SND_SOC_DAPM_INPUT("MIC2M"),
	SND_SOC_DAPM_INPUT("MIC3P"),
	SND_SOC_DAPM_INPUT("MIC3M"),
	SND_SOC_DAPM_INPUT("MIC4P"),
	SND_SOC_DAPM_INPUT("MIC4M"),

	/* Input Selection to MIC_PGA */
	SND_SOC_DAPM_MUX("MIC1P P-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic1p),
	SND_SOC_DAPM_MUX("MIC2P P-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic2p),
	SND_SOC_DAPM_MUX("MIC3P P-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic3p),
	SND_SOC_DAPM_MUX("MIC4P P-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic4p),

	SND_SOC_DAPM_MUX("MIC1M M-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic1m),
	SND_SOC_DAPM_MUX("MIC2M M-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic2m),
	SND_SOC_DAPM_MUX("MIC3M M-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic3m),
	SND_SOC_DAPM_MUX("MIC4M M-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic4m),

	/* Enabling & Disabling MIC Gain Ctl */
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH1", ADC5140_CH1_CFG1,
			 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH2", ADC5140_CH2_CFG1,
			 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH3", ADC5140_CH3_CFG1,
			 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH4", ADC5140_CH4_CFG1,
			 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH5", ADC5140_CH5_CFG1,
			 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH6", ADC5140_CH6_CFG1,
			 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH7", ADC5140_CH7_CFG1,
			 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH8", ADC5140_CH8_CFG1,
			 7, 1, NULL, 0),

	SND_SOC_DAPM_ADC("ADC", "Capture", 0, 0, 0),

	SND_SOC_DAPM_AIF_OUT("AIF OUT", "Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route adc5140_audio_map[] = {
	/* Mic input */
	{"MIC1P P-Terminal", "2.5 kOhm", "MIC1P"},
	{"MIC1P P-Terminal", "10 kOhm", "MIC1P"},
	{"MIC1P P-Terminal", "20 kOhm", "MIC1P"},

	{"MIC2P P-Terminal", "2.5 kOhm", "MIC2P"},
	{"MIC2P P-Terminal", "10 kOhm", "MIC2P"},
	{"MIC2P P-Terminal", "20 kOhm", "MIC2P"},

	{"MIC3P P-Terminal", "2.5 kOhm", "MIC3P"},
	{"MIC3P P-Terminal", "10 kOhm", "MIC3P"},
	{"MIC3P P-Terminal", "20 kOhm", "MIC3P"},

	{"MIC4P P-Terminal", "2.5 kOhm", "MIC4P"},
	{"MIC4P P-Terminal", "10 kOhm", "MIC4P"},
	{"MIC4P P-Terminal", "20 kOhm", "MIC4P"},

	{"MIC1M M-Terminal", "2.5 kOhm", "MIC1M"},
	{"MIC1M M-Terminal", "10 kOhm", "MIC1M"},
	{"MIC1M M-Terminal", "20 kOhm", "MIC1M"},

	{"MIC2M M-Terminal", "2.5 kOhm", "MIC2M"},
	{"MIC2M M-Terminal", "10 kOhm", "MIC2M"},
	{"MIC2M M-Terminal", "20 kOhm", "MIC2M"},

	{"MIC3M M-Terminal", "2.5 kOhm", "MIC3M"},
	{"MIC3M M-Terminal", "10 kOhm", "MIC3M"},
	{"MIC3M M-Terminal", "20 kOhm", "MIC3M"},

	{"MIC4M M-Terminal", "2.5 kOhm", "MIC4M"},
	{"MIC4M M-Terminal", "10 kOhm", "MIC4M"},
	{"MIC4M M-Terminal", "20 kOhm", "MIC4M"},

	{"MIC_GAIN_CTL_CH1", NULL, "MIC1P P-Terminal"},
	{"MIC_GAIN_CTL_CH3", NULL, "MIC2P P-Terminal"},
	{"MIC_GAIN_CTL_CH5", NULL, "MIC3P P-Terminal"},
	{"MIC_GAIN_CTL_CH7", NULL, "MIC4P P-Terminal"},
	{"MIC_GAIN_CTL_CH2", NULL, "MIC1M M-Terminal"},
	{"MIC_GAIN_CTL_CH4", NULL, "MIC2M M-Terminal"},
	{"MIC_GAIN_CTL_CH6", NULL, "MIC3M M-Terminal"},
	{"MIC_GAIN_CTL_CH8", NULL, "MIC4M M-Terminal"},

	{"ADC", NULL, "MIC_GAIN_CTL_CH1"},
	{"ADC", NULL, "MIC_GAIN_CTL_CH2"},
	{"ADC", NULL, "MIC_GAIN_CTL_CH3"},
	{"ADC", NULL, "MIC_GAIN_CTL_CH4"},
	{"ADC", NULL, "MIC_GAIN_CTL_CH5"},
	{"ADC", NULL, "MIC_GAIN_CTL_CH6"},
	{"ADC", NULL, "MIC_GAIN_CTL_CH7"},
	{"ADC", NULL, "MIC_GAIN_CTL_CH8"},

	{"AIF OUT", NULL, "ADC"},

	{"MIC1", NULL, "MIC1P P-Terminal"},
	{"MIC1", NULL, "MIC1M M-Terminal"},
};

static const struct snd_kcontrol_new adc5140_snd_controls[] = {
	SOC_SINGLE_TLV("Digital Channel 1 Capture Volume", ADC5140_CH1_CFG2,
			4, 4, 1, adc_fgain_tlv),
	SOC_SINGLE_TLV("Digital Channel 2 Capture Volume", ADC5140_CH2_CFG2,
			4, 4, 1, adc_fgain_tlv),
	SOC_SINGLE_TLV("Digital Channel 3 Capture Volume", ADC5140_CH3_CFG2,
			4, 4, 1, adc_fgain_tlv),
	SOC_SINGLE_TLV("Digital Channel 4 Capture Volume", ADC5140_CH4_CFG2,
			4, 4, 1, adc_fgain_tlv),
	SOC_SINGLE_TLV("Digital Channel 5 Capture Volume", ADC5140_CH5_CFG2,
			4, 4, 1, adc_fgain_tlv),
	SOC_SINGLE_TLV("Digital Channel 6 Capture Volume", ADC5140_CH6_CFG2,
			4, 4, 1, adc_fgain_tlv),
	SOC_SINGLE_TLV("Digital Channel 7 Capture Volume", ADC5140_CH7_CFG2,
			4, 4, 1, adc_fgain_tlv),
	SOC_SINGLE_TLV("Digital Channel 8 Capture Volume", ADC5140_CH8_CFG2,
			4, 4, 1, adc_fgain_tlv),
};

static int adc5140_reset(struct adc5140_priv *adc5140)
{
	int ret = 0;

	if (adc5140->gpio_reset) {
		gpiod_direction_output(adc5140->gpio_reset, 0);
		msleep(10);
		gpiod_direction_output(adc5140->gpio_reset, 1);
	} else {
		ret = regmap_write(adc5140->regmap, ADC5140_SW_RESET,
				   ADC5140_RESET);
	}

	return ret;
}

static int adc5140_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	u8 data = 0;

	dev_dbg(component->dev, "## %s: width %d rate %d\n",
		__func__, params_width(params),
		params_rate(params));

	switch (params_width(params)) {
	case 16:
		break;
	case 20:
		data = ADC5140_20_BIT_WORD;
		break;
	case 24:
		data = ADC5140_24_BIT_WORD;
		break;
	case 32:
		data = ADC5140_32_BIT_WORD;
		break;
	default:
		dev_err(component->dev, "%s: Unsupported width %d\n",
			__func__, params_width(params));
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, ADC5140_ASI_CFG0,
			    ADC5140_WORD_LEN_MSK, data);

	return 0;
}

static int adc5140_set_dai_fmt(struct snd_soc_dai *codec_dai,
			       unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 iface_reg1 = 0;
	u8 iface_reg2 = 0;

	dev_dbg(component->dev, "## %s: fmt = 0x%x\n", __func__, fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface_reg2 |= ADC5140_BCLK_FSYNC_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
	case SND_SOC_DAIFMT_CBM_CFS:
		break;
	default:
		dev_err(component->dev, "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* signal polarity */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface_reg1 |= ADC5140_BCLKINV_BIT;
		break;
	default:
		dev_err(component->dev, "Invalid DAI clock signal polarity\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface_reg1 |= ADC5140_I2S_MODE_BIT;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg1 |= ADC5140_LEFT_JUST_BIT;
		break;
	default:
		dev_err(component->dev, "Invalid DAI interface format\n");
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, ADC5140_ASI_CFG0,
				      ADC5140_BCLKINV_BIT,
				      iface_reg1);
	snd_soc_component_update_bits(component, ADC5140_MST_CFG0,
			    ADC5140_BCLK_FSYNC_MASTER,
			    iface_reg2);

	return 0;
}

static int adc5140_dac_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_component *component = codec_dai->component;

	dev_dbg(component->dev, "## %s: mute = 0x%x\n", __func__, mute);

	return 0;
}

static int adc5140_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct adc5140_priv *adc5140 = snd_soc_component_get_drvdata(component);
	int i;

	dev_dbg(component->dev, "## %s: clk_id = %d, freq = %d, dir = %d\n",
		__func__, clk_id, freq, dir);

	for (i = 1; i < 8; i++)
		if (freq / i <= 20000000)
			break;

	if (freq/i > 20000000) {
		dev_err(adc5140->dev, "%s: Too high mclk frequency %u\n",
				__func__, freq);
		return -EINVAL;
	}

	adc5140->p_div = i;
	adc5140->sysclk = freq;

	return 0;
}

static const struct snd_soc_dai_ops adc5140_dai_ops = {
	.hw_params	= adc5140_hw_params,
	.set_sysclk	= adc5140_set_dai_sysclk,
	.set_fmt	= adc5140_set_dai_fmt,
	.digital_mute	= adc5140_dac_mute,
};

struct adc5140_hack_table {
	uint8_t reg;
	uint8_t value;
};

static struct adc5140_hack_table tlv320adc5140_dev0[] = {
	{ ADC5140_ASI_CFG0, 0x35 },
	{ ADC5140_ASI_CFG1, 0xa0 },
	{ ADC5140_ASI_CH1, 0x00 },
	{ ADC5140_ASI_CH2, 0x01 },
	{ ADC5140_ASI_CH3, 0x02 },
	{ ADC5140_ASI_CH4, 0x03 },
	{ ADC5140_CH1_CFG1, 0x00 },
	{ ADC5140_CH2_CFG1, 0x00 },
	{ ADC5140_CH3_CFG1, 0x00 },
	{ ADC5140_CH4_CFG1, 0x00 },
	{ ADC5140_CH1_CFG2, 0xff },
	{ ADC5140_CH2_CFG2, 0xff },
	{ ADC5140_CH3_CFG2, 0xff },
	{ ADC5140_CH4_CFG2, 0xff },
	{ ADC5140_DRE_CFG0, 0xcb },
	{ ADC5140_CH1_CFG0, 0x01 },
	{ ADC5140_CH2_CFG0, 0x01 },
	{ ADC5140_CH3_CFG0, 0x01 },
	{ ADC5140_CH4_CFG0, 0x01 },
	{ ADC5140_IN_CH_EN, 0xf0 },
	{ ADC5140_ASI_OUT_CH_EN, 0xf0 },
};

static struct adc5140_hack_table tlv320adc5140_dev1[] = {
	{ ADC5140_ASI_CFG0, 0x35 },
	{ ADC5140_ASI_CFG1, 0x80 },
	{ ADC5140_CH1_CFG1, 0x00 },
	{ ADC5140_CH2_CFG1, 0x00 },
	{ ADC5140_CH3_CFG1, 0x00 },
	{ ADC5140_CH4_CFG1, 0x00 },
	{ ADC5140_CH1_CFG2, 0xff },
	{ ADC5140_CH2_CFG2, 0xff },
	{ ADC5140_CH3_CFG2, 0xff },
	{ ADC5140_CH4_CFG2, 0xff },
	{ ADC5140_DRE_CFG0, 0xcb },
	{ ADC5140_ASI_CH1, 0x04 },
	{ ADC5140_ASI_CH2, 0x05 },
	{ ADC5140_ASI_CH3, 0x06 },
	{ ADC5140_ASI_CH4, 0x07 },
	{ ADC5140_CH1_CFG0, 0x01 },
	{ ADC5140_CH2_CFG0, 0x01 },
	{ ADC5140_CH3_CFG0, 0x01 },
	{ ADC5140_CH4_CFG0, 0x01 },
	{ ADC5140_IN_CH_EN, 0xf0 },
	{ ADC5140_ASI_OUT_CH_EN, 0xf0 },
};

static int adc5410_init_hack(struct adc5140_priv *adc5140)
{
	struct adc5140_hack_table *regs_to_write;
	unsigned i, reg_count;
	int ret;

	ret = regmap_write(adc5140->regmap, ADC5140_SLEEP_CFG, 0x81);
	if (ret)
		goto out;

	ret = adc5140_reset(adc5140);
	if (ret)
		goto out;

	ret = regmap_write(adc5140->regmap, ADC5140_SLEEP_CFG, 0x81);
	if (ret)
		goto out;

	msleep(10);

	if (adc5140->device_number) {
		regs_to_write = &tlv320adc5140_dev1[0];
		reg_count = sizeof(tlv320adc5140_dev1) / sizeof(tlv320adc5140_dev1[0]);
	} else {
		regs_to_write = &tlv320adc5140_dev0[0];
		reg_count = sizeof(tlv320adc5140_dev0) / sizeof(tlv320adc5140_dev0[0]);
	}

	for (i = 0; i < reg_count; i++) {
		ret = regmap_write(adc5140->regmap, regs_to_write[i].reg,
				   regs_to_write[i].value);
		if (ret)
			goto out;
	}

out:
	return ret;
}

static int adc5140_codec_probe(struct snd_soc_codec *codec)
{
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	int ret;

	dev_dbg(adc5140->dev, "## %s\n", __func__);

	regcache_cache_only(adc5140->regmap, true);
	regcache_mark_dirty(adc5140->regmap);

	adc5410_init_hack(adc5140);

	snd_soc_add_codec_controls(codec, adc5140_snd_controls,
			ARRAY_SIZE(adc5140_snd_controls));

	ret = snd_soc_dapm_new_controls(dapm, adc5140_dapm_widgets,
			ARRAY_SIZE(adc5140_dapm_widgets));
	if (ret)
		return ret;

	ret = snd_soc_dapm_add_routes(dapm, adc5140_audio_map,
				      ARRAY_SIZE(adc5140_audio_map));
	if (ret)
		return ret;

	return 0;
}

static int adc5140_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_dbg(adc5140->dev, "## %s:Enter\n", __func__);
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		ret = regmap_write(adc5140->regmap, ADC5140_PWR_CFG, 0xe0);
		break;
	case SND_SOC_BIAS_OFF:
		ret = regmap_write(adc5140->regmap, ADC5140_PWR_CFG, 0x00);
		break;
	}

	return ret;
}

static int adc5140_codec_remove(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "## %s:Enter\n", __func__);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_driver_adc5140 = {
	.probe			= adc5140_codec_probe,
	.remove			= adc5140_codec_remove,
	.controls		= adc5140_snd_controls,
	.num_controls		= ARRAY_SIZE(adc5140_snd_controls),
	.dapm_widgets		= adc5140_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(adc5140_dapm_widgets),
	.dapm_routes		= adc5140_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(adc5140_audio_map),
	.set_bias_level 	= adc5140_set_bias_level,
};

static struct snd_soc_dai_driver adc5140_dai_driver[] = {
	{
		.name = "tlv320adc5140-codec",
		.capture = {
			.stream_name	 = "Capture",
			.channels_min	 = 2,
			.channels_max	 = 8,
			.rates		 = ADC5140_RATES,
			.formats	 = ADC5140_FORMATS,
		},
		.ops = &adc5140_dai_ops,
		.symmetric_rates = 1,
	}
};

static const struct of_device_id tlv320adc5140_of_match[] = {
	{ .compatible = "ti,tlv320adc5140" },
	{},
};
MODULE_DEVICE_TABLE(of, tlv320adc5140_of_match);

static int adc5140_parse_node(struct adc5140_priv *adc5140)
{
	int ret;

	adc5140->gpio_reset = devm_gpiod_get_optional(adc5140->dev,
						   "reset", GPIOD_OUT_LOW);
	if (IS_ERR(adc5140->gpio_reset))
		dev_info(adc5140->dev, "Reset GPIO not defined\n");

	/* This is a hack for multiple devices */
	ret = device_property_read_u32_array(adc5140->dev, "dev_num",
					     &adc5140->device_number, 1);
	if (ret) {
		dev_err(adc5140->dev, "dev_num DT property missing\n");
		return ret;
	}

	return ret;
}

static int adc5140_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{

	struct adc5140_priv *adc5140;
	int ret;

	dev_dbg(&i2c->dev, "## %s: %s codec_type = %d\n", __func__,
		id->name, (int)id->driver_data);

	adc5140 = devm_kzalloc(&i2c->dev, sizeof(*adc5140), GFP_KERNEL);
	if (!adc5140)
		return -ENOMEM;

	adc5140->regmap = devm_regmap_init_i2c(i2c, &adc5140_i2c_regmap);
	if (IS_ERR(adc5140->regmap)) {
		ret = PTR_ERR(adc5140->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}
	adc5140->dev = &i2c->dev;

	i2c_set_clientdata(i2c, adc5140);

	adc5140_parse_node(adc5140);

#ifdef TLV320ADC5140_REG_DEBUG
	/* For Debug only */
	tlv320adc5140_init_debug(adc5140);
#endif

	return snd_soc_register_codec(&i2c->dev,
				&soc_codec_driver_adc5140,
				adc5140_dai_driver, 1);
}

static const struct i2c_device_id adc5140_i2c_id[] = {
	{ "tlv320adc5140", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, adc5140_i2c_id);

static struct i2c_driver adc5140_i2c_driver = {
	.driver = {
		.name	= "tlv320adc5140-codec",
		.of_match_table = of_match_ptr(tlv320adc5140_of_match),
	},
	.probe		= adc5140_i2c_probe,
	.id_table	= adc5140_i2c_id,
};
module_i2c_driver(adc5140_i2c_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_DESCRIPTION("ASoC TLV320ADC5140 CODEC Driver");
MODULE_LICENSE("GPL v2");
