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

#define ADC5140_RESET	BIT(0)

#define ADC5140_BCLKINV_BIT	BIT(2)
#define ADC5140_BCLK_FSYNC_MASTER	BIT(7)
#define ADC5140_I2S_MODE_BIT	BIT(6)
#define ADC5140_LEFT_JUST_BIT	BIT(7)

#define ADC5140_16_BIT_WORD	0x0
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

/* Digital Volume control. From -100 to 27 dB in 0.5 dB steps */
static DECLARE_TLV_DB_SCALE(dig_vol_tlv, -10000, 50, 0);

/* ADC gain. From 0 to 42 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(adc_tlv, 0, 100, 0);

static const char * const resistor_text[] = {
	"2.5 kOhm", "10 kOhm", "20 kOhm"
};
/* Left mixer pins */
static SOC_ENUM_SINGLE_DECL(in1_resistor_enum, ADC5140_CH1_CFG0, 2, resistor_text);
static SOC_ENUM_SINGLE_DECL(in2_resistor_enum, ADC5140_CH2_CFG0, 2, resistor_text);
static SOC_ENUM_SINGLE_DECL(in3_resistor_enum, ADC5140_CH3_CFG0, 2, resistor_text);
static SOC_ENUM_SINGLE_DECL(in4_resistor_enum, ADC5140_CH4_CFG0, 2, resistor_text);

static const struct snd_kcontrol_new in1_resistor_controls[] = {
	SOC_DAPM_ENUM("CH1 Resistor Select", in1_resistor_enum),
};
static const struct snd_kcontrol_new in2_resistor_controls[] = {
	SOC_DAPM_ENUM("CH2 Resistor Select", in2_resistor_enum),
};
static const struct snd_kcontrol_new in3_resistor_controls[] = {
	SOC_DAPM_ENUM("CH3 Resistor Select", in3_resistor_enum),
};
static const struct snd_kcontrol_new in4_resistor_controls[] = {
	SOC_DAPM_ENUM("CH4 Resistor Select", in4_resistor_enum),
};

/* Analog/Digital Selection */
static const char *adc5140_mic_sel_text[] = {"Analog", "Line In", "Digital"};
static const char *adc5140_analog_sel_text[] = {"Analog", "Line In"};

static SOC_ENUM_SINGLE_DECL(adc5140_mic1p_enum,
			    ADC5140_CH1_CFG0, 5,
			    adc5140_mic_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic1p_control =
SOC_DAPM_ENUM("MIC1P MUX", adc5140_mic1p_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic1_analog_enum,
			    ADC5140_CH1_CFG0, 7,
			    adc5140_analog_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic1_analog_control =
SOC_DAPM_ENUM("MIC1 Analog MUX", adc5140_mic1_analog_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic1m_enum,
			    ADC5140_CH1_CFG0, 5,
			    adc5140_mic_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic1m_control =
SOC_DAPM_ENUM("MIC1M MUX", adc5140_mic1m_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic2p_enum,
			    ADC5140_CH2_CFG0, 5,
			    adc5140_mic_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic2p_control =
SOC_DAPM_ENUM("MIC2P MUX", adc5140_mic2p_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic2_analog_enum,
			    ADC5140_CH2_CFG0, 7,
			    adc5140_analog_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic2_analog_control =
SOC_DAPM_ENUM("MIC2 Analog MUX", adc5140_mic2_analog_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic2m_enum,
			    ADC5140_CH2_CFG0, 5,
			    adc5140_mic_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic2m_control =
SOC_DAPM_ENUM("MIC2M MUX", adc5140_mic2m_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic3p_enum,
			    ADC5140_CH3_CFG0, 5,
			    adc5140_mic_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic3p_control =
SOC_DAPM_ENUM("MIC3P MUX", adc5140_mic3p_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic3_analog_enum,
			    ADC5140_CH3_CFG0, 7,
			    adc5140_analog_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic3_analog_control =
SOC_DAPM_ENUM("MIC3 Analog MUX", adc5140_mic3_analog_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic3m_enum,
			    ADC5140_CH3_CFG0, 5,
			    adc5140_mic_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic3m_control =
SOC_DAPM_ENUM("MIC3M MUX", adc5140_mic3m_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic4p_enum,
			    ADC5140_CH4_CFG0, 5,
			    adc5140_mic_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic4p_control =
SOC_DAPM_ENUM("MIC4P MUX", adc5140_mic4p_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic4_analog_enum,
			    ADC5140_CH4_CFG0, 7,
			    adc5140_analog_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic4_analog_control =
SOC_DAPM_ENUM("MIC4 Analog MUX", adc5140_mic4_analog_enum);

static SOC_ENUM_SINGLE_DECL(adc5140_mic4m_enum,
			    ADC5140_CH4_CFG0, 5,
			    adc5140_mic_sel_text);

static const struct snd_kcontrol_new adc5140_dapm_mic4m_control =
SOC_DAPM_ENUM("MIC4M MUX", adc5140_mic4m_enum);

/* Output Mixer */
static const struct snd_kcontrol_new adc5140_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("Digital CH1 Switch", 0, 0, 0, 0),
	SOC_DAPM_SINGLE("Digital CH2 Switch", 0, 0, 0, 0),
	SOC_DAPM_SINGLE("Digital CH3 Switch", 0, 0, 0, 0),
	SOC_DAPM_SINGLE("Digital CH4 Switch", 0, 0, 0, 0),
};

static const struct snd_soc_dapm_widget adc5140_dapm_widgets[] = {

	/* Analog Differential Inputs */
	SND_SOC_DAPM_INPUT("MIC1P"),
	SND_SOC_DAPM_INPUT("MIC1M"),
	SND_SOC_DAPM_INPUT("MIC2P"),
	SND_SOC_DAPM_INPUT("MIC2M"),
	SND_SOC_DAPM_INPUT("MIC3P"),
	SND_SOC_DAPM_INPUT("MIC3M"),
	SND_SOC_DAPM_INPUT("MIC4P"),
	SND_SOC_DAPM_INPUT("MIC4M"),

	SND_SOC_DAPM_OUTPUT("CH1_OUT"),
	SND_SOC_DAPM_OUTPUT("CH2_OUT"),
	SND_SOC_DAPM_OUTPUT("CH3_OUT"),
	SND_SOC_DAPM_OUTPUT("CH4_OUT"),
	SND_SOC_DAPM_OUTPUT("CH5_OUT"),
	SND_SOC_DAPM_OUTPUT("CH6_OUT"),
	SND_SOC_DAPM_OUTPUT("CH7_OUT"),
	SND_SOC_DAPM_OUTPUT("CH8_OUT"),

	SND_SOC_DAPM_MIXER("Output Mixer", SND_SOC_NOPM, 0, 0,
		&adc5140_output_mixer_controls[0],
		ARRAY_SIZE(adc5140_output_mixer_controls)),

	/* Input Selection to MIC_PGA */
	SND_SOC_DAPM_MUX("MIC1P Input Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic1p_control),
	SND_SOC_DAPM_MUX("MIC2P Input Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic2p_control),
	SND_SOC_DAPM_MUX("MIC3P Input Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic3p_control),
	SND_SOC_DAPM_MUX("MIC4P Input Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic4p_control),

	/* Input Selection to MIC_PGA */
	SND_SOC_DAPM_MUX("MIC1 Analog Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic1_analog_control),
	SND_SOC_DAPM_MUX("MIC2 Analog Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic2_analog_control),
	SND_SOC_DAPM_MUX("MIC3 Analog Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic3_analog_control),
	SND_SOC_DAPM_MUX("MIC4 Analog Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic4_analog_control),

	SND_SOC_DAPM_MUX("MIC1M Input Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic1m_control),
	SND_SOC_DAPM_MUX("MIC2M Input Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic2m_control),
	SND_SOC_DAPM_MUX("MIC3M Input Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic3m_control),
	SND_SOC_DAPM_MUX("MIC4M Input Mux", SND_SOC_NOPM, 0, 0,
			 &adc5140_dapm_mic4m_control),

	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH3", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL_CH4", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_ADC("CH1_ADC", "CH1 Capture", ADC5140_IN_CH_EN, 7, 0),
	SND_SOC_DAPM_ADC("CH2_ADC", "CH2 Capture", ADC5140_IN_CH_EN, 6, 0),
	SND_SOC_DAPM_ADC("CH3_ADC", "CH3 Capture", ADC5140_IN_CH_EN, 5, 0),
	SND_SOC_DAPM_ADC("CH4_ADC", "CH4 Capture", ADC5140_IN_CH_EN, 4, 0),

	SND_SOC_DAPM_MUX("IN1 Analog Mic Resistor", SND_SOC_NOPM, 0, 0,
			in1_resistor_controls),
	SND_SOC_DAPM_MUX("IN2 Analog Mic Resistor", SND_SOC_NOPM, 0, 0,
			in2_resistor_controls),
	SND_SOC_DAPM_MUX("IN3 Analog Mic Resistor", SND_SOC_NOPM, 0, 0,
			in3_resistor_controls),
	SND_SOC_DAPM_MUX("IN4 Analog Mic Resistor", SND_SOC_NOPM, 0, 0,
			in4_resistor_controls),
};

static const struct snd_soc_dapm_route adc5140_audio_map[] = {
	/* Outputs */
	{"CH1_OUT", NULL, "Output Mixer"},
	{"CH2_OUT", NULL, "Output Mixer"},
	{"CH3_OUT", NULL, "Output Mixer"},
	{"CH4_OUT", NULL, "Output Mixer"},

	/* Mic input */
	{"CH1_ADC", NULL, "MIC_GAIN_CTL_CH1"},
	{"CH2_ADC", NULL, "MIC_GAIN_CTL_CH2"},
	{"CH3_ADC", NULL, "MIC_GAIN_CTL_CH3"},
	{"CH4_ADC", NULL, "MIC_GAIN_CTL_CH4"},

	{"MIC_GAIN_CTL_CH1", NULL, "IN1 Analog Mic Resistor"},
	{"MIC_GAIN_CTL_CH1", NULL, "IN1 Analog Mic Resistor"},
	{"MIC_GAIN_CTL_CH2", NULL, "IN2 Analog Mic Resistor"},
	{"MIC_GAIN_CTL_CH2", NULL, "IN2 Analog Mic Resistor"},
	{"MIC_GAIN_CTL_CH3", NULL, "IN3 Analog Mic Resistor"},
	{"MIC_GAIN_CTL_CH3", NULL, "IN3 Analog Mic Resistor"},
	{"MIC_GAIN_CTL_CH4", NULL, "IN4 Analog Mic Resistor"},
	{"MIC_GAIN_CTL_CH4", NULL, "IN4 Analog Mic Resistor"},

	{"IN1 Analog Mic Resistor", "2.5 kOhm", "MIC1P Input Mux"},
	{"IN1 Analog Mic Resistor", "10 kOhm", "MIC1P Input Mux"},
	{"IN1 Analog Mic Resistor", "20 kOhm", "MIC1P Input Mux"},

	{"IN1 Analog Mic Resistor", "2.5 kOhm", "MIC1M Input Mux"},
	{"IN1 Analog Mic Resistor", "10 kOhm", "MIC1M Input Mux"},
	{"IN1 Analog Mic Resistor", "20 kOhm", "MIC1M Input Mux"},

	{"IN2 Analog Mic Resistor", "2.5 kOhm", "MIC2P Input Mux"},
	{"IN2 Analog Mic Resistor", "10 kOhm", "MIC2P Input Mux"},
	{"IN2 Analog Mic Resistor", "20 kOhm", "MIC2P Input Mux"},

	{"IN2 Analog Mic Resistor", "2.5 kOhm", "MIC2M Input Mux"},
	{"IN2 Analog Mic Resistor", "10 kOhm", "MIC2M Input Mux"},
	{"IN2 Analog Mic Resistor", "20 kOhm", "MIC2M Input Mux"},

	{"IN3 Analog Mic Resistor", "2.5 kOhm", "MIC3P Input Mux"},
	{"IN3 Analog Mic Resistor", "10 kOhm", "MIC3P Input Mux"},
	{"IN3 Analog Mic Resistor", "20 kOhm", "MIC3P Input Mux"},

	{"IN3 Analog Mic Resistor", "2.5 kOhm", "MIC3M Input Mux"},
	{"IN3 Analog Mic Resistor", "10 kOhm", "MIC3M Input Mux"},
	{"IN3 Analog Mic Resistor", "20 kOhm", "MIC3M Input Mux"},

	{"IN4 Analog Mic Resistor", "2.5 kOhm", "MIC4P Input Mux"},
	{"IN4 Analog Mic Resistor", "10 kOhm", "MIC4P Input Mux"},
	{"IN4 Analog Mic Resistor", "20 kOhm", "MIC4P Input Mux"},

	{"IN4 Analog Mic Resistor", "2.5 kOhm", "MIC4M Input Mux"},
	{"IN4 Analog Mic Resistor", "10 kOhm", "MIC4M Input Mux"},
	{"IN4 Analog Mic Resistor", "20 kOhm", "MIC4M Input Mux"},

	{"MIC1 Analog Mux", "Line In", "MIC1P"},
	{"MIC2 Analog Mux", "Line In", "MIC2P"},
	{"MIC3 Analog Mux", "Line In", "MIC3P"},
	{"MIC4 Analog Mux", "Line In", "MIC4P"},

	{"MIC1P Input Mux", "Analog", "MIC1P"},
	{"MIC1M Input Mux", "Analog", "MIC1M"},
	{"MIC2P Input Mux", "Analog", "MIC2P"},
	{"MIC2M Input Mux", "Analog", "MIC2M"},
	{"MIC3P Input Mux", "Analog", "MIC3P"},
	{"MIC3M Input Mux", "Analog", "MIC3M"},
	{"MIC4P Input Mux", "Analog", "MIC4P"},
	{"MIC4M Input Mux", "Analog", "MIC4M"},
};

static const struct snd_kcontrol_new adc5140_snd_controls[] = {
	SOC_SINGLE_TLV("Analog CH1 Mic Gain", ADC5140_CH1_CFG1, 0, 0x52, 0,
			adc_tlv),
	SOC_SINGLE_TLV("Analog CH2 Mic Gain", ADC5140_CH1_CFG2, 0, 0x52, 0,
			adc_tlv),
	SOC_SINGLE_TLV("Analog CH3 Mic Gain", ADC5140_CH1_CFG3, 0, 0x52, 0,
			adc_tlv),
	SOC_SINGLE_TLV("Analog CH4 Mic Gain", ADC5140_CH1_CFG4, 0, 0x52, 0,
			adc_tlv),

	SOC_SINGLE_TLV("Digital CH1 Out Volume", ADC5140_CH1_CFG2,
			0, 0xff, 0, dig_vol_tlv),
	SOC_SINGLE_TLV("Digital CH2 Out Volume", ADC5140_CH2_CFG2,
			0, 0xff, 0, dig_vol_tlv),
	SOC_SINGLE_TLV("Digital CH3 Out Volume", ADC5140_CH3_CFG2,
			0, 0xff, 0, dig_vol_tlv),
	SOC_SINGLE_TLV("Digital CH4 Out Volume", ADC5140_CH4_CFG2,
			0, 0xff, 0, dig_vol_tlv),
	SOC_SINGLE_TLV("Digital CH5 Out Volume", ADC5140_CH5_CFG2,
			0, 0xff, 0, dig_vol_tlv),
	SOC_SINGLE_TLV("Digital CH6 Out Volume", ADC5140_CH6_CFG2,
			0, 0xff, 0, dig_vol_tlv),
	SOC_SINGLE_TLV("Digital CH7 Out Volume", ADC5140_CH7_CFG2,
			0, 0xff, 0, dig_vol_tlv),
	SOC_SINGLE_TLV("Digital CH8 Out Volume", ADC5140_CH8_CFG2,
			0, 0xff, 0, dig_vol_tlv),
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
		data = ADC5140_16_BIT_WORD;
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

static int adc5140_mute(struct snd_soc_dai *codec_dai, int mute)
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
	.digital_mute	= adc5140_mute,
};

static int adc5410_init_hack(struct adc5140_priv *adc5140)
{
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

	ret = regmap_write(adc5140->regmap, ADC5140_ASI_CFG0, 0x30);
	if (ret)
		goto out;

	ret = regmap_write(adc5140->regmap, ADC5140_CH1_CFG0, 0x4c);
	if (ret)
		goto out;

	ret = regmap_write(adc5140->regmap, ADC5140_CH2_CFG0, 0x4c);
	if (ret)
		goto out;

	ret = regmap_write(adc5140->regmap, ADC5140_CH3_CFG0, 0x4c);
	if (ret)
		goto out;

	ret = regmap_write(adc5140->regmap, ADC5140_CH4_CFG0, 0x4c);
	if (ret)
		goto out;

	ret = regmap_write(adc5140->regmap, ADC5140_IN_CH_EN, 0xf0);
	if (ret)
		goto out;

	ret = regmap_write(adc5140->regmap, ADC5140_ASI_OUT_CH_EN, 0xf0);
	if (ret)
		goto out;

	ret = regmap_write(adc5140->regmap, ADC5140_PWR_CFG, 0xe0);
	if (ret)
		goto out;

out:
	return ret;
}

static int adc5140_codec_probe(struct snd_soc_component *component)
{
	struct adc5140_priv *adc5140 = snd_soc_component_get_drvdata(component);

	dev_dbg(adc5140->dev, "## %s\n", __func__);

	regcache_cache_only(adc5140->regmap, true);
	regcache_mark_dirty(adc5140->regmap);

	return 0;
}

static int adc5140_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	struct adc5140_priv *adc5140 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(component->dev, "## %s: %d -> %d\n", __func__,
		snd_soc_component_get_bias_level(component), level);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		ret = regmap_write(adc5140->regmap, ADC5140_PWR_CFG, 0xe0);
		break;
	case SND_SOC_BIAS_OFF:
		ret = regmap_write(adc5140->regmap, ADC5140_PWR_CFG, 0x00);
		break;
	}

	return ret;
}

static void adc5140_codec_remove(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "## %s:Enter\n", __func__);
}

static const struct snd_soc_component_driver soc_codec_driver_adc5140 = {
	.probe			= adc5140_codec_probe,
	.remove			= adc5140_codec_remove,
	.set_bias_level		= adc5140_set_bias_level,
	.controls		= adc5140_snd_controls,
	.num_controls		= ARRAY_SIZE(adc5140_snd_controls),
	.dapm_widgets		= adc5140_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(adc5140_dapm_widgets),
	.dapm_routes		= adc5140_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(adc5140_audio_map),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
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

	adc5140->gpio_reset = devm_gpiod_get_optional(adc5140->dev,
						      "reset", GPIOD_OUT_LOW);
	if (IS_ERR(adc5140->gpio_reset))
		dev_info(&i2c->dev, "Reset GPIO not defined\n");


	adc5140->regmap = devm_regmap_init_i2c(i2c, &adc5140_i2c_regmap);
	if (IS_ERR(adc5140->regmap)) {
		ret = PTR_ERR(adc5140->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}
	adc5140->dev = &i2c->dev;

	i2c_set_clientdata(i2c, adc5140);

	/* For Debug only */
	tlv320adc5140_init_debug(adc5140);

	adc5410_init_hack(adc5140);

	return devm_snd_soc_register_component(&i2c->dev,
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
