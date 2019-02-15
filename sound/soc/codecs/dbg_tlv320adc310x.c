// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * ALSA SoC TLV320ADC310X codec driver
 *
 * Author:      Dan Murphy <dmurphy@ti.com>
 * Copyright (C) 2017-2018 Texas Instruments, Inc.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tlv320adc310x.h"

/* The registers can be accessed via
 * cat /sys/class/i2c-adapter/i2c-2/2-00xx/registers
 * And written through echo for example
 * echo "RESET 0x00" > /sys/class/i2c-adapter/i2c-2/2-00xx/registers
 */
struct tlv320adc310x_reg {
	const char *name;
	uint16_t reg;
	int writeable;
} tlv320adc310x_regs[] = {
	{
	"PAGE_CONTROL", ADC310X_PAGE_SELECT, 1}, {
	"RESET", ADC310X_RESET, 1}, {
	"CLK_GEN_MUX", ADC310X_CLK_GEN_MUX, 1}, {
	"PLL_R_VAL", ADC310X_PLL_PR_VAL, 1}, {
	"PLL_J_VAL", ADC310X_PLL_J_VAL, 1}, {
	"PLL_D_MSB", ADC310X_PLL_D_VAL_MSB, 1}, {
	"PLL_D_LSB", ADC310X_PLL_D_VAL_LSB, 1}, {
	"NADC_CLK", ADC310X_NADC_CLK, 1}, {
	"MADC_CLK", ADC310X_MADC_CLK, 1}, {
	"AOSR", ADC310X_AOSR, 1}, {
	"ADC_IADC", ADC310X_ADC_IADC, 1}, {
	"DSP_DECI", ADC310X_DSP_DECI, 1}, {
	"CLK_OUT_MUX", ADC310X_CLK_OUT_MUX, 1}, {
	"CLK_OUT_MDIV", ADC310X_CLK_OUT_MDIV, 1}, {
	"INTF_CTRL_1", ADC310X_ADC_INTF_CTRL_1, 1}, {
	"DATA_SLOT_CH1", ADC310X_DATA_SLOT_OFF_CH_1, 1}, {
	"INTF_CTRL_2", ADC310X_ADC_INTF_CTRL_2, 1}, {
	"BLCK_N_DIV", ADC310X_BLCK_N_DIV, 1}, {
	"SEC_INTF_CTRL_1", ADC310X_SEC_INTF_CTRL_1, 1}, {
	"SEC_INTF_CTRL_2", ADC310X_SEC_INTF_CTRL_2, 1}, {
	"SEC_INTF_CTRL_3", ADC310X_SEC_INTF_CTRL_3, 1}, {
	"I2S_SYNC", ADC310X_I2S_SYNC, 1}, {
	"ADC_FLAG", ADC310X_ADC_FLAG, 0},	/* RO */
	{
	"DATA_SLOT_CH2", ADC310X_DATA_SLOT_OFF_CH_2, 1}, {
	"I2S_TDM_CTRL", ADC310X_I2S_TDM_CTRL, 1}, {
	"INT_FLAG1", ADC310X_INT_FLAG_1, 0}, {
	"INT_FLAG2", ADC310X_INT_FLAG_2, 0}, {
	"ADC_INT_FLAG1", ADC310X_ADC_INT_FLAG_1, 0}, {
	"ADC_INT_FLAG2", ADC310X_ADC_INT_FLAG_2, 0}, {
	"ADC_INT_1_CTRL", ADC310X_ADC_INT_1_CTRL, 1}, {
	"ADC_INT_2_CTRL", ADC310X_ADC_INT_2_CTRL, 1}, {
	"DMCLK_GPIO2", ADC310X_DMCLK_GPIO2, 1}, {
	"DMDIN_GPIO1", ADC310X_DMDIN_GPIO1, 1}, {
	"DOUT_CTRL", ADC310X_DOUT_CTRL, 1}, {
	"ADC_SYNC_CTRL_1", ADC310X_ADC_SYNC_CTRL_1, 1}, {
	"ADC_SYNC_CTRL_1", ADC310X_ADC_SYNC_CTRL_2, 1}, {
	"ADC_CIC_FILT_GN", ADC310X_ADC_CIC_FILT_GN, 1}, {
	"ADC_PROC_BLK", ADC310X_ADC_PROC_BLK, 1}, {
	"INST_MD_CTRL", ADC310X_INST_MD_CTRL, 1}, {
	"ADC_DIG", ADC310X_ADC_DIGITAL, 1}, {
	"VOL_CTRL", ADC310X_ADC_VOL_CTRL, 1}, {
	"VOL_L", ADC310X_ADC_VOL_L, 1}, {
	"VOL_R", ADC310X_ADC_VOL_R, 1}, {
	"PHASE_COMP_L", ADC310X_ADC_PHASE_COMP_L, 1}, {
	"DMIC_POL", ADC310X_DMIC_POL, 1}, {
	"GAIN_CTRL_1_L", ADC310X_AGC_GAIN_CTRL_L_1, 1}, {
	"GAIN_CTRL_2_L", ADC310X_AGC_GAIN_CTRL_L_2, 1}, {
	"MAX_GAIN_L", ADC310X_AGC_MAX_GAIN_L, 1}, {
	"ATTACK_TIME_L", ADC310X_AGC_ATTACK_TIME_L, 1}, {
	"DECAY_TIME_L", ADC310X_AGC_DECAY_TIME_L, 1}, {
	"NOISE_DEBOUNCE_L", ADC310X_AGC_NOISE_DEBNCE_L, 1}, {
	"SIG_DEBOUNCE_L", ADC310X_AGC_SIGNAL_DEBNCE_L, 1}, {
	"GAIN_L_APP", ADC310X_AGC_GAIN_APPLIED_L, 0}, {
	"GAIN_CTRL_1_R", ADC310X_AGC_GAIN_CTRL_R_1, 1}, {
	"GAIN_CTRL_2_R", ADC310X_AGC_GAIN_CTRL_R_2, 1}, {
	"MAX_GAIN_R", ADC310X_AGC_MAX_GAIN_R, 1}, {
	"ATTACK_TIME_R", ADC310X_AGC_ATTACK_TIME_R, 1}, {
	"DECAY_TIME_R", ADC310X_AGC_DECAY_TIME_R, 1}, {
	"NOISE_DEBOUNCE_R", ADC310X_AGC_NOISE_DEBNCE_R, 1}, {
	"SIG_DEBOUNCE_R", ADC310X_AGC_SIGNAL_DEBNCE_R, 1}, {
	"GAIN_R_APP", ADC310X_AGC_GAIN_APPLIED_R, 0},
	    /* Page 1 */
	{
	"DITHER", ADC310X_DITHER_CTRL, 1}, {
	"MIC_BIAS", ADC310X_MIC_BIAS_CTRL, 1}, {
	"IN_PGA_L_1", ADC310X_INPUT_SEL_PGA_L_1, 1}, {
	"IN_PGA_L_2", ADC310X_INPUT_SEL_PGA_L_2, 1}, {
	"IN_PGA_R_1", ADC310X_INPUT_SEL_PGA_R_1, 1}, {
	"IN_PGA_R_2", ADC310X_INPUT_SEL_PGA_R_2, 1}, {
	"ANALOG_L", ADC310X_PGA_ANALOG_L, 1}, {
	"ANALOG_R", ADC310X_PGA_ANALOG_R, 1}, {
	"LOW_CURRENT", ADC310X_LOW_CURRENT, 1}, {
	"ANALOG_FLAGS", ADC310X_PGA_ANALOG_FLAGS, 0},
	    /* Page 4 */
	{
	"FOIIR_N0_L1", ADC310X_ADC_FOIIR_N0_L1, 1}, {
	"FOIIR_N0_L2", ADC310X_ADC_FOIIR_N0_L2, 1}, {
	"FOIIR_N1_L1", ADC310X_ADC_FOIIR_N1_L1, 1}, {
	"FOIIR_N1_L2", ADC310X_ADC_FOIIR_N1_L2, 1}, {
	"FOIIR_D1_L1", ADC310X_ADC_FOIIR_D1_L1, 1}, {
	"FOIIR_D1_L2", ADC310X_ADC_FOIIR_D1_L2, 1}, {
	"BQA_N0_L1", ADC310X_ADC_BQA_N0_L1, 1}, {
	"BQA_N0_L2", ADC310X_ADC_BQA_N0_L2, 1}, {
	"BQA_N1_L1", ADC310X_ADC_BQA_N1_L1, 1}, {
	"BQA_N1_L2", ADC310X_ADC_BQA_N1_L2, 1}, {
	"BQA_N2_L1", ADC310X_ADC_BQA_N2_L1, 1}, {
	"BQA_N2_L2", ADC310X_ADC_BQA_N2_L2, 1}, {
	"BQA_D1_L1", ADC310X_ADC_BQA_D1_L1, 1}, {
	"BQA_D1_L2", ADC310X_ADC_BQA_D1_L2, 1}, {
	"BQA_D2_L1", ADC310X_ADC_BQA_D2_L1, 1}, {
	"BQA_D2_L2", ADC310X_ADC_BQA_D2_L2, 1}, {
	"BQB_N0_L1", ADC310X_ADC_BQB_N0_L1, 1}, {
	"BQB_N0_L2", ADC310X_ADC_BQB_N0_L2, 1}, {
	"BQB_N1_L1", ADC310X_ADC_BQB_N1_L1, 1}, {
	"BQB_N1_L2", ADC310X_ADC_BQB_N1_L2, 1}, {
	"BQB_N2_L1", ADC310X_ADC_BQB_N2_L1, 1}, {
	"BQB_N2_L2", ADC310X_ADC_BQB_N2_L2, 1}, {
	"BQB_D1_L1", ADC310X_ADC_BQB_D1_L1, 1}, {
	"BQB_D1_L2", ADC310X_ADC_BQB_D1_L2, 1}, {
	"BQB_D2_L1", ADC310X_ADC_BQB_D2_L1, 1}, {
	"BQB_D2_L2", ADC310X_ADC_BQB_D2_L2, 1}, {
	"BQC_N0_L1", ADC310X_ADC_BQC_N0_L1, 1}, {
	"BQC_N0_L2", ADC310X_ADC_BQC_N0_L2, 1}, {
	"BQC_N1_L1", ADC310X_ADC_BQC_N1_L1, 1}, {
	"BQC_N1_L2", ADC310X_ADC_BQC_N1_L2, 1}, {
	"BQC_N2_L1", ADC310X_ADC_BQC_N2_L1, 1}, {
	"BQC_N2_L2", ADC310X_ADC_BQC_N2_L2, 1}, {
	"BQC_D1_L1", ADC310X_ADC_BQC_D1_L1, 1}, {
	"BQC_D1_L2", ADC310X_ADC_BQC_D1_L2, 1}, {
	"BQC_D2_L1", ADC310X_ADC_BQC_D2_L1, 1}, {
	"BQC_D2_L2", ADC310X_ADC_BQC_D2_L2, 1}, {
	"BQD_N0_L1", ADC310X_ADC_BQD_N0_L1, 1}, {
	"BQD_N0_L2", ADC310X_ADC_BQD_N0_L2, 1}, {
	"BQD_N1_L1", ADC310X_ADC_BQD_N1_L1, 1}, {
	"BQD_N1_L2", ADC310X_ADC_BQD_N1_L2, 1}, {
	"BQD_N2_L1", ADC310X_ADC_BQD_N2_L1, 1}, {
	"BQD_N2_L2", ADC310X_ADC_BQD_N2_L2, 1}, {
	"BQD_D1_L1", ADC310X_ADC_BQD_D1_L1, 1}, {
	"BQD_D1_L2", ADC310X_ADC_BQD_D1_L2, 1}, {
	"BQD_D2_L1", ADC310X_ADC_BQD_D2_L1, 1}, {
	"BQD_D2_L2", ADC310X_ADC_BQD_D2_L2, 1}, {
	"BQE_N0_L1", ADC310X_ADC_BQE_N0_L1, 1}, {
	"BQE_N0_L2", ADC310X_ADC_BQE_N0_L2, 1}, {
	"BQE_N1_L1", ADC310X_ADC_BQE_N1_L1, 1}, {
	"BQE_N1_L2", ADC310X_ADC_BQE_N1_L2, 1}, {
	"BQE_N2_L1", ADC310X_ADC_BQE_N2_L1, 1}, {
	"BQE_N2_L2", ADC310X_ADC_BQE_N2_L2, 1}, {
	"BQE_D1_L1", ADC310X_ADC_BQE_D1_L1, 1}, {
	"BQE_D1_L2", ADC310X_ADC_BQE_D1_L2, 1}, {
	"BQE_D2_L1", ADC310X_ADC_BQE_D2_L1, 1}, {
	"BQE_D2_L2", ADC310X_ADC_BQE_D2_L2, 1}, {
	"FIR0_L1", ADC310X_ADC_FIR0_L1, 1}, {
	"FIR0_L2", ADC310X_ADC_FIR0_L2, 1}, {
	"FIR1_L1", ADC310X_ADC_FIR1_L1, 1}, {
	"FIR1_L2", ADC310X_ADC_FIR1_L2, 1}, {
	"FIR2_L1", ADC310X_ADC_FIR2_L1, 1}, {
	"FIR2_L2", ADC310X_ADC_FIR2_L2, 1}, {
	"FIR3_L1", ADC310X_ADC_FIR3_L1, 1}, {
	"FIR3_L2", ADC310X_ADC_FIR3_L2, 1}, {
	"FIR4_L1", ADC310X_ADC_FIR4_L1, 1}, {
	"FIR4_L2", ADC310X_ADC_FIR4_L2, 1}, {
	"FIR5_L1", ADC310X_ADC_FIR5_L1, 1}, {
	"FIR5_L2", ADC310X_ADC_FIR5_L2, 1}, {
	"FIR6_L1", ADC310X_ADC_FIR6_L1, 1}, {
	"FIR6_L2", ADC310X_ADC_FIR6_L2, 1}, {
	"FIR7_L1", ADC310X_ADC_FIR7_L1, 1}, {
	"FIR7_L2", ADC310X_ADC_FIR7_L2, 1}, {
	"FIR8_L1", ADC310X_ADC_FIR8_L1, 1}, {
	"FIR8_L2", ADC310X_ADC_FIR8_L2, 1}, {
	"FIR9_L1", ADC310X_ADC_FIR9_L1, 1}, {
	"FIR9_L2", ADC310X_ADC_FIR9_L2, 1}, {
	"FIR10_L1", ADC310X_ADC_FIR10_L1, 1}, {
	"FIR10_L2", ADC310X_ADC_FIR10_L2, 1}, {
	"FIR11_L1", ADC310X_ADC_FIR11_L1, 1}, {
	"FIR11_L2", ADC310X_ADC_FIR11_L2, 1}, {
	"FIR12_L1", ADC310X_ADC_FIR12_L1, 1}, {
	"FIR12_L2", ADC310X_ADC_FIR12_L2, 1}, {
	"FIR13_L1", ADC310X_ADC_FIR13_L1, 1}, {
	"FIR13_L2", ADC310X_ADC_FIR13_L2, 1}, {
	"FIR14_L1", ADC310X_ADC_FIR14_L1, 1}, {
	"FIR14_L2", ADC310X_ADC_FIR14_L2, 1}, {
	"FIR15_L1", ADC310X_ADC_FIR15_L1, 1}, {
	"FIR15_L2", ADC310X_ADC_FIR15_L2, 1}, {
	"FIR16_L1", ADC310X_ADC_FIR16_L1, 1}, {
	"FIR16_L2", ADC310X_ADC_FIR16_L2, 1}, {
	"FIR17_L1", ADC310X_ADC_FIR17_L1, 1}, {
	"FIR17_L2", ADC310X_ADC_FIR17_L2, 1}, {
	"FIR18_L1", ADC310X_ADC_FIR18_L1, 1}, {
	"FIR18_L2", ADC310X_ADC_FIR18_L2, 1}, {
	"FIR19_L1", ADC310X_ADC_FIR19_L1, 1}, {
	"FIR19_L2", ADC310X_ADC_FIR19_L2, 1}, {
	"FIR20_L1", ADC310X_ADC_FIR20_L1, 1}, {
	"FIR20_L2", ADC310X_ADC_FIR20_L2, 1}, {
	"FIR21_L1", ADC310X_ADC_FIR21_L1, 1}, {
	"FIR21_L2", ADC310X_ADC_FIR21_L2, 1}, {
	"FIR22_L1", ADC310X_ADC_FIR22_L1, 1}, {
	"FIR22_L2", ADC310X_ADC_FIR22_L2, 1}, {
	"FIR23_L1", ADC310X_ADC_FIR23_L1, 1}, {
	"FIR23_L2", ADC310X_ADC_FIR23_L2, 1}, {
	"FIR24_L1", ADC310X_ADC_FIR24_L1, 1}, {
	"FIR24_L2", ADC310X_ADC_FIR24_L2, 1}, {
	"FOIIR_N0_R1", ADC310X_ADC_FOIIR_N0_R1, 1}, {
	"FOIIR_N0_R2", ADC310X_ADC_FOIIR_N0_R2, 1}, {
	"FOIIR_N1_R1", ADC310X_ADC_FOIIR_N1_R1, 1}, {
	"FOIIR_N1_R2", ADC310X_ADC_FOIIR_N1_R2, 1}, {
	"FOIIR_D1_R1", ADC310X_ADC_FOIIR_D1_R1, 1}, {
	"FOIIR_D1_R2", ADC310X_ADC_FOIIR_D1_R2, 1}, {
	"BQA_N0_R1", ADC310X_ADC_BQA_N0_R1, 1}, {
	"BQA_N0_R2", ADC310X_ADC_BQA_N0_R2, 1}, {
	"BQA_N1_R1", ADC310X_ADC_BQA_N1_R1, 1}, {
	"BQA_N1_R2", ADC310X_ADC_BQA_N1_R2, 1}, {
	"BQA_N2_R1", ADC310X_ADC_BQA_N2_R1, 1}, {
	"BQA_N2_R2", ADC310X_ADC_BQA_N2_R2, 1}, {
	"BQA_D1_R1", ADC310X_ADC_BQA_D1_R1, 1}, {
	"BQA_D1_R2", ADC310X_ADC_BQA_D1_R2, 1}, {
	"BQA_D2_R1", ADC310X_ADC_BQA_D2_R1, 1}, {
	"BQA_D2_R2", ADC310X_ADC_BQA_D2_R2, 1}, {
	"BQB_N0_R1", ADC310X_ADC_BQB_N0_R1, 1}, {
	"BQB_N0_R2", ADC310X_ADC_BQB_N0_R2, 1}, {
	"BQB_N1_R1", ADC310X_ADC_BQB_N1_R1, 1}, {
	"BQB_N1_R2", ADC310X_ADC_BQB_N1_R2, 1}, {
	"BQB_N2_R1", ADC310X_ADC_BQB_N2_R1, 1}, {
	"BQB_N2_R2", ADC310X_ADC_BQB_N2_R2, 1}, {
	"BQB_D1_R1", ADC310X_ADC_BQB_D1_R1, 1}, {
	"BQB_D1_R2", ADC310X_ADC_BQB_D1_R2, 1}, {
	"BQB_D2_R1", ADC310X_ADC_BQB_D2_R1, 1}, {
	"BQB_D2_R2", ADC310X_ADC_BQB_D2_R2, 1}, {
	"BQC_N0_R1", ADC310X_ADC_BQC_N0_R1, 1}, {
	"BQC_N0_R2", ADC310X_ADC_BQC_N0_R2, 1}, {
	"BQC_N1_R1", ADC310X_ADC_BQC_N1_R1, 1}, {
	"BQC_N1_R2", ADC310X_ADC_BQC_N1_R2, 1}, {
	"BQC_N2_R1", ADC310X_ADC_BQC_N2_R1, 1}, {
	"BQC_N2_R2", ADC310X_ADC_BQC_N2_R2, 1}, {
	"BQC_D1_R1", ADC310X_ADC_BQC_D1_R1, 1}, {
	"BQC_D1_R2", ADC310X_ADC_BQC_D1_R2, 1}, {
	"BQC_D2_R1", ADC310X_ADC_BQC_D2_R1, 1}, {
	"BQC_D2_R2", ADC310X_ADC_BQC_D2_R2, 1}, {
	"BQD_N0_R1", ADC310X_ADC_BQD_N0_R1, 1}, {
	"BQD_N0_R2", ADC310X_ADC_BQD_N0_R2, 1}, {
	"BQD_N1_R1", ADC310X_ADC_BQD_N1_R1, 1}, {
	"BQD_N1_R2", ADC310X_ADC_BQD_N1_R2, 1}, {
	"BQD_N2_R1", ADC310X_ADC_BQD_N2_R1, 1}, {
	"BQD_N2_R2", ADC310X_ADC_BQD_N2_R2, 1}, {
	"BQD_D1_R1", ADC310X_ADC_BQD_D1_R1, 1}, {
	"BQD_D1_R2", ADC310X_ADC_BQD_D1_R2, 1}, {
	"BQD_D2_R1", ADC310X_ADC_BQD_D2_R1, 1}, {
	"BQD_D2_R2", ADC310X_ADC_BQD_D2_R2, 1}, {
	"BQE_N0_R1", ADC310X_ADC_BQE_N0_R1, 1}, {
	"BQE_N0_R2", ADC310X_ADC_BQE_N0_R2, 1}, {
	"BQE_N1_R1", ADC310X_ADC_BQE_N1_R1, 1}, {
	"BQE_N1_R2", ADC310X_ADC_BQE_N1_R2, 1}, {
	"BQE_N2_R1", ADC310X_ADC_BQE_N2_R1, 1}, {
	"BQE_N2_R2", ADC310X_ADC_BQE_N2_R2, 1}, {
	"BQE_D1_R1", ADC310X_ADC_BQE_D1_R1, 1}, {
	"BQE_D1_R2", ADC310X_ADC_BQE_D1_R2, 1}, {
	"BQE_D2_R1", ADC310X_ADC_BQE_D2_R1, 1}, {
	"BQE_D2_R2", ADC310X_ADC_BQE_D2_R2, 1}, {
	"FIR0_R1", ADC310X_ADC_FIR0_R1, 1}, {
	"FIR0_R2", ADC310X_ADC_FIR0_R2, 1}, {
	"FIR1_R1", ADC310X_ADC_FIR1_R1, 1}, {
	"FIR1_R2", ADC310X_ADC_FIR1_R2, 1}, {
	"FIR2_R1", ADC310X_ADC_FIR2_R1, 1}, {
	"FIR2_R2", ADC310X_ADC_FIR2_R2, 1}, {
	"FIR3_R1", ADC310X_ADC_FIR3_R1, 1}, {
	"FIR3_R2", ADC310X_ADC_FIR3_R2, 1}, {
	"FIR4_R1", ADC310X_ADC_FIR4_R1, 1}, {
	"FIR4_R2", ADC310X_ADC_FIR4_R2, 1}, {
	"FIR5_R1", ADC310X_ADC_FIR5_R1, 1}, {
	"FIR5_R2", ADC310X_ADC_FIR5_R2, 1}, {
	"FIR6_R1", ADC310X_ADC_FIR6_R1, 1}, {
	"FIR6_R2", ADC310X_ADC_FIR6_R2, 1}, {
	"FIR7_R1", ADC310X_ADC_FIR7_R1, 1}, {
	"FIR7_R2", ADC310X_ADC_FIR7_R2, 1}, {
	"FIR8_R1", ADC310X_ADC_FIR8_R1, 1}, {
	"FIR8_R2", ADC310X_ADC_FIR8_R2, 1}, {
	"FIR9_R1", ADC310X_ADC_FIR9_R1, 1}, {
	"FIR9_R2", ADC310X_ADC_FIR9_R2, 1}, {
	"FIR10_R1", ADC310X_ADC_FIR10_R1, 1}, {
	"FIR10_R2", ADC310X_ADC_FIR10_R2, 1}, {
	"FIR11_R1", ADC310X_ADC_FIR11_R1, 1}, {
	"FIR11_R2", ADC310X_ADC_FIR11_R2, 1}, {
	"FIR12_R1", ADC310X_ADC_FIR12_R1, 1}, {
	"FIR12_R2", ADC310X_ADC_FIR12_R2, 1}, {
	"FIR13_R1", ADC310X_ADC_FIR13_R1, 1}, {
	"FIR13_R2", ADC310X_ADC_FIR13_R2, 1}, {
	"FIR14_R1", ADC310X_ADC_FIR14_R1, 1}, {
	"FIR14_R2", ADC310X_ADC_FIR14_R2, 1}, {
	"FIR15_R1", ADC310X_ADC_FIR15_R1, 1}, {
	"FIR15_R2", ADC310X_ADC_FIR15_R2, 1}, {
	"FIR16_R1", ADC310X_ADC_FIR16_R1, 1}, {
	"FIR16_R2", ADC310X_ADC_FIR16_R2, 1}, {
	"FIR17_R1", ADC310X_ADC_FIR17_R1, 1}, {
	"FIR17_R2", ADC310X_ADC_FIR17_R2, 1}, {
	"FIR18_R1", ADC310X_ADC_FIR18_R1, 1}, {
	"FIR18_R2", ADC310X_ADC_FIR18_R2, 1}, {
	"FIR19_R1", ADC310X_ADC_FIR19_R1, 1}, {
	"FIR19_R2", ADC310X_ADC_FIR19_R2, 1}, {
	"FIR20_R1", ADC310X_ADC_FIR20_R1, 1}, {
	"FIR20_R2", ADC310X_ADC_FIR20_R2, 1}, {
	"FIR21_R1", ADC310X_ADC_FIR21_R1, 1}, {
	"FIR21_R2", ADC310X_ADC_FIR21_R2, 1}, {
	"FIR22_R1", ADC310X_ADC_FIR22_R1, 1}, {
	"FIR22_R2", ADC310X_ADC_FIR22_R2, 1}, {
	"FIR23_R1", ADC310X_ADC_FIR23_R1, 1}, {
	"FIR23_R2", ADC310X_ADC_FIR23_R2, 1}, {
	"FIR24_R1", ADC310X_ADC_FIR24_R1, 1}, {
"FIR24_R2", ADC310X_ADC_FIR24_R2, 1},};

static ssize_t tlv320adc310x_registers_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	unsigned i, n, reg_count;
	unsigned int read_buf;
	struct adc310x_priv *data = dev_get_drvdata(dev);

	reg_count = sizeof(tlv320adc310x_regs) / sizeof(tlv320adc310x_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		regmap_read(data->regmap, tlv320adc310x_regs[i].reg, &read_buf);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%s %02X\n",
			       tlv320adc310x_regs[i].name, read_buf);
	}
	return n;
}

static ssize_t tlv320adc310x_registers_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];
	struct adc310x_priv *data = dev_get_drvdata(dev);

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(tlv320adc310x_regs) / sizeof(tlv320adc310x_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, tlv320adc310x_regs[i].name)) {
			if (tlv320adc310x_regs[i].writeable == 1) {
				regcache_cache_only(data->regmap, false);
				error =
				    regmap_write(data->regmap,
						 tlv320adc310x_regs[i].reg,
						 value);
				if (error) {
					pr_err("%s:Failed to write %s\n",
					       __func__, name);
					return -1;
				}
				if (!strcmp(name, "RESET") && (value == 0x01)) {
					unsigned int rv = 0;

					do {
						regmap_read(data->regmap,
							    tlv320adc310x_regs
							    [i].reg, &rv);
					} while (rv & 0x01);
					pr_err("%s: Re-sync after RESET\n",
					       __func__);
					/* We need to re-sync the registers
					 * following a soft reset
					 */
					regcache_mark_dirty(data->regmap);
				}
			} else {
				pr_err("%s:Register %s is not writeable\n",
				       __func__, name);
				return -1;
			}
			return count;
		}
	}
	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		   tlv320adc310x_registers_show, tlv320adc310x_registers_store);

static struct attribute *tlv320adc310x_attrs[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group tlv320adc310x_attr_group = {
	.attrs = tlv320adc310x_attrs,
};

int tlv320adc310x_init_debug(struct adc310x_priv *adc310x)
{
	int ret;
	struct adc310x_priv *dbg_adc310x;

	printk("%s: Init debug\n", __func__);
	dbg_adc310x = adc310x;

	ret =
	    sysfs_create_group(&dbg_adc310x->i2c->dev.kobj,
			       &tlv320adc310x_attr_group);
	if (ret < 0)
		dev_err(&dbg_adc310x->i2c->dev, "Failed to create sysfs: %d\n",
			ret);

	return ret;
}
EXPORT_SYMBOL_GPL(tlv320adc310x_init_debug);

MODULE_DESCRIPTION("ASoC TLV320ADC310X debug");
MODULE_AUTHOR("Dan Murphy");
MODULE_LICENSE("Dual BSD/GPL");
