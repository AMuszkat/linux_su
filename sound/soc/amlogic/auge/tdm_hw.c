/*
 * sound/soc/amlogic/auge/tdm_hw.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <sound/soc.h>

#include "tdm_hw.h"
#include "iomap.h"

#define MST_CLK_INVERT_PH0_PAD_BCLK       (1 << 0)
#define MST_CLK_INVERT_PH0_PAD_FCLK       (1 << 1)
#define MST_CLK_INVERT_PH1_TDMIN_BCLK     (1 << 2)
#define MST_CLK_INVERT_PH1_TDMIN_FCLK     (1 << 3)
#define MST_CLK_INVERT_PH2_TDMOUT_BCLK    (1 << 4)
#define MST_CLK_INVERT_PH2_TDMOUT_FCLK    (1 << 5)

/* without audio handler, it should be improved */
void aml_tdm_enable(
	struct aml_audio_controller *actrl,
	int stream, int index,
	bool is_enable)
{
	unsigned int offset, reg;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_info("tdm playback enable\n");

		offset = EE_AUDIO_TDMOUT_B_CTRL0
				- EE_AUDIO_TDMOUT_A_CTRL0;
		reg = EE_AUDIO_TDMOUT_A_CTRL0 + offset * index;
		aml_audiobus_update_bits(actrl, reg, 1<<31, is_enable<<31);
	} else {
		pr_info("tdm capture enable\n");

		offset = EE_AUDIO_TDMIN_B_CTRL
				- EE_AUDIO_TDMIN_A_CTRL;
		reg = EE_AUDIO_TDMIN_A_CTRL + offset * index;
		aml_audiobus_update_bits(actrl, reg, 1<<31, is_enable<<31);
	}

}

void aml_tdm_arb_config(struct aml_audio_controller *actrl)
{
	/* config ddr arb */
	aml_audiobus_write(actrl, EE_AUDIO_ARB_CTRL, 1<<31|0xff<<0);
}

void aml_tdm_fifo_reset(
	struct aml_audio_controller *actrl,
	int stream, int index)
{
	unsigned int reg, offset;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		offset = EE_AUDIO_TDMOUT_B_CTRL0
				- EE_AUDIO_TDMOUT_A_CTRL0;
		reg = EE_AUDIO_TDMOUT_A_CTRL0 + offset * index;
		/* reset afifo */
		aml_audiobus_update_bits(actrl, reg, 3<<28, 0);
		aml_audiobus_update_bits(actrl, reg, 1<<29, 1<<29);
		aml_audiobus_update_bits(actrl, reg, 1<<28, 1<<28);
	} else {
		offset = EE_AUDIO_TDMIN_B_CTRL
				- EE_AUDIO_TDMIN_A_CTRL;
		reg = EE_AUDIO_TDMIN_A_CTRL + offset * index;
		/* reset afifo */
		aml_audiobus_update_bits(actrl, reg, 3<<28, 0);
		aml_audiobus_update_bits(actrl, reg, 1<<29, 1<<29);
		aml_audiobus_update_bits(actrl, reg, 1<<28, 1<<28);
	}

}


void tdm_enable(int tdm_index, int is_enable)
{
	unsigned int offset, reg;

	if (tdm_index < 3) {
		pr_info("tdmout is_enable:%d\n", is_enable);

		offset = EE_AUDIO_TDMOUT_B_CTRL0
			- EE_AUDIO_TDMOUT_A_CTRL0;
		reg = EE_AUDIO_TDMOUT_A_CTRL0 + offset * tdm_index;

		audiobus_update_bits(reg, 1<<31, is_enable<<31);
	} else if (tdm_index < 6) {
		pr_info("tdmin is_enable:%d\n", is_enable);

		tdm_index -= 3;
		offset = EE_AUDIO_TDMIN_B_CTRL
			- EE_AUDIO_TDMIN_A_CTRL;
		reg = EE_AUDIO_TDMIN_A_CTRL + offset * tdm_index;

		audiobus_update_bits(reg, 1<<31, is_enable<<31);
	}
}

void tdm_fifo_enable(int tdm_index, int is_enable)
{
	unsigned int reg, offset;

	if (tdm_index < 3) {
		offset = EE_AUDIO_TDMOUT_B_CTRL0
			- EE_AUDIO_TDMOUT_A_CTRL0;
		reg = EE_AUDIO_TDMOUT_A_CTRL0 + offset * tdm_index;

		if (is_enable) {
			audiobus_update_bits(reg, 1<<29, 1<<29);
			audiobus_update_bits(reg, 1<<28, 1<<28);
		} else
			audiobus_update_bits(reg, 3<<28, 0);

	} else if (tdm_index < 6) {
		tdm_index -= 3;
		offset = EE_AUDIO_TDMIN_B_CTRL
				- EE_AUDIO_TDMIN_A_CTRL;
		reg = EE_AUDIO_TDMIN_A_CTRL + offset * tdm_index;

		if (is_enable) {
			audiobus_update_bits(reg, 1<<29, 1<<29);
			audiobus_update_bits(reg, 1<<28, 1<<28);
		} else
			audiobus_update_bits(reg, 3<<28, 0);
	}
}

void aml_tdm_fifo_ctrl(
	struct aml_audio_controller *actrl,
	int bitwidth, int stream,
	int index, unsigned int fifo_id)
{
	unsigned int frddr_type;
	unsigned int reg, offset;

	switch (bitwidth) {
	case 8:
		frddr_type = 0;
		break;
	case 16:
		frddr_type = 2;
		break;
	case 24:
	case 32:
		frddr_type = 4;
		break;
	default:
		pr_err("invalid bit_depth: %d\n",
			bitwidth);
		return;
	}

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_info("tdm prepare----playback\n");
		// from ddr, 63bit split into 2 samples
		offset = EE_AUDIO_TDMOUT_B_CTRL1
				- EE_AUDIO_TDMOUT_A_CTRL1;
		reg = EE_AUDIO_TDMOUT_A_CTRL1 + offset * index;
		aml_audiobus_update_bits(actrl, reg,
				0x3<<24|0x1f<<8|0x7<<4,
				fifo_id<<24|(bitwidth-1)<<8|frddr_type<<4);
	} else {
		pr_info("tdm prepare----capture\n");
	}

}

void aml_tdm_set_format(
	struct aml_audio_controller *actrl,
	struct pcm_setting *p_config,
	unsigned int clk_sel,
	unsigned int index,
	unsigned int fmt,
	unsigned int capture_active,
	unsigned int playback_active)
{
	unsigned int binv_in, binv_out, finv, id;
	unsigned int valb, valf;
	unsigned int reg_in, reg_out, off_set;
	int bclkin_skew, bclkout_skew;
	int master_mode;
	unsigned int clkctl = 0;

	id = index;

	binv_in = 0;
	binv_out = 0;
	finv = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		valb = SLAVE_A + id;
		valf = SLAVE_A + id;
		master_mode = 0;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		valb = MASTER_A + clk_sel;
		valf = MASTER_A + clk_sel;
		master_mode = 1;
		break;
	default:
		return;
	}

	//TODO: clk tree
	reg_out = EE_AUDIO_CLK_TDMOUT_A_CTRL + id;
	reg_in = EE_AUDIO_CLK_TDMIN_A_CTRL + id;
	aml_audiobus_update_bits(actrl,
		reg_out,
		0xff<<20,
		valb<<24|valf<<20);
	aml_audiobus_update_bits(actrl,
		reg_in,
		0xff<<20,
		valb<<24|valf<<20);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		bclkout_skew = 1;
		bclkin_skew = 3;

		clkctl |= MST_CLK_INVERT_PH0_PAD_FCLK;
		if (!master_mode)
			finv = 1;

		if (master_mode) {
			clkctl |= MST_CLK_INVERT_PH0_PAD_BCLK;
			if (capture_active)
				binv_in |= 1;
		} else {
			if (playback_active)
				binv_out |= 1;
		}

		break;
	case SND_SOC_DAIFMT_DSP_A:
		/*
		 * Frame high, 1clk before data, one bit for frame sync,
		 * frame sync starts one serial clock cycle earlier,
		 * that is, together with the last bit of the previous
		 * data word.
		 */
		bclkout_skew = 1;
		bclkin_skew = 3;

		if (capture_active)
			binv_in |= 1;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_DSP_B:
		/*
		 * Frame high, one bit for frame sync,
		 * frame sync asserts with the first bit of the frame.
		 */
		bclkout_skew = 2;
		bclkin_skew = 2;

		if (capture_active)
			binv_in |= 1;
		break;
	default:
		return;
	}

	p_config->pcm_mode = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	pr_info("pad clk ctl value:%x\n", clkctl);
	/* set lrclk/bclk invertion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		/* Invert both clocks */
		if (!master_mode) {
			if (capture_active)
				binv_in ^= 1;
			if (playback_active)
				binv_out ^= 1;
		}

		finv |= 1;
		clkctl ^= MST_CLK_INVERT_PH0_PAD_BCLK;
		clkctl ^= MST_CLK_INVERT_PH0_PAD_FCLK;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		/* Invert bit clock */
		if (!master_mode) {
			if (capture_active)
				binv_in ^= 1;
			if (playback_active)
				binv_out ^= 1;
		}
		clkctl ^= MST_CLK_INVERT_PH0_PAD_BCLK;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/* Invert frame clock */
		finv ^= 1;
		clkctl ^= MST_CLK_INVERT_PH0_PAD_FCLK;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		/* normal cases */
		break;
	default:
		return;
	}
	pr_info("sclk_ph0 (pad) clk ctl set:%x\n", clkctl);
	/* clk ctrl: delay line and invert clk */
	/*clkctl |= 0x88880000;*/
	if (master_mode) {
		off_set = EE_AUDIO_MST_B_SCLK_CTRL1 - EE_AUDIO_MST_A_SCLK_CTRL1;
		reg_out = EE_AUDIO_MST_A_SCLK_CTRL1 + off_set * id;

		aml_audiobus_update_bits(actrl, reg_out, 0x3f, clkctl);
	}

	pr_info("master_mode(%d), binv_in(%d), binv_out(%d), finv(%d) out_skew(%d), in_skew(%d)\n",
			master_mode, binv_in, binv_out, finv, bclkout_skew, bclkin_skew);

	/* TDM out */
	if (playback_active) {

		reg_out = EE_AUDIO_CLK_TDMOUT_A_CTRL + id;
		aml_audiobus_update_bits(actrl, reg_out,
			0x3<<30, 0x3<<30);

		aml_audiobus_update_bits(actrl, reg_out,
			0x1<<29, binv_out<<29);

		off_set = EE_AUDIO_TDMOUT_B_CTRL1 - EE_AUDIO_TDMOUT_A_CTRL1;
		reg_out = EE_AUDIO_TDMOUT_A_CTRL1 + off_set * id;
		aml_audiobus_update_bits(actrl, reg_out, 0x1<<28, finv<<28);

		off_set = EE_AUDIO_TDMOUT_B_CTRL0 - EE_AUDIO_TDMOUT_A_CTRL0;
		reg_out = EE_AUDIO_TDMOUT_A_CTRL0 + off_set * id;
		aml_audiobus_update_bits(actrl, reg_out,
			0x1f<<15, bclkout_skew<<15);
	}

	/* TDM in */
	if (capture_active) {
		reg_in = EE_AUDIO_CLK_TDMIN_A_CTRL + id;
		aml_audiobus_update_bits(actrl, reg_in,
			0x3<<30, 0x3<<30);

		if (master_mode)
			aml_audiobus_update_bits(actrl, reg_in,
				0x1<<29, binv_in<<29);

		off_set = EE_AUDIO_TDMIN_B_CTRL - EE_AUDIO_TDMIN_A_CTRL;
		reg_in = EE_AUDIO_TDMIN_A_CTRL + off_set * id;
		aml_audiobus_update_bits(actrl, reg_in,
			3<<26|0x7<<16, 3<<26|bclkin_skew<<16);

		aml_audiobus_update_bits(actrl, reg_in,
			0x1<<25, finv<<25);

		if (p_config->pcm_mode == SND_SOC_DAIFMT_I2S)
			aml_audiobus_update_bits(actrl, reg_in,
				1<<30,
				1<<30);
	}
}

void aml_tdm_set_slot(
	struct aml_audio_controller *actrl,
	int slots, int slot_width, int index)
{
	unsigned int reg, offset;

	offset = EE_AUDIO_TDMOUT_B_CTRL0 - EE_AUDIO_TDMOUT_A_CTRL0;
	reg = EE_AUDIO_TDMOUT_A_CTRL0 + offset * index;
	aml_audiobus_update_bits(actrl, reg,
				0x3ff, ((slots - 1) << 5) | (slot_width - 1));

	offset = EE_AUDIO_TDMIN_B_CTRL - EE_AUDIO_TDMIN_A_CTRL;
	reg = EE_AUDIO_TDMIN_A_CTRL + offset * index;
	aml_audiobus_update_bits(actrl, reg,
		0xf<<20|0x1f, index<<20|(slot_width-1));
}

void aml_tdm_set_channel_mask(
	struct aml_audio_controller *actrl,
	int stream, int index, int lane, int mask)
{
	unsigned int offset, reg;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		offset = EE_AUDIO_TDMOUT_B_MASK0 - EE_AUDIO_TDMOUT_A_MASK0;
		reg = EE_AUDIO_TDMOUT_A_MASK0 + offset * index;
	} else {
		offset = EE_AUDIO_TDMIN_B_MASK0 - EE_AUDIO_TDMIN_A_MASK0;
		reg = EE_AUDIO_TDMIN_A_MASK0 + offset * index;
	}

	aml_audiobus_write(actrl, reg + lane, mask);
}

void aml_tdm_set_lane_channel_swap(
	struct aml_audio_controller *actrl,
	int stream, int index, int swap)
{
	unsigned int offset, reg;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		// set lanes mask acordingly
		offset = EE_AUDIO_TDMOUT_B_MASK0 - EE_AUDIO_TDMOUT_A_MASK0;
		reg = EE_AUDIO_TDMOUT_A_MASK0 + offset * index;

		pr_info("\ttdmout swap val = %#x\n", swap);
		offset = EE_AUDIO_TDMOUT_B_SWAP - EE_AUDIO_TDMOUT_A_SWAP;
		reg = EE_AUDIO_TDMOUT_A_SWAP + offset * index;
		aml_audiobus_write(actrl, reg, swap);
	} else {
		offset = EE_AUDIO_TDMIN_B_MASK0 - EE_AUDIO_TDMIN_A_MASK0;
		reg = EE_AUDIO_TDMIN_A_MASK0 + offset * index;

		pr_info("\ttdmin swap val = %#x\n", swap);
		offset = EE_AUDIO_TDMIN_B_SWAP - EE_AUDIO_TDMIN_A_SWAP;
		reg = EE_AUDIO_TDMIN_A_SWAP + offset * index;
		aml_audiobus_write(actrl, reg, swap);
	}
}

void aml_tdm_set_bclk_ratio(
	struct aml_audio_controller *actrl,
	int clk_sel, int lrclk_hi, int bclk_ratio)
{
	unsigned int reg, reg_step = 2;

	reg = EE_AUDIO_MST_A_SCLK_CTRL0 + reg_step * clk_sel;

	aml_audiobus_update_bits(actrl, reg,
				(3 << 30)|0x3ff<<10|0x3ff,
				(3 << 30)|lrclk_hi<<10|bclk_ratio);
}

void aml_tdm_set_lrclkdiv(
	struct aml_audio_controller *actrl,
	int clk_sel, int ratio)
{
	unsigned int reg, reg_step = 2;

	pr_info("aml_dai_set_clkdiv, clksel(%d), ratio(%d)\n",
			clk_sel, ratio);

	reg = EE_AUDIO_MST_A_SCLK_CTRL0 + reg_step * clk_sel;

	aml_audiobus_update_bits(actrl, reg,
		(3 << 30)|(0x3ff << 20),
		(3 << 30)|(ratio << 20));
}
