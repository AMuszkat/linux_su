/*
 * sound/soc/amlogic/auge/pdm.c
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

#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/cma.h>
#include <linux/dma-contiguous.h>

#include <sound/soc.h>
#include <sound/tlv.h>

#include "pdm.h"
#include "pdm_hw.h"
#include "audio_io.h"
#include "iomap.h"
#include "regs.h"
#include "ddr_mngr.h"
#include "audio_utils.h"

static size_t pdm_paddr_base;
static size_t pdm_paddr_size;
static size_t pdm_toaddr_cur;
static size_t pdm_toaddr_off = 1024*1024;
static size_t pdm_readaddr_idx;
static size_t pdm_endddr_bak;
static char *reserved_vbuf;
static int s_pdm_wake_sys;
module_param(s_pdm_wake_sys, int, 0644);
MODULE_PARM_DESC(s_pdm_wake_sys, "pwr detect wake sys");

static struct snd_pcm_hardware aml_pdm_hardware = {
	.info			=
					SNDRV_PCM_INFO_MMAP |
					SNDRV_PCM_INFO_MMAP_VALID |
					SNDRV_PCM_INFO_INTERLEAVED |
					SNDRV_PCM_INFO_PAUSE |
					SNDRV_PCM_INFO_RESUME,

	.formats		=	SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24 |
					SNDRV_PCM_FMTBIT_S32,

	.rate_min			=	8000,
	.rate_max			=	48000,

	.channels_min		=	PDM_CHANNELS_MIN,
	.channels_max		=	PDM_CHANNELS_MAX,

	.buffer_bytes_max	=	128 * 1024,
	.period_bytes_max	=	64 * 1024,
	.period_bytes_min	=	32,
	.periods_min		=	2,
	.periods_max		=	1024,
	.fifo_size		=	0,
};

static unsigned int period_sizes[] = {
	64, 128, 256, 512, 1024, 2048, 4096,
	8192, 16384, 32768, 65536, 65536 * 2, 65536 * 4
};

static struct snd_pcm_hw_constraint_list hw_constraints_period_sizes = {
	.count = ARRAY_SIZE(period_sizes),
	.list = period_sizes,
	.mask = 0
};

static int s_pdm_filter_mode;

static const char *const pdm_filter_mode_texts[] = {
	"Filter Mode 0",
	"Filter Mode 1",
	"Filter Mode 2",
	"Filter Mode 3",
	"Filter Mode 4"
};

static const struct soc_enum pdm_filter_mode_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_filter_mode_texts),
			pdm_filter_mode_texts);

static int aml_pdm_filter_mode_get_enum(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = s_pdm_filter_mode;

	return 0;
}

static int aml_pdm_filter_mode_set_enum(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	s_pdm_filter_mode = ucontrol->value.enumerated.item[0];

	return 0;
}

int pdm_hcic_shift_gain;

static const char *const pdm_hcic_shift_gain_texts[] = {
	"keep with coeff",
	"shift with -0x4",
};

static const struct soc_enum pdm_hcic_shift_gain_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_hcic_shift_gain_texts),
			pdm_hcic_shift_gain_texts);

static int pdm_hcic_shift_gain_get_enum(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = pdm_hcic_shift_gain;

	return 0;
}

static int pdm_hcic_shift_gain_set_enum(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pdm_hcic_shift_gain = ucontrol->value.enumerated.item[0];

	return 0;
}

int pdm_dclk;

static const char *const pdm_dclk_texts[] = {
	"pdm dclk 3.072m, support 8k/16k/32k/48k",
	"pdm dclk 1.024m, support 8k/16k",
	"pdm dclk   768k, support 8k/16k",
};

static const struct soc_enum pdm_dclk_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_dclk_texts),
			pdm_dclk_texts);

static int pdm_dclk_get_enum(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = pdm_dclk;

	return 0;
}

static int pdm_dclk_set_enum(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pdm_dclk = ucontrol->value.enumerated.item[0];

	return 0;
}

static unsigned int pdm_pwr_thr_hi = 0x70000;
module_param(pdm_pwr_thr_hi, int, 0644);
MODULE_PARM_DESC(pdm_pwr_thr_hi, "pwr thr hi");
static unsigned int pdm_pwr_thr_lo = 0x16000;
module_param(pdm_pwr_thr_lo, int, 0644);
MODULE_PARM_DESC(pdm_pwr_thr_lo, "pwr thr lo");

static int pdm_pwr_thr_hi_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = pdm_pwr_thr_hi;

	return 0;
}

static int pdm_pwr_thr_hi_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pdm_pwr_thr_hi = ucontrol->value.enumerated.item[0];

	return 0;
}

static int pdm_pwr_thr_lo_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = pdm_pwr_thr_lo;

	return 0;
}

static int pdm_pwr_thr_lo_set(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pdm_pwr_thr_lo = ucontrol->value.enumerated.item[0];

	return 0;
}

static const struct snd_kcontrol_new snd_pdm_controls[] = {
	/* which set */
	SOC_ENUM_EXT("PDM Filter Mode",
		     pdm_filter_mode_enum,
		     aml_pdm_filter_mode_get_enum,
		     aml_pdm_filter_mode_set_enum),

	/* fix HCIC shift gain according current dmic */
	SOC_ENUM_EXT("HCIC shift gain from coeff",
		     pdm_hcic_shift_gain_enum,
		     pdm_hcic_shift_gain_get_enum,
		     pdm_hcic_shift_gain_set_enum),


	SOC_ENUM_EXT("pdm dclk",
		     pdm_dclk_enum,
		     pdm_dclk_get_enum,
		     pdm_dclk_set_enum),

	SOC_SINGLE_EXT("pdm pwr thr upper",
		     0, 0, 0x7fffffff, 0,
		     pdm_pwr_thr_hi_get,
		     pdm_pwr_thr_hi_set),
	SOC_SINGLE_EXT("pdm pwr thr low",
		     0, 0, 0x7fffffff, 0,
		     pdm_pwr_thr_lo_get,
		     pdm_pwr_thr_lo_set),
};

static irqreturn_t aml_pdm_isr_handler(int irq, void *data)
{
	struct snd_pcm_substream *substream =
		(struct snd_pcm_substream *)data;

	pr_debug("%s\n", __func__);

	snd_pcm_period_elapsed(substream);

	return IRQ_HANDLED;
}

static int aml_pdm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct aml_pdm *p_pdm = (struct aml_pdm *)
							dev_get_drvdata(dev);
	int ret;

	pr_info("%s, stream:%d\n",
		__func__, substream->stream);

	snd_soc_set_runtime_hwparams(substream, &aml_pdm_hardware);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	/* Ensure that period size is a multiple of 32bytes */
	ret = snd_pcm_hw_constraint_list(runtime, 0,
					   SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					   &hw_constraints_period_sizes);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"%s() setting constraints failed: %d\n",
			__func__, ret);
		return -EINVAL;
	}

	/* Ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"%s() setting constraints failed: %d\n",
			__func__, ret);
		return -EINVAL;
	}

	runtime->private_data = p_pdm;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		p_pdm->tddr = aml_audio_register_toddr
			(dev, p_pdm->actrl, aml_pdm_isr_handler, substream);
		if (p_pdm->tddr == NULL) {
			dev_err(dev, "failed to claim to ddr\n");
			return -ENXIO;
		}
	}

	return 0;
}

static int aml_pdm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct aml_pdm *p_pdm = (struct aml_pdm *)dev_get_drvdata(dev);

	pr_info("enter %s type: %d\n",
		__func__, substream->stream);

	aml_audio_unregister_toddr(p_pdm->dev, substream);

	return 0;
}


static int aml_pdm_hw_params(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	pr_info("enter %s\n", __func__);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);
	memset(runtime->dma_area, 0, runtime->dma_bytes);

	return ret;
}

static int aml_pdm_hw_free(struct snd_pcm_substream *substream)
{
	pr_info("%s\n", __func__);
	snd_pcm_lib_free_pages(substream);

	return 0;
}

static int aml_pdm_prepare(
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aml_pdm *p_pdm = runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		struct toddr *to = p_pdm->tddr;
		unsigned int start_addr, end_addr, int_addr;

		start_addr = runtime->dma_addr;
		end_addr = start_addr + runtime->dma_bytes - 8;
		int_addr = frames_to_bytes(runtime, runtime->period_size) / 8;

		pdm_endddr_bak = end_addr;
		aml_toddr_set_buf(to, start_addr, end_addr);
		aml_toddr_set_intrpt(to, int_addr);
	}

	return 0;
}

static int aml_pdm_trigger(
	struct snd_pcm_substream *substream, int cmd)
{
	return 0;
}

static snd_pcm_uframes_t aml_pdm_pointer(
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aml_pdm *p_pdm = runtime->private_data;
	unsigned int addr = 0, start_addr = 0;

	start_addr = runtime->dma_addr;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		addr = aml_toddr_get_position(p_pdm->tddr);

	return bytes_to_frames(runtime, addr - start_addr);
}

static int aml_pdm_mmap(
	struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_coherent(substream->pcm->card->dev, vma,
			runtime->dma_area, runtime->dma_addr,
			runtime->dma_bytes);
}

static int aml_pdm_silence(
	struct snd_pcm_substream *substream,
	int channel, snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned char *ppos = NULL;
	ssize_t n;

	pr_info("%s\n", __func__);

	n = frames_to_bytes(runtime, count);
	ppos = runtime->dma_area + frames_to_bytes(runtime, pos);
	memset(ppos, 0, n);

	return 0;
}

static int aml_pdm_pcm_new(struct snd_soc_pcm_runtime *soc_runtime)
{
	struct snd_pcm *pcm = soc_runtime->pcm;
	struct snd_pcm_substream *substream;
	struct snd_soc_dai *dai = soc_runtime->cpu_dai;
	int size = aml_pdm_hardware.buffer_bytes_max;
	int ret = -EINVAL;

	pr_info("%s dai->name: %s dai->id: %d\n",
		__func__, dai->name, dai->id);

	/* only capture for PDM */
	substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	if (pdm_paddr_size > size)
		size = pdm_paddr_size;
	if (substream) {
		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
					soc_runtime->platform->dev,
					size, &substream->dma_buffer);
		if (ret) {
			dev_err(soc_runtime->dev, "Cannot allocate buffer\n");
			return ret;
		} else if (pdm_paddr_size > 0) {
			pr_info("%s dma.addr:0x%llx, dma.area:%p\n", __func__,
					substream->dma_buffer.addr,
					substream->dma_buffer.area);
			reserved_vbuf = substream->dma_buffer.area;
			if (reserved_vbuf == NULL)
				pr_err("%s failed dma_alloc_cma\n", __func__);
		}
	}

	return 0;
}

static void aml_pdm_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;

	pr_info("%s\n", __func__);

	substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	if (substream) {
		snd_dma_free_pages(&substream->dma_buffer);
		substream->dma_buffer.area = NULL;
		substream->dma_buffer.addr = 0;
	}
}

static struct snd_pcm_ops aml_pdm_ops = {
	.open = aml_pdm_open,
	.close = aml_pdm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = aml_pdm_hw_params,
	.hw_free = aml_pdm_hw_free,
	.prepare = aml_pdm_prepare,
	.trigger = aml_pdm_trigger,
	.pointer = aml_pdm_pointer,
	.mmap = aml_pdm_mmap,
	.silence = aml_pdm_silence,
};

static int aml_pdm_probe(struct snd_soc_platform *platform)
{
	pr_info("%s\n", __func__);

	return 0;
}

struct snd_soc_platform_driver aml_soc_platform_pdm = {
	.pcm_new = aml_pdm_pcm_new,
	.pcm_free = aml_pdm_pcm_free,
	.ops = &aml_pdm_ops,
	.probe = aml_pdm_probe,
};
EXPORT_SYMBOL_GPL(aml_soc_platform_pdm);

static int aml_pdm_dai_hw_params(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int aml_pdm_dai_set_fmt(
	struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	return 0;
}


static int aml_pdm_dai_prepare(
	struct snd_pcm_substream *substream,
	struct snd_soc_dai *cpu_dai)
{
	struct aml_pdm *p_pdm = snd_soc_dai_get_drvdata(cpu_dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int bitwidth;
	unsigned int toddr_type, lsb;

	/* set bclk */
	bitwidth = snd_pcm_format_width(runtime->format);
	lsb = 32 - bitwidth;

	switch (bitwidth) {
	case 16:
	case 32:
		toddr_type = 0;
		break;
	case 24:
		toddr_type = 4;
		break;
	default:
		pr_err("invalid bit_depth: %d\n",
				bitwidth);
		return -1;
	}

	pr_info("%s rate:%d, bits:%d, channels:%d\n",
		__func__,
		runtime->rate,
		bitwidth,
		runtime->channels);


	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		struct toddr *to = p_pdm->tddr;
		struct toddr_fmt fmt;
		unsigned int osr = 192;

		/* to ddr pdmin */
		fmt.type      = toddr_type;
		fmt.msb       = 31;
		fmt.lsb       = lsb;
		fmt.endian    = 0;
		fmt.ch_num    = runtime->channels;
		fmt.bit_depth = bitwidth;
		aml_toddr_select_src(to, PDMIN);
		aml_toddr_set_format(to, &fmt);
		aml_toddr_set_fifos(to, 0x40);

		aml_pdm_ctrl(p_pdm->actrl,
			bitwidth, runtime->channels);

		/* filter for pdm */
		if (pdm_dclk == 1) {
			if (runtime->rate == 16000)
				osr = 64;
			else if (runtime->rate == 8000)
				osr = 128;
			else {
				pr_err("Not support rate:%d\n", runtime->rate);
				return -EINVAL;
			}
		} else if (pdm_dclk == 2) {
			if (runtime->rate == 16000)
				osr = 48;
			else if (runtime->rate == 8000)
				osr = 96;
			else {
				pr_err("Not support rate:%d\n", runtime->rate);
				return -EINVAL;
			}
		} else {
			if (runtime->rate == 48000)
				osr = 64;
			else if (runtime->rate == 32000)
				osr = 96;
			else if (runtime->rate == 16000)
				osr = 192;
			else if (runtime->rate == 8000)
				osr = 384;
			else {
				pr_err("Not support rate:%d\n", runtime->rate);
				return -EINVAL;
			}
		}
		p_pdm->filter_mode = s_pdm_filter_mode;
		aml_pdm_filter_ctrl(osr, p_pdm->filter_mode);
	}

	return 0;
}

static int aml_pdm_dai_trigger(
	struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *cpu_dai)
{
	struct aml_pdm *p_pdm = snd_soc_dai_get_drvdata(cpu_dai);

	pr_info("%s\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pdm_fifo_reset();

		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			dev_info(substream->pcm->card->dev, "pdm capture enable\n");
			aml_toddr_enable(p_pdm->tddr, 1);
			pdm_enable(1);
		}

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			dev_info(substream->pcm->card->dev, "pdm capture disable\n");
			pdm_enable(0);
			aml_toddr_enable(p_pdm->tddr, 0);
		}

		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int aml_pdm_dai_set_sysclk(struct snd_soc_dai *cpu_dai,
				int clk_id, unsigned int freq, int dir)
{
	struct aml_pdm *p_pdm = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int sysclk_srcpll_freq, dclk_srcpll_freq;

	sysclk_srcpll_freq = clk_get_rate(p_pdm->sysclk_srcpll);
	dclk_srcpll_freq = clk_get_rate(p_pdm->dclk_srcpll);

	clk_set_rate(p_pdm->clk_pdm_sysclk,
		sysclk_srcpll_freq / 5);
	if (dclk_srcpll_freq == 0)
		clk_set_rate(p_pdm->dclk_srcpll, 24576000);

	if (pdm_dclk == 1)
		clk_set_rate(p_pdm->clk_pdm_dclk, 1024000);
	else if (pdm_dclk == 2)
		clk_set_rate(p_pdm->clk_pdm_dclk, 768000);
	else
		clk_set_rate(p_pdm->clk_pdm_dclk, 3072000);

	return 0;
}

static int aml_pdm_dai_probe(struct snd_soc_dai *cpu_dai)
{
	int ret = 0;

	ret = snd_soc_add_dai_controls(cpu_dai, snd_pdm_controls,
				ARRAY_SIZE(snd_pdm_controls));
	if (ret < 0) {
		pr_err("%s, failed add snd pdm controls\n", __func__);
		return ret;
	}

	pr_info("%s\n", __func__);

	return 0;
}

int aml_pdm_dai_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *cpu_dai)
{
	struct aml_pdm *p_pdm = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	/* enable clock gate */
	ret = clk_prepare_enable(p_pdm->clk_gate);

	/* enable clock */
	ret = clk_prepare_enable(p_pdm->sysclk_srcpll);
	if (ret) {
		pr_err("Can't enable pcm sysclk_srcpll clock: %d\n", ret);
		goto err;
	}

	ret = clk_prepare_enable(p_pdm->dclk_srcpll);
	if (ret) {
		pr_err("Can't enable pcm dclk_srcpll clock: %d\n", ret);
		goto err;
	}

	ret = clk_prepare_enable(p_pdm->clk_pdm_sysclk);
	if (ret) {
		pr_err("Can't enable pcm clk_pdm_sysclk clock: %d\n", ret);
		goto err;
	}

	ret = clk_prepare_enable(p_pdm->clk_pdm_dclk);
	if (ret) {
		pr_err("Can't enable pcm clk_pdm_dclk clock: %d\n", ret);
		goto err;
	}

	return 0;
err:
	pr_err("failed enable clock\n");
	return -EINVAL;
}

void aml_pdm_dai_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *cpu_dai)
{
	struct aml_pdm *p_pdm = snd_soc_dai_get_drvdata(cpu_dai);

	/* disable clock and gate */
	clk_disable_unprepare(p_pdm->clk_pdm_dclk);
	clk_disable_unprepare(p_pdm->clk_pdm_sysclk);
	clk_disable_unprepare(p_pdm->sysclk_srcpll);
	clk_disable_unprepare(p_pdm->dclk_srcpll);
	clk_disable_unprepare(p_pdm->clk_gate);
}

static ssize_t show_cur(struct class *cla,
					  struct class_attribute *attr,
						char *buf)
{
	ssize_t ret = sprintf(buf, "%lu", pdm_readaddr_idx);
	return ret;
}
static struct class_attribute pdm_class_attrs[] = {
	__ATTR(cur, 0500, show_cur, NULL),
	__ATTR_NULL

};

static struct class pdm_class = {
	.name = "pdm_da",
	.class_attrs = pdm_class_attrs,
};
static int pdm_cdev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int pdm_cdev_release(struct inode *inode, struct file *file)
{
	pdm_readaddr_idx = 0;
	return 0;
}

static ssize_t pdm_cdev_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	int ret = 0;
	char *pbuf = reserved_vbuf;
	unsigned int off = 0;

	if (pdm_readaddr_idx == 0)
		return 0;
	if (pdm_readaddr_idx >= pdm_toaddr_cur) {
		if (count > (pdm_paddr_base + pdm_paddr_size
				- pdm_readaddr_idx))
			count = pdm_paddr_base + pdm_paddr_size
				- pdm_readaddr_idx;
		off = pdm_readaddr_idx - pdm_paddr_base;
		ret = copy_to_user((void *)buf, (void *)(pbuf + off), count);
		if (ret) {
			pr_err("%s cpy to user failed, count:%lu, idx:0x%lx\n",
					__func__, count, pdm_readaddr_idx);
			return 0;
		}
		pdm_readaddr_idx += count;
		if (pdm_readaddr_idx >= (pdm_paddr_base + pdm_paddr_size))
			pdm_readaddr_idx = pdm_paddr_base + pdm_toaddr_off;
	} else {
		if (count > pdm_toaddr_cur - pdm_readaddr_idx)
			count = pdm_toaddr_cur - pdm_readaddr_idx;
		off = pdm_readaddr_idx - pdm_paddr_base;
		if (copy_to_user((void *)buf, (void *)(pbuf + off), count)) {
			pr_err("%s cpy to user failed,count:%lu\n",
					__func__, count);
			return 0;
		}
		pdm_readaddr_idx += count;
		if (pdm_readaddr_idx == pdm_toaddr_cur)
			pdm_readaddr_idx = 0;
	}

	return count;
}

static dev_t pdm_devno;
static struct cdev pdm_cdev;
static const struct file_operations pdm_fops = {
	.owner      = THIS_MODULE,
	.open       = pdm_cdev_open,
	.release    = pdm_cdev_release,
	.read       = pdm_cdev_read,
};

static struct snd_soc_dai_ops aml_pdm_dai_ops = {
	.set_fmt        = aml_pdm_dai_set_fmt,
	.hw_params      = aml_pdm_dai_hw_params,
	.prepare        = aml_pdm_dai_prepare,
	.trigger        = aml_pdm_dai_trigger,
	.set_sysclk     = aml_pdm_dai_set_sysclk,
	.startup = aml_pdm_dai_startup,
	.shutdown = aml_pdm_dai_shutdown,
};

static int pdm_cdev_init(struct platform_device *pdev)
{
	int ret = 0;
	struct device *devp;

	ret = alloc_chrdev_region(&pdm_devno, 0, 1, "pdm_da");
	if (ret < 0) {
		dev_err(&pdev->dev, "pdm: failed to allocate major number\n");
		ret = -ENODEV;
		goto out;
	}
	ret = class_register(&pdm_class);
	if (ret)
		goto out;
	/* connect the file operations with cdev */
	cdev_init(&pdm_cdev, &pdm_fops);
	pdm_cdev.owner = THIS_MODULE;
	/* connect the major/minor number to the cdev */
	ret = cdev_add(&pdm_cdev, pdm_devno, 1);
	if (ret) {
		dev_err(&pdev->dev, "pdm: failed to add device\n");
		goto out;
	}
	devp = device_create(&pdm_class, NULL, pdm_devno, NULL, "pdm_da");
	if (IS_ERR(devp)) {
		dev_err(&pdev->dev, "pdm: failed to create device node\n");
		ret = PTR_ERR(devp);
		goto out;
	}
	pr_info("%s done\n", __func__);
out:
	return ret;
}

struct snd_soc_dai_driver aml_pdm_dai[] = {
	{
		.name = "PDM",
		.capture = {
			.channels_min =	PDM_CHANNELS_MIN,
			.channels_max = PDM_CHANNELS_MAX,
			.rates        = PDM_RATES,
			.formats      = PDM_FORMATS,
		},
		.probe            = aml_pdm_dai_probe,
		.ops     = &aml_pdm_dai_ops,
	},
};
EXPORT_SYMBOL_GPL(aml_pdm_dai);

static const struct snd_soc_component_driver aml_pdm_component = {
	.name = DRV_NAME,
};

static int aml_pdm_platform_probe(struct platform_device *pdev)
{
	struct aml_pdm *p_pdm;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *node_prt = NULL;
	struct platform_device *pdev_parent;
	struct aml_audio_controller *actrl = NULL;
	struct device *dev = &pdev->dev;
	#ifdef CONFIG_CMA
	struct cma *cma;
	#endif
	unsigned int thr_arr[2];

	int ret;

	p_pdm = devm_kzalloc(&pdev->dev,
			sizeof(struct aml_pdm),
			GFP_KERNEL);
	if (!p_pdm) {
		/*dev_err(&pdev->dev, "Can't allocate pcm_p\n");*/
		ret = -ENOMEM;
		goto err;
	}
	/* init reserved memory */
	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret != 0) {
		pr_info("%s failed to get reserved mem\n", __func__);
	} else {
		cma = dev_get_cma_area(&pdev->dev);
		if (cma) {
			pdm_paddr_base = cma_get_base(cma);
			pdm_paddr_size = cma_get_size(cma);
			pr_info("%s reserved memory base:%llx, size:%lx\n",
				__func__, cma_get_base(cma), cma_get_size(cma));
		} else
			pr_info("%s no reserved mem in CMA\n", __func__);
	}

	/* get audio controller */
	node_prt = of_get_parent(node);
	if (node_prt == NULL)
		return -ENXIO;

	pdev_parent = of_find_device_by_node(node_prt);
	of_node_put(node_prt);
	actrl = (struct aml_audio_controller *)
				platform_get_drvdata(pdev_parent);
	p_pdm->actrl = actrl;
	/* clock gate */
	p_pdm->clk_gate = devm_clk_get(&pdev->dev, "gate");
	if (IS_ERR(p_pdm->clk_gate)) {
		dev_err(&pdev->dev,
			"Can't get pdm gate\n");
		return PTR_ERR(p_pdm->clk_gate);
	}

	/* pinmux */
	p_pdm->pdm_pins = devm_pinctrl_get_select(&pdev->dev, "pdm_pins");
	if (IS_ERR(p_pdm->pdm_pins)) {
		p_pdm->pdm_pins = NULL;
		dev_err(&pdev->dev,
			"Can't get pdm pinmux\n");
		return PTR_ERR(p_pdm->pdm_pins);
	}

	p_pdm->sysclk_srcpll = devm_clk_get(&pdev->dev, "sysclk_srcpll");
	if (IS_ERR(p_pdm->sysclk_srcpll)) {
		dev_err(&pdev->dev,
			"Can't retrieve pll clock\n");
		ret = PTR_ERR(p_pdm->sysclk_srcpll);
		goto err;
	}

	p_pdm->dclk_srcpll = devm_clk_get(&pdev->dev, "dclk_srcpll");
	if (IS_ERR(p_pdm->dclk_srcpll)) {
		dev_err(&pdev->dev,
			"Can't retrieve data src clock\n");
		ret = PTR_ERR(p_pdm->dclk_srcpll);
		goto err;
	}

	p_pdm->clk_pdm_sysclk = devm_clk_get(&pdev->dev, "pdm_sysclk");
	if (IS_ERR(p_pdm->clk_pdm_sysclk)) {
		dev_err(&pdev->dev,
			"Can't retrieve clk_pdm_sysclk clock\n");
		ret = PTR_ERR(p_pdm->clk_pdm_sysclk);
		goto err;
	}

	p_pdm->clk_pdm_dclk = devm_clk_get(&pdev->dev, "pdm_dclk");
	if (IS_ERR(p_pdm->clk_pdm_dclk)) {
		dev_err(&pdev->dev,
			"Can't retrieve clk_pdm_dclk clock\n");
		ret = PTR_ERR(p_pdm->clk_pdm_dclk);
		goto err;
	}

	ret = clk_set_parent(p_pdm->clk_pdm_sysclk, p_pdm->sysclk_srcpll);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't set clk_pdm_sysclk parent clock\n");
		ret = PTR_ERR(p_pdm->clk_pdm_sysclk);
		goto err;
	}

	ret = clk_set_parent(p_pdm->clk_pdm_dclk, p_pdm->dclk_srcpll);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't set clk_pdm_dclk parent clock\n");
		ret = PTR_ERR(p_pdm->clk_pdm_dclk);
		goto err;
	}

	s_pdm_wake_sys = of_property_read_bool(node, "wake_sys");

	ret = of_property_read_u32(node, "filter_mode",
			&p_pdm->filter_mode);
	if (ret < 0) {
		/* defulat set 1 */
		p_pdm->filter_mode = 1;
	}
	s_pdm_filter_mode = p_pdm->filter_mode;
	pr_info("%s pdm filter mode from dts:%d\n",
		__func__, p_pdm->filter_mode);

	ret = of_property_read_u32_array(node, "pwr_det_threshd",
			thr_arr, 2);
	if (!ret) {
		pdm_pwr_thr_hi = thr_arr[0];
		pdm_pwr_thr_lo = thr_arr[1];
	}

	p_pdm->dev = dev;
	dev_set_drvdata(&pdev->dev, p_pdm);

	/*config ddr arb */
	aml_pdm_arb_config(p_pdm->actrl);

	ret = snd_soc_register_component(&pdev->dev,
				&aml_pdm_component,
				aml_pdm_dai,
				ARRAY_SIZE(aml_pdm_dai));

	if (ret) {
		dev_err(&pdev->dev,
			"snd_soc_register_component failed\n");
		goto err;
	}

	pr_info("%s, register soc platform\n", __func__);

	pdm_cdev_init(pdev);
	return snd_soc_register_platform(&pdev->dev, &aml_soc_platform_pdm);

err:
	return ret;

}

static int aml_pdm_platform_remove(struct platform_device *pdev)
{
	struct aml_pdm *pdm_priv = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(pdm_priv->sysclk_srcpll);
	clk_disable_unprepare(pdm_priv->clk_pdm_dclk);

	snd_soc_unregister_component(&pdev->dev);

	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static int aml_pdm_platform_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct aml_pdm *p_pdm = dev_get_drvdata(&pdev->dev);
	unsigned int start_addr, end_addr;
	int ret = 0;

	if (!s_pdm_wake_sys)
		return 0;

	start_addr = pdm_paddr_base + pdm_toaddr_off;
	end_addr = start_addr + pdm_paddr_size/2;

	if (reserved_vbuf != NULL)
		memset(reserved_vbuf + pdm_toaddr_off, 0, pdm_paddr_size/2);
	else
		return 0;

	pr_info("%s enable all pdm clk\n", __func__);
	/* enable clock gate */
	ret = clk_prepare_enable(p_pdm->clk_gate);

	/* enable clock */
	ret = clk_prepare_enable(p_pdm->sysclk_srcpll);
	if (ret) {
		pr_err("Can't enable pcm sysclk_srcpll clock: %d\n", ret);
		goto err;
	}

	ret = clk_prepare_enable(p_pdm->dclk_srcpll);
	if (ret) {
		pr_err("Can't enable pcm dclk_srcpll clock: %d\n", ret);
		goto err;
	}

	ret = clk_prepare_enable(p_pdm->clk_pdm_sysclk);
	if (ret) {
		pr_err("Can't enable pcm clk_pdm_sysclk clock: %d\n", ret);
		goto err;
	}

	ret = clk_prepare_enable(p_pdm->clk_pdm_dclk);
	if (ret) {
		pr_err("Can't enable pcm clk_pdm_dclk clock: %d\n", ret);
		goto err;
	}
	aud_toddr_conf_start(0, start_addr);
	aud_toddr_conf_end(0, end_addr);

	aud_pwr_detect_set_threshold_hi(pdm_pwr_thr_hi);
	aud_pwr_detect_set_threshold_lo(pdm_pwr_thr_lo);
err:
	return 0;
}
static int aml_pdm_platform_resume(struct platform_device *pdev)
{
	unsigned int start_addr;

	if (!s_pdm_wake_sys)
		return 0;

	if (reserved_vbuf == NULL)
		return 0;

	start_addr = aud_toddr_fetch_start(0);
	pdm_toaddr_cur = aud_toddr_fetch_posi(0);
	pdm_readaddr_idx = pdm_toaddr_cur;
	pr_info("%s start:0x%x, cur_addr :0x%lx\n",
		__func__, start_addr, pdm_toaddr_cur);

	pdm_enable(0);
	if (pdm_endddr_bak == 0)
		pdm_endddr_bak = start_addr + 4096;
	aud_toddr_conf_start(0, pdm_paddr_base);
	aud_toddr_conf_end(0, pdm_endddr_bak);
	pdm_endddr_bak = 0;

	return 0;
}
static const struct of_device_id aml_pdm_device_id[] = {
	{ .compatible = "amlogic, snd-pdm" },
	{}
};
MODULE_DEVICE_TABLE(of, aml_pdm_device_id);

struct platform_driver aml_pdm_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aml_pdm_device_id),
	},
	.probe = aml_pdm_platform_probe,
	.suspend = aml_pdm_platform_suspend,
	.resume  = aml_pdm_platform_resume,
	.remove = aml_pdm_platform_remove,
};
module_platform_driver(aml_pdm_driver);


MODULE_AUTHOR("AMLogic, Inc.");
MODULE_DESCRIPTION("Amlogic PDM ASoc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
