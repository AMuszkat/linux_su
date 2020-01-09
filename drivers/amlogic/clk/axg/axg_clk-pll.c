/*
 * drivers/amlogic/clk/axg/axg_clk-pll.c
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

/*
 * In the most basic form, a Meson PLL is composed as follows:
 *
 *                     PLL
 *      +------------------------------+
 *      |                              |
 * in -----[ /N ]---[ *M ]---[ >>OD ]----->> out
 *      |         ^        ^           |
 *      +------------------------------+
 *                |        |
 *               FREF     VCO
 *
 * out = (in * M / N) >> OD
 */

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/amlogic/cpu_version.h>
#include <dt-bindings/clock/amlogic,axg-clkc.h>

#ifdef CONFIG_ARM64
#include "../clkc.h"
#else
#include "m8b/clkc.h"
#endif

#define MESON_PLL_RESET				BIT(29)
#define MESON_PLL_ENABLE			BIT(30)
#define MESON_PLL_LOCK				BIT(31)

/* GXL TXL */
#define GXL_GP0_CNTL1 0xc084a000
#define GXL_GP0_CNTL2 0xb75020be
#define GXL_GP0_CNTL3 0x0a59a288
#define GXL_GP0_CNTL4 0xc000004d
#define GXL_GP0_CNTL5 0x00078000
/* AXG */
#define AXG_MIPI_CNTL0_ENABLE   BIT(29)
#define AXG_MIPI_CNTL0_BANDGAP  BIT(26)
#define AXG_PCIE_PLL_CNTL 0x400106c8
#define AXG_PCIE_PLL_CNTL1 0x0084a2aa
#define AXG_PCIE_PLL_CNTL2 0xb75020be
#define AXG_PCIE_PLL_CNTL3 0x0a47488e
#define AXG_PCIE_PLL_CNTL4 0xc000004d
#define AXG_PCIE_PLL_CNTL5 0x00078000
#define AXG_PCIE_PLL_CNTL6 0x002323c6

#define AXG_HIFI_PLL_CNTL1 0xc084b000
#define AXG_HIFI_PLL_CNTL2 0xb75020be
#define AXG_HIFI_PLL_CNTL3 0x0a6a3a88
#define AXG_HIFI_PLL_CNTL4 0xc000004d
#define AXG_HIFI_PLL_CNTL5 0x000581eb

#define to_meson_clk_pll(_hw) container_of(_hw, struct meson_clk_pll, hw)

static unsigned long meson_axg_pll_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct meson_clk_pll *pll = to_meson_clk_pll(hw);
	struct parm *p;
	unsigned long parent_rate_mhz = parent_rate;
	unsigned long rate_mhz;
	u16 n, m, frac = 0, od, od2 = 0;
	u32 reg;

	p = &pll->n;
	reg = readl(pll->base + p->reg_off);
	n = PARM_GET(p->width, p->shift, reg);

	p = &pll->m;
	reg = readl(pll->base + p->reg_off);
	m = PARM_GET(p->width, p->shift, reg);

	p = &pll->od;
	reg = readl(pll->base + p->reg_off);
	od = PARM_GET(p->width, p->shift, reg);

	p = &pll->od2;
	if (p->width) {
		reg = readl(pll->base + p->reg_off);
		od2 = PARM_GET(p->width, p->shift, reg);
	}

	p = &pll->frac;

	if (p->width) {
		reg = readl(pll->base + p->reg_off);
		frac = PARM_GET(p->width, p->shift, reg);
		rate_mhz = (parent_rate_mhz * m +
				(parent_rate_mhz * frac >> 12)) / n;
		rate_mhz = rate_mhz >> od >> od2;
	} else
		rate_mhz = (parent_rate_mhz * m / n) >> od >> od2;

	return rate_mhz;
}

/*
 * This calculates the fractional part based on the formula:
 *
 *          (rate * n * 2^od * 2^od2 - parent_rate * m) * 8192
 *  frac = ----------------------------------------------------
 *                            parent_rate
 */
static s16 calc_frac(unsigned long rate, unsigned long parent_rate, u16 m, u16 n, u16 od, u16 od2)
{
	return (((((s64)rate * n) << od << od2) - ((s64)parent_rate * m)) * 8192) / (s64)parent_rate;
}

/*
 * This calculates the rate for a given parent_rate, m, n, od, od2 and frac value:
 *
 *        /                      parent_rate * frac  \            1
 * rate = | parent_rate * m +  --------------------- | * -------------------
 *        \                             8192         /     n * 2^od * n^od2
 */
unsigned long calc_rate(unsigned long parent_rate, u16 m, u16 n, u16 od, u16 od2, s16 frac)
{
	return ((((u64)parent_rate * m) + ((s64)parent_rate * frac) / 8192) / n) >> od >> od2;
}

static unsigned long meson_axg_frac_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct meson_clk_pll *pll = to_meson_clk_pll(hw);
	struct parm *p;
	u16 n, m, frac_raw = 0, od, od2 = 0;
	s16 frac;
	u32 reg;

	p = &pll->n;
	reg = readl(pll->base + p->reg_off);
	n = PARM_GET(p->width, p->shift, reg);

	p = &pll->m;
	reg = readl(pll->base + p->reg_off);
	m = PARM_GET(p->width, p->shift, reg);

	p = &pll->od;
	reg = readl(pll->base + p->reg_off);
	od = PARM_GET(p->width, p->shift, reg);

	p = &pll->od2;
	reg = readl(pll->base + p->reg_off);
	od2 = PARM_GET(p->width, p->shift, reg);

	p = &pll->frac;
	reg = readl(pll->base + p->reg_off);
	frac_raw = PARM_GET(p->width, p->shift, reg);

	/*
	 * Sign extend if required
	 */
	if (frac_raw & (1 << 14))
		frac_raw |= (1 << 15);

	frac = (s16)frac_raw;

	return calc_rate(parent_rate, m, n, od, od2, frac);
}

static const struct pll_rate_table *get_closest_rate_entry(struct meson_clk_pll *pll, unsigned long desired_rate)
{
	int min_error = INT_MAX;
	int min_entry = -1;
	int i;

	for (i = 0; i < (pll->rate_count - 1); i++) {
		int error = desired_rate - pll->rate_table[i].rate;

		if (abs(error) < abs(min_error)) {
			min_error = error;
			min_entry = i;
		}
	}

	if (abs(min_error) > 1000000)
		pr_warn("%s: PLL base frequency error is higher than 1 MHz, probably a rate_table entry is missing\n", __func__);

	BUG_ON(min_entry == -1);

	return &(pll->rate_table[min_entry]);
}

static long meson_axg_frac_pll_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate)
{
	struct meson_clk_pll *pll = to_meson_clk_pll(hw);
	const struct pll_rate_table *rate_entry;
	unsigned long rounded_rate;
	s16 frac;

	/* Get the closest rate */
	rate_entry = get_closest_rate_entry(pll, rate);
	rounded_rate = rate_entry->rate;

	/*
	 * Try to get a closer rate to the requested one by skewing the frac value
	 * NOTE: The frac part has to be in the range below, otherwise the PLL is unstable
	 */
	frac = calc_frac(rate, *parent_rate, rate_entry->m, rate_entry->n, rate_entry->od, rate_entry->od2);
	if (rate_entry->frac != frac && (frac > -16128 && frac < 16128)) {
		rounded_rate = calc_rate(*parent_rate, rate_entry->m, rate_entry->n, rate_entry->od, rate_entry->od2, frac);
	}

	return rounded_rate;
}

static long meson_axg_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct meson_clk_pll *pll = to_meson_clk_pll(hw);
	const struct pll_rate_table *rate_table = pll->rate_table;
	int i;

	for (i = 0; i < pll->rate_count; i++) {
		if (rate <= rate_table[i].rate)
			return rate_table[i].rate;
	}

	/* else return the smallest value */
	return rate_table[0].rate;
}

static const struct pll_rate_table *meson_axg_get_pll_settings
	(struct meson_clk_pll *pll, unsigned long rate)
{
	const struct pll_rate_table *rate_table = pll->rate_table;
	int i;

	for (i = 0; i < pll->rate_count; i++) {
		if (rate == rate_table[i].rate)
			return &rate_table[i];
	}
	return NULL;
}

static int meson_axg_pll_wait_lock(struct meson_clk_pll *pll,
				   struct parm *p_n)
{
	int delay = 24000000;
	u32 reg;

	while (delay > 0) {
		reg = readl(pll->base + p_n->reg_off);

		if (reg & MESON_PLL_LOCK)
			return 0;
		delay--;
	}
	return -ETIMEDOUT;
}

/* Load default config and (re-)enable the PLL */
static void meson_axg_pll_load_default(struct clk_hw *hw, bool enable)
{
	if (!strcmp(clk_hw_get_name(hw), "gp0_pll")
		|| !strcmp(clk_hw_get_name(hw), "hifi_pll")
		|| !strcmp(clk_hw_get_name(hw), "pcie_pll")) {

		struct meson_clk_pll *pll = to_meson_clk_pll(hw);
		struct parm *p = &pll->n;
		void *cntlbase = pll->base + p->reg_off;

		if (!strcmp(clk_hw_get_name(hw), "pcie_pll")) {
			writel(AXG_PCIE_PLL_CNTL, cntlbase + (u64)(0*4));
			writel(AXG_PCIE_PLL_CNTL1, cntlbase + (u64)(1*4));
			writel(AXG_PCIE_PLL_CNTL2, cntlbase + (u64)(2*4));
			writel(AXG_PCIE_PLL_CNTL3, cntlbase + (u64)(3*4));
			writel(AXG_PCIE_PLL_CNTL4, cntlbase + (u64)(4*4));
			writel(AXG_PCIE_PLL_CNTL5, cntlbase + (u64)(5*4));
			writel(AXG_PCIE_PLL_CNTL6, cntlbase + (u64)(6*4));
		} else if (!strcmp(clk_hw_get_name(hw), "hifi_pll")) {
			writel(AXG_HIFI_PLL_CNTL1, cntlbase + (u64)6*4);
			writel(AXG_HIFI_PLL_CNTL2, cntlbase + (u64)1*4);
			writel(AXG_HIFI_PLL_CNTL3, cntlbase + (u64)2*4);
			writel(AXG_HIFI_PLL_CNTL4, cntlbase + (u64)3*4);
			writel(AXG_HIFI_PLL_CNTL5, cntlbase + (u64)4*4);
		} else {
			writel(GXL_GP0_CNTL1, cntlbase + (u64)6*4);
			writel(GXL_GP0_CNTL2, cntlbase + (u64)1*4);
			writel(GXL_GP0_CNTL3, cntlbase + (u64)2*4);
			writel(GXL_GP0_CNTL4, cntlbase + (u64)3*4);
			writel(GXL_GP0_CNTL5, cntlbase + (u64)4*4);
		}

		/* Only enable the PLL when explicitly requested */
		if (enable) {
			u32 reg;
			reg = readl(pll->base + p->reg_off);
			writel(((reg | (MESON_PLL_ENABLE)) &
				(~MESON_PLL_RESET)), pll->base + p->reg_off);
		}
	}
}

static int meson_axg_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct meson_clk_pll *pll = to_meson_clk_pll(hw);
	struct parm *p;
	const struct pll_rate_table *rate_set;
	unsigned long old_rate;
	int ret = 0;
	u32 reg;
	unsigned long flags = 0;

	if (parent_rate == 0 || rate == 0)
		return -EINVAL;

	old_rate = rate;

	rate_set = meson_axg_get_pll_settings(pll, rate);
	if (!rate_set)
		return -EINVAL;

	p = &pll->n;

	if (pll->lock)
		spin_lock_irqsave(pll->lock, flags);

	if (readl(pll->base + p->reg_off) & MESON_PLL_ENABLE) {
		old_rate = meson_axg_pll_recalc_rate(hw,
			clk_get_rate(clk_get_parent(hw->clk)));
		old_rate = meson_axg_pll_round_rate(hw, old_rate, NULL);

		if (old_rate == rate) {
			if (pll->lock)
				spin_unlock_irqrestore(pll->lock, flags);
			return ret;
		}
	}

	meson_axg_pll_load_default(hw, true);

	reg = readl(pll->base + p->reg_off);

	reg = PARM_SET(p->width, p->shift, reg, rate_set->n);
	writel(reg, pll->base + p->reg_off);

	p = &pll->m;
	reg = readl(pll->base + p->reg_off);
	reg = PARM_SET(p->width, p->shift, reg, rate_set->m);
	writel(reg, pll->base + p->reg_off);

	p = &pll->od;
	reg = readl(pll->base + p->reg_off);
	reg = PARM_SET(p->width, p->shift, reg, rate_set->od);
	writel(reg, pll->base + p->reg_off);

	p = &pll->od2;
	if (p->width) {
		reg = readl(pll->base + p->reg_off);
		reg = PARM_SET(p->width, p->shift, reg, rate_set->od2);
		writel(reg, pll->base + p->reg_off);
	}

	p = &pll->frac;
	if (p->width) {
		reg = readl(pll->base + p->reg_off);
		reg = PARM_SET(p->width, p->shift, reg, rate_set->frac);
		writel(reg, pll->base + p->reg_off);
	}

	p = &pll->n;

	/* PLL reset */
	reg = readl(pll->base + p->reg_off);
	writel(reg | MESON_PLL_RESET, pll->base + p->reg_off);
	udelay(10);
	writel(reg & (~MESON_PLL_RESET), pll->base + p->reg_off);

	ret = meson_axg_pll_wait_lock(pll, p);

	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);

	if (ret) {
		pr_warn("%s: pll did not lock, trying to lock rate %lu again\n",
			__func__, rate);
		meson_axg_pll_set_rate(hw, rate, parent_rate);
	}

	return ret;
}

static int meson_axg_frac_pll_get_current_settings(struct meson_clk_pll *pll, struct pll_rate_table *current_settings)
{
	struct parm *p;
	u32 reg;

	p = &pll->n;
	if (!(readl(pll->base + p->reg_off) & MESON_PLL_ENABLE))
		return -EINVAL;

	p = &pll->n;
	reg = readl(pll->base + p->reg_off);
	current_settings->n = PARM_GET(p->width, p->shift, reg);

	p = &pll->m;
	reg = readl(pll->base + p->reg_off);
	current_settings->m = PARM_GET(p->width, p->shift, reg);

	p = &pll->od;
	reg = readl(pll->base + p->reg_off);
	current_settings->od = PARM_GET(p->width, p->shift, reg);

	p = &pll->od2;
	reg = readl(pll->base + p->reg_off);
	current_settings->od2 = PARM_GET(p->width, p->shift, reg);

	p = &pll->frac;
	reg = readl(pll->base + p->reg_off);
	current_settings->frac = PARM_GET(p->width, p->shift, reg);

	/*
	 * Sign extend if required
	 */
	if (current_settings->frac & (1 << 14))
		current_settings->frac |= (1 << 15);

	return 0;
}

static int meson_axg_frac_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct meson_clk_pll *pll = to_meson_clk_pll(hw);
	const struct pll_rate_table *rate_entry;
	struct pll_rate_table current_settings;
	bool reset = false;
	int ret;
	s16 frac;
	u32 reg;

	rate_entry = get_closest_rate_entry(pll, rate);
	frac = rate_entry->frac;

	frac = calc_frac(rate, parent_rate, rate_entry->m, rate_entry->n, rate_entry->od, rate_entry->od2);
	if (frac <= -16128 && frac >= 16128) {
		pr_warn("%s: PLL frac value outside of safe range, probably a rate_table entry is missing\n", __func__);
		frac = rate_entry->frac;
	}

	ret = meson_axg_frac_pll_get_current_settings(pll, &current_settings);
	/* PLL is not running or in a strange state, force a reset */
	if (ret < 0)
		reset = true;

	/*
	 * If a reset is not required and we only changed the frac part, we can just update it
	 * without doing any resets, etc.
	 */
	if (!reset && current_settings.m == rate_entry->m && current_settings.n == rate_entry->n &&
		current_settings.od == rate_entry->od && current_settings.od2 == rate_entry->od2) {
		struct parm *p;

		/* If the fractional part is the same as well, we just do nothing */
		if (current_settings.frac == (u16)frac)
			return 0;

		p = &pll->frac;
		reg = readl(pll->base + p->reg_off);
		reg = PARM_SET(p->width, p->shift, reg, (u16)frac);
		writel(reg, pll->base + p->reg_off);

		return 0;
	} else {
		/* m, n, od, or od2 might have changed, we need to re-initialize the PLL */
		struct parm *p;

		pr_info("%s: re-initializing the PLL\n", __func__);

		meson_axg_pll_load_default(hw, false);

		/* Load the new configuration */
		p = &pll->n;
		reg = readl(pll->base + p->reg_off);
		reg = PARM_SET(p->width, p->shift, reg, rate_entry->n);
		writel(reg, pll->base + p->reg_off);

		p = &pll->m;
		reg = readl(pll->base + p->reg_off);
		reg = PARM_SET(p->width, p->shift, reg, rate_entry->m);
		writel(reg, pll->base + p->reg_off);

		p = &pll->od;
		reg = readl(pll->base + p->reg_off);
		reg = PARM_SET(p->width, p->shift, reg, rate_entry->od);
		writel(reg, pll->base + p->reg_off);

		p = &pll->od2;
		reg = readl(pll->base + p->reg_off);
		reg = PARM_SET(p->width, p->shift, reg, rate_entry->od2);
		writel(reg, pll->base + p->reg_off);

		p = &pll->frac;
		reg = readl(pll->base + p->reg_off);
		reg = PARM_SET(p->width, p->shift, reg, (u16)frac);
		writel(reg, pll->base + p->reg_off);


		/* Perform the reset and set enable bit */
		p = &pll->n;
		reg = readl(pll->base + p->reg_off);
		reg |= MESON_PLL_RESET | MESON_PLL_ENABLE;
		writel(reg, pll->base + p->reg_off);
		udelay(10);
		writel(reg & (~MESON_PLL_RESET), pll->base + p->reg_off);

		ret = meson_axg_pll_wait_lock(pll, p);
		if (ret) {
			pr_err("%s: failed to lock the PLL\n", __func__);
			return ret;
		}
	}

	return 0;
}

static int meson_axg_pll_enable(struct clk_hw *hw)
{
	struct meson_clk_pll *pll = to_meson_clk_pll(hw);
	struct parm *p;
	int ret = 0;
	unsigned long flags = 0;
	unsigned long first_set = 1;
	struct clk *parent;
	unsigned long rate;

	p = &pll->n;

	if (pll->lock)
		spin_lock_irqsave(pll->lock, flags);

	if (readl(pll->base + p->reg_off) & MESON_PLL_ENABLE) {
		if (pll->lock)
			spin_unlock_irqrestore(pll->lock, flags);
		return ret;
	}

	if (!strcmp(clk_hw_get_name(hw), "gp0_pll")
		|| !strcmp(clk_hw_get_name(hw), "hifi_pll")
		|| !strcmp(clk_hw_get_name(hw), "pcie_pll")) {
		void *cntlbase = pll->base + p->reg_off;

		if (!strcmp(clk_hw_get_name(hw), "pcie_pll")) {
			if (readl(cntlbase + (u64)(6*4)) == AXG_PCIE_PLL_CNTL6)
				first_set = 0;
		} else if (!strcmp(clk_hw_get_name(hw), "hifi_pll")) {
			if (readl(cntlbase + (u64)(4*4)) == AXG_HIFI_PLL_CNTL5)
				first_set = 0;
		} else {
			if (readl(cntlbase + (u64)(4*4)) == GXL_GP0_CNTL5)
				first_set = 0;
		}
	}

	parent = clk_get_parent(hw->clk);

	/*First init, just set minimal rate.*/
	if (first_set)
		rate = pll->rate_table[0].rate;
	else {
		rate = meson_axg_pll_recalc_rate(hw, clk_get_rate(parent));
		rate = meson_axg_pll_round_rate(hw, rate, NULL);
	}

	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);

	ret = meson_axg_pll_set_rate(hw, rate, clk_get_rate(parent));

	return ret;
}

static void meson_axg_pll_disable(struct clk_hw *hw)
{
	struct meson_clk_pll *pll = to_meson_clk_pll(hw);
	struct parm *p = &pll->n;
	unsigned long flags = 0;

	if (!strcmp(clk_hw_get_name(hw), "gp0_pll")
			|| !strcmp(clk_hw_get_name(hw), "hifi_pll")
			|| !strcmp(clk_hw_get_name(hw), "pcie_pll")) {
		if (pll->lock)
			spin_lock_irqsave(pll->lock, flags);

		writel(readl(pll->base + p->reg_off) & (~MESON_PLL_ENABLE),
			pll->base + p->reg_off);

		if (pll->lock)
			spin_unlock_irqrestore(pll->lock, flags);
	}

}

const struct clk_ops meson_axg_pll_ops = {
	.recalc_rate	= meson_axg_pll_recalc_rate,
	.round_rate	= meson_axg_pll_round_rate,
	.set_rate	= meson_axg_pll_set_rate,
	.enable		= meson_axg_pll_enable,
	.disable	= meson_axg_pll_disable,
};

const struct clk_ops meson_axg_frac_pll_ops = {
	.recalc_rate	= meson_axg_frac_pll_recalc_rate,
	.round_rate	= meson_axg_frac_pll_round_rate,
	.set_rate	= meson_axg_frac_pll_set_rate,
	.enable		= meson_axg_pll_enable,
	.disable	= meson_axg_pll_disable,
};

const struct clk_ops meson_axg_pll_ro_ops = {
	.recalc_rate	= meson_axg_pll_recalc_rate,
};

