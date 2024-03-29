/*
 * sound/soc/amlogic/auge/resample_hw.h
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
#ifndef __AML_AUDIO_RESAMPLE_HW_H__
#define __AML_AUDIO_RESAMPLE_HW_H__
#include <linux/clk.h>

#include "regs.h"
#include "iomap.h"

extern void resample_enable(bool enable);
extern int resample_init(int input_sr);
extern int resample_disable(void);
extern int resample_set_hw_param(int index);
extern void resample_src_select(int src);
extern void resample_format_set(int ch_num, int bits);

#endif
