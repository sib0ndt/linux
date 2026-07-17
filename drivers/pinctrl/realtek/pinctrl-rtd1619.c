/*
 * Realtek pin controller driver
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/seq_file.h>

#include "pinctrl-rtd1619.h"
#include "../core.h"

#ifdef CONFIG_RTK_XEN_SUPPORT
#include <xen/xen.h>
#endif

/* =========================================================================
 * 1. ARRAY DEFINITIONS (Harus diletakkan di atas sebelum digunakan)
 * ========================================================================= */

static const struct rtk_pinmux_reg pinmux_reg_list[] = {
	/* ISO muxpad */
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x000},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x004},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x008},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x00c},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x010},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x048},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x04C},
	/* ISO  pfunc */
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x014},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x018},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x01C},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x020},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x024},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x028},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x02C},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x030},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x034},
	{.reg_base = PMUX_BASE_ISO, .reg_offset = 0x038},
	/* MISC muxpad */
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x000},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x004},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x008},
	/* MISC  pfunc */
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x00C},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x010},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x014},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x018},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x01C},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x020},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x024},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x028},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x02C},
	{.reg_base = PMUX_BASE_MISC, .reg_offset = 0x030},
};

static const struct rtk_pin_regmap pin_regmap[] = {
	/* GPIO */
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit =  0, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x014, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit =  1, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x014, .pcof_regbit = 5, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit =  2, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 9, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit =  5, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 13, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit =  8, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 17, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 11, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 21, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 14, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 25, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 17, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x014, .pcof_regbit = 29, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 18, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x018, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 19, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x018, .pcof_regbit = 5, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 20, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x018, .pcof_regbit = 9, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 21, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x018, .pcof_regbit = 13, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 22, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x018, .pcof_regbit = 17, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 24, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x018, .pcof_regbit = 21, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 26, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x018, .pcof_regbit = 25, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 28, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x018, .pcof_regbit = 29, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 30, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x01C, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x000, .pmux_regbit = 31, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x01C, .pcof_regbit = 5, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit =  0, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x01C, .pcof_regbit = 9, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit =  1, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x01C, .pcof_regbit = 13, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit =  2, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x01C, .pcof_regbit = 17, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit =  4, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x01C, .pcof_regbit = 21, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit =  6, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x01C, .pcof_regbit = 25, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit =  8, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x01C, .pcof_regbit = 29, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 0, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x00C, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 10, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x020, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 13, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x020, .pcof_regbit = 5, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 16, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x020, .pcof_regbit = 9, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 19, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x020, .pcof_regbit = 13, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 22, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x020, .pcof_regbit = 17, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 25, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x020, .pcof_regbit = 21, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 26, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x020, .pcof_regbit = 25, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 3, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x00C, .pcof_regbit = 4, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 6, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x00C, .pcof_regbit = 16, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 27, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x020, .pcof_regbit = 29, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x004, .pmux_regbit = 29, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x024, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 9, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x00C, .pcof_regbit = 28, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 12, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x010, .pcof_regbit = 8, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 15, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x010, .pcof_regbit = 20, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 17, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit =  0, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x024, .pcof_regbit = 4, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit =  3, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x024, .pcof_regbit = 15, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit =  6, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x024, .pcof_regbit = 26, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit =  9, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x028, .pcof_regbit = 5, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit = 12, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x028, .pcof_regbit = 16, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit = 15, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x028, .pcof_regbit = 27, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit = 18, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x02C, .pcof_regbit = 7, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit = 21, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x02C, .pcof_regbit = 11, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit = 23, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x02C, .pcof_regbit = 15, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit = 24, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x02C, .pcof_regbit = 19, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit = 25, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x02C, .pcof_regbit = 23, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x008, .pmux_regbit = 27, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x02C, .pcof_regbit = 27, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit =  0, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x030, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit =  3, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x030, .pcof_regbit = 3, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit =  5, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x030, .pcof_regbit = 7, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit =  6, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x030, .pcof_regbit = 11, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x010, .pmux_regbit =  0, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit =  7, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x030, .pcof_regbit = 15, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 10, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x030, .pcof_regbit = 19, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 12, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x030, .pcof_regbit = 23, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 15, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x030, .pcof_regbit = 27, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 18, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x030, .pcof_regbit = 31, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 21, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x034, .pcof_regbit = 3, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 24, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x034, .pcof_regbit = 7, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 25, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x034, .pcof_regbit = 11, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 26, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x034, .pcof_regbit = 15, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 28, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x034, .pcof_regbit = 19, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 29, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x034, .pcof_regbit = 23, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 30, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x034, .pcof_regbit = 27, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x010, .pmux_regbit =  1, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 20, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 13, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 23, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 17, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 26, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 21, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit = 29, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x014, .pcof_regbit = 25, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x008, .pmux_regbit =  0, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x014, .pcof_regbit = 29, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x008, .pmux_regbit =  3, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x018, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x008, .pmux_regbit =  6, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x018, .pcof_regbit = 5, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x008, .pmux_regbit =  9, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x018, .pcof_regbit = 9, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x008, .pmux_regbit = 12, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x018, .pcof_regbit = 13, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x008, .pmux_regbit = 15, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x018, .pcof_regbit = 17, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x008, .pmux_regbit = 18, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x018, .pcof_regbit = 21, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 25, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x02C, .pcof_regbit = 9, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 27, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x02C, .pcof_regbit = 21, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 29, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x030, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x004, .pmux_regbit =  0, .pmux_regbitmsk = 0x7, .pcof_regoff = 0x030, .pcof_regbit = 13, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x00C, .pmux_regbit = 31, .pmux_regbitmsk = 0x1, .pcof_regoff = 0x038, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_4_8},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit =  0, .pmux_regbitmsk = 0x3, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit =  2, .pmux_regbitmsk = 0x3, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit =  4, .pmux_regbitmsk = 0x3, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit =  6, .pmux_regbitmsk = 0x3, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit =  8, .pmux_regbitmsk = 0x3, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 10, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 11, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 12, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 13, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 14, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 15, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 16, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 17, .pmux_regbitmsk = 0x3, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 21, .pmux_regbitmsk = 0x3, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 23, .pmux_regbitmsk = 0x1, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_ISO, .pmux_regoff = 0x048, .pmux_regbit = 24, .pmux_regbitmsk = 0x3, .pcof_regoff = PCOF_UNSUPPORT, .pcof_regbit = 0, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit =  1, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x018, .pcof_regbit = 24, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit =  3, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x01C, .pcof_regbit = 4, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit =  5, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x01C, .pcof_regbit = 16, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit =  7, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x01C, .pcof_regbit = 28, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit =  9, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x020, .pcof_regbit = 9, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 11, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x020, .pcof_regbit = 21, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 13, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x024, .pcof_regbit = 1, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 15, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x024, .pcof_regbit = 13, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 17, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x024, .pcof_regbit = 25, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 19, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x028, .pcof_regbit = 5, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 21, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x028, .pcof_regbit = 17, .pcof_cur_strgh = PADDRI_UNSUPPORT},
	{.pmux_base = PMUX_BASE_MISC, .pmux_regoff = 0x000, .pmux_regbit = 23, .pmux_regbitmsk = 0x3, .pcof_regoff = 0x028, .pcof_regbit = 29, .pcof_cur_strgh = PADDRI_UNSUPPORT},
};

static const struct rtk_desc_pin rtk_pins[] = {
	/* GPIO */
	RTK_PIN(RTK_PINCTRL_PIN_gpio_0, RTK_FUNCTION(0x0, "gpio")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_1, RTK_FUNCTION(0x0, "gpio")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_2, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart2_loc0"), RTK_FUNCTION(0x2, "standby_dbg"), RTK_FUNCTION(0x3, "gspi_loc0"), RTK_FUNCTION(0x5, "scpu_ejtag_loc0"), RTK_FUNCTION(0x6, "lx_ejtag_loc0"), RTK_FUNCTION(0x7, "vfd")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_3, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart2_loc0"), RTK_FUNCTION(0x2, "standby_dbg"), RTK_FUNCTION(0x3, "gspi_loc0"), RTK_FUNCTION(0x4, "scan_debug"), RTK_FUNCTION(0x5, "scpu_ejtag_loc0"), RTK_FUNCTION(0x6, "lx_ejtag_loc0"), RTK_FUNCTION(0x7, "vfd")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_4, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart2_loc0"), RTK_FUNCTION(0x3, "gspi_loc0"), RTK_FUNCTION(0x4, "scan_debug"), RTK_FUNCTION(0x5, "scpu_ejtag_loc0"), RTK_FUNCTION(0x6, "lx_ejtag_loc0"), RTK_FUNCTION(0x7, "vfd")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_5, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart2_loc0"), RTK_FUNCTION(0x2, "i2c5"), RTK_FUNCTION(0x3, "gspi_loc0"), RTK_FUNCTION(0x4, "scan_debug"), RTK_FUNCTION(0x5, "scpu_ejtag_loc0"), RTK_FUNCTION(0x6, "lx_ejtag_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_6, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "i2c5"), RTK_FUNCTION(0x4, "scan_debug"), RTK_FUNCTION(0x5, "scpu_ejtag_loc0"), RTK_FUNCTION(0x6, "lx_ejtag_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_7, RTK_FUNCTION(0x0, "gpio")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_8, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_9, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_10, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_11, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_12, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "i2c0"), RTK_FUNCTION(0x2, "pwm0")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_13, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "i2c0"), RTK_FUNCTION(0x2, "pwm1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_14, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "etn_led"), RTK_FUNCTION(0x2, "pwm2")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_15, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "etn_led"), RTK_FUNCTION(0x2, "pwm3")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_16, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "i2c1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_17, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "i2c1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_18, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "i2c2")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_19, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "i2c2")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_20, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "pwm0")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_21, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "spdif"), RTK_FUNCTION(0x2, "pwm1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_22, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "pwm2")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_23, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "pwm3")),
	RTK_PIN(RTK_PINCTRL_PIN_demod_agc, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "qam_agc_if")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_25, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart2_loc1"), RTK_FUNCTION(0x2, "rtc_dig"), RTK_FUNCTION(0x3, "gspi_loc1"), RTK_FUNCTION(0x4, "rtc_ana"), RTK_FUNCTION(0x5, "spi")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_26, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart2_loc1"), RTK_FUNCTION(0x3, "gspi_loc1"), RTK_FUNCTION(0x5, "spi")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_27, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart2_loc1"), RTK_FUNCTION(0x3, "gspi_loc1"), RTK_FUNCTION(0x5, "spi")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_28, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart2_loc1"), RTK_FUNCTION(0x3, "gspi_loc1"), RTK_FUNCTION(0x5, "spi")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_29, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x3, "gphy"), RTK_FUNCTION(0x4, "extphy")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_30, RTK_FUNCTION(0x0, "gpio")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_31, RTK_FUNCTION(0x0, "gpio")),
	RTK_PIN(RTK_PINCTRL_PIN_sd3_cmd, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "sd"), RTK_FUNCTION(0x2, "sdio_loc0"), RTK_FUNCTION(0x4, "sc1"), RTK_FUNCTION(0x5, "scpu_ejtag_loc1"), RTK_FUNCTION(0x6, "lx_ejtag_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_sd3_clk, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "sd"), RTK_FUNCTION(0x2, "sdio_loc0"), RTK_FUNCTION(0x4, "sc1"), RTK_FUNCTION(0x5, "scpu_main2"), RTK_FUNCTION(0x6, "lx_ejtag_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_34, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "sd"), RTK_FUNCTION(0x2, "i2c3")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_35, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "sd"), RTK_FUNCTION(0x2, "i2c3")),
	RTK_PIN(RTK_PINCTRL_PIN_sd3_data_0, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "sd"), RTK_FUNCTION(0x2, "sdio_loc0"), RTK_FUNCTION(0x4, "sc1"), RTK_FUNCTION(0x5, "scpu_ejtag_loc1"), RTK_FUNCTION(0x6, "lx_ejtag_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_sd3_data_1, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "sd"), RTK_FUNCTION(0x2, "sdio_loc0"), RTK_FUNCTION(0x4, "sc1"), RTK_FUNCTION(0x5, "scpu_ejtag_loc1"), RTK_FUNCTION(0x6, "lx_ejtag_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_sd3_data_2, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "sd"), RTK_FUNCTION(0x2, "sdio_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_sd3_data_3, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "sd"), RTK_FUNCTION(0x2, "sdio_loc0"), RTK_FUNCTION(0x5, "scpu_ejtag_loc1"), RTK_FUNCTION(0x6, "lx_ejtag_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_40, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "sdio_loc1"), RTK_FUNCTION(0x3, "dmic_loc1"), RTK_FUNCTION(0x4, "tdm_ai_loc1"), RTK_FUNCTION(0x5, "ai_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_41, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "sdio_loc1"), RTK_FUNCTION(0x3, "dmic_loc1"), RTK_FUNCTION(0x4, "tdm_ai_loc1"), RTK_FUNCTION(0x5, "ai_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_42, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "sdio_loc1"), RTK_FUNCTION(0x3, "dmic_loc1"), RTK_FUNCTION(0x4, "tdm_ai_loc1"), RTK_FUNCTION(0x5, "ai_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_43, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "sdio_loc1"), RTK_FUNCTION(0x3, "dmic_loc1"), RTK_FUNCTION(0x4, "tdm_ai_loc1"), RTK_FUNCTION(0x5, "ai_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_44, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "sdio_loc1"), RTK_FUNCTION(0x3, "dmic_loc1"), RTK_FUNCTION(0x5, "ai_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_45, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x2, "sdio_loc1"), RTK_FUNCTION(0x3, "dmic_loc1"), RTK_FUNCTION(0x5, "ai_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_46, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "pcie0"), RTK_FUNCTION(0x3, "gphy"), RTK_FUNCTION(0x4, "extphy")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_47, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "ir_tx"), RTK_FUNCTION(0x2, "dc_fan_sensor")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_48, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "pll_test_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_49, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "pll_test_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_50, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "spdif"), RTK_FUNCTION(0x2, "test_loop_dis")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_51, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "pll_test_loc0"), RTK_FUNCTION(0x2, "debug_p2s"), RTK_FUNCTION(0x3, "etn_led"), RTK_FUNCTION(0x4, "scan_debug")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_52, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "pll_test_loc0"), RTK_FUNCTION(0x2, "debug_p2s"), RTK_FUNCTION(0x4, "scan_debug")),
	RTK_PIN(RTK_PINCTRL_PIN_ir_rx, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "ir_rx"), RTK_FUNCTION(0x2, "standby_dbg")),
	RTK_PIN(RTK_PINCTRL_PIN_ur0_rx, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart0")),
	RTK_PIN(RTK_PINCTRL_PIN_ur0_tx, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "uart0")),
	RTK_PIN(RTK_PINCTRL_PIN_usb_cc1, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "usb_cc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_57, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x3, "dmic_loc1"), RTK_FUNCTION(0x5, "ai_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_58, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x3, "dmic_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_59, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x3, "dmic_loc0"), RTK_FUNCTION(0x4, "sc0"), RTK_FUNCTION(0x5, "ai_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_60, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x3, "dmic_loc0"), RTK_FUNCTION(0x4, "sc0"), RTK_FUNCTION(0x5, "ai_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_61, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x3, "dmic_loc0"), RTK_FUNCTION(0x4, "sc0"), RTK_FUNCTION(0x5, "ai_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_62, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x3, "dmic_loc0"), RTK_FUNCTION(0x4, "sc0"), RTK_FUNCTION(0x5, "ai_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_63, RTK_FUNCTION(0x0, "gpio")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_64, RTK_FUNCTION(0x0, "gpio")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_65, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "pcie1"), RTK_FUNCTION(0x2, "hdd_pwr")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_66, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "pcie1")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_67, RTK_FUNCTION(0x0, "gpio")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_68, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "spdif")),
	RTK_PIN(RTK_PINCTRL_PIN_usb_cc2, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "usb_cc2")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_sync, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x2, "tdm_ai_loc0"), RTK_FUNCTION(0x3, "dmic_loc0"), RTK_FUNCTION(0x6, "ai_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_valid, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x2, "tdm_ai_loc0"), RTK_FUNCTION(0x3, "dmic_loc0"), RTK_FUNCTION(0x4, "scan_debug"), RTK_FUNCTION(0x6, "ai_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_clk, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x2, "tdm_ai_loc0"), RTK_FUNCTION(0x3, "dmic_loc0"), RTK_FUNCTION(0x6, "ai_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_data_0, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x2, "tdm_ai_loc0"), RTK_FUNCTION(0x3, "dmic_loc0")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_data_1, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x2, "tp1"), RTK_FUNCTION(0x6, "ao")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_data_2, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x2, "tp1"), RTK_FUNCTION(0x6, "ao")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_data_3, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x2, "tp1"), RTK_FUNCTION(0x6, "ao")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_data_4, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x2, "tp1"), RTK_FUNCTION(0x6, "ao")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_data_5, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x6, "ao")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_data_6, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x6, "ao")),
	RTK_PIN(RTK_PINCTRL_PIN_tp_data_7, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "tp0"), RTK_FUNCTION(0x3, "ao")),
	RTK_PIN(RTK_PINCTRL_PIN_hif_data_0, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "hi")),
	RTK_PIN(RTK_PINCTRL_PIN_hif_rdy, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "hi")),
	RTK_PIN(RTK_PINCTRL_PIN_hif_clk, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "hi"), RTK_FUNCTION(0x3, "i2c4")),
	RTK_PIN(RTK_PINCTRL_PIN_hif_en, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "hi"), RTK_FUNCTION(0x3, "i2c4"), RTK_FUNCTION(0x4, "edp")),
	RTK_PIN(RTK_PINCTRL_PIN_gpio_85, RTK_FUNCTION(0x0, "gpio"), RTK_FUNCTION(0x1, "spdif")),
	
	/* OTHERS */
	RTK_PIN(RTK_PINCTRL_PIN_ur2_loc, RTK_FUNCTION(0x0, "uart2_disable"), RTK_FUNCTION(0x1, "uart2_loc0"), RTK_FUNCTION(0x2, "uart2_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_gspi_loc, RTK_FUNCTION(0x0, "gspi_disable"), RTK_FUNCTION(0x1, "gspi_loc0"), RTK_FUNCTION(0x2, "gspi_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_sdio_loc, RTK_FUNCTION(0x0, "sdio_disable"), RTK_FUNCTION(0x1, "sdio_loc0"), RTK_FUNCTION(0x2, "sdio_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_hi_loc, RTK_FUNCTION(0x0, "hi_loc_disable"), RTK_FUNCTION(0x1, "hi_loc0"), RTK_FUNCTION(0x2, "hi_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_hi_width, RTK_FUNCTION(0x0, "hi_width_disable"), RTK_FUNCTION(0x1, "hi_width_1bit"), RTK_FUNCTION(0x2, "hi_width_8bit"), RTK_FUNCTION(0x3, "hi_width_16bit")),
	RTK_PIN(RTK_PINCTRL_PIN_debug_p2s_enable, RTK_FUNCTION(0x0, "p2s_disable"), RTK_FUNCTION(0x1, "debug_p2s")),
	RTK_PIN(RTK_PINCTRL_PIN_sf_en, RTK_FUNCTION(0x0, "sf_disable"), RTK_FUNCTION(0x1, "sf_enable")),
	RTK_PIN(RTK_PINCTRL_PIN_arm_trace_dbg_en, RTK_FUNCTION(0x0, "arm_trace_debug_disable"), RTK_FUNCTION(0x1, "arm_trace_debug_enable")),
	RTK_PIN(RTK_PINCTRL_PIN_pwm_01_open_drain_en_loc0, RTK_FUNCTION(0x0, "pwm_normal"), RTK_FUNCTION(0x1, "pwm_open_drain")),
	RTK_PIN(RTK_PINCTRL_PIN_pwm_23_open_drain_en_loc0, RTK_FUNCTION(0x0, "pwm_normal"), RTK_FUNCTION(0x1, "pwm_open_drain")),
	RTK_PIN(RTK_PINCTRL_PIN_pwm_01_open_drain_en_loc1, RTK_FUNCTION(0x0, "pwm_normal"), RTK_FUNCTION(0x1, "pwm_open_drain")),
	RTK_PIN(RTK_PINCTRL_PIN_pwm_23_open_drain_en_loc1, RTK_FUNCTION(0x0, "pwm_normal"), RTK_FUNCTION(0x1, "pwm_open_drain")),
	RTK_PIN(RTK_PINCTRL_PIN_ejtag_avcpu_loc, RTK_FUNCTION(0x0, "lx_ejtag_disable"), RTK_FUNCTION(0x1, "lx_ejtag_loc0"), RTK_FUNCTION(0x2, "lx_ejtag_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_ejtag_scpu_loc, RTK_FUNCTION(0x0, "scpu_ejtag_disable"), RTK_FUNCTION(0x1, "scpu_ejtag_loc0"), RTK_FUNCTION(0x2, "scpu_ejtag_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_i2c_tg, RTK_FUNCTION(0x1, "i2c_tg_disable"), RTK_FUNCTION(0x2, "i2c_tg_enable")),
	RTK_PIN(RTK_PINCTRL_PIN_dmic_loc, RTK_FUNCTION(0x1, "dmic_loc0"), RTK_FUNCTION(0x2, "dmic_loc1")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_rst, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_dd_sb, RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_clk, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_cmd, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_data_0, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_data_1, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_data_2, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_data_3, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_data_4, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_data_5, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_data_6, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
	RTK_PIN(RTK_PINCTRL_PIN_emmc_data_7, RTK_FUNCTION(0x1, "nf"), RTK_FUNCTION(0x2, "emmc")),
};

/* =========================================================================
 * 2. DRIVER IMPLEMENTATION
 * ========================================================================= */

static const struct rtk_pinctrl_desc rtk1619_pinctrl_data = {
	.pins = rtk_pins,
	.npins = ARRAY_SIZE(rtk_pins),
};

static void rtk_pmx_set(struct pinctrl_dev *pctldev, unsigned int pin, u8 config)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *addr;
	u32 val;
	u32 mask;

	if (pin_regmap[pin].pmux_regoff == PMUX_UNSUPPORT)
		return;

	switch (pin_regmap[pin].pmux_base) {
	case PMUX_BASE_ISO:
		addr = pctl->iso_membase + pin_regmap[pin].pmux_regoff;
		break;
	case PMUX_BASE_MISC:
		addr = pctl->misc_membase + pin_regmap[pin].pmux_regoff;
		break;
	default:
		dev_err(pctl->dev, "%s: Unknown pmux_base\n", __func__);
		return;
	}

	dev_dbg(pctl->dev, "%s: Addr(0x%p), bit=%u, config=%u\n",
		__func__, addr, pin_regmap[pin].pmux_regbit, config);

	val = readl(addr);
	mask = pin_regmap[pin].pmux_regbitmsk << pin_regmap[pin].pmux_regbit;
	writel(((val & ~mask) | (config << pin_regmap[pin].pmux_regbit)), addr);

	dev_dbg(pctl->dev, "%s: Addr(0x%p) final_val=0x%08x\n",
		__func__, addr, readl(addr));
}

static struct rtk_pinctrl_group *
rtk_pinctrl_find_group_by_name(struct rtk_pinctrl *pctl, const char *group)
{
	int i;

	for (i = 0; i < pctl->ngroups; i++) {
		struct rtk_pinctrl_group *grp = pctl->groups + i;

		if (!strcmp(grp->name, group))
			return grp;
	}

	return NULL;
}

static struct rtk_pinctrl_function *
rtk_pinctrl_find_function_by_name(struct rtk_pinctrl *pctl, const char *name)
{
	struct rtk_pinctrl_function *func = pctl->functions;
	int i;

	for (i = 0; i < pctl->nfunctions; i++) {
		if (!func[i].name)
			break;

		if (!strcmp(func[i].name, name))
			return func + i;
	}

	return NULL;
}

static struct rtk_desc_function *
rtk_pinctrl_desc_find_function_by_name(struct rtk_pinctrl *pctl,
				       const char *pin_name,
				       const char *func_name)
{
	int i;

	for (i = 0; i < pctl->desc->npins; i++) {
		const struct rtk_desc_pin *pin = pctl->desc->pins + i;

		if (!strcmp(pin->pin.name, pin_name)) {
			struct rtk_desc_function *func = pin->functions;

			while (func->name) {
				if (!strcmp(func->name, func_name))
					return func;

				func++;
			}
		}
	}

	return NULL;
}

static int rtk_pctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->ngroups;
}

static const char *rtk_pctrl_get_group_name(struct pinctrl_dev *pctldev,
					    unsigned int group)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->groups[group].name;
}

static int rtk_pctrl_get_group_pins(struct pinctrl_dev *pctldev,
				    unsigned int group,
				    const unsigned int **pins,
				    unsigned int *num_pins)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = (unsigned int *)&pctl->groups[group].pin;
	*num_pins = 1;

	return 0;
}

static int rtk_pctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
				    struct device_node *node,
				    struct pinctrl_map **map,
				    unsigned int *num_maps)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long *pinconfig;
	struct property *prop;
	const char *function;
	const char *group;
	int ret, nmaps, i = 0;
	u32 val;

	*map = NULL;
	*num_maps = 0;

	ret = of_property_read_string(node, "realtek,function", &function);
	if (ret) {
		dev_err(pctl->dev, "missing realtek,function property in node %pOF\n", node);
		return -EINVAL;
	}

	nmaps = of_property_count_strings(node, "realtek,pins") * 2;
	if (nmaps < 0) {
		dev_err(pctl->dev, "missing realtek,pins property in node %pOF\n", node);
		return -EINVAL;
	}

	*map = kcalloc(nmaps, sizeof(struct pinctrl_map), GFP_KERNEL);
	if (!*map)
		return -ENOMEM;

	of_property_for_each_string(node, "realtek,pins", prop, group) {
		struct rtk_pinctrl_group *grp = rtk_pinctrl_find_group_by_name(pctl, group);
		int j = 0, configlen = 0;

		if (!grp) {
			dev_err(pctl->dev, "unknown pin %s\n", group);
			continue;
		}

		if (!rtk_pinctrl_desc_find_function_by_name(pctl, grp->name, function)) {
			dev_err(pctl->dev, "unsupported function %s on pin %s\n",
				function, group);
			continue;
		}

		(*map)[i].type = PIN_MAP_TYPE_MUX_GROUP;
		(*map)[i].data.mux.group = group;
		(*map)[i].data.mux.function = function;

		i++;

		(*map)[i].type = PIN_MAP_TYPE_CONFIGS_GROUP;
		(*map)[i].data.configs.group_or_pin = group;

		if (of_find_property(node, "realtek,schmitt", NULL))
			configlen++;
		if (of_find_property(node, "realtek,drive", NULL))
			configlen++;
		if (of_find_property(node, "realtek,pull_en", NULL))
			configlen++;
		if (of_find_property(node, "realtek,pull_sel", NULL))
			configlen++;

		if (configlen) {
			pinconfig = kcalloc(configlen, sizeof(*pinconfig), GFP_KERNEL);
		} else {
			configlen = 1;
			pinconfig = kcalloc(1, sizeof(*pinconfig), GFP_KERNEL);
			pinconfig[j++] = pinconf_to_config_packed(PIN_CONFIG_END, 0);
		}

		if (!of_property_read_u32(node, "realtek,schmitt", &val)) {
			u16 schmitt_enable = val ? 1 : 0;
			pinconfig[j++] = pinconf_to_config_packed(
				PIN_CONFIG_INPUT_SCHMITT_ENABLE, schmitt_enable);
		}

		if (!of_property_read_u32(node, "realtek,drive", &val)) {
			pinconfig[j++] = pinconf_to_config_packed(
				PIN_CONFIG_DRIVE_STRENGTH, val);
		}

		if (!of_property_read_u32(node, "realtek,pull_en", &val)) {
			enum pin_config_param pull = PIN_CONFIG_END;

			if (val == 0)
				pull = PIN_CONFIG_BIAS_DISABLE;
			else if (val == 1)
				pull = PIN_CONFIG_DRIVE_PUSH_PULL;
			pinconfig[j++] = pinconf_to_config_packed(pull, 0);
		}

		if (!of_property_read_u32(node, "realtek,pull_sel", &val)) {
			enum pin_config_param pull = PIN_CONFIG_END;

			if (val == 0)
				pull = PIN_CONFIG_BIAS_PULL_DOWN;
			else if (val == 1)
				pull = PIN_CONFIG_BIAS_PULL_UP;
			pinconfig[j++] = pinconf_to_config_packed(pull, 1);
		}

		(*map)[i].data.configs.configs = pinconfig;
		(*map)[i].data.configs.num_configs = configlen;

		i++;
	}

	*num_maps = nmaps;

	return 0;
}

static void rtk_pctrl_dt_free_map(struct pinctrl_dev *pctldev,
				  struct pinctrl_map *map,
				  unsigned int num_maps)
{
	int i;

	for (i = 0; i < num_maps; i++) {
		if (map[i].type == PIN_MAP_TYPE_CONFIGS_GROUP)
			kfree(map[i].data.configs.configs);
	}

	kfree(map);
}

static void rtk_pctrl_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
			       unsigned int offset)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct rtk_desc_function *func = rtk_pins[offset].functions;
	void __iomem *addr;
	u32 val;
	u32 mask;
	u8 pin_val;
	int is_map = 0;

	if (pin_regmap[offset].pmux_regoff == PMUX_UNSUPPORT)
		return;

	switch (pin_regmap[offset].pmux_base) {
	case PMUX_BASE_ISO:
		addr = pctl->iso_membase + pin_regmap[offset].pmux_regoff;
		break;
	case PMUX_BASE_MISC:
		addr = pctl->misc_membase + pin_regmap[offset].pmux_regoff;
		break;
	default:
		return;
	}

	val = readl(addr);
	mask = pin_regmap[offset].pmux_regbitmsk << pin_regmap[offset].pmux_regbit;
	pin_val = (val & mask) >> pin_regmap[offset].pmux_regbit;

	seq_printf(s, "function: ");
	while (func && func->name) {
		if (func->muxval == pin_val) {
			is_map = 1;
			seq_printf(s, "[%s] ", func->name);
		} else {
			seq_printf(s, "%s ", func->name);
		}
		func++;
	}
	if (!is_map)
		seq_printf(s, "[not defined] ");
}

static const struct pinctrl_ops rtk_pctrl_ops = {
	.dt_node_to_map		= rtk_pctrl_dt_node_to_map,
	.dt_free_map		= rtk_pctrl_dt_free_map,
	.get_groups_count	= rtk_pctrl_get_groups_count,
	.get_group_name		= rtk_pctrl_get_group_name,
	.get_group_pins		= rtk_pctrl_get_group_pins,
	.pin_dbg_show		= rtk_pctrl_dbg_show,
};

static int rtk_pconf_parse_conf(struct pinctrl_dev *pctldev,
				unsigned int pin, enum pin_config_param param,
				enum pin_config_param arg)
{
	void __iomem *addr;
	u8 set_val;
	u16 strength;
	u32 val, mask;
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	int pconf_pulsel, pconf_pulen, pconf_schm, pconf_curr;

	if (pin_regmap[pin].pcof_regoff == PCOF_UNSUPPORT)
		return 0;

	pconf_schm = RTK_PCONF_SCHM;
	pconf_curr = RTK_PCONF_CURR;

	if (pin == 55 || pin == 104) {
		pconf_pulsel = 0;
		pconf_pulen = 1;
	} else {
		pconf_pulsel = RTK_PCONF_PULSEL;
		pconf_pulen = RTK_PCONF_PULEN;
	}

	switch (pin_regmap[pin].pmux_base) {
	case PMUX_BASE_ISO:
		addr = pctl->iso_membase + pin_regmap[pin].pcof_regoff;
		break;
	case PMUX_BASE_MISC:
		addr = pctl->misc_membase + pin_regmap[pin].pcof_regoff;
		break;
	default:
		dev_err(pctl->dev, "%s: Unknown pmux_base\n", __func__);
		return -EINVAL;
	}

	switch (param) {
	case PIN_CONFIG_INPUT_SCHMITT:
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		set_val = arg ? 1 : 0;
		val = readl(addr);
		mask = 1 << (pin_regmap[pin].pcof_regbit + pconf_schm);
		writel(((val & ~mask) | (set_val << (pin_regmap[pin].pcof_regbit + pconf_schm))), addr);
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		strength = arg;
		switch (pin_regmap[pin].pcof_cur_strgh) {
		case PADDRI_4_8:
			if (strength == 4)
				set_val = 0;
			else if (strength == 8)
				set_val = 1;
			else
				return -EINVAL;
			break;
		case PADDRI_2_4:
			if (strength == 2)
				set_val = 0;
			else if (strength == 4)
				set_val = 1;
			else
				return -EINVAL;
			break;
		case PADDRI_UNSUPPORT:
		default:
			return -EINVAL;
		}
		val = readl(addr);
		mask = 1 << (pin_regmap[pin].pcof_regbit + pconf_curr);
		writel(((val & ~mask) | (set_val << (pin_regmap[pin].pcof_regbit + pconf_curr))), addr);
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		val = readl(addr);
		mask = 1 << (pin_regmap[pin].pcof_regbit + pconf_pulen);
		writel(((val & ~mask) | (1 << (pin_regmap[pin].pcof_regbit + pconf_pulen))), addr);
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		val = readl(addr);
		mask = 1 << (pin_regmap[pin].pcof_regbit + pconf_pulen);
		writel(((val & ~mask) | (0 << (pin_regmap[pin].pcof_regbit + pconf_pulen))), addr);
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (pin == 52) {
			val = readl(addr - 0x4);
			mask = 1 << 31;
			writel((val & ~mask) | (1 << 31), addr - 0x4);
		} else if (pin == 61) {
			val = readl(addr + 0x4);
			mask = 1 << 0;
			writel((val & ~mask) | (1 << 0), addr + 0x4);
		} else {
			val = readl(addr);
			mask = 1 << (pin_regmap[pin].pcof_regbit + pconf_pulsel);
			writel(((val & ~mask) | (1 << (pin_regmap[pin].pcof_regbit + pconf_pulsel))), addr);
		}
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (pin == 52) {
			val = readl(addr - 0x4);
			mask = 1 << 31;
			writel((val & ~mask) | (0 << 31), addr - 0x4);
		} else if (pin == 61) {
			val = readl(addr + 0x4);
			mask = 1 << 0;
			writel((val & ~mask) | (0 << 0), addr + 0x4);
		} else {
			val = readl(addr);
			mask = 1 << (pin_regmap[pin].pcof_regbit + pconf_pulsel);
			writel(((val & ~mask) | (0 << (pin_regmap[pin].pcof_regbit + pconf_pulsel))), addr);
		}
		break;
	default:
		break;
	}

	return 0;
}

static int rtk_pconf_group_get(struct pinctrl_dev *pctldev,
			       unsigned int group,
			       unsigned long *config)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*config = pctl->groups[group].config;
	return 0;
}

static int rtk_pconf_group_set(struct pinctrl_dev *pctldev,
			       unsigned int group,
			       unsigned long *configs,
			       unsigned int num_configs)
{
	u32 i;
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct rtk_pinctrl_group *g = &pctl->groups[group];

	if (pin_regmap[g->pin].pcof_regoff == PCOF_UNSUPPORT) {
		dev_dbg(pctl->dev, "%s: pin(%d) name(%s) not support pin config\n",
			__func__, g->pin, g->name);
		g->config = configs[num_configs - 1];
		return 0;
	}

	for (i = 0; i < num_configs; i++) {
		rtk_pconf_parse_conf(pctldev, g->pin,
				     pinconf_to_config_param(configs[i]),
				     pinconf_to_config_argument(configs[i]));
		g->config = configs[i];
	}

	return 0;
}

static const struct pinconf_ops rtk_pconf_ops = {
	.pin_config_group_get	= rtk_pconf_group_get,
	.pin_config_group_set	= rtk_pconf_group_set,
};

static int rtk_pmx_get_funcs_cnt(struct pinctrl_dev *pctldev)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->nfunctions;
}

static const char *rtk_pmx_get_func_name(struct pinctrl_dev *pctldev,
					 unsigned int function)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->functions[function].name;
}

static int rtk_pmx_get_func_groups(struct pinctrl_dev *pctldev,
				   unsigned int function,
				   const char * const **groups,
				   unsigned int * const num_groups)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctl->functions[function].groups;
	*num_groups = pctl->functions[function].ngroups;

	return 0;
}

static int rtk_pmx_enable(struct pinctrl_dev *pctldev,
			  unsigned int function,
			  unsigned int group)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct rtk_pinctrl_group *g = pctl->groups + group;
	struct rtk_pinctrl_function *func = pctl->functions + function;
	struct rtk_desc_function *desc =
		rtk_pinctrl_desc_find_function_by_name(pctl, g->name, func->name);

	if (!desc)
		return -EINVAL;

	rtk_pmx_set(pctldev, g->pin, desc->muxval);

	return 0;
}

void gpio_pinctrl_control(struct gpio_chip *gc, int gpio, int direction, int value)
{
	struct device_node *np = dev_of_node(gc->parent);
	struct of_phandle_args pinspec;
	struct pinctrl_dev *pctldev;
	int ret, pin;

	ret = of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3, 0, &pinspec);
	if (ret) {
		pr_err("rtk-pinctrl: Can't get pinspec\n");
		return;
	}

	pctldev = of_pinctrl_get(pinspec.np);
	pin = gpio + pinspec.args[1];

	if (direction) {
		if (value == 0 || value == 1) {
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_BIAS_DISABLE, 0);
		} else if (value == 2 || value == 3) {
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_DRIVE_PUSH_PULL, 0);
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_BIAS_PULL_UP, 0);
		} else if (value == 4 || value == 5) {
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_DRIVE_PUSH_PULL, 0);
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_BIAS_PULL_DOWN, 0);
		}
	} else {
		if (value == 1) {
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_DRIVE_PUSH_PULL, 0);
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_BIAS_PULL_UP, 0);
		} else if (value == 0) {
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_DRIVE_PUSH_PULL, 0);
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_BIAS_PULL_DOWN, 0);
		} else if (value == 2) {
			rtk_pconf_parse_conf(pctldev, pin, PIN_CONFIG_BIAS_DISABLE, 0);
		}
	}
}
EXPORT_SYMBOL_GPL(gpio_pinctrl_control);

static int rtk_pmx_gpio_request_enable(struct pinctrl_dev *pctldev,
				       struct pinctrl_gpio_range *range,
				       unsigned int offset)
{
	struct rtk_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct rtk_pinctrl_group *g = pctl->groups + offset;
	struct rtk_desc_function *desc;
	const char *func = "gpio";

	desc = rtk_pinctrl_desc_find_function_by_name(pctl, g->name, func);
	if (!desc) {
		dev_err(pctl->dev, "Set gpio pinmux fail, Pin(%s) offset=%u\n",
			g->name, offset);
		return -EINVAL;
	}

	rtk_pmx_set(pctldev, offset, desc->muxval);

	return 0;
}

static void rtk_pmx_gpio_disable_free(struct pinctrl_dev *pctldev,
				      struct pinctrl_gpio_range *range,
				      unsigned int offset)
{
	/* TODO: need to add gpio related api */
}

static const struct pinmux_ops rtk_pmx_ops = {
	.get_functions_count = rtk_pmx_get_funcs_cnt,
	.get_function_name = rtk_pmx_get_func_name,
	.get_function_groups = rtk_pmx_get_func_groups,
	.set_mux = rtk_pmx_enable,
	.gpio_request_enable = rtk_pmx_gpio_request_enable,
	.gpio_disable_free = rtk_pmx_gpio_disable_free,
};

static struct pinctrl_desc rtk_pctrl_desc = {
	.confops	= &rtk_pconf_ops,
	.pctlops	= &rtk_pctrl_ops,
	.pmxops		= &rtk_pmx_ops,
};

static const struct of_device_id rtk_pinctrl_match[] = {
	{ .compatible = "realtek,rtk1619-pinctrl", .data = (void *)&rtk1619_pinctrl_data },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, rtk_pinctrl_match);

static int rtk_pinctrl_add_function(struct rtk_pinctrl *pctl, const char *name)
{
	struct rtk_pinctrl_function *func = pctl->functions;

	while (func->name) {
		if (strcmp(func->name, name) == 0) {
			func->ngroups++;
			return -EEXIST;
		}
		func++;
	}

	func->name = name;
	func->ngroups = 1;
	pctl->nfunctions++;

	return 0;
}

static int rtk_pinctrl_build_state(struct platform_device *pdev)
{
	struct rtk_pinctrl *pctl = platform_get_drvdata(pdev);
	int i;

	pctl->ngroups = pctl->desc->npins;

	pctl->groups = devm_kcalloc(&pdev->dev, pctl->ngroups,
				    sizeof(*pctl->groups), GFP_KERNEL);
	if (!pctl->groups)
		return -ENOMEM;

	for (i = 0; i < pctl->desc->npins; i++) {
		const struct rtk_desc_pin *pin = pctl->desc->pins + i;
		struct rtk_pinctrl_group *group = pctl->groups + i;

		group->name = pin->pin.name;
		group->pin = pin->pin.number;
	}

	pctl->functions = devm_kcalloc(&pdev->dev, pctl->desc->npins,
				       sizeof(*pctl->functions), GFP_KERNEL);
	if (!pctl->functions)
		return -ENOMEM;

	for (i = 0; i < pctl->desc->npins; i++) {
		const struct rtk_desc_pin *pin = pctl->desc->pins + i;
		struct rtk_desc_function *func = pin->functions;

		while (func && func->name) {
			rtk_pinctrl_add_function(pctl, func->name);
			func++;
		}
	}

	pctl->functions = devm_krealloc_array(&pdev->dev, pctl->functions,
					      pctl->nfunctions,
					      sizeof(*pctl->functions),
					      GFP_KERNEL | __GFP_ZERO);

	for (i = 0; i < pctl->desc->npins; i++) {
		const struct rtk_desc_pin *pin = pctl->desc->pins + i;
		struct rtk_desc_function *func = pin->functions;

		while (func && func->name) {
			struct rtk_pinctrl_function *func_item;
			const char **func_grp;

			func_item = rtk_pinctrl_find_function_by_name(pctl, func->name);
			if (!func_item)
				return -EINVAL;

			if (!func_item->groups) {
				func_item->groups = devm_kcalloc(&pdev->dev,
								 func_item->ngroups,
								 sizeof(*func_item->groups),
								 GFP_KERNEL);
				if (!func_item->groups)
					return -ENOMEM;
			}

			func_grp = func_item->groups;
			while (*func_grp)
				func_grp++;

			*func_grp = pin->pin.name;
			func++;
		}
	}

	return 0;
}

static int rtk_pinctrl_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *device;
	struct pinctrl_pin_desc *pins;
	struct rtk_pinctrl *pctl;
	int i, ret;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;
	platform_set_drvdata(pdev, pctl);

	pctl->iso_membase = devm_of_iomap(&pdev->dev, node, 0, NULL);
	if (IS_ERR(pctl->iso_membase)) {
		dev_err(&pdev->dev, "of_iomap iso_membase fail\n");
		return PTR_ERR(pctl->iso_membase);
	}

	pctl->misc_membase = devm_of_iomap(&pdev->dev, node, 1, NULL);
	if (IS_ERR(pctl->misc_membase)) {
		dev_err(&pdev->dev, "of_iomap misc_membase fail\n");
		return PTR_ERR(pctl->misc_membase);
	}

	device = of_match_device(rtk_pinctrl_match, &pdev->dev);
	if (!device) {
		dev_err(&pdev->dev, "of_match_device fail\n");
		return -ENODEV;
	}

	pctl->desc = (struct rtk_pinctrl_desc *)device->data;

	ret = rtk_pinctrl_build_state(pdev);
	if (ret) {
		dev_err(&pdev->dev, "rtk_pinctrl_build_state fail\n");
		return ret;
	}

	pins = devm_kcalloc(&pdev->dev, pctl->desc->npins, sizeof(*pins), GFP_KERNEL);
	if (!pins)
		return -ENOMEM;

	for (i = 0; i < pctl->desc->npins; i++)
		pins[i] = pctl->desc->pins[i].pin;

	rtk_pctrl_desc.name = dev_name(&pdev->dev);
	rtk_pctrl_desc.owner = THIS_MODULE;
	rtk_pctrl_desc.pins = pins;
	rtk_pctrl_desc.npins = pctl->desc->npins;
	pctl->dev = &pdev->dev;
	
	pctl->pctl_dev = devm_pinctrl_register(&pdev->dev, &rtk_pctrl_desc, pctl);
	if (IS_ERR(pctl->pctl_dev)) {
		dev_err(&pdev->dev, "register pinctrl driver fail\n");
		return PTR_ERR(pctl->pctl_dev);
	}

	dev_info(&pdev->dev, "Realtek RTD1619 pinctrl initialized\n");

	return 0;
}

static int __maybe_unused rtk_pinctrl_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pinctrl *pctl = platform_get_drvdata(pdev);
	void __iomem *addr;
	unsigned int i;

#ifdef CONFIG_RTK_XEN_SUPPORT
	if (xen_domain() && !xen_initial_domain())
		return 0;
#endif

	if (!pctl->reg_values) {
		pctl->reg_values = devm_kcalloc(dev, ARRAY_SIZE(pinmux_reg_list),
						sizeof(u32), GFP_KERNEL);
		if (!pctl->reg_values)
			return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(pinmux_reg_list); i++) {
		switch (pinmux_reg_list[i].reg_base) {
		case PMUX_BASE_ISO:
			addr = pctl->iso_membase + pinmux_reg_list[i].reg_offset;
			break;
		case PMUX_BASE_MISC:
			addr = pctl->misc_membase + pinmux_reg_list[i].reg_offset;
			break;
		default:
			dev_err(dev, "%s: Unknown reg_base\n", __func__);
			return -EINVAL;
		}
		pctl->reg_values[i] = readl(addr);
	}

	return 0;
}

static int __maybe_unused rtk_pinctrl_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pinctrl *pctl = platform_get_drvdata(pdev);
	void __iomem *addr;
	unsigned int i;

#ifdef CONFIG_RTK_XEN_SUPPORT
	if (xen_domain() && !xen_initial_domain())
		return 0;
#endif

	if (!pctl->reg_values)
		return 0;

	for (i = 0; i < ARRAY_SIZE(pinmux_reg_list); i++) {
		switch (pinmux_reg_list[i].reg_base) {
		case PMUX_BASE_ISO:
			addr = pctl->iso_membase + pinmux_reg_list[i].reg_offset;
			break;
		case PMUX_BASE_MISC:
			addr = pctl->misc_membase + pinmux_reg_list[i].reg_offset;
			break;
		default:
			dev_err(dev, "%s: Unknown reg_base\n", __func__);
			return -EINVAL;
		}
		writel(pctl->reg_values[i], addr);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(rtk_pinctrl_pm_ops, rtk_pinctrl_suspend, rtk_pinctrl_resume);

static struct platform_driver rtk_pinctrl_driver = {
	.probe = rtk_pinctrl_probe,
	.driver = {
		.name = "rtk-pinctrl",
		.of_match_table = rtk_pinctrl_match,
		.pm = &rtk_pinctrl_pm_ops,
	},
};

static int __init rtk_pinctrl_init(void)
{
	return platform_driver_register(&rtk_pinctrl_driver);
}
postcore_initcall(rtk_pinctrl_init);

MODULE_DESCRIPTION("Realtek RTD1619 pinctrl driver");
MODULE_LICENSE("GPL");