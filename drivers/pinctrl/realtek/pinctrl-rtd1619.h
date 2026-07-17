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

#ifndef __PINCTRL_RTD1619_H
#define __PINCTRL_RTD1619_H

#include <linux/kernel.h>
#include <linux/pinctrl/pinctrl.h>

#define P_ISO_BASE			0

#define RTK_PCONF_SCHM			2
#define RTK_PCONF_PULEN			0
#define RTK_PCONF_PULSEL		1
#define RTK_PCONF_CURR			-1

/* ISO muxpad0 */
#define RTK_PINCTRL_PIN_gpio_0			PINCTRL_PIN(P_ISO_BASE + 0, "gpio_0")
#define RTK_PINCTRL_PIN_gpio_1			PINCTRL_PIN(P_ISO_BASE + 1, "gpio_1")
#define RTK_PINCTRL_PIN_gpio_2			PINCTRL_PIN(P_ISO_BASE + 2, "gpio_2")
#define RTK_PINCTRL_PIN_gpio_3			PINCTRL_PIN(P_ISO_BASE + 3, "gpio_3")
#define RTK_PINCTRL_PIN_gpio_4			PINCTRL_PIN(P_ISO_BASE + 4, "gpio_4")
#define RTK_PINCTRL_PIN_gpio_5			PINCTRL_PIN(P_ISO_BASE + 5, "gpio_5")
#define RTK_PINCTRL_PIN_gpio_6			PINCTRL_PIN(P_ISO_BASE + 6, "gpio_6")
#define RTK_PINCTRL_PIN_gpio_7			PINCTRL_PIN(P_ISO_BASE + 7, "gpio_7")
#define RTK_PINCTRL_PIN_gpio_8			PINCTRL_PIN(P_ISO_BASE + 8, "gpio_8")
#define RTK_PINCTRL_PIN_gpio_9			PINCTRL_PIN(P_ISO_BASE + 9, "gpio_9")
#define RTK_PINCTRL_PIN_gpio_10			PINCTRL_PIN(P_ISO_BASE + 10, "gpio_10")
#define RTK_PINCTRL_PIN_gpio_11			PINCTRL_PIN(P_ISO_BASE + 11, "gpio_11")
#define RTK_PINCTRL_PIN_gpio_12			PINCTRL_PIN(P_ISO_BASE + 12, "gpio_12")
#define RTK_PINCTRL_PIN_gpio_13			PINCTRL_PIN(P_ISO_BASE + 13, "gpio_13")
#define RTK_PINCTRL_PIN_gpio_14			PINCTRL_PIN(P_ISO_BASE + 14, "gpio_14")
#define RTK_PINCTRL_PIN_gpio_15			PINCTRL_PIN(P_ISO_BASE + 15, "gpio_15")
#define RTK_PINCTRL_PIN_gpio_16			PINCTRL_PIN(P_ISO_BASE + 16, "gpio_16")
#define RTK_PINCTRL_PIN_gpio_17			PINCTRL_PIN(P_ISO_BASE + 17, "gpio_17")

/* ISO muxpad1 */
#define RTK_PINCTRL_PIN_gpio_18			PINCTRL_PIN(P_ISO_BASE + 18, "gpio_18")
#define RTK_PINCTRL_PIN_gpio_19			PINCTRL_PIN(P_ISO_BASE + 19, "gpio_19")
#define RTK_PINCTRL_PIN_gpio_20			PINCTRL_PIN(P_ISO_BASE + 20, "gpio_20")
#define RTK_PINCTRL_PIN_gpio_21			PINCTRL_PIN(P_ISO_BASE + 21, "gpio_21")
#define RTK_PINCTRL_PIN_gpio_22			PINCTRL_PIN(P_ISO_BASE + 22, "gpio_22")
#define RTK_PINCTRL_PIN_gpio_23			PINCTRL_PIN(P_ISO_BASE + 23, "gpio_23")
#define RTK_PINCTRL_PIN_gpio_25			PINCTRL_PIN(P_ISO_BASE + 25, "gpio_25")
#define RTK_PINCTRL_PIN_gpio_26			PINCTRL_PIN(P_ISO_BASE + 26, "gpio_26")
#define RTK_PINCTRL_PIN_gpio_27			PINCTRL_PIN(P_ISO_BASE + 27, "gpio_27")
#define RTK_PINCTRL_PIN_gpio_28			PINCTRL_PIN(P_ISO_BASE + 28, "gpio_28")
#define RTK_PINCTRL_PIN_gpio_29			PINCTRL_PIN(P_ISO_BASE + 29, "gpio_29")
#define RTK_PINCTRL_PIN_gpio_30			PINCTRL_PIN(P_ISO_BASE + 30, "gpio_30")
#define RTK_PINCTRL_PIN_gpio_31			PINCTRL_PIN(P_ISO_BASE + 31, "gpio_31")
#define RTK_PINCTRL_PIN_gpio_34			PINCTRL_PIN(P_ISO_BASE + 34, "gpio_34")
#define RTK_PINCTRL_PIN_gpio_35			PINCTRL_PIN(P_ISO_BASE + 35, "gpio_35")

/* ISO muxpad2 */
#define RTK_PINCTRL_PIN_gpio_40			PINCTRL_PIN(P_ISO_BASE + 40, "gpio_40")
#define RTK_PINCTRL_PIN_gpio_41			PINCTRL_PIN(P_ISO_BASE + 41, "gpio_41")
#define RTK_PINCTRL_PIN_gpio_42			PINCTRL_PIN(P_ISO_BASE + 42, "gpio_42")
#define RTK_PINCTRL_PIN_gpio_43			PINCTRL_PIN(P_ISO_BASE + 43, "gpio_43")
#define RTK_PINCTRL_PIN_gpio_44			PINCTRL_PIN(P_ISO_BASE + 44, "gpio_44")
#define RTK_PINCTRL_PIN_gpio_45			PINCTRL_PIN(P_ISO_BASE + 45, "gpio_45")
#define RTK_PINCTRL_PIN_gpio_46			PINCTRL_PIN(P_ISO_BASE + 46, "gpio_46")
#define RTK_PINCTRL_PIN_gpio_47			PINCTRL_PIN(P_ISO_BASE + 47, "gpio_47")
#define RTK_PINCTRL_PIN_gpio_48			PINCTRL_PIN(P_ISO_BASE + 48, "gpio_48")
#define RTK_PINCTRL_PIN_gpio_49			PINCTRL_PIN(P_ISO_BASE + 49, "gpio_49")
#define RTK_PINCTRL_PIN_gpio_50			PINCTRL_PIN(P_ISO_BASE + 50, "gpio_50")
#define RTK_PINCTRL_PIN_gpio_51			PINCTRL_PIN(P_ISO_BASE + 51, "gpio_51")

/* ISO muxpad3 */
#define RTK_PINCTRL_PIN_gpio_52			PINCTRL_PIN(P_ISO_BASE + 52, "gpio_52")
#define RTK_PINCTRL_PIN_ir_rx			PINCTRL_PIN(P_ISO_BASE + 53, "ir_rx")
#define RTK_PINCTRL_PIN_ur0_rx			PINCTRL_PIN(P_ISO_BASE + 54, "ur0_rx")
#define RTK_PINCTRL_PIN_ur0_tx			PINCTRL_PIN(P_ISO_BASE + 55, "ur0_tx")
#define RTK_PINCTRL_PIN_usb_cc1			PINCTRL_PIN(P_ISO_BASE + 56, "usb_cc1")
#define RTK_PINCTRL_PIN_gpio_57			PINCTRL_PIN(P_ISO_BASE + 57, "gpio_57")
#define RTK_PINCTRL_PIN_gpio_58			PINCTRL_PIN(P_ISO_BASE + 58, "gpio_58")
#define RTK_PINCTRL_PIN_gpio_59			PINCTRL_PIN(P_ISO_BASE + 59, "gpio_59")
#define RTK_PINCTRL_PIN_gpio_60			PINCTRL_PIN(P_ISO_BASE + 60, "gpio_60")
#define RTK_PINCTRL_PIN_gpio_61			PINCTRL_PIN(P_ISO_BASE + 61, "gpio_61")
#define RTK_PINCTRL_PIN_gpio_62			PINCTRL_PIN(P_ISO_BASE + 62, "gpio_62")
#define RTK_PINCTRL_PIN_gpio_63			PINCTRL_PIN(P_ISO_BASE + 63, "gpio_63")
#define RTK_PINCTRL_PIN_gpio_64			PINCTRL_PIN(P_ISO_BASE + 64, "gpio_64")
#define RTK_PINCTRL_PIN_gpio_65			PINCTRL_PIN(P_ISO_BASE + 65, "gpio_65")
#define RTK_PINCTRL_PIN_gpio_66			PINCTRL_PIN(P_ISO_BASE + 66, "gpio_66")
#define RTK_PINCTRL_PIN_gpio_67			PINCTRL_PIN(P_ISO_BASE + 67, "gpio_67")
#define RTK_PINCTRL_PIN_gpio_68			PINCTRL_PIN(P_ISO_BASE + 68, "gpio_68")
#define RTK_PINCTRL_PIN_usb_cc2			PINCTRL_PIN(P_ISO_BASE + 69, "usb_cc2")
#define RTK_PINCTRL_PIN_gpio_85			PINCTRL_PIN(P_ISO_BASE + 85, "gpio_85")

/* ISO muxpad6 */
#define RTK_PINCTRL_PIN_ur2_loc			PINCTRL_PIN(P_ISO_BASE + 86, "ur2_loc")
#define RTK_PINCTRL_PIN_gspi_loc		PINCTRL_PIN(P_ISO_BASE + 87, "gspi_loc")
#define RTK_PINCTRL_PIN_sdio_loc		PINCTRL_PIN(P_ISO_BASE + 88, "sdio_loc")
#define RTK_PINCTRL_PIN_hi_loc			PINCTRL_PIN(P_ISO_BASE + 89, "hi_loc")
#define RTK_PINCTRL_PIN_hi_width		PINCTRL_PIN(P_ISO_BASE + 90, "hi_width")
#define RTK_PINCTRL_PIN_debug_p2s_enable	PINCTRL_PIN(P_ISO_BASE + 91, "debug_p2s_enable")
#define RTK_PINCTRL_PIN_sf_en			PINCTRL_PIN(P_ISO_BASE + 92, "sf_en")
#define RTK_PINCTRL_PIN_arm_trace_dbg_en	PINCTRL_PIN(P_ISO_BASE + 93, "arm_trace_dbg_en")
#define RTK_PINCTRL_PIN_pwm_01_open_drain_en_loc0	PINCTRL_PIN(P_ISO_BASE + 94, "pwm_01_open_drain_en_loc0")
#define RTK_PINCTRL_PIN_pwm_23_open_drain_en_loc0	PINCTRL_PIN(P_ISO_BASE + 95, "pwm_23_open_drain_en_loc0")
#define RTK_PINCTRL_PIN_pwm_01_open_drain_en_loc1	PINCTRL_PIN(P_ISO_BASE + 96, "pwm_01_open_drain_en_loc1")
#define RTK_PINCTRL_PIN_pwm_23_open_drain_en_loc1	PINCTRL_PIN(P_ISO_BASE + 97, "pwm_23_open_drain_en_loc1")
#define RTK_PINCTRL_PIN_ejtag_avcpu_loc		PINCTRL_PIN(P_ISO_BASE + 98, "ejtag_avcpu_loc")
#define RTK_PINCTRL_PIN_ejtag_scpu_loc		PINCTRL_PIN(P_ISO_BASE + 99, "ejtag_scpu_loc")
#define RTK_PINCTRL_PIN_i2c_tg			PINCTRL_PIN(P_ISO_BASE + 100, "i2c_tg")
#define RTK_PINCTRL_PIN_dmic_loc		PINCTRL_PIN(P_ISO_BASE + 101, "dmic_loc")

/* MISC muxpad0 */
#define RTK_PINCTRL_PIN_demod_agc		PINCTRL_PIN(P_ISO_BASE + 24, "demod_agc")
#define RTK_PINCTRL_PIN_emmc_rst		PINCTRL_PIN(P_ISO_BASE + 102, "emmc_rst")
#define RTK_PINCTRL_PIN_emmc_dd_sb		PINCTRL_PIN(P_ISO_BASE + 103, "emmc_dd_sb")
#define RTK_PINCTRL_PIN_emmc_clk		PINCTRL_PIN(P_ISO_BASE + 104, "emmc_clk")
#define RTK_PINCTRL_PIN_emmc_cmd		PINCTRL_PIN(P_ISO_BASE + 105, "emmc_cmd")
#define RTK_PINCTRL_PIN_emmc_data_0		PINCTRL_PIN(P_ISO_BASE + 106, "emmc_data_0")
#define RTK_PINCTRL_PIN_emmc_data_1		PINCTRL_PIN(P_ISO_BASE + 107, "emmc_data_1")
#define RTK_PINCTRL_PIN_emmc_data_2		PINCTRL_PIN(P_ISO_BASE + 108, "emmc_data_2")
#define RTK_PINCTRL_PIN_emmc_data_3		PINCTRL_PIN(P_ISO_BASE + 109, "emmc_data_3")
#define RTK_PINCTRL_PIN_emmc_data_4		PINCTRL_PIN(P_ISO_BASE + 110, "emmc_data_4")
#define RTK_PINCTRL_PIN_emmc_data_5		PINCTRL_PIN(P_ISO_BASE + 111, "emmc_data_5")
#define RTK_PINCTRL_PIN_emmc_data_6		PINCTRL_PIN(P_ISO_BASE + 112, "emmc_data_6")
#define RTK_PINCTRL_PIN_emmc_data_7		PINCTRL_PIN(P_ISO_BASE + 113, "emmc_data_7")
#define RTK_PINCTRL_PIN_hif_data_0		PINCTRL_PIN(P_ISO_BASE + 81, "hif_data_0")
#define RTK_PINCTRL_PIN_hif_rdy			PINCTRL_PIN(P_ISO_BASE + 82, "hif_rdy")
#define RTK_PINCTRL_PIN_hif_clk			PINCTRL_PIN(P_ISO_BASE + 83, "hif_clk")

/* MISC muxpad1 */
#define RTK_PINCTRL_PIN_hif_en			PINCTRL_PIN(P_ISO_BASE + 84, "hif_en")
#define RTK_PINCTRL_PIN_sd3_cmd			PINCTRL_PIN(P_ISO_BASE + 32, "sd3_cmd")
#define RTK_PINCTRL_PIN_sd3_clk			PINCTRL_PIN(P_ISO_BASE + 33, "sd3_clk")
#define RTK_PINCTRL_PIN_sd3_data_0		PINCTRL_PIN(P_ISO_BASE + 36, "sd3_data_0")
#define RTK_PINCTRL_PIN_sd3_data_1		PINCTRL_PIN(P_ISO_BASE + 37, "sd3_data_1")
#define RTK_PINCTRL_PIN_sd3_data_2		PINCTRL_PIN(P_ISO_BASE + 38, "sd3_data_2")
#define RTK_PINCTRL_PIN_sd3_data_3		PINCTRL_PIN(P_ISO_BASE + 39, "sd3_data_3")
#define RTK_PINCTRL_PIN_tp_sync			PINCTRL_PIN(P_ISO_BASE + 70, "tp_sync")
#define RTK_PINCTRL_PIN_tp_valid		PINCTRL_PIN(P_ISO_BASE + 71, "tp_valid")
#define RTK_PINCTRL_PIN_tp_clk			PINCTRL_PIN(P_ISO_BASE + 72, "tp_clk")
#define RTK_PINCTRL_PIN_tp_data_0		PINCTRL_PIN(P_ISO_BASE + 73, "tp_data_0")

/* MISC muxpad2 */
#define RTK_PINCTRL_PIN_tp_data_1		PINCTRL_PIN(P_ISO_BASE + 74, "tp_data_1")
#define RTK_PINCTRL_PIN_tp_data_2		PINCTRL_PIN(P_ISO_BASE + 75, "tp_data_2")
#define RTK_PINCTRL_PIN_tp_data_3		PINCTRL_PIN(P_ISO_BASE + 76, "tp_data_3")
#define RTK_PINCTRL_PIN_tp_data_4		PINCTRL_PIN(P_ISO_BASE + 77, "tp_data_4")
#define RTK_PINCTRL_PIN_tp_data_5		PINCTRL_PIN(P_ISO_BASE + 78, "tp_data_5")
#define RTK_PINCTRL_PIN_tp_data_6		PINCTRL_PIN(P_ISO_BASE + 79, "tp_data_6")
#define RTK_PINCTRL_PIN_tp_data_7		PINCTRL_PIN(P_ISO_BASE + 80, "tp_data_7")

enum rtk_pmux_base_type {
	PMUX_BASE_ISO,
	PMUX_BASE_MISC
};

struct rtk_pin_regmap {
	u8 pmux_base;
	u16 pmux_regoff;
	u16 pmux_regbit;
	u16 pmux_regbitmsk;
	u16 pcof_regoff;
	u16 pcof_regbit;
	u16 pcof_cur_strgh; /* 0: 2&4mA, 1: 4&8mA */
};

struct rtk_pinmux_reg {
	u8 reg_base;
	u16 reg_offset;
};

#define PADDRI_4_8		1
#define PADDRI_2_4		0
#define PADDRI_UNSUPPORT	0xFFFF

#define PCOF_UNSUPPORT		0xFFFF
#define PMUX_UNSUPPORT		0xFF

struct rtk_desc_function {
	const char	*name;
	u8		muxval;
};

struct rtk_desc_pin {
	struct pinctrl_pin_desc		pin;
	struct rtk_desc_function	*functions;
};

struct rtk_pinctrl_desc {
	const struct rtk_desc_pin	*pins;
	int				npins;
	struct pinctrl_gpio_range	*ranges;
	int				nranges;
};

struct rtk_pinctrl_function {
	const char	*name;
	const char	**groups;
	unsigned int	ngroups;
};

struct rtk_pinctrl_group {
	const char	*name;
	unsigned long	config;
	unsigned int	pin;
};

struct rtk_pinctrl {
	void __iomem			*iso_membase;
	void __iomem			*misc_membase;
	struct rtk_pinctrl_desc		*desc;
	struct device			*dev;
	struct rtk_pinctrl_function	*functions;
	unsigned int			nfunctions;
	struct rtk_pinctrl_group	*groups;
	unsigned int			ngroups;
	struct pinctrl_dev		*pctl_dev;
	u32				*reg_values;
};

#define RTK_PIN(_pin, ...)					\
	{							\
		.pin = _pin,					\
		.functions = (struct rtk_desc_function[]){	\
			__VA_ARGS__, { } },			\
	}

#define RTK_FUNCTION(_val, _name)				\
	{							\
		.name = _name,					\
		.muxval = _val,					\
	}


void gpio_pinctrl_control(struct gpio_chip *gc, int gpio, int direction, int value);

#endif /* __PINCTRL_RTD1619_H */