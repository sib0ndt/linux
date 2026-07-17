/*
 * Realtek RTD129x/RTD16xx Clock Gate Controller Driver
 *
 * Rewritten for Mainline Linux compatibility.
 * Replaces legacy proprietary Realtek CCF implementations.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

struct rtk_cgc {
	struct device *dev;
	void __iomem *reg;
	bool has_write_en;
	u32 pm_mask;
	u32 pm_data;
	spinlock_t lock;
	struct clk_hw_onecell_data *hw_data;
};

struct rtk_gate {
	struct clk_hw hw;
	void __iomem *reg;
	u8 bit_idx;
	bool has_write_en;
	spinlock_t *lock;
};

#define to_rtk_gate(_hw) container_of(_hw, struct rtk_gate, hw)

static int rtk_gate_enable(struct clk_hw *hw)
{
	struct rtk_gate *gate = to_rtk_gate(hw);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(gate->lock, flags);
	
	if (gate->has_write_en) {
		/* Bit N is enable, Bit N+1 is write mask */
		val = BIT(gate->bit_idx) | BIT(gate->bit_idx + 1);
		writel(val, gate->reg);
	} else {
		val = readl(gate->reg);
		val |= BIT(gate->bit_idx);
		writel(val, gate->reg);
	}
	
	spin_unlock_irqrestore(gate->lock, flags);
	return 0;
}

static void rtk_gate_disable(struct clk_hw *hw)
{
	struct rtk_gate *gate = to_rtk_gate(hw);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(gate->lock, flags);
	
	if (gate->has_write_en) {
		/* Write enable bit high (N+1), data bit low (N) */
		val = BIT(gate->bit_idx + 1);
		writel(val, gate->reg);
	} else {
		val = readl(gate->reg);
		val &= ~BIT(gate->bit_idx);
		writel(val, gate->reg);
	}
	
	spin_unlock_irqrestore(gate->lock, flags);
}

static int rtk_gate_is_enabled(struct clk_hw *hw)
{
	struct rtk_gate *gate = to_rtk_gate(hw);
	u32 val = readl(gate->reg);

	return !!(val & BIT(gate->bit_idx));
}

static const struct clk_ops rtk_gate_ops = {
	.enable = rtk_gate_enable,
	.disable = rtk_gate_disable,
	.is_enabled = rtk_gate_is_enabled,
};

static int rtk_cgc_init_clocks(struct platform_device *pdev, struct rtk_cgc *cgc)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int num_names, max_clks;
	int i, j;

	num_names = of_property_count_strings(np, "clock-output-names");
	if (num_names < 0)
		return num_names;

	/*
	 * To maintain DT ABI compatibility with legacy Realtek bindings,
	 * if 'has-write-en' is present, the clock indices requested by DT
	 * are shifted (multiplied by 2).
	 */
	max_clks = cgc->has_write_en ? num_names * 2 : num_names;

	cgc->hw_data = devm_kzalloc(dev, struct_size(cgc->hw_data, hws, max_clks), GFP_KERNEL);
	if (!cgc->hw_data)
		return -ENOMEM;

	cgc->hw_data->num = max_clks;
	
	/* Inisialisasi slot kosong agar driver tidak crash saat mencari index */
	for (j = 0; j < max_clks; j++)
		cgc->hw_data->hws[j] = ERR_PTR(-ENOENT);

	for (i = 0; i < num_names; i++) {
		const char *name;
		struct rtk_gate *gate;
		struct clk_init_data init = { 0 };
		int idx = cgc->has_write_en ? i * 2 : i;
		int ret;

		ret = of_property_read_string_index(np, "clock-output-names", i, &name);
		if (ret || !name || !name[0])
			continue;

		gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
		if (!gate)
			return -ENOMEM;

		init.name = name;
		init.ops = &rtk_gate_ops;
		init.flags = 0;

		gate->reg = cgc->reg;
		gate->bit_idx = idx;
		gate->has_write_en = cgc->has_write_en;
		gate->lock = &cgc->lock;
		gate->hw.init = &init;

		ret = devm_clk_hw_register(dev, &gate->hw);
		if (ret) {
			dev_err(dev, "failed to register clock %s: %d\n", name, ret);
			return ret;
		}

		cgc->hw_data->hws[idx] = &gate->hw;
		cgc->pm_mask |= BIT(idx);
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, cgc->hw_data);
}

static int rtk_cgc_suspend(struct device *dev)
{
	struct rtk_cgc *cgc = dev_get_drvdata(dev);

	cgc->pm_data = readl(cgc->reg);
	return 0;
}

static int rtk_cgc_resume(struct device *dev)
{
	struct rtk_cgc *cgc = dev_get_drvdata(dev);

	if (cgc->has_write_en)
		writel(cgc->pm_data | (cgc->pm_mask << 1), cgc->reg);
	else
		writel(cgc->pm_data, cgc->reg);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(rtk_cgc_pm_ops, rtk_cgc_suspend, rtk_cgc_resume);

static int rtk_cgc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_cgc *cgc;
	int ret;

	cgc = devm_kzalloc(dev, sizeof(*cgc), GFP_KERNEL);
	if (!cgc)
		return -ENOMEM;

	cgc->dev = dev;
	spin_lock_init(&cgc->lock);

	cgc->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cgc->reg))
		return PTR_ERR(cgc->reg);

	if (of_property_read_bool(dev->of_node, "has-write-en"))
		cgc->has_write_en = true;

	platform_set_drvdata(pdev, cgc);

	ret = rtk_cgc_init_clocks(pdev, cgc);
	if (ret)
		return ret;

	dev_info(dev, "Realtek CGC initialized\n");
	return 0;
}

static const struct of_device_id rtk_cgc_match[] = {
	{ .compatible = "realtek,clock-gate-controller" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtk_cgc_match);

static struct platform_driver rtk_cgc_driver = {
	.probe = rtk_cgc_probe,
	.driver = {
		.name = "rtk-cgc",
		.of_match_table = rtk_cgc_match,
		.pm = pm_sleep_ptr(&rtk_cgc_pm_ops),
	},
};

static int __init rtk_cgc_init(void)
{
	return platform_driver_register(&rtk_cgc_driver);
}
core_initcall(rtk_cgc_init);

MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_DESCRIPTION("Realtek RTD SoC Clock Gate Controller Driver");
MODULE_LICENSE("GPL v2");