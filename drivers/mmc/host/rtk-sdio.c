/*
 * Realtek SDIO driver for RTD16xx Architecture
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/reset.h>
#include <soc/realtek/rtk_chip.h>
#include "sdhci.h"
#include "sdhci-pltfm.h"
#include "rtk-sdio.h"

#define SD_INTERFACE_SDIO_1619   /* pad config needed for SDIO pins on this board */

//#define RTK_DEBUG
#ifdef RTK_DEBUG
#define RTK_debug(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define RTK_debug(fmt, ...)
#endif

#define DRIVER_NAME    "realtek-sdio"
#define BANNER         "Realtek RTD1619 SDIO Host Driver"
#define SDIO_CLKEN_REGOFF    0x0C
#define SDIO_CLKEN_REGBIT    30

#define MAX_PHASE    31
#define TUNING_CNT   3

void set_SDIO_version(int version);
int get_SDIO_version(void);
void rtk_register_set(void);
void rtk_sdhci_platform_init(void);
void rtk_sdhci_close_clk(void);

static void __iomem *sdio_membase;
static void __iomem *crt_membase;
struct gpio_desc *sdio_gpio_23 = NULL;
bool SDIO_module = false;
bool clock_enable = true;
bool SDIO_card = true;
int SDIO_version = 3;
struct sdhci_host *G_host;

unsigned int drive_temp = 0;

struct reset_control *rstc_sdio;
struct clk * clk_en_sdio;
struct clk * clk_en_sdio_ip;

struct sdhci_rtk_sdio_data {
    const struct sdhci_pltfm_data *pdata;
};

struct timing_phase_path {
    int start;
    int end;
    int mid;
    int len;
};

void set_SDIO_version(int version)
{
    SDIO_version = version;
}
EXPORT_SYMBOL(set_SDIO_version);

int get_SDIO_version(void)
{
    return SDIO_version;
}
EXPORT_SYMBOL(get_SDIO_version);

#ifdef CONFIG_RTK_SDIO_CLK_ADJUST
void rtk_adjust_clock_for_wifi(unsigned int channel, unsigned int offset, unsigned int bwmode)
{
    wifi_sdio_clk_tbl rtk_wifi_sdio_clk_tbl[] = {
        { 1, 1, 0, {{0x7757893, 0x554388}, {0, 0}}},
        { 2, 1, 0, {{0x4fd7893, 0x544388}, {0, 0}}},
        { 3, 1, 0, {{0x4bd7893, 0x564388}, {0, 0}}},
        { 4, 1, 0, {{0x57d7893, 0x564388}, {0, 0}}},
        { 5, 1, 1, {{0x73d7893, 0x544388}, {0x7757893, 0x554388}}},
        { 6, 1, 1, {{0x7fd7893, 0x544388}, {0x4fd7893, 0x544388}}},
        { 7, 1, 1, {{0x4757893, 0x554388}, {0x4bd7893, 0x564388}}},
        { 8, 1, 1, {{0x53d7893, 0x554388}, {0x57d7893, 0x564388}}},
        { 9, 1, 1, {{0x5f57893, 0x554388}, {0x73d7893, 0x544388}}},
        {10, 0, 1, {{0, 0}, {0x7fd7893, 0x544388}}},
        {11, 0, 1, {{0, 0}, {0x4757893, 0x554388}}},
        {12, 0, 1, {{0, 0}, {0x53d7893, 0x554388}}},
        {13, 0, 1, {{0, 0}, {0x5f57893, 0x554388}}},
    };

    int i, ret = -1, idx = 0;
    u32 value = 0;

    printk("%s: channel=%d, offset=%d, bwmode=%d\n", __func__, channel, offset, bwmode);

    for (i = 0; i < ARRAY_SIZE(rtk_wifi_sdio_clk_tbl); i++) {
        if (channel == rtk_wifi_sdio_clk_tbl[i].channel) {
            if ((offset == PRI_CH_OFFSET_LOWER && rtk_wifi_sdio_clk_tbl[i].lower)
                || (offset == PRI_CH_OFFSET_DONT_CARE && rtk_wifi_sdio_clk_tbl[i].lower))
                idx = 0;
            else if ((offset == PRI_CH_OFFSET_UPPER && rtk_wifi_sdio_clk_tbl[i].upper)
                || (offset == PRI_CH_OFFSET_DONT_CARE && rtk_wifi_sdio_clk_tbl[i].upper))
                idx = 1;
            else
                break;

            value = readl(crt_membase + 0x10a58);
            if (value & 0x40000000)
                writel(value & ~(0x40000000), crt_membase + 0x10a58);
            else
                writel(value | 0x40000000, crt_membase + 0x10a58);
            writel(rtk_wifi_sdio_clk_tbl[i].map[idx].reg1a4, crt_membase + 0x01A4);
            mdelay(2);
            writel(rtk_wifi_sdio_clk_tbl[i].map[idx].reg1a8, crt_membase + 0x01A8);
            mdelay(2);
            writel(value, crt_membase + 0x10a58);

            ret = 0;
            break;
        }
    }

    if (ret)
        printk("invalid settings: ch=%u, offset=%u, bwmode=%u\n", channel, offset, bwmode);
}
EXPORT_SYMBOL_GPL(rtk_adjust_clock_for_wifi);
#endif

static void rtk_sdhci_buswidth(struct sdhci_host *host, int bus_width)
{
    u32 ctrl = 0;

    ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
    if ((host->mmc->caps & MMC_CAP_8_BIT_DATA) &&
        (bus_width == MMC_BUS_WIDTH_8)) {
        ctrl &= ~SDHCI_CTRL_4BITBUS;
        ctrl |= SDHCI_CTRL_8BITBUS;
    } else {
        ctrl &= ~SDHCI_CTRL_8BITBUS;
        if (bus_width == MMC_BUS_WIDTH_4)
            ctrl |= SDHCI_CTRL_4BITBUS;
        else
            ctrl &= ~SDHCI_CTRL_4BITBUS;
    }
    sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static void rtk_sdhci_reset(struct sdhci_host *host, u8 mask)
{
    unsigned long timeout = 0;

    sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);
    if (mask & SDHCI_RESET_ALL)
        host->clock = 0;

    /* Wait max 100 ms */
    timeout = 100;

    /* hw clears the bit when it's done */
    while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
        if (timeout == 0) {
            pr_err("%s: Reset 0x%x never completed.\n",
                mmc_hostname(host->mmc), (int)mask);
            return;
        }
        timeout--;
        mdelay(1);
    }
}

static void rtk_sdhci_uhs_signaling(struct sdhci_host *host, unsigned timing)
{
    u16 ctrl_2 = 0;

    sdhci_writew(host, sdhci_readw(host, SDHCI_CLOCK_CONTROL) | SDHCI_CLOCK_CARD_EN, SDHCI_CLOCK_CONTROL);

    ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
    /* Select Bus Speed Mode for host */
    ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
    if ((timing == MMC_TIMING_MMC_HS200) ||
        (timing == MMC_TIMING_UHS_SDR104))
        ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
    else if (timing == MMC_TIMING_UHS_SDR12)
        ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
    else if (timing == MMC_TIMING_UHS_SDR25)
        ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
    else if (timing == MMC_TIMING_UHS_SDR50)
        ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
    else if ((timing == MMC_TIMING_UHS_DDR50) ||
         (timing == MMC_TIMING_MMC_DDR52))
        ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
    else if (timing == MMC_TIMING_MMC_HS400)
        ctrl_2 |= SDHCI_CTRL_HS400; /* Non-standard */

    sdhci_writew(host, ctrl_2 | SDHCI_CTRL_VDD_180, SDHCI_HOST_CONTROL2);
}

static u16 rtk_sdhci_get_preset_value(struct sdhci_host *host)
{
    u16 preset = 0;

    switch (host->timing) {
    case MMC_TIMING_UHS_SDR12:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR12);
        break;
    case MMC_TIMING_UHS_SDR25:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR25);
        break;
    case MMC_TIMING_UHS_SDR50:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR50);
        break;
    case MMC_TIMING_UHS_SDR104:
    case MMC_TIMING_MMC_HS200:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR104);
        break;
    case MMC_TIMING_UHS_DDR50:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_DDR50);
        break;
    case MMC_TIMING_MMC_HS400:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_HS400);
        break;
    default:
        pr_warn("%s: Invalid UHS-I mode selected\n",
            mmc_hostname(host->mmc));
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR12);
        break;
    }
    return preset;
}

static void rtk_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
    int div = 0;
    int real_div = div, clk_mul = 1;
    u16 clk = 0;
    u16 pre_val = 0;
    unsigned long timeout = 0;

    if (clock && clock == host->clock)
        return;

    host->mmc->actual_clock = 0;

    sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

    if (clock == 0)
        goto out;

    if (host->version >= SDHCI_SPEC_300) {
        if (sdhci_readw(host, SDHCI_HOST_CONTROL2) & SDHCI_CTRL_PRESET_VAL_ENABLE) {
            clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
            pre_val = rtk_sdhci_get_preset_value(host);
            
            div = (pre_val & SDHCI_PRESET_SDCLK_FREQ_MASK);
            if (host->clk_mul && (pre_val & SDHCI_PRESET_CLKGEN_SEL)) {
                clk = SDHCI_PROG_CLOCK_MODE;
                real_div = div + 1;
                clk_mul = host->clk_mul;
            } else {
                real_div = max_t(int, 1, div << 1);
            }
            goto clock_set;
        }

        if (host->clk_mul) {
            for (div = 1; div <= 1024; div++) {
                if ((host->max_clk * host->clk_mul / div) <= clock)
                    break;
            }
            clk = SDHCI_PROG_CLOCK_MODE;
            real_div = div;
            clk_mul = host->clk_mul;
            div--;
        } else {
            if (host->max_clk <= clock)
                div = 1;
            else {
                for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2) {
                    if ((host->max_clk / div) <= clock)
                        break;
                }
            }
            real_div = div;
            div >>= 1;
        }
    } else {
        for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
            if ((host->max_clk / div) <= clock)
                break;
        }
        real_div = div;
        div >>= 1;
    }

clock_set:
    if (real_div)
        host->mmc->actual_clock = (host->max_clk * clk_mul) / real_div;

    clk |= (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
    clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN) << SDHCI_DIVIDER_HI_SHIFT;
    clk |= SDHCI_CLOCK_INT_EN;
    sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

    timeout = 20;
    while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL)) & SDHCI_CLOCK_INT_STABLE)) {
        if (timeout == 0) {
            pr_err("%s: Internal clock never stabilised.\n", mmc_hostname(host->mmc));
            return;
        }
        timeout--;
        mdelay(1);
    }

    clk |= SDHCI_CLOCK_CARD_EN;
    sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

    if (readl(host->ioaddr + SDHCI_CLOCK_CONTROL) == 0xe8c7) {
        SDIO_card = false;
    }
out:
    host->clock = clock;
}

void rtk_register_set(void)
{
    if (get_SDIO_version() == 4) {
        printk(KERN_ERR "SDIO SDR104 mode\n");
        writel(0x1d, crt_membase + 0x10a58); 
        
        writel(0x00000006, crt_membase + 0x01AC);
        writel(0x04517893, crt_membase + 0x01A4);
        writel(0x00ae4388, crt_membase + 0x01A8); 
        mdelay(2);
        writel(0x00000007, crt_membase + 0x01AC);
        udelay(200);
    }
    else if (get_SDIO_version() == 3) {
        writel((readl(crt_membase + 0x1A0) & ~0x000000F8) | (0x1d << 3), crt_membase + 0x1A0);
        
        writel((readl(crt_membase + 0x4E024) & 0xf) | 0xAF75EEB0, crt_membase + 0x4E024);
        writel(0x5EEBDD7B, crt_membase + 0x4E028);
        writel((readl(crt_membase + 0x4E02c) & 0xffffffc0) | 0x37, crt_membase + 0x4E02c);

        writel(0x00000000, crt_membase + 0x010A40);

        if (readl(crt_membase + 0x1a204) == 0x00) {
            printk(KERN_ERR "SDIO 3.0 A00 version\n");
        } else {
            printk(KERN_ERR "SDIO 3.0 A01 version\n");
        }
    }
    else if (get_SDIO_version() == 2) {
        writel(0x00000003, crt_membase + 0x01A0);
        if (readl(crt_membase + 0x1a204) == 0x00) {
            printk(KERN_ERR "SDIO 2.0 A00 version\n");
        } else {
            printk(KERN_ERR "SDIO 2.0 A01 version\n");
        }
    }
}
EXPORT_SYMBOL(rtk_register_set);

void rtk_sdhci_platform_init(void)
{
#ifdef SD_INTERFACE_SDIO_1619
    unsigned int tmp = 0;
#endif

    if (sdio_gpio_23) {
        gpiod_direction_output(sdio_gpio_23, 0);
        mdelay(50);
        gpiod_direction_output(sdio_gpio_23, 1);
        mdelay(10);
    } else {
        printk(KERN_WARNING "%s: sdio_gpio_23 is NULL, SDIO module may not power on\n", __func__);
    }

    if (rstc_sdio) {
        reset_control_deassert(rstc_sdio);
    }
    clk_prepare_enable(clk_en_sdio);
    clk_prepare_enable(clk_en_sdio_ip);

    if(readl(crt_membase+0x1a204)==0x00) {
        writel(0x00000006, crt_membase + 0x01AC);
        writel(0x04517893, crt_membase + 0x01A4);
        writel(0x00ae4388, crt_membase + 0x01A8);
        mdelay(2);
        writel(0x00000007, crt_membase + 0x01AC);
        udelay(200);
    } else {
        writel(0x00000006, crt_membase + 0x01AC);
        writel(0x04517893, crt_membase + 0x01A4);
        writel(0x00ae4388, crt_membase + 0x01A8);
        mdelay(2);
        writel(0x00000007, crt_membase + 0x01AC);
        udelay(200);
    }
    writel(0x00000000, crt_membase + 0x10A40);
    writel(0x00000011, crt_membase + 0x010A34);

#ifdef SD_INTERFACE_SDIO_1619
    writel(readl(crt_membase+0x4E004)&(~(0x1<<25)),crt_membase+0x4E004);
    writel((readl(crt_membase+0x4E020)&0xff0fffff)|0x00600000 ,crt_membase+0x4E020);
    mdelay(100);
    writel(readl(crt_membase+0x4E020) & 0xffbfffff ,crt_membase+0x4E020);
    mdelay(100);

    tmp = (0x3243243 << 4) | (readl(crt_membase + 0x4f00c) & 0xf);
    writel(tmp,crt_membase + 0x4f00c);
    writel(0xd83d83d8,crt_membase + 0x4f010);
    tmp = (readl(crt_membase + 0x4f014) & 0xfffff000) | 0xd83;
    writel(tmp,crt_membase + 0x4f014);

    writel((readl(crt_membase+0x4E048) & ~(0x3 << 4)) | (0x2 << 4), crt_membase+0x4E048); /* sdio_loc1 */
    writel((readl(crt_membase+0x4F004)&0xfff00000)|0x52492, crt_membase+0x4F004);
    writel(0x3,crt_membase + 0x1e0);
    writel(0x4003,crt_membase + 0x1e0);
    writel(0x6003,crt_membase + 0x1e0);
#endif
}
EXPORT_SYMBOL(rtk_sdhci_platform_init);

void rtk_sdhci_close_clk(void)
{
    if (SDIO_module == true) {
        clock_enable = false;
        G_host->mmc->caps2 |= (MMC_CAP2_NO_SD | MMC_CAP2_NO_MMC | MMC_CAP2_NO_SDIO);
        
        printk(KERN_ERR "Do not detect the SDIO card and disable sdio irq\n");
        mdelay(10);
        
        writel(0x0, crt_membase + 0x01AC);
        reset_control_assert(rstc_sdio);
        clk_disable_unprepare(clk_en_sdio);
        clk_disable_unprepare(clk_en_sdio_ip);
        wmb();
        mdelay(10);

        if (get_rtd_chip_revision() == RTD_CHIP_A01) {
            writel(readl(crt_membase + 0x1A0) & (~0x1), crt_membase + 0x1A0);
            writel(readl(crt_membase + 0x1A4) & (~0x1), crt_membase + 0x1A4);
            udelay(200);
        }
    }
}
EXPORT_SYMBOL(rtk_sdhci_close_clk);

static int rtk_sdhci_change_rx_phase(u8 sample_point)
{
    unsigned int temp_reg = 0;
    writel(readl(crt_membase + 0x1A0) & 0xfffffffd, crt_membase + 0x1A0);
    temp_reg = readl(crt_membase + 0x1A0);
    temp_reg = (temp_reg & ~0x00001F00) | (sample_point << 8);
    writel(temp_reg, crt_membase + 0x1A0);
    writel(readl(crt_membase + 0x1A0) | 0x2, crt_membase + 0x1A0);
    udelay(100);
    return 0;
}

#ifdef sdhci_tx_tuning
static int rtk_sdhci_change_tx_phase(u8 sample_point)
{
    unsigned int temp_reg = 0;
    writel(readl(crt_membase + 0x1A0) & 0xfffffffd, crt_membase + 0x1A0);
    temp_reg = readl(crt_membase + 0x1A0);
    temp_reg = (temp_reg & ~0x000000F8) | (sample_point << 3);
    writel(temp_reg, crt_membase + 0x1A0);
    writel(readl(crt_membase + 0x1A0) | 0x2, crt_membase + 0x1A0);
    udelay(100);
    return 0;
}
#endif

static u8 rtk_sdhci_search_final_phase(u32 phase_map)
{
    struct timing_phase_path path[MAX_PHASE + 1];
    struct timing_phase_path swap;
    int i = 0, j = 0, k = 0;
    int cont_path_cnt = 0, new_block = 1, max_len = 0, final_path_idx = 0;
    u8 final_phase = 0xFF;

    for (i = 0; i < MAX_PHASE + 1; i++) {
        if (phase_map & (1 << i)) {
            if (new_block) {
                new_block = 0;
                j = cont_path_cnt++;
                path[j].start = i;
                path[j].end = i;
            } else
                path[j].end = i;
        } else {
            new_block = 1;
            if (cont_path_cnt) {
                int idx = cont_path_cnt - 1;
                path[idx].len = path[idx].end - path[idx].start + 1;
                path[idx].mid = path[idx].start + path[idx].len / 2;
            }
        }
    }

    if (cont_path_cnt == 0) {
        goto finish;
    } else {
        int idx = cont_path_cnt - 1;
        path[idx].len = path[idx].end - path[idx].start + 1;
        path[idx].mid = path[idx].start + path[idx].len / 2;
    }

    if (!path[0].start && (path[cont_path_cnt - 1].end == MAX_PHASE)) {
        path[0].start = path[cont_path_cnt - 1].start - MAX_PHASE - 1;
        path[0].len += path[cont_path_cnt - 1].len;
        path[0].mid = path[0].start + path[0].len / 2;
        if (path[0].mid < 0)
            path[0].mid += MAX_PHASE + 1;
        cont_path_cnt--;
    }

    for (k = 0; k < cont_path_cnt; ++k) {
        for (i = 0; i < cont_path_cnt - 1 - k; ++i) {
            if (path[i].len < path[i+1].len) {
                swap = path[i+1];
                path[i+1] = path[i];
                path[i] = swap;
            }
        }
    }

    for (i = 0; i < cont_path_cnt; i++) {
        if (path[i].len > max_len) {
            max_len = path[i].len;
            if (max_len > 6)
                final_phase = (u8)path[i].mid;
            else
                final_phase = 0xFF;
            final_path_idx = i;
        }
    }

finish:
    return final_phase;
}

static int rtk_sdhci_tuning_rx(struct sdhci_host *host)
{
    int sample_point = 0, ret = 0, i = 0;
    u32 raw_phase_map[TUNING_CNT] = {0}, phase_map = 0;
    u8 final_phase = 0;
    struct mmc_host *mmc = host->mmc;

    for (sample_point = 0; sample_point <= MAX_PHASE; sample_point++) {
        for (i = 0; i < TUNING_CNT; i++) {
            rtk_sdhci_change_rx_phase((u8)sample_point);
            ret = mmc_send_tuning(mmc, MMC_SEND_TUNING_BLOCK, NULL);
            if (ret == 0)
                raw_phase_map[i] |= (1 << sample_point);
        }
    }

    phase_map = 0xFFFFFFFF;
    for (i = 0; i < TUNING_CNT; i++) {
        phase_map &= raw_phase_map[i];
    }

    if (phase_map) {
        final_phase = rtk_sdhci_search_final_phase(phase_map);
        if (final_phase == 0xFF) {
            ret = -EINVAL;
            goto out;
        }
        rtk_sdhci_change_rx_phase(final_phase);
        ret = 0;
    } else {
        ret = -EINVAL;
    }
out:
    return ret;
}

#ifdef sdhci_tx_tuning
static int rtk_sdhci_tuning_tx(struct sdhci_host *host)
{
    int sample_point = 0, ret = 0, i = 0;
    u32 raw_phase_map[TUNING_CNT] = {0}, phase_map = 0;
    u8 final_phase = 0;
    struct mmc_host *mmc = host->mmc;

    for (sample_point = 0; sample_point <= MAX_PHASE; sample_point++) {
        for (i = 0; i < TUNING_CNT; i++) {
            rtk_sdhci_change_tx_phase((u8)sample_point);
            writew(0x1, host->ioaddr + 0x36);
            writew(0x1, host->ioaddr + 0x3A);
            
            ret = mmc_send_tuning(mmc, MMC_SEND_TUNING_BLOCK, NULL);
            if (ret == 0)
                raw_phase_map[i] |= (1 << sample_point);
        }
    }

    phase_map = 0xFFFFFFFF;
    for (i = 0; i < TUNING_CNT; i++) {
        phase_map &= raw_phase_map[i];
    }

    if (phase_map) {
        final_phase = rtk_sdhci_search_final_phase(phase_map);
        if (final_phase == 0xFF) {
            ret = -EINVAL;
            goto out;
        }
        rtk_sdhci_change_tx_phase(final_phase);
        ret = 0;
    } else {
        ret = -EINVAL;
    }
out:
    return ret;
}
#endif

static int rtk_sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{
    int ret = 0;
    unsigned int reg_tmp = 0;
    unsigned int reg_tmp2 = 0;
    unsigned int reg_tuned3318 = 0;

    reg_tmp2 = readl(crt_membase + 0x01A4);
    writel((reg_tmp2 & 0xFFFF1FFF), crt_membase + 0x01A4);

#ifdef sdhci_tx_tuning
    ret = rtk_sdhci_tuning_tx(host);
#endif

    do {
        ret = rtk_sdhci_tuning_rx(host);
        if (ret) {
            reg_tmp = readl(crt_membase + 0x01A8);
            reg_tuned3318 = (reg_tmp & 0x03FF0000) >> 16;
            if (reg_tuned3318 <= 100) {
                printk(KERN_ERR "%s: Tuning RX fail\n", __func__);
                return ret;
            }
            reg_tmp = ((reg_tmp & (~0x3FF0000)) | ((reg_tuned3318 - 8) << 16)); 

            writel(0x00000006, crt_membase + 0x01AC);
            writel(reg_tmp, crt_membase + 0x01A8);
            mdelay(2);
            writel(0x00000007, crt_membase + 0x01AC);
            udelay(200);
        }
    } while (ret);

    writel(reg_tmp2, crt_membase + 0x01A4);

    return 0;
}

static const struct sdhci_ops rtk_sdhci_ops = {
    .reset = rtk_sdhci_reset,
    .set_bus_width = rtk_sdhci_buswidth,
    .set_uhs_signaling = rtk_sdhci_uhs_signaling,
    .set_clock = rtk_sdhci_set_clock,
    .platform_execute_tuning = rtk_sdhci_execute_tuning,
};

static const struct sdhci_pltfm_data sdhci_rtk_sdio_pdata = {
    .quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
              SDHCI_QUIRK_SINGLE_POWER_WRITE |
              SDHCI_QUIRK_NO_HISPD_BIT |
              SDHCI_QUIRK_BROKEN_CARD_DETECTION |
              SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC,
    .quirks2 = SDHCI_QUIRK2_BROKEN_DDR50 |
               SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
    .ops  = &rtk_sdhci_ops,
};

static struct sdhci_rtk_sdio_data sdhci_rtk_sdio_data = {
    .pdata = &sdhci_rtk_sdio_pdata,
};

static const struct of_device_id sdhci_rtk_dt_match[] = {
    { .compatible = "realtek,rtk1619-sdio", .data = &sdhci_rtk_sdio_data },
    {}
};
MODULE_DEVICE_TABLE(of, sdhci_rtk_dt_match);
static int sdhci_rtk_probe(struct platform_device *pdev)
{
    const struct of_device_id *match;
    const struct sdhci_rtk_sdio_data *soc_data;
    struct sdhci_host *host;
    struct sdhci_pltfm_host *pltfm_host;
    int rc = 0;
    struct device_node *node = pdev->dev.of_node;

    SDIO_module = true;

    sdio_membase = of_iomap(node, 0);
    if (!sdio_membase)
        return -ENOMEM;

    crt_membase = of_iomap(node, 1);
    if (!crt_membase)
        return -ENOMEM;

    sdio_gpio_23 = devm_gpiod_get_index_optional(&pdev->dev, NULL, 0, GPIOD_ASIS);
    if (IS_ERR(sdio_gpio_23)) {
        rc = PTR_ERR(sdio_gpio_23);
        if (rc == -EPROBE_DEFER) return rc;
        printk(KERN_WARNING "%s: can't request gpio 0 (err=%d)\n", __func__, rc);
        sdio_gpio_23 = NULL;
    }
    
    match = of_match_device(sdhci_rtk_dt_match, &pdev->dev);
    if (!match)
        return -EINVAL;
    soc_data = match->data;

    rstc_sdio = devm_reset_control_get(&pdev->dev, NULL);
    if (IS_ERR(rstc_sdio)) {
        rc = PTR_ERR(rstc_sdio);
        if (rc == -EPROBE_DEFER) return rc;
        printk(KERN_WARNING "%s: reset_control_get() returns %d\n", __func__, rc);
        rstc_sdio = NULL;
    }
    
    clk_en_sdio = devm_clk_get(&pdev->dev, "clk_en_sdio");
    if (IS_ERR(clk_en_sdio)) {
        rc = PTR_ERR(clk_en_sdio);
        if (rc == -EPROBE_DEFER) return rc;
        printk(KERN_WARNING "%s: clk_get() returns %d\n", __func__, rc);
        clk_en_sdio = NULL;
    }
    
    clk_en_sdio_ip = devm_clk_get(&pdev->dev, "clk_en_sdio_ip");
    if (IS_ERR(clk_en_sdio_ip)) {
        rc = PTR_ERR(clk_en_sdio_ip);
        if (rc == -EPROBE_DEFER) return rc;
        printk(KERN_WARNING "%s: clk_get() returns %d\n", __func__, rc);
        clk_en_sdio_ip = NULL;
    }

    rtk_sdhci_platform_init();

    if (get_rtd_chip_revision() == RTD_CHIP_A01) {
        writel(readl(crt_membase + 0x1A0) | 0x1, crt_membase + 0x1A0);
        writel(readl(crt_membase + 0x1A4) | 0x1, crt_membase + 0x1A4);
        udelay(200);
    }
 
    host = sdhci_pltfm_init(pdev, soc_data->pdata, 0);
    if (IS_ERR(host)) {
        rc = PTR_ERR(host);
        goto err_pltfm_init;
    }
    pltfm_host = sdhci_priv(host);

    host->mmc->caps = MMC_CAP_4_BIT_DATA |
                      MMC_CAP_SD_HIGHSPEED |
                      MMC_CAP_MMC_HIGHSPEED |
                      MMC_CAP_UHS_SDR12 |
                      MMC_CAP_UHS_SDR25 |
                      MMC_CAP_UHS_SDR50 |
                      MMC_CAP_NONREMOVABLE;

    host->mmc->caps2 = MMC_CAP2_NO_SD | MMC_CAP2_NO_MMC;
    host->mmc->pm_caps = MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ;

    host->ocr_mask = MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 |
                     MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 |
                     MMC_VDD_31_32 | MMC_VDD_32_33 | MMC_VDD_33_34 |
                     MMC_VDD_165_195;
    host->mmc->ocr_avail = host->ocr_mask;

    G_host = host;

    rc = sdhci_add_host(host);
    if (rc)
        goto err_add_host;

    return 0;

err_add_host:

err_pltfm_init:
    if (clk_en_sdio) clk_disable_unprepare(clk_en_sdio);
    if (clk_en_sdio_ip) clk_disable_unprepare(clk_en_sdio_ip);
    return rc;
}

static void sdhci_rtk_remove(struct platform_device *pdev)
{
    struct sdhci_host *host = platform_get_drvdata(pdev);
    int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xFFFFFFFF);

    sdhci_remove_host(host, dead);
    
    if (clk_en_sdio) clk_disable_unprepare(clk_en_sdio);
    if (clk_en_sdio_ip) clk_disable_unprepare(clk_en_sdio_ip);
}

static int sdhci_rtk_suspend(struct device *dev)
{
    printk(KERN_ERR "[SDIO] sdhci_rtk_suspend start\n");
    drive_temp = readl(crt_membase + 0x01A0);
    if (clock_enable == true) {
        struct sdhci_host *host = dev_get_drvdata(dev);
        sdhci_suspend_host(host);
    }
    printk(KERN_ERR "[SDIO] sdhci_rtk_suspend OK\n");
    return 0;
}

static void sdhci_rtk_shutdown(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    printk(KERN_ERR "[SDIO] sdhci_rtk_shutdown\n");
    if (clock_enable == true) {
        struct sdhci_host *host = dev_get_drvdata(dev);
        sdhci_suspend_host(host);
    }
}

static int sdhci_rtk_resume(struct device *dev)
{
    struct sdhci_host *host = dev_get_drvdata(dev);
    void __iomem *addr;
    unsigned int val;

    host->clock = 0;
    printk(KERN_ERR "[SDIO] sdhci_rtk_resume start\n");

    if (clock_enable == true) {
        addr = crt_membase + SDIO_CLKEN_REGOFF;

        if (get_SDIO_version() == 3 || get_SDIO_version() == 4) {
            writel(drive_temp, crt_membase + 0x01A0);
            writel((readl(crt_membase + 0x4E024) & 0xf) | 0x8F31E630, crt_membase + 0x4E024);
            writel(0x1E63CC79, crt_membase + 0x4E028);
            writel((readl(crt_membase + 0x4E02c) & 0xffffffc0) | 0x33, crt_membase + 0x4E02c);

            if (readl(crt_membase + 0x1a204) == 0x00) {
                if (get_SDIO_version() == 3) {
                    writel(0x00000006, crt_membase + 0x01AC);
                    writel(0x04517893, crt_membase + 0x01A4);
                    writel(0x00ae4388, crt_membase + 0x01A8); 
                    mdelay(2);
                    writel(0x00000007, crt_membase + 0x01AC);
                    udelay(200);
                } else {
                    writel(0x1d, crt_membase + 0x10a58);
                    writel(0x00000006, crt_membase + 0x01AC);
                    writel(0x04517893, crt_membase + 0x01A4);
                    writel(0x00ae4388, crt_membase + 0x01A8); 
                    mdelay(2);
                    writel(0x00000007, crt_membase + 0x01AC);
                    udelay(200);
                }
                writel(0x00000000, crt_membase + 0x010A40);
                writel(0x00000011, crt_membase + 0x010A34);
            } else {
                reset_control_deassert(rstc_sdio);
                if (get_SDIO_version() == 3) {
                    writel(0x00000006, crt_membase + 0x01AC);
                    writel(0x04517893, crt_membase + 0x01A4);
                    writel(0x00ae4388, crt_membase + 0x01A8); 
                    mdelay(2);
                    writel(0x00000007, crt_membase + 0x01AC);
                    udelay(200);
                } else {
                    writel(0x1d, crt_membase + 0x10a58);
                    writel(0x00000006, crt_membase + 0x01AC);
                    writel(0x04517893, crt_membase + 0x01A4);
                    writel(0x00ae4388, crt_membase + 0x01A8); 
                    mdelay(2);
                    writel(0x00000007, crt_membase + 0x01AC);
                    udelay(200);
                }
                writel(0x00000000, crt_membase + 0x010A40);
                writel(0x00000011, crt_membase + 0x010A34);
            }
        } else {
            if (readl(crt_membase + 0x1a204) == 0x00) {
                writel(0x00000006, crt_membase + 0x01AC);
                writel(0x04517893, crt_membase + 0x01A4);
                writel(0x00ae4388, crt_membase + 0x01A8); 
                mdelay(2);
                writel(0x00000007, crt_membase + 0x01AC);
                udelay(200);
                reset_control_deassert(rstc_sdio);
                writel(0x00000011, crt_membase + 0x010A34);
            } else {
                reset_control_deassert(rstc_sdio);
                writel(0x00000006, crt_membase + 0x01AC);
                writel(0x04517893, crt_membase + 0x01A4);
                writel(0x00ae4388, crt_membase + 0x01A8); 
                mdelay(2);
                writel(0x00000007, crt_membase + 0x01AC);
                udelay(200);
                writel(0x00000011, crt_membase + 0x010A34);
            }
        }
        
        writel(0x00000003, crt_membase + 0x010A10);
        
        val = readl(addr);
        val |= 1 << (SDIO_CLKEN_REGBIT);
        writel(val, addr);

        val = readl(host->ioaddr + 0x28);
        val |= 0x0f << 8;
        writel(val, host->ioaddr + 0x28);
        val = readl(host->ioaddr + 0x28);

        sdhci_resume_host(host);
    }
    printk(KERN_ERR "[SDIO] sdhci_rtk_resume OK\n");
    return 0;
}

const struct dev_pm_ops sdhci_rtk_pmops = {
    .suspend = sdhci_rtk_suspend,
    .resume = sdhci_rtk_resume,
};

static struct platform_driver sdhci_rtk_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = sdhci_rtk_dt_match,
        .pm = &sdhci_rtk_pmops,
    },
    .probe = sdhci_rtk_probe,
    .remove = sdhci_rtk_remove,
    .shutdown = sdhci_rtk_shutdown,
};

module_platform_driver(sdhci_rtk_driver);
MODULE_DESCRIPTION("SDHCI host driver for Realtek 16xx");