// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2022-2023, CTCaer.
 */

#include <fdt_support.h>
#include <i2c.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pinmux.h>
#include <asm/arch-tegra/fuse.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/arch-tegra/tegra.h>
#include "../../nvidia/p2571/max77620_init.h"
#include "pinmux-config-nintendo-switch.h"

#define FUSE_OPT_LOT_CODE_0	0x208

/*
 * hekate hands over with every pad in whatever state the Nintendo bootloader
 * left it, so U-Boot has to program the board pinmux itself. The two console
 * UART TX pins come back as GPIOs, which would override the mux, hence the
 * SFIO entries in the GPIO table.
 */
void pinmux_init(void)
{
	gpio_config_table(nintendo_switch_gpio_inits,
			  ARRAY_SIZE(nintendo_switch_gpio_inits));

	pinmux_config_pingrp_table(nintendo_switch_pingrps,
				   ARRAY_SIZE(nintendo_switch_pingrps));
}

void pin_mux_mmc(void)
{
	struct pmc_ctlr *const pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	struct udevice *dev;
	u32 reg_val;
	uchar val;
	int ret;

	/* Power cycle the SDMMC1 pad rail before changing its voltage */
	reg_val = readl(&pmc->pmc_no_iopower);
	reg_val |= BIT(12);
	writel(reg_val, &pmc->pmc_no_iopower);
	readl(&pmc->pmc_no_iopower);
	udelay(1000);
	reg_val &= ~BIT(12);
	writel(reg_val, &pmc->pmc_no_iopower);
	readl(&pmc->pmc_no_iopower);

	/* Tell the IO pads the rail is about to be 3.3V */
	reg_val = readl(&pmc->pmc_pwr_det_val);
	reg_val |= BIT(12);
	writel(reg_val, &pmc->pmc_pwr_det_val);
	readl(&pmc->pmc_pwr_det_val);

	ret = i2c_get_chip_for_busnum(0, MAX77620_I2C_ADDR_7BIT, 1, &dev);
	if (ret) {
		printf("%s: Cannot find MAX77620 I2C chip\n", __func__);
		return;
	}

	/* Turn on LDO2 to 3.3V for SD card power */
	val = 0xf2;
	ret = dm_i2c_write(dev, MAX77620_CNFG1_L2_REG, &val, 1);
	if (ret)
		printf("Failed to enable 3.3V LDO for SD Card IO: %d\n", ret);

	/* Disable LDO4 discharge, it feeds VDD_RTC */
	ret = dm_i2c_read(dev, MAX77620_CNFG2_L4_REG, &val, 1);
	if (ret) {
		printf("Failed to read LDO4 register: %d\n", ret);
	} else {
		val &= ~BIT(1); /* ADE */
		ret = dm_i2c_write(dev, MAX77620_CNFG2_L4_REG, &val, 1);
		if (ret)
			printf("Failed to disable ADE in LDO4: %d\n", ret);
	}
}

/* The derivation the vendor bootloader uses, from the lot code fuse */
static void switch_mac(u8 mac[6], u8 last)
{
	u32 lot0;

	tegra_fuse_init();
	lot0 = readl(NV_PA_FUSE_BASE + FUSE_OPT_LOT_CODE_0);

	mac[0] = 0x98;
	mac[1] = 0xb6;
	mac[2] = 0xe9;
	mac[3] = lot0 >> 16;
	mac[4] = lot0 >> 8;
	mac[5] = lot0 + last;
}

static void set_addr(void *blob, const char *compat, const char *prop,
		     const u8 *addr)
{
	int node, ret;

	node = fdt_node_offset_by_compatible(blob, -1, compat);
	if (node < 0)
		return;

	ret = fdt_setprop(blob, node, prop, addr, 6);
	if (ret)
		printf("Failed to set %s: %s\n", prop, fdt_strerror(ret));
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	u8 mac[6], bdaddr[6];
	int i;

	/* the Bluetooth binding wants the address byte-reversed */
	switch_mac(mac, 1);
	for (i = 0; i < 6; i++)
		bdaddr[i] = mac[5 - i];
	set_addr(blob, "brcm,bcm43438-bt", "local-bd-address", bdaddr);

	switch_mac(mac, 2);
	set_addr(blob, "pci14e4,43ec", "local-mac-address", mac);

	return 0;
}
