// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2018-2019
 * NVIDIA Corporation <www.nvidia.com>
 *
 */

#include <fdtdec.h>
#include <i2c.h>
#include <linux/bitops.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>
#include <pca953x.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mc.h>
#include <asm/arch/pinmux.h>
#include <asm/arch-tegra/board.h>
#include "../p2571/max77620_init.h"
#include "mc-security.h"
#include "pinmux-config-p3450-0000.h"

void tegra210_emc_ft_setup(void *fdt);

/*
 * On a stock boot nvtboot/cboot programs the board pinmux out of the bl-dtb
 * before it hands over, so U-Boot has never had to. The RAM-boot flow jumps
 * straight from nvtboot into U-Boot, which leaves every pad at its reset
 * value - tristated, so nothing the SoC drives actually reaches the board.
 * Program the table here so both boot paths end up in the same state.
 */
void pinmux_init(void)
{
	pinmux_config_pingrp_table(p3450_0000_pingrps,
				   ARRAY_SIZE(p3450_0000_pingrps));
}

/*
 * The SMMU enable, the VPR lock and the GPU's WPR carveout are all
 * TrustZone-protected and are cboot's job on a stock boot. The RAM-boot flow
 * skips cboot, so U-Boot makes them here instead; see mc-security.h for what
 * each one is for and what breaks without it. The same setup has to be redone
 * from EL3 after SC7, which is why it lives in a header.
 *
 * On the QSPI boot path U-Boot runs non-secure, so these writes do nothing;
 * that is fine, cboot has already made them.
 */
int nvidia_board_init(void)
{
	tegra_mc_security_setup();

	return 0;
}

void pin_mux_mmc(void)
{
	struct udevice *dev;
	uchar val;
	int ret;

	/* Turn on MAX77620 LDO2 to 3.3V for SD card power */
	debug("%s: Set LDO2 for VDDIO_SDMMC_AP power to 3.3V\n", __func__);
	ret = i2c_get_chip_for_busnum(0, MAX77620_I2C_ADDR_7BIT, 1, &dev);
	if (ret) {
		printf("%s: Cannot find MAX77620 I2C chip\n", __func__);
		return;
	}
	/* 0xF2 for 3.3v, enabled: bit7:6 = 11 = enable, bit5:0 = voltage */
	val = 0xF2;
	ret = dm_i2c_write(dev, MAX77620_CNFG1_L2_REG, &val, 1);
	if (ret)
		printf("i2c_write 0 0x3c 0x27 failed: %d\n", ret);

	/* Disable LDO4 discharge */
	ret = dm_i2c_read(dev, MAX77620_CNFG2_L4_REG, &val, 1);
	if (ret) {
		printf("i2c_read 0 0x3c 0x2c failed: %d\n", ret);
	} else {
		val &= ~BIT(1); /* ADE */
		ret = dm_i2c_write(dev, MAX77620_CNFG2_L4_REG, &val, 1);
		if (ret)
			printf("i2c_write 0 0x3c 0x2c failed: %d\n", ret);
	}

	/* Set MBLPD */
	ret = dm_i2c_read(dev, MAX77620_CNFGGLBL1_REG, &val, 1);
	if (ret) {
		printf("i2c_write 0 0x3c 0x00 failed: %d\n", ret);
	} else {
		val |= BIT(6); /* MBLPD */
		ret = dm_i2c_write(dev, MAX77620_CNFGGLBL1_REG, &val, 1);
		if (ret)
			printf("i2c_write 0 0x3c 0x00 failed: %d\n", ret);
	}
}

#ifdef CONFIG_PCI_TEGRA
int tegra_pcie_board_init(void)
{
	struct udevice *dev;
	uchar val;
	int ret;

	/* Turn on MAX77620 LDO1 to 1.05V for PEX power */
	debug("%s: Set LDO1 for PEX power to 1.05V\n", __func__);
	ret = i2c_get_chip_for_busnum(0, MAX77620_I2C_ADDR_7BIT, 1, &dev);
	if (ret) {
		printf("%s: Cannot find MAX77620 I2C chip\n", __func__);
		return -1;
	}
	/* 0xCA for 1.05v, enabled: bit7:6 = 11 = enable, bit5:0 = voltage */
	val = 0xCA;
	ret = dm_i2c_write(dev, MAX77620_CNFG1_L1_REG, &val, 1);
	if (ret)
		printf("i2c_write 0 0x3c 0x25 failed: %d\n", ret);

	return 0;
}
#endif /* PCI */

static const char * const nodes[] = {
	"/host1x@50000000/dc@54200000",
	"/host1x@50000000/dc@54240000",
	"/external-memory-controller@7001b000",
};

int ft_board_setup(void *fdt, struct bd_info *bd)
{
	ft_mac_address_setup(fdt);
	ft_carveout_setup(fdt, nodes, ARRAY_SIZE(nodes));
	tegra210_emc_ft_setup(fdt);

	return 0;
}
