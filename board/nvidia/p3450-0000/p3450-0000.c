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
 * MC_SMMU_CONFIG is TrustZone-protected: the TRM (18.5.4.1, step 10) spells out
 * that a non-secure write to it is silently dropped. On a stock boot cboot runs
 * secure and turns the SMMU on, and Linux's tegra-smmu driver simply assumes
 * that happened - its own SMMU_CONFIG write from non-secure EL1 is a no-op. The
 * RAM-boot flow skips cboot, so without this every SMMU client puts raw IOVAs on
 * the bus: the GPU falls over in ACR ("gpusrd: EMEM address decode error", ACR
 * boot -110) and the display controller buries the MC in decode errors as soon
 * as it starts scanning out.
 *
 * Only the global enable is needed here. The per-client MC_SMMU_<engine>_ASID
 * registers are zero out of reset, so clients keep bypassing translation until
 * Linux attaches them - and those writes are not TZ-restricted.
 *
 * On the QSPI boot path U-Boot runs non-secure, so this write does nothing; that
 * is fine, cboot has already done it.
 */
/*
 * The GPU's write-protected region, where the ACR firmware keeps the signed
 * low-secure ucode it bootstraps FECS/GPCCS/PMU from. It is one of the memory
 * controller's five general security carveouts, and only secure mode can set it
 * up - so, like the SMMU enable above, it is cboot's job on a stock boot and
 * ours on the RAM-boot path.
 *
 * The base and size below are what a cboot boot leaves behind, read back out of
 * the GPU (nouveau's acr subdev logs "WPR region is from 0xff500000-0xff540000"
 * at trace level). It sits in the carveout gap above the top of DRAM bank 0,
 * which rp1.dts caps at 0xfee00000, and below the LP0 resume vector at
 * 0xff780000, so nothing else claims it.
 *
 * Without this the GPU is handed a garbage aperture. nouveau only range-checks
 * it (gm20b_acr_wpr_alloc), and a nonsense window is comfortably larger than the
 * ~72 KiB WPR image, so the check passes and ACR reports success while writing
 * outside any protected region. The low-secure ucode then fails validation, the
 * PMU falcon halts before it can post its INIT message, and everything
 * downstream times out: "pmu:hpq: timeout waiting for queue ready",
 * "gr: init failed, -110", then a gf100_gr_fecs_bind_pointer timeout on the
 * first GL client.
 *
 * The carveout register layout is not in the TRM; the field meanings and this
 * configuration follow hekate (bdk/mem/mc_t210.h, bootloader/l4t/l4t.c), whose
 * L4T launcher programs the same carveouts for the same kernel.
 */
#define GPU_WPR_CARVEOUT	1	/* MC_SECURITY_CARVEOUT2 */
#define GPU_WPR_BASE		0xff500000
#define GPU_WPR_SIZE		SZ_256K

static void gpu_wpr_carveout_init(struct mc_ctlr *mc)
{
	struct mc_sec_carveout *gsc = &mc->mc_security_carveout[GPU_WPR_CARVEOUT];
	int i;

	writel(GPU_WPR_BASE, &gsc->bom);
	writel(0, &gsc->bom_hi);
	writel(GPU_WPR_SIZE / SZ_128K, &gsc->size_128kb);

	for (i = 0; i < 5; i++) {
		writel(0, &gsc->client_access[i]);
		writel(0, &gsc->client_force_internal_access[i]);
	}
	writel(TEGRA_MC_SEC_CARVEOUT_CA2_R_GPU |
	       TEGRA_MC_SEC_CARVEOUT_CA2_W_GPU, &gsc->client_access[2]);
	writel(TEGRA_MC_SEC_CARVEOUT_CA4_R_GPU2 |
	       TEGRA_MC_SEC_CARVEOUT_CA4_W_GPU2, &gsc->client_access[4]);

	/*
	 * LOCKED is not optional: the TRM (18.6.9) notes the memory controller
	 * only publishes the VPR and WPR apertures to the GPU once they are
	 * locked. VPR is already locked by tegra_gpu_config(), which board_init()
	 * runs before it gets here.
	 */
	writel(TEGRA_MC_SEC_CARVEOUT_CFG_LOCKED |
	       TEGRA_MC_SEC_CARVEOUT_CFG_UNTRANSLATED_ONLY |
	       TEGRA_MC_SEC_CARVEOUT_CFG_RD_NS |
	       TEGRA_MC_SEC_CARVEOUT_CFG_RD_SEC |
	       TEGRA_MC_SEC_CARVEOUT_CFG_RD_FALCON_LS |
	       TEGRA_MC_SEC_CARVEOUT_CFG_RD_FALCON_HS |
	       TEGRA_MC_SEC_CARVEOUT_CFG_WR_FALCON_LS |
	       TEGRA_MC_SEC_CARVEOUT_CFG_WR_FALCON_HS |
	       TEGRA_MC_SEC_CARVEOUT_CFG_APERTURE_ID(GPU_WPR_CARVEOUT + 1) |
	       TEGRA_MC_SEC_CARVEOUT_CFG_SEND_CFG_TO_GPU |
	       TEGRA_MC_SEC_CARVEOUT_CFG_FORCE_APERTURE_ID_MATCH, &gsc->cfg0);
	readl(&gsc->cfg0);	/* flush the posted write */
}

int nvidia_board_init(void)
{
	struct mc_ctlr *mc = (struct mc_ctlr *)NV_PA_MC_BASE;

	writel(TEGRA_MC_SMMU_CONFIG_ENABLE, &mc->mc_smmu_config);
	readl(&mc->mc_smmu_config);	/* flush the posted write */

	gpu_wpr_carveout_init(mc);

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
