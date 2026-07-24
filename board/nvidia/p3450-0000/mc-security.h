/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Memory-controller security setup for the Jetson Nano.
 *
 * These are the three MC settings that only secure mode can make and that
 * Linux therefore assumes some earlier stage already made. On a stock boot
 * that stage is cboot; on ours it is U-Boot, from nvidia_board_init().
 *
 * They also have to be made a second time, from EL3, on the way out of SC7:
 * the memory controller sits in the VDD_CORE domain and comes back completely
 * reset, and nothing in the kernel puts these back (see
 * tegra_sc7_resume_finish() in psci.c). That is why this lives in a header as
 * an inline: the SC7 resume path runs from the relocated ._secure section and
 * cannot call into ordinary U-Boot text, so it needs its own copy of the code
 * rather than a shared out-of-line function.
 */

#ifndef __P3450_MC_SECURITY_H
#define __P3450_MC_SECURITY_H

#include <linux/sizes.h>
#include <asm/io.h>
#include <asm/arch/mc.h>
#include <asm/arch/tegra.h>

/*
 * The GPU's write-protected region, where the ACR firmware keeps the signed
 * low-secure ucode it bootstraps FECS/GPCCS/PMU from. It is one of the memory
 * controller's five general security carveouts.
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

static inline void tegra_mc_security_setup(void)
{
	struct mc_ctlr *mc = (struct mc_ctlr *)NV_PA_MC_BASE;
	struct mc_sec_carveout *gsc = &mc->mc_security_carveout[GPU_WPR_CARVEOUT];
	int i;

	/*
	 * Turn the video-protect region off and lock it. This is what
	 * tegra_gpu_config() does on the cold-boot path, repeated here because
	 * the carveout below is only published to the GPU once VPR is locked
	 * (TRM 18.6.9) and because an SC7 exit gets no other chance to do it.
	 */
	writel(0, &mc->mc_video_protect_size_mb);
	writel(TEGRA_MC_VIDEO_PROTECT_REG_WRITE_ACCESS_DISABLED,
	       &mc->mc_video_protect_reg_ctrl);
	readl(&mc->mc_video_protect_reg_ctrl);

	/*
	 * MC_SMMU_CONFIG is TrustZone-protected: the TRM (18.5.4.1, step 10)
	 * spells out that a non-secure write to it is silently dropped. Linux's
	 * tegra-smmu driver simply assumes an earlier secure stage turned the
	 * SMMU on - its own SMMU_CONFIG write from non-secure EL1 is a no-op.
	 * Without this every SMMU client puts raw IOVAs on the bus: the GPU
	 * falls over in ACR ("gpusrd: EMEM address decode error", ACR boot
	 * -110) and the display controller buries the MC in decode errors as
	 * soon as it starts scanning out.
	 *
	 * Only the global enable is needed. The per-client MC_SMMU_<engine>_ASID
	 * registers are zero out of reset, so clients keep bypassing translation
	 * until Linux attaches them - and those writes are not TZ-restricted.
	 */
	writel(TEGRA_MC_SMMU_CONFIG_ENABLE, &mc->mc_smmu_config);
	readl(&mc->mc_smmu_config);

	/* The GPU WPR carveout. */
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
	 * LOCKED is not optional either: the memory controller only publishes
	 * the aperture to the GPU once the carveout is locked.
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

#endif /* __P3450_MC_SECURITY_H */
