/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Cortex-A57 erratum workarounds for Tegra210.
 *
 * On the stock boot path NVIDIA's Trusted Firmware applies these at EL3 in its
 * CPU reset handler, for every core, every time a core comes out of reset (see
 * lib/cpus/aarch64/cortex_a57.S and the ERRATA_A57_* / A57_* switches in
 * plat/nvidia/tegra/soc/t210/platform_t210.mk). When U-Boot is RAM-booted via
 * RCM that chain is skipped and nothing applies them, so every core runs with
 * CPUACTLR_EL1 at its reset value. CPUACTLR_EL1 is only writable at EL3, so
 * Linux cannot make up for it later - it has to happen here.
 *
 * The T210 A57s are r1p1, which is in scope for all of the below.
 *
 * Erratum 828024 is deliberately not applied: its workaround disables write
 * streaming, which costs 2.6x of CPU write bandwidth to DRAM, and the errata
 * notice scopes it to multi-cluster systems using DVM transactions, which
 * T210 is not. NVIDIA's own shipped monitor does not apply it either.
 *
 * This is applied in three places, matching the three ways a core starts
 * executing:
 *   - the boot CPU, via the weak apply_core_errata hook in start.S;
 *   - a secondary brought up by PSCI CPU_ON;
 *   - any core warm-booting after a CPU_SUSPEND core power-gate,
 * the latter two through tegra_secondary_entry (psci_secondary.S).
 *
 * It lives in ._secure.text because the last of those happens long after Linux
 * has reclaimed the rest of U-Boot's memory.
 */

#include <config.h>
#include <linux/linkage.h>

/*
 * CPUACTLR_EL1 (S3_1_c15_c2_0) bits, named as in ARM Trusted Firmware's
 * include/lib/cpus/aarch64/cortex_a57.h. The combined mask is:
 *
 *   (1 << 59)  DIS_LOAD_PASS_DMB    erratum 826974  (<= r1p1)
 *   (1 << 54)  GRE_NGRE_AS_NGNRE    erratum 826977  (<= r1p1)
 *   (1 << 52)  DIS_OVERREAD         LDNP/STNP over-read (<= r1p2)
 *   (1 << 38)  FORCE_FPSCR_FLUSH    erratum 833471  (<= r1p2)
 *   (1 << 24)  EN_NC_LOAD_FWD       non-cacheable load forwarding (perf)
 *
 * = 0x0850004001000000. Built with movz/movk since it is not encodable as a
 * logical immediate.
 */
#define CPUACTLR_ERRATA_HI16	0x0850	/* bits 63:48 */
#define CPUACTLR_ERRATA_HM16	0x0040	/* bits 47:32 */
#define CPUACTLR_ERRATA_LM16	0x0100	/* bits 31:16 */

#define MIDR_PART_CORTEX_A57	0xd07
#define CURRENTEL_EL3		0xc

.pushsection ._secure.text, "ax"
.align 3

/*
 * void tegra_a57_apply_errata(void)
 *
 * Leaf, clobbers x0/x1 only, uses no stack: it is called from the boot CPU's
 * reset path and from the secondary trampoline, neither of which has a stack
 * set up yet.
 */
ENTRY(tegra_a57_apply_errata)
	/* CPUACTLR_EL1 is implementation defined - check this really is an A57. */
	mrs	x0, midr_el1
	ubfx	x0, x0, #4, #12		/* MIDR_EL1.PartNum */
	cmp	x0, #MIDR_PART_CORTEX_A57
	b.ne	1f

	/*
	 * Only EL3 may write CPUACTLR_EL1. On the eMMC/QSPI boot path U-Boot
	 * runs non-secure below a Trusted Firmware that has already applied
	 * these, so skip rather than take an exception.
	 */
	mrs	x0, CurrentEL
	cmp	x0, #CURRENTEL_EL3
	b.ne	1f

	movz	x1, #CPUACTLR_ERRATA_HI16, lsl #48
	movk	x1, #CPUACTLR_ERRATA_HM16, lsl #32
	movk	x1, #CPUACTLR_ERRATA_LM16, lsl #16

	mrs	x0, S3_1_c15_c2_0	/* CPUACTLR_EL1 */
	orr	x0, x0, x1
	msr	S3_1_c15_c2_0, x0	/* CPUACTLR_EL1 */
	isb
1:
	ret
ENDPROC(tegra_a57_apply_errata)

/*
 * Override the weak apply_core_errata in arch/arm/cpu/armv8/start.S so the boot
 * CPU picks the workarounds up at reset, before the MMU and caches come on.
 * The generic implementation is driven by CONFIG_ARM_ERRATA_* and covers only a
 * subset of what Tegra210 needs (it has no 826977, LDNP/STNP over-read or
 * non-cacheable load forwarding), so we replace it wholesale instead of
 * enabling those symbols, and keep one definition of the T210 erratum set.
 */
ENTRY(apply_core_errata)
	b	tegra_a57_apply_errata
ENDPROC(apply_core_errata)

.popsection
