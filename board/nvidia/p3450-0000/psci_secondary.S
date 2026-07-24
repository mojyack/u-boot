/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Tegra210 (T210) secondary-CPU entry trampoline for U-Boot's ARMv8 PSCI
 * backend (see psci.c).
 *
 * A secondary Cortex-A57 that psci_cpu_on() power-ungates comes out of reset
 * at EL3 with the MMU and caches off, fetching its first instruction from the
 * Tegra secure reset-vector registers, which psci_cpu_on() points here. This
 * stub brings the core into the same state the boot CPU had on entry to Linux
 * (SMP coherency, arch-timer frequency, GIC CPU interface, EL3 vectors), then
 * drops it to Non-secure EL2 at the kernel entry point recorded by
 * psci_cpu_on() - mirroring how the primary was launched via
 * armv8_switch_to_el2().
 *
 * Everything here lives in ._secure.text so it survives in the memreserved
 * region after the kernel has taken over the rest of U-Boot's memory. cpu is
 * kept in x20 (callee-saved) so it survives the C helper calls.
 */

#include <config.h>
#include <linux/linkage.h>
#include <asm/macro.h>
#include <asm/psci.h>
#include <asm/system.h>

.pushsection ._secure.text, "ax"
.align 3

ENTRY(tegra_secondary_entry)
	/* cpu = MPIDR_EL1.Aff0 (the four A57s share a single cluster on T210) */
	mrs	x20, mpidr_el1
	and	x20, x20, #0xff

	/*
	 * Enable A57 SMP coherency (CPUECTLR_EL1.SMPEN) before any cache or MMU
	 * use, exactly as the bootstub does for CPU0. Without this the core
	 * neither snoops nor is snooped, so once the kernel turns on its caches
	 * on this CPU it loses coherency with the boot CPU and SMP wedges.
	 */
	mrs	x0, S3_1_C15_C2_1	/* CPUECTLR_EL1 */
	orr	x0, x0, #(1 << 6)	/* SMPEN */
	msr	S3_1_C15_C2_1, x0
	isb

	/*
	 * Program the architected-timer frequency for this core. CNTFRQ_EL0 is a
	 * per-CPU register that only firmware (EL3) can write; the boot CPU had
	 * it set already, so the kernel treats a differing value on a secondary
	 * as a fatal feature variation ("Unexpected variation in SYS_CNTFRQ_EL0").
	 */
	adr	x0, tegra_boot_cntfrq
	ldr	x0, [x0]
	msr	cntfrq_el0, x0

	/*
	 * Per-CPU EL3 stack, matching psci_get_cpu_stack_top():
	 *   sp = __secure_stack_end - (cpu << ARM_PSCI_STACK_SHIFT)
	 * Needed before the C helper calls below.
	 */
	adr	x0, __secure_stack_end
	lsl	x1, x20, #ARM_PSCI_STACK_SHIFT
	sub	x0, x0, x1
	mov	sp, x0

	/* Enable this core's GICv2 CPU interface (else "GIC CPU mask not found") */
	bl	tegra_gic_secondary_setup

	/* Install the EL3 exception vectors shared with the primary CPU */
	adr	x0, el3_exception_vectors
	msr	vbar_el3, x0

	/*
	 * Fetch the kernel entry point and PSCI context ID that psci_cpu_on()
	 * stashed for this CPU. The context ID must land in x0 on entry to the
	 * kernel, per the PSCI spec.
	 */
	adr	x0, tegra_cpu_entry
	ldr	x4, [x0, x20, lsl #3]
	adr	x0, tegra_cpu_context
	ldr	x0, [x0, x20, lsl #3]

	/* Drop to Non-secure EL2 (AArch64) at x4, preserving x0 = context ID */
	mov	x5, #ES_TO_AARCH64
	isb
	armv8_switch_to_el2_m x4, x5, x6
ENDPROC(tegra_secondary_entry)

/*
 * void tegra_cpu_powerdn_wfi(void)
 *
 * Cortex-A57 core power-down, called from psci_cpu_suspend_64() at EL3 with the
 * MMU and caches off. Cleans and invalidates the L1 data cache by set/way
 * (level 1 only, so the shared cluster L2 that the still-running cores rely on
 * is left intact), exits SMP coherency (CPUECTLR_EL1.SMPEN = 0), then WFIs. The
 * flow controller, armed by the caller, power-gates the core on WFI, so this
 * normally does not return: the core later warm-boots through
 * tegra_secondary_entry (above), which re-asserts SMPEN. If a wake event was
 * already pending the FC skips gating and WFI falls through; we then re-enter
 * coherency and return so the caller can report the core did not suspend.
 *
 * A private copy of the L1 set/way flush lives here rather than calling
 * U-Boot's __asm_dcache_level, because only ._secure.text is memreserved once
 * Linux has taken over the rest of U-Boot's memory. A57 is ARMv8.0 (no
 * FEAT_CCIDX), so the legacy CCSIDR_EL1 field layout is assumed.
 */
ENTRY(tegra_cpu_powerdn_wfi)
	dsb	sy
	msr	csselr_el1, xzr		/* select L1 (level 0), data cache */
	isb
	mrs	x1, ccsidr_el1
	ubfx	x2, x1, #0, #3		/* x2 = log2(line bytes) - 4 */
	add	x2, x2, #4		/* x2 = log2(line bytes) */
	ubfx	x3, x1, #3, #10		/* x3 = associativity - 1 (ways) */
	ubfx	x4, x1, #13, #15	/* x4 = number of sets - 1 */
	clz	w5, w3			/* w5 = bit position of the way field */
1:	/* set loop */
	mov	x6, x3			/* x6 = working copy of ways */
2:	/* way loop */
	lsl	x7, x6, x5		/* way << way-shift */
	lsl	x8, x4, x2		/* set << line-shift */
	orr	x7, x7, x8		/* set/way, cache level 0 (bits[3:1]=0) */
	dc	cisw, x7		/* clean & invalidate by set/way */
	subs	x6, x6, #1		/* next way */
	b.ge	2b
	subs	x4, x4, #1		/* next set */
	b.ge	1b
	dsb	sy
	isb

	/* Exit SMP coherency before the core rail drops. */
	mrs	x0, S3_1_C15_C2_1	/* CPUECTLR_EL1 */
	bic	x0, x0, #(1 << 6)	/* clear SMPEN */
	msr	S3_1_C15_C2_1, x0
	isb
	dsb	sy

	wfi				/* flow controller power-gates the core here */

	/* Spurious wake (FC skipped gating): restore coherency and return. */
	mrs	x0, S3_1_C15_C2_1
	orr	x0, x0, #(1 << 6)	/* set SMPEN */
	msr	S3_1_C15_C2_1, x0
	isb
	dsb	sy
	ret
ENDPROC(tegra_cpu_powerdn_wfi)

.popsection
