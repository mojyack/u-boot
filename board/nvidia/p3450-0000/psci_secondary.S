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
	/*
	 * Invalidate the branch predictor, as the T210's own firmware does on
	 * every core power-up and as nothing else here does.
	 *
	 * An A57 coming out of a Tegra core power-gate carries whatever its
	 * branch-target buffer happens to hold, and IC IALLU on its own does not
	 * clear it: on this part the BTB is only invalidated together with the
	 * I-cache, and only while CPUACTLR_EL1[0] is set. ARM Trusted Firmware
	 * does exactly this at the top of tegra_secure_entrypoint, gated on
	 * ERRATA_TEGRA_INVALIDATE_BTB_AT_BOOT, which its build enables for T210
	 * (and only T210 - the B01 respin is skipped at runtime).
	 *
	 * Without it a core that has just been powered up occasionally follows a
	 * stale branch prediction into nowhere and is never seen again. It shows
	 * up as a CPU_ON that times out ("CPUn: failed to come online", ~8% of
	 * bring-ups on this board's CPU3, 0% under ARM-TF) and as a core lost on
	 * a C7 resume, both of them dying in the same handful of instructions
	 * after the eret with memory demonstrably intact.
	 *
	 * This has to be the first thing the core does: CPUACTLR_EL1 may only be
	 * written "after a powerup reset, before the MMU is enabled and before
	 * any ACE or CHI traffic begins" (Cortex-A57 TRM 4.3.65), and the
	 * uncached read below is bus traffic. x0/x1 are dead here - this is a
	 * reset vector, entered with no live state.
	 */
	mrs	x0, S3_1_C15_C2_0		/* CPUACTLR_EL1 */
	orr	x1, x0, #1			/* invalidate BTB with I$ */
	msr	S3_1_C15_C2_0, x1
	dsb	sy
	isb
	ic	iallu				/* the actual invalidate */
	dsb	sy
	isb
	msr	S3_1_C15_C2_0, x0		/* restore */
	dsb	sy
	isb
	.rept	7				/* as ARM-TF has it: wait */
	nop
	.endr

	/*
	 * ...and then reset the core, once, before it runs anything else.
	 *
	 * This is the other half of the same T210 workaround. ARM Trusted
	 * Firmware, immediately after the invalidate above, checks OSLSR_EL1.OSLK
	 * and - if it is set, which on an A57 means this core has just come out
	 * of a real reset rather than out of the warm reset below - turns off
	 * regional clock gating and requests a warm reset of the core through
	 * RMR_EL3, then parks in WFI until it happens.  The core re-enters here
	 * with OSLK clear and falls through.
	 *
	 * So a T210 A57 never actually executes firmware in the state a
	 * power-gate leaves it in: the first thing it does is reset itself into a
	 * known one.  We were the exception, and the price was a core that
	 * occasionally never came back - a CPU_ON that timed out, or a lost core
	 * on a C7 resume, both dying within a few instructions of the eret.
	 *
	 * The reset has to be taken on *every* power-up. ARM-TF breaks the loop
	 * with OSLSR_EL1.OSLK, on the grounds that a reset sets it and the warm
	 * reset requested here does not; that test holds for a CPU_ON bring-up on
	 * this chain but not for a C7 resume, where it silently skips the reset -
	 * which leaves the failure alive on exactly the cores that gate rarely and
	 * gate long, and looks fixed under any load. A per-CPU word in
	 * ._secure.data is used instead: set on the way in, cleared on the way
	 * out, so it depends on nothing surviving the reset and is true once per
	 * power-up by construction.
	 * The trailing nops are ARM-TF's, to keep speculative execution away from
	 * the code after the reset request.
	 */
	mrs	x2, mpidr_el1
	and	x2, x2, #0xff
	adr	x1, tegra_warmboot_reset_pending
	ldr	w0, [x1, x2, lsl #2]
	cbnz	w0, 2f				/* pass two: the reset happened */
	mov	w0, #1
	str	w0, [x1, x2, lsl #2]
	dsb	sy
	msr	oslar_el1, xzr			/* as ARM-TF does before the reset */
	movz	x0, #0x8000, lsl #48		/* CPUACTLR_EL1: RCG off */
	msr	S3_1_C15_C2_0, x0
	isb
	mov	x0, #3				/* RMR_EL3: RR | AA64 */
	msr	rmr_el3, x0
	isb
	dsb	sy
1:	wfi
	b	1b
	.rept	65
	nop
	.endr
2:	str	wzr, [x1, x2, lsl #2]		/* arm it again for the next one */
	dsb	sy
	mov	x0, #1				/* restore the OS lock */
	msr	oslar_el1, x0
	isb

	/* I-cache on: with SCTLR_EL3.I clear every fetch is an uncached read */
	mrs	x0, sctlr_el3
	orr	x0, x0, #CR_I
	msr	sctlr_el3, x0
	isb

	/*
	 * Re-apply the Cortex-A57 erratum workarounds (errata.S). CPUACTLR_EL1
	 * is per-CPU and returns to its reset value whenever a core comes out
	 * of reset, so this is needed both for a CPU_ON bring-up and on every
	 * warm boot after a CPU_SUSPEND core power-gate - exactly as ARM
	 * Trusted Firmware re-runs its CPU reset handler on those paths. Must
	 * come before coherency and caches, and works without a stack.
	 */
	bl	tegra_a57_apply_errata

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
	 * Only now drop whatever this core's caches and TLBs came back with. A
	 * core reaching here has just come out of reset, so their contents are
	 * undefined and must not be trusted.
	 *
	 * This has to run with coherency already on. Doing IC IALLU, DC ISW and
	 * TLBI ALLE3 with SMPEN clear works for hundreds of C7 cycles and then
	 * locks the core up mid-sequence; ARM Trusted Firmware never has that
	 * window, because cortex_a57_reset_func() sets SMP in the reset handler
	 * and everything else follows.
	 */
	bl	tegra_cpu_warmboot_invalidate

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

	/*
	 * Re-do the SoC-level setup that an SC7 exit needs, now that the CPU
	 * interface is up. No-op on every other warm-boot path. See psci.c.
	 */
	bl	tegra_sc7_resume_finish

	/* Install the EL3 exception vectors shared with the primary CPU */
	adr	x0, el3_exception_vectors
	msr	vbar_el3, x0

	/*
	 * Fetch the kernel entry point and PSCI context ID that psci_cpu_on()
	 * stashed for this CPU. The context ID must land in x0 on entry to the
	 * kernel, per the PSCI spec.
	 *
	 * The entry is consumed: it is zeroed once read, so each armed entry
	 * matches exactly one warm boot. A core that gets here with no armed
	 * entry was power-gated outside any PSCI call - e.g. by a flow
	 * controller left armed across a plain kernel WFI - and jumping through
	 * the stale pointer would be a wild resume into a reused stack frame.
	 * Park the core instead, so the fault is a missing CPU rather than a
	 * silent corruption of whatever that frame has become.
	 */
	adr	x0, tegra_cpu_entry
	ldr	x4, [x0, x20, lsl #3]
	str	xzr, [x0, x20, lsl #3]
	cbz	x4, 9f
	adr	x0, tegra_cpu_context
	ldr	x0, [x0, x20, lsl #3]

	/* Drop to Non-secure EL2 (AArch64) at x4, preserving x0 = context ID */
	mov	x5, #ES_TO_AARCH64
	isb
	armv8_switch_to_el2_m x4, x5, x6, CR_I

9:	/* Unexpected warm boot: no armed entry, so there is nowhere to go. */
1:	wfi
	b	1b
ENDPROC(tegra_secondary_entry)

/*
 * void tegra_cpu_warmboot_invalidate(void)
 *
 * Warm-boot cache/TLB invalidation for a core that has just come out of reset,
 * run at EL3 with the MMU and caches off. This is the counterpart of ARM
 * Trusted Firmware running its CPU reset handler on every warm boot; without it
 * the core resumes from a flow-controller power-gate carrying whatever its L1
 * tags and data RAMs happen to hold.
 *
 * That was not theoretical: leaving it out corrupted page-cache pages. A page
 * freshly DMA'd from the SD card would come back with one 64-byte line (an A57
 * cache line) still holding the *previous* occupant of that physical page,
 * because a stale valid line in this core's L1 satisfied the load instead of
 * the DMA'd data. It showed up as random ELF/firmware files reading back wrong
 * - most visibly sshd's session binary, which segfaulted on every connection
 * until the page cache was dropped. Disabling the cpu-sleep idle state made it
 * go away; this makes it go away with cpu-sleep left on.
 *
 * Invalidate ONLY - never clean. After a power-gate the L1 contents are
 * undefined, so a clean would push undefined data out into memory.
 *
 * Leaf, no stack (it runs before the per-CPU EL3 stack is set up), clobbers
 * x1-x8. A57 is ARMv8.0 (no FEAT_CCIDX), so the legacy CCSIDR_EL1 layout is
 * assumed, as in tegra_cpu_powerdn_wfi() below.
 */
ENTRY(tegra_cpu_warmboot_invalidate)
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
	dc	isw, x7			/* invalidate by set/way */
	subs	x6, x6, #1		/* next way */
	b.ge	2b
	subs	x4, x4, #1		/* next set */
	b.ge	1b
	dsb	sy

	ic	iallu			/* L1 instruction cache + branch predictor */
	tlbi	alle3			/* EL3 TLB (we are at EL3 with MMU off) */
	dsb	sy
	isb
	ret
ENDPROC(tegra_cpu_warmboot_invalidate)

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
