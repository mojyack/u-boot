// SPDX-License-Identifier: GPL-2.0+
/*
 * Minimal Tegra210 (T210) ARMv8 PSCI backend for U-Boot.
 *
 * When U-Boot is RAM-booted on the Jetson Nano via RCM (see tegra-rcm), the
 * NVIDIA secure-world chain (nvtboot_cpu/TOS) is not loaded, so nothing at EL3
 * answers the kernel's PSCI SMCs. U-Boot itself enters at EL3 here (the RCM
 * bootstub keeps us there), so U-Boot installs the generic ARMv8 PSCI monitor
 * (arch/arm/cpu/armv8/psci.S) and drops the kernel to EL2-NS. This file
 * provides the T210-specific PSCI functions the generic layer leaves as weak
 * NOT_IMPLEMENTED stubs.
 *
 * Scope: CPU0 boot, SoC reset, and CPU_ON bring-up of the three secondary
 * A57s (the CAR-reset + PMC power-ungate sequence used by ARM Trusted
 * Firmware's t210 port). CPU_OFF is not implemented, so once a secondary is up
 * it stays up; that is enough for full SMP at boot. The woken cores land at EL3
 * in the tegra_secondary_entry trampoline (psci_secondary.S), which drops them
 * to EL2-NS at the kernel entry recorded below.
 *
 * CPU_SUSPEND implements the Tegra210 DT cpu-sleep idle-state (extended-format
 * param 0x40000007 = single-core power-down, PSTATE_ID_CORE_POWERDN): it arms
 * the flow controller to power-gate this core on WFI, cleans the L1 D-cache and
 * exits coherency, then WFIs. On the wake interrupt the flow controller powers
 * the core back up and it warm-boots through the same tegra_secondary_entry
 * trampoline, returning to the kernel's cpu_resume entry point. Modeled on ARM
 * Trusted Firmware's t210 tegra_fc_cpu_powerdn() path. See psci_cpu_suspend_64()
 * / psci_features().
 *
 * SYSTEM_SUSPEND implements SC7 ("deep" suspend-to-RAM): it hands the SC7 entry
 * firmware to the BPMP and power-gates with the CPU rail off, per ARM Trusted
 * Firmware's tegra_soc_pwr_domain_power_down_wfi(). Advertising it is what makes
 * the kernel offer /sys/power/state "deep" at all. SC7 turns VDD_CORE off, so
 * the resumed core comes back to a freshly reset SoC and has to put a good deal
 * of it back before the kernel can run again - see tegra_sc7_resume_finish().
 */

#include <asm/arch-tegra/tegra.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/io.h>
#include <asm/psci.h>
#include <asm/secure.h>
#include <linux/types.h>

#include "mc-security.h"

/*
 * Deliberately not including <asm/system.h>: it declares a single-argument
 * psci_features() prototype for the PSCI *client*, which conflicts with the
 * two-argument secure-monitor entry point below (same pattern as the imx8m
 * and stm32mp1 PSCI backends). wfi is issued inline instead.
 */
#define tegra_wfi()	asm volatile("wfi" ::: "memory")

#define MPIDR_CPU_MASK			0xff
#define SCTLR_EL3_I			BIT(12)

/*
 * PSCI_FEATURES return flag for CPU_SUSPEND: bit 1 advertises the extended
 * StateID power-state parameter format (PSCI_1_0_FEATURES_CPU_SUSPEND_PF_MASK
 * in the kernel). The Tegra210 cpu-sleep idle-state uses the extended format
 * (param 0x40000007); without this the kernel validates it against the shorter
 * original format and rejects it.
 */
#define PSCI_CPU_SUSPEND_EXT_POWER_STATE	(1 << 1)

/* Clock-and-Reset (CAR): CPU complex reset clear */
#define CLK_RST_CPU_CMPLX_RESET_CLR	0x454
#define CPU_CORE_RESET_MASK		0x10001

/* Flow controller (NV_PA_FLOW_BASE): per-CPU CSR and halt-event registers */
#define FLOWCTRL_WAITEVENT		(2 << 29)	/* HALT: FLOW_MODE_WAITEVENT */
#define FLOWCTRL_HALT_SCLK		(1 << 27)
#define FLOWCTRL_HALT_GIC_IRQ		(1 << 9)	/* HALT: wake on GIC IRQ */
#define FLOWCTRL_HALT_GIC_FIQ		(1 << 8)	/* HALT: wake on GIC FIQ */
#define FLOWCTRL_CSR_INTR_FLAG		(1 << 15)	/* CSR: W1C interrupt status */
#define FLOWCTRL_CSR_EVENT_FLAG		(1 << 14)	/* CSR: W1C event status */
#define FLOWCTRL_WAIT_WFI_BITMAP	(1 << 8)	/* CSR: gate when CPU<<n in WFI */
#define FLOWCTRL_CSR_ENABLE		(1 << 0)	/* CSR: power-gate enable (HW-cleared) */

/*
 * SYSTEM_SUSPEND (SC7) additions.
 *
 * SC7 turns off VDD_CPU and VDD_CORE and puts DRAM into self-refresh, so the
 * final sequence cannot run on the A57 at all: it runs on the BPMP, out of
 * IRAM, from the "SC7 entry firmware" blob. nvtboot has already placed that
 * blob for us (UART: "SC7 Entry Firmware - 0xff700000, 0x4000"); the carveout
 * sits above the top of System RAM (0xfedfffff) so it is still intact when the
 * kernel suspends. Ported from ARM Trusted Firmware's t210
 * tegra_soc_pwr_domain_power_down_wfi().
 */
#define FLOWCTRL_CC4_CORE0_CTRL		0x6c
#define FLOWCTRL_L2_FLUSH_CONTROL	0x94
#define FLOWCTRL_HALT_BPMP_EVENTS	0x04
#define FLOWCTRL_ENABLE_EXT		12
#define FLOWCTRL_TURNOFF_CPURAIL	0x2

#define TEGRA_IRAM_BASE			0x40000000
#define TEGRA_IRAM_A_SIZE		0x10000
#define TEGRA_RES_SEMA_BASE		0x60001000
#define RES_SEMA_STA			0x00
#define BPMP_SIGN_OF_LIFE		0xaaaaaaaa

#define TEGRA_EVP_BASE			0x6000f000
#define EVP_BPMP_RESET_VECTOR		0x200

#define CLK_RST_DEV_L_SET		0x300
#define CLK_RST_DEV_L_CLR		0x304
#define CLK_BPMP_RST			(1 << 1)
#define CLK_RST_BOND_OUT_H		0x74
#define CLK_RST_BOND_OUT_U		0x78
#define AHB_DMA_LOCK_BIT		(1 << 1)
#define APB_DMA_LOCK_BIT		(1 << 2)
#define IRAM_B_LOCK_BIT			(1 << 21)
#define IRAM_C_LOCK_BIT			(1 << 22)
#define IRAM_D_LOCK_BIT			(1 << 23)

#define APB_SLAVE_SECURITY_ENABLE	0xc00
#define PMC_SECURITY_EN_BIT		(1 << 13)

/*
 * Where nvtboot leaves the SC7 entry firmware, and the size of the header in
 * front of the code the BPMP actually executes.
 */
#define SC7ENTRY_FW_BASE		0xff700000
#define SC7ENTRY_FW_MAGIC		0x46374353	/* "SC7F" */
#define SC7ENTRY_FW_HEADER_SIZE		0x400
#define SC7ENTRY_FW_MAX_SIZE		0x10000

/* PMC (NV_PA_PMC_BASE): CPU-partition power gating + secure reset vector */
#define PMC_PWRGATE_TOGGLE		0x30
#define PMC_PWRGATE_TOGGLE_START	(1 << 8)
#define PMC_PWRGATE_STATUS		0x38
#define PMC_SECURE_SCRATCH34		0x368	/* AA64 reset vector, low */
#define PMC_SECURE_SCRATCH35		0x36c	/* AA64 reset vector, high */

/* Secure Boot (0x6000C200): AArch64 non-secure reset vector */
#define NV_PA_SB_BASE			0x6000c200
#define SB_AA64_RESET_LOW		0x30
#define SB_AA64_RESET_HIGH		0x34

/* GICv2 (GIC-400): per-CPU interface enable, mirroring gic_init_secure_percpu */
#define TEGRA_GICD_BASE			0x50041000
#define TEGRA_GICC_BASE			0x50042000
#define GICD_CTLR			0x000
#define GICD_TYPER			0x004
#define GICD_TYPER_ITLINES		0x1f
#define GICD_IGROUPR0			0x080
#define GICD_ISENABLER0			0x100
#define GICD_ICENABLER0			0x180
#define GICD_IPRIORITYR0		0x400
#define GICD_ITARGETSR0			0x800
#define GICD_ICFGR0			0xc00
#define GICC_CTLR			0x000
#define GICC_PMR			0x004
#define GICC_BPR			0x008

/*
 * Interrupt-ID banks to save across SC7. The GIC-400 on T210 reports
 * ITLinesNumber = 6, i.e. 7 banks of 32 = 224 interrupt IDs; the arrays are
 * sized for that and the actual count is clamped to it at run time.
 */
#define GIC_MAX_BANKS			7
#define GIC_MAX_INTIDS			(GIC_MAX_BANKS * 32)

/* MSELECT: the AXI switch between the CPU cluster and the peripheral bus */
#define TEGRA_MSELECT_BASE		0x50060000
#define MSELECT_CONFIG			0x00
#define MSELECT_UNSUP_TX_ERRORS		((1 << 25) | (1 << 24))
#define MSELECT_WRAP_TO_INCR_BURSTS	((1 << 29) | (1 << 28) | (1 << 27))

/* PMC: IO-pad deep-power-down, cleared on the way out of SC7 */
#define PMC_IO_DPD_SAMPLE		0x20
#define PMC_DPD_ENABLE			0x24

/*
 * PMC power-partition ID for each CPU, and per-CPU flow-controller register
 * offsets, as in the ARM-TF t210 port.
 */
static u32 pmc_cpu_powergate_id[4] __secure_data = { 14, 9, 10, 11 };
static u32 flowctrl_csr_off[4] __secure_data = { 0x08, 0x18, 0x20, 0x28 };
static u32 flowctrl_halt_off[4] __secure_data = { 0x00, 0x14, 0x1c, 0x24 };

/*
 * Kernel entry point and PSCI context ID handed to each secondary. Written by
 * psci_cpu_on() (EL3, MMU/caches off => straight to DRAM) and read by the
 * tegra_secondary_entry trampoline. Non-static so the trampoline can address
 * them. cpu_started tracks whether a core has ever been power-ungated (first
 * bring-up goes through the PMC, a later one through the flow controller).
 */
u64 tegra_cpu_entry[4] __secure_data;
u64 tegra_cpu_context[4] __secure_data;
/*
 * Set on the way into the warm-boot self-reset (psci_secondary.S) and cleared
 * on the way out, so the reset is taken exactly once per power-up, without
 * depending on any architectural bit surviving it.
 */
u32 tegra_warmboot_reset_pending[4] __secure_data;
u64 tegra_boot_cntfrq __secure_data;
/*
 * Set by psci_system_suspend_64() just before the SoC powers down, so that the
 * warm boot that follows can tell an SC7 exit (the whole SoC came back from
 * reset) from an ordinary core power-up. Cleared by tegra_sc7_resume_finish().
 */
static u32 tegra_sc7_resume_pending __secure_data;

/*
 * GIC context saved on the way into SC7 and put back on the way out. See
 * tegra_gic_save()/tegra_gic_restore().
 */
static struct {
	u32 ctlr;
	u32 banks;
	u32 igroupr[GIC_MAX_BANKS];
	u32 isenabler[GIC_MAX_BANKS];
	u32 ipriorityr[GIC_MAX_INTIDS / 4];
	u32 itargetsr[GIC_MAX_INTIDS / 4];
	u32 icfgr[GIC_MAX_INTIDS / 16];
	u32 cpu_ctlr;
	u32 cpu_pmr;
	u32 cpu_bpr;
} tegra_gic_ctx __secure_data;
static u8 cpu_started[4] __secure_data;
static u8 psci_cpu_state[4] __secure_data = {
	PSCI_AFFINITY_LEVEL_ON,
	PSCI_AFFINITY_LEVEL_OFF,
	PSCI_AFFINITY_LEVEL_OFF,
	PSCI_AFFINITY_LEVEL_OFF,
};

extern void tegra_secondary_entry(void);

/*
 * EL3 core power-down helper (psci_secondary.S): cleans the L1 D-cache, exits
 * SMP coherency and WFIs so the armed flow controller can power-gate this core.
 * Normally does not return (the core warm-boots via tegra_secondary_entry);
 * returns only if a pending wake made the FC skip gating.
 */
extern void tegra_cpu_powerdn_wfi(void);

u32 __secure psci_version(void)
{
	return ARM_PSCI_VER_1_0;
}

/*
 * Enable this core's GICv2 CPU interface at EL3, mirroring what U-Boot's
 * gic_init_secure_percpu() did for the boot CPU. Without it the distributor
 * never associates a CPU-interface id with this core, so the kernel reads its
 * GICD_ITARGETSR mask as 0 ("GIC CPU mask not found - kernel will fail to
 * boot") and SMP wedges. Called from the trampoline on the secondary at EL3.
 */
void __secure tegra_gic_secondary_setup(void)
{
	/* SGIs+PPIs to Group1, enable SGI0 (banked to this CPU). */
	writel(0xffffffff, TEGRA_GICD_BASE + GICD_IGROUPR0);
	writel(0x1, TEGRA_GICD_BASE + GICD_ISENABLER0);
	/* Secure GICC_CTLR: EnableGrp0|Grp1, AckCtl, FIQEn, bypass disabled. */
	writel(0x1e7, TEGRA_GICC_BASE + GICC_CTLR);
	/* Allow non-secure access to GICC_PMR. */
	writel(0x1 << 7, TEGRA_GICC_BASE + GICC_PMR);
}

/*
 * Save and restore the GIC across SC7.
 *
 * SC7 drops VDD_CORE, so the GIC-400 comes back completely reset - distributor
 * disabled, every interrupt masked, priorities, targets and trigger types all
 * zero. Linux does not put any of that back: on the PSCI SYSTEM_SUSPEND path it
 * issues no cpu_pm notifications (those are a cpuidle thing), so neither
 * gic_dist_restore() nor gic_cpu_restore() ever runs, and its own resume only
 * re-enables the individual device interrupts it had disabled. In particular
 * nothing re-enables the boot CPU's private arch-timer PPI - which is why
 * without this the resumed kernel wedges in the first driver resume that sleeps.
 *
 * So EL3 owns this: take a copy of the distributor and CPU-interface state while
 * it is still valid, and put it back before the kernel gets control again.
 *
 * Only the boot CPU's banked registers (INTIDs 0-31) are captured, because it is
 * the only core still running by the time SYSTEM_SUSPEND is called. The other
 * cores are brought back with CPU_ON afterwards, and Linux initialises their
 * banked state itself from gic_starting_cpu().
 */
static void __secure tegra_gic_save(void)
{
	u32 banks = (readl(TEGRA_GICD_BASE + GICD_TYPER) & GICD_TYPER_ITLINES) + 1;
	u32 i;

	if (banks > GIC_MAX_BANKS)
		banks = GIC_MAX_BANKS;
	tegra_gic_ctx.banks = banks;

	tegra_gic_ctx.ctlr = readl(TEGRA_GICD_BASE + GICD_CTLR);

	for (i = 0; i < banks; i++) {
		tegra_gic_ctx.igroupr[i] =
			readl(TEGRA_GICD_BASE + GICD_IGROUPR0 + i * 4);
		tegra_gic_ctx.isenabler[i] =
			readl(TEGRA_GICD_BASE + GICD_ISENABLER0 + i * 4);
	}
	for (i = 0; i < banks * 32 / 4; i++) {
		tegra_gic_ctx.ipriorityr[i] =
			readl(TEGRA_GICD_BASE + GICD_IPRIORITYR0 + i * 4);
		tegra_gic_ctx.itargetsr[i] =
			readl(TEGRA_GICD_BASE + GICD_ITARGETSR0 + i * 4);
	}
	for (i = 0; i < banks * 32 / 16; i++)
		tegra_gic_ctx.icfgr[i] =
			readl(TEGRA_GICD_BASE + GICD_ICFGR0 + i * 4);

	tegra_gic_ctx.cpu_ctlr = readl(TEGRA_GICC_BASE + GICC_CTLR);
	tegra_gic_ctx.cpu_pmr = readl(TEGRA_GICC_BASE + GICC_PMR);
	tegra_gic_ctx.cpu_bpr = readl(TEGRA_GICC_BASE + GICC_BPR);

	asm volatile("dsb sy" ::: "memory");
}

static void __secure tegra_gic_restore(void)
{
	u32 banks = tegra_gic_ctx.banks;
	u32 i;

	if (!banks)
		return;

	/* Configure with the distributor off, exactly as Linux's own restore. */
	writel(0, TEGRA_GICD_BASE + GICD_CTLR);

	for (i = 0; i < banks * 32 / 16; i++)
		writel(tegra_gic_ctx.icfgr[i],
		       TEGRA_GICD_BASE + GICD_ICFGR0 + i * 4);
	for (i = 0; i < banks * 32 / 4; i++) {
		writel(tegra_gic_ctx.ipriorityr[i],
		       TEGRA_GICD_BASE + GICD_IPRIORITYR0 + i * 4);
		writel(tegra_gic_ctx.itargetsr[i],
		       TEGRA_GICD_BASE + GICD_ITARGETSR0 + i * 4);
	}
	for (i = 0; i < banks; i++)
		writel(tegra_gic_ctx.igroupr[i],
		       TEGRA_GICD_BASE + GICD_IGROUPR0 + i * 4);

	/* Enables last, and only after clearing whatever reset left behind. */
	for (i = 0; i < banks; i++) {
		writel(0xffffffff, TEGRA_GICD_BASE + GICD_ICENABLER0 + i * 4);
		writel(tegra_gic_ctx.isenabler[i],
		       TEGRA_GICD_BASE + GICD_ISENABLER0 + i * 4);
	}

	writel(tegra_gic_ctx.ctlr, TEGRA_GICD_BASE + GICD_CTLR);

	writel(tegra_gic_ctx.cpu_pmr, TEGRA_GICC_BASE + GICC_PMR);
	writel(tegra_gic_ctx.cpu_bpr, TEGRA_GICC_BASE + GICC_BPR);
	writel(tegra_gic_ctx.cpu_ctlr, TEGRA_GICC_BASE + GICC_CTLR);

	asm volatile("dsb sy" ::: "memory");
}

/*
 * SoC-level fixups for an SC7 exit, run at EL3 on the resumed boot CPU from the
 * warm-boot trampoline, straight after tegra_gic_secondary_setup() and before
 * the drop to the kernel. Does nothing on the CPU_ON and CPU_SUSPEND warm-boot
 * paths, where the SoC never lost power.
 *
 * SC7 turns VDD_CORE off, so everything outside the always-on PMC domain comes
 * back at its reset values. The kernel does not expect that: on the PSCI
 * SYSTEM_SUSPEND path it issues no cpu_pm notifications at all (those come from
 * cpuidle only), so nothing re-runs gic_cpu_init()/gic_cpu_restore() or any
 * other CPU-level re-init on the way out. Whatever the resumed CPU needs, EL3
 * has to put back - which is what ARM Trusted Firmware does from
 * tegra_pwr_domain_on_finish()/tegra_soc_pwr_domain_on_finish() for the
 * PSTATE_ID_SOC_POWERDN case.
 *
 * The GIC CPU interface is the one that matters: without this the board wakes,
 * resumes the kernel and then wedges in the first driver resume that sleeps,
 * because no interrupt - not even the arch timer - can reach the CPU.
 */
void __secure tegra_sc7_resume_finish(void)
{
	u32 val;

	if (!tegra_sc7_resume_pending)
		return;
	tegra_sc7_resume_pending = 0;

	/*
	 * Put the GIC back the way Linux had it. This overwrites what
	 * tegra_gic_secondary_setup() just programmed, which is deliberate:
	 * that function sets up a *fresh* CPU interface, which is right for a
	 * core being brought up but not for one rejoining a running kernel.
	 */
	tegra_gic_restore();

	/*
	 * Take the IO pads back out of deep power down. The SC7 entry firmware
	 * put them there on the way down and only the PMC survived to remember
	 * it (ARM-TF's tegra_pmc_resume()).
	 */
	writel(0, NV_PA_PMC_BASE + PMC_IO_DPD_SAMPLE);
	writel(0, NV_PA_PMC_BASE + PMC_DPD_ENABLE);

	/*
	 * Re-apply the MSELECT settings the AXI switch loses with VDD_CORE:
	 * convert WRAP bursts to INCR on the way to the peripheral bus, and do
	 * not raise errors for unsupported transactions (ARM-TF does this on
	 * the same path).
	 */
	val = readl(TEGRA_MSELECT_BASE + MSELECT_CONFIG);
	val &= ~MSELECT_UNSUP_TX_ERRORS;
	val |= MSELECT_WRAP_TO_INCR_BURSTS;
	writel(val, TEGRA_MSELECT_BASE + MSELECT_CONFIG);

	/*
	 * The memory controller is in the VDD_CORE domain too, so the secure-only
	 * settings U-Boot made for Linux at cold boot - SMMU enable, VPR lock,
	 * GPU WPR carveout - are gone. Redo them; without the carveout the GPU
	 * comes back with a garbage WPR aperture and nouveau's resume fails in
	 * ACR with -110.
	 */
	tegra_mc_security_setup();
}

/* Undo cleanup_before_linux()'s I-cache disable for the boot CPU's EL3 */
void __secure psci_arch_init(void)
{
	u64 sctlr;

	asm volatile("mrs %0, sctlr_el3" : "=r"(sctlr));
	asm volatile("msr sctlr_el3, %0; isb" : : "r"(sctlr | SCTLR_EL3_I));
}

/* Point the Tegra secure reset vectors at our secondary-CPU trampoline. */
static void __secure tegra_setup_cpu_reset_vector(void)
{
	u64 addr = (u64)(uintptr_t)tegra_secondary_entry;
	u32 lo = ((u32)addr) | 1;	/* bit0 selects AArch64 */
	u32 hi = (u32)(addr >> 32) & 0x7ff;

	writel(lo, NV_PA_SB_BASE + SB_AA64_RESET_LOW);
	writel(hi, NV_PA_SB_BASE + SB_AA64_RESET_HIGH);
	writel(lo, NV_PA_PMC_BASE + PMC_SECURE_SCRATCH34);
	writel(hi, NV_PA_PMC_BASE + PMC_SECURE_SCRATCH35);
}

/* First-time bring-up: power-ungate the CPU partition via the PMC. */
static void __secure tegra_pmc_cpu_on(u32 cpu)
{
	u32 id = pmc_cpu_powergate_id[cpu];

	if (readl(NV_PA_PMC_BASE + PMC_PWRGATE_STATUS) & (1 << id))
		return;

	/* Wait for any in-progress toggle to finish. */
	while (readl(NV_PA_PMC_BASE + PMC_PWRGATE_TOGGLE) & PMC_PWRGATE_TOGGLE_START)
		;

	writel(id | PMC_PWRGATE_TOGGLE_START, NV_PA_PMC_BASE + PMC_PWRGATE_TOGGLE);

	/* Wait for the toggle to be accepted, then for the partition to power up. */
	while (readl(NV_PA_PMC_BASE + PMC_PWRGATE_TOGGLE) & PMC_PWRGATE_TOGGLE_START)
		;
	while (!(readl(NV_PA_PMC_BASE + PMC_PWRGATE_STATUS) & (1 << id)))
		;
}

/* Subsequent bring-up: unhalt the CPU via the flow controller. */
static void __secure tegra_fc_cpu_on(u32 cpu)
{
	writel(FLOWCTRL_CSR_ENABLE, NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
	writel(FLOWCTRL_WAITEVENT | FLOWCTRL_HALT_SCLK,
	       NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
}

s32 __secure psci_cpu_on_64(u32 __always_unused function_id, u64 mpidr,
			    u64 entry_point, u64 context_id)
{
	u32 cpu = mpidr & MPIDR_CPU_MASK;

	if (mpidr & ~((u64)MPIDR_CPU_MASK))
		return ARM_PSCI_RET_INVAL;
	if (cpu == 0 || cpu >= 4)
		return ARM_PSCI_RET_INVAL;
	if (psci_cpu_state[cpu] == PSCI_AFFINITY_LEVEL_ON)
		return ARM_PSCI_RET_ALREADY_ON;

	tegra_cpu_entry[cpu] = entry_point;
	tegra_cpu_context[cpu] = context_id;
	asm volatile("mrs %0, cntfrq_el0" : "=r"(tegra_boot_cntfrq));
	tegra_setup_cpu_reset_vector();
	psci_cpu_state[cpu] = PSCI_AFFINITY_LEVEL_ON;

	/*
	 * Order the hand-off stores before the target CPU is released. No cache
	 * maintenance is needed: U-Boot's cleanup_before_linux() turned the EL3
	 * MMU and caches off, so these stores are non-cacheable and reach DRAM
	 * directly - the same DRAM the secondary reads with its caches still off.
	 */
	asm volatile("dsb sy; isb" ::: "memory");

	/* Deassert the CPU's reset in the clock-and-reset block. */
	writel(CPU_CORE_RESET_MASK << cpu,
	       NV_PA_CLK_RST_BASE + CLK_RST_CPU_CMPLX_RESET_CLR);

	if (!cpu_started[cpu]) {
		tegra_pmc_cpu_on(cpu);
		cpu_started[cpu] = 1;
	} else {
		tegra_fc_cpu_on(cpu);
	}

	return ARM_PSCI_RET_SUCCESS;
}

s32 __secure psci_cpu_on(u32 function_id, u32 mpidr, u32 entry_point,
			 u32 context_id)
{
	return psci_cpu_on_64(function_id, mpidr, entry_point, context_id);
}

/*
 * Arm the flow controller to power-gate this core the next time it executes
 * WFI, waking on any GIC interrupt. Mirrors ARM-TF t210's
 * tegra_fc_prepare_suspend(cpu, 0) for the core-power-down case. The CSR ENABLE
 * bit is cleared by hardware once the power-gate sequence completes, so no
 * disarm is needed on resume - a subsequent plain kernel WFI will not re-gate.
 */
/*
 * Busy-wait, using the architectural counter. udelay() lives in normal-world
 * U-Boot text, which the relocated secure section must not call into, so spin
 * on CNTPCT_EL0 instead.
 */
static void __secure tegra_udelay(u32 us)
{
	u64 freq, start, now;

	asm volatile("mrs %0, cntfrq_el0" : "=r"(freq));
	asm volatile("isb; mrs %0, cntpct_el0" : "=r"(start));
	do {
		asm volatile("isb; mrs %0, cntpct_el0" : "=r"(now));
	} while ((now - start) < ((freq * us) / 1000000));
}

/* ARM-TF's tegra_fc_bpmp_off()/on(): park or launch the BPMP. */
static void __secure tegra_fc_bpmp_off(void)
{
	writel(FLOWCTRL_WAITEVENT, NV_PA_FLOW_BASE + FLOWCTRL_HALT_BPMP_EVENTS);
	writel(CLK_BPMP_RST, NV_PA_CLK_RST_BASE + CLK_RST_DEV_L_SET);
	writel(0, TEGRA_EVP_BASE + EVP_BPMP_RESET_VECTOR);
	while (readl(TEGRA_EVP_BASE + EVP_BPMP_RESET_VECTOR) != 0)
		;
}

static void __secure tegra_fc_bpmp_on(u32 entrypoint)
{
	writel(FLOWCTRL_WAITEVENT, NV_PA_FLOW_BASE + FLOWCTRL_HALT_BPMP_EVENTS);
	writel(CLK_BPMP_RST, NV_PA_CLK_RST_BASE + CLK_RST_DEV_L_SET);

	writel(entrypoint, TEGRA_EVP_BASE + EVP_BPMP_RESET_VECTOR);
	while (readl(TEGRA_EVP_BASE + EVP_BPMP_RESET_VECTOR) != entrypoint)
		;

	tegra_udelay(2);

	writel(CLK_BPMP_RST, NV_PA_CLK_RST_BASE + CLK_RST_DEV_L_CLR);
	writel(0, NV_PA_FLOW_BASE + FLOWCTRL_HALT_BPMP_EVENTS);
}

/*
 * ARM-TF's tegra_fc_prepare_suspend(): arm the flow controller to gate this
 * core once it WFIs. csr_extra carries the extra CSR bits the SoC power-down
 * needs (turn the CPU rail off) and is 0 for a plain core power-down.
 */
static void __secure tegra_fc_prepare_suspend(u32 cpu, u32 csr_extra)
{
	u32 halt = FLOWCTRL_HALT_GIC_IRQ | FLOWCTRL_HALT_GIC_FIQ |
		   FLOWCTRL_WAITEVENT;
	u32 csr = FLOWCTRL_CSR_INTR_FLAG | FLOWCTRL_CSR_EVENT_FLAG |
		  FLOWCTRL_CSR_ENABLE | (FLOWCTRL_WAIT_WFI_BITMAP << cpu);

	writel(halt, NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
	writel(csr | csr_extra, NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
}

static void __secure tegra_fc_cpu_powerdn(u32 cpu)
{
	tegra_fc_prepare_suspend(cpu, 0);
}

/*
 * ARM-TF's tegra_fc_soc_powerdn(): same as a core power-down, plus turning the
 * CPU rail off and asking for an L2 flush, and with the HALT register
 * overwritten afterwards so only the wait-event mode remains.
 */
static void __secure tegra_fc_soc_powerdn(u32 cpu)
{
	/* Disable CC4 (cluster retention) arbitration for this core. */
	writel(0, NV_PA_FLOW_BASE + FLOWCTRL_CC4_CORE0_CTRL + (cpu * 4));
	readl(NV_PA_FLOW_BASE + FLOWCTRL_CC4_CORE0_CTRL + (cpu * 4));

	writel(1, NV_PA_FLOW_BASE + FLOWCTRL_L2_FLUSH_CONTROL);
	readl(NV_PA_FLOW_BASE + FLOWCTRL_L2_FLUSH_CONTROL);

	tegra_fc_prepare_suspend(cpu, FLOWCTRL_TURNOFF_CPURAIL << FLOWCTRL_ENABLE_EXT);

	writel(FLOWCTRL_WAITEVENT, NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
}

/*
 * Hand the SC7 entry firmware to the BPMP and start it. The BPMP is the only
 * thing that can finish the sequence: it puts DRAM into self-refresh and drives
 * the PMIC to drop the rails, both of which outlive the A57.
 *
 * Returns false if the firmware does not look present or the BPMP never signs
 * on, in which case the caller must not power down.
 */
static bool __secure tegra_bpmp_sc7_start(void)
{
	const u32 *src = (const u32 *)(uintptr_t)(SC7ENTRY_FW_BASE +
						  SC7ENTRY_FW_HEADER_SIZE);
	u32 *dst = (u32 *)(uintptr_t)TEGRA_IRAM_BASE;
	u32 size, val, i;
	int timeout;

	/*
	 * nvtboot loads the partition verbatim, so this is the usual signed
	 * boot-component container: u32 reserved, 4-byte magic, u32 total
	 * length. Check the magic before trusting the length.
	 */
	if (readl(SC7ENTRY_FW_BASE + 4) != SC7ENTRY_FW_MAGIC)
		return false;
	size = readl(SC7ENTRY_FW_BASE + 8);
	if (size <= SC7ENTRY_FW_HEADER_SIZE || size > SC7ENTRY_FW_MAX_SIZE)
		return false;
	size -= SC7ENTRY_FW_HEADER_SIZE;

	/* Park the BPMP while we rewrite its IRAM. */
	tegra_fc_bpmp_off();

	/* Bond out the IRAM banks and the DMA engines the firmware must own. */
	writel(IRAM_B_LOCK_BIT | IRAM_C_LOCK_BIT | IRAM_D_LOCK_BIT,
	       NV_PA_CLK_RST_BASE + CLK_RST_BOND_OUT_U);
	writel(APB_DMA_LOCK_BIT | AHB_DMA_LOCK_BIT,
	       NV_PA_CLK_RST_BASE + CLK_RST_BOND_OUT_H);

	/* The BPMP runs non-secure, so it needs the PMC opened up to it. */
	val = readl(NV_PA_APB_MISC_BASE + APB_SLAVE_SECURITY_ENABLE);
	val &= ~PMC_SECURITY_EN_BIT;
	writel(val, NV_PA_APB_MISC_BASE + APB_SLAVE_SECURITY_ENABLE);

	for (i = 0; i < TEGRA_IRAM_A_SIZE / 4; i++)
		dst[i] = 0;
	for (i = 0; i < size / 4; i++)
		dst[i] = src[i];
	asm volatile("dsb sy" ::: "memory");

	tegra_fc_bpmp_on(TEGRA_IRAM_BASE);

	/* Wait for the firmware to sign on before we let the A57 go. */
	for (timeout = 100000; timeout > 0; timeout--) {
		if (readl(TEGRA_RES_SEMA_BASE + RES_SEMA_STA) == BPMP_SIGN_OF_LIFE)
			return true;
		tegra_udelay(1);
	}

	return false;
}

/*
 * Undo an arm that did not result in a gate.
 *
 * CSR ENABLE is cleared by hardware only when a power-gate sequence actually
 * completes (TRM 17.2.3). If the WFI below falls through instead - WFI completes
 * immediately whenever a physical interrupt is pending, regardless of PSTATE
 * masking, and we leave the GIC CPU interface enabled - the flow controller
 * stays armed while we return to the kernel. The next *plain* kernel WFI
 * (cpuidle state0, entered hundreds of times a second on an idle core) would
 * then power-gate the core outside any PSCI call, and it would warm boot into
 * the stale entry of the previous suspend. Clear the latched wake status and
 * drop the arm, then put the wake enables back so the HALT register reads as it
 * did before.
 *
 * ARM-TF never needs this because its power-down WFI path is __dead2; we return
 * per the PSCI spec's shallow-wake option, so we must disarm.
 */
static void __secure tegra_fc_cpu_disarm(u32 cpu)
{
	u32 halt = FLOWCTRL_HALT_GIC_IRQ | FLOWCTRL_HALT_GIC_FIQ;

	writel(FLOWCTRL_CSR_INTR_FLAG | FLOWCTRL_CSR_EVENT_FLAG,
	       NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
	writel(halt, NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
}

/*
 * Disable this core's GICv2 CPU interface, as ARM-TF does from
 * tegra_gic_cpuif_deactivate() at the end of tegra_pwr_domain_off(): with the
 * interface off nothing can assert an interrupt at this core, so it stays
 * power-gated until a CPU_ON ungates it. tegra_gic_secondary_setup() brings the
 * interface back wholesale on the warm-boot path.
 */
static void __secure tegra_gic_cpuif_deactivate(void)
{
	writel(0, TEGRA_GICC_BASE + GICC_CTLR);
	readl(TEGRA_GICC_BASE + GICC_CTLR);
}

/*
 * Arm the flow controller to power-gate this core permanently: same as
 * tegra_fc_cpu_powerdn() above but with no wake-event enables in HALT, so no
 * GIC or LIC interrupt can bring the core back.
 */
static void __secure tegra_fc_cpu_off(u32 cpu)
{
	u32 csr = FLOWCTRL_CSR_INTR_FLAG | FLOWCTRL_CSR_EVENT_FLAG |
		  FLOWCTRL_CSR_ENABLE | (FLOWCTRL_WAIT_WFI_BITMAP << cpu);

	writel(csr, NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
	writel(FLOWCTRL_WAITEVENT, NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
}

/*
 * CPU_OFF: take the calling core out of the system until some other core brings
 * it back with CPU_ON. This is what backs Linux CPU hotplug, and the secondary
 * half of reboot and kexec; without it the generic NOT_IMPLEMENTED stub makes
 * the kernel BUG in cpu_die().
 *
 * Modelled on ARM-TF t210's tegra_soc_pwr_domain_off() plus the
 * tegra_gic_cpuif_deactivate() that tegra_pwr_domain_off() appends. It differs
 * from CPU_SUSPEND in exactly two ways: the flow controller is armed with no
 * wake-event enables, and this core's GIC CPU interface is switched off - so
 * nothing can assert an interrupt at it and it stays gated. CPU_SUSPEND must do
 * neither, since the GIC is its only wake source.
 *
 * The affinity state is published before the gate, not after: the core calling
 * this is the one being switched off, so no code of ours runs afterwards, and
 * the kernel's psci_cpu_kill() spins on AFFINITY_INFO from another core until it
 * reads OFF.
 */
s32 __secure psci_cpu_off(void)
{
	u64 mpidr;
	u32 cpu;

	asm volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
	cpu = mpidr & MPIDR_CPU_MASK;
	if (cpu == 0 || cpu >= 4)
		return ARM_PSCI_RET_DENIED;

	/* Retire any resume entry: "entry is armed" must keep meaning "a gate is
	 * armed and still pending" for the trampoline's stale-entry trap. */
	tegra_cpu_entry[cpu] = 0;
	tegra_cpu_context[cpu] = 0;
	psci_cpu_state[cpu] = PSCI_AFFINITY_LEVEL_OFF;
	asm volatile("dsb sy; isb" ::: "memory");

	tegra_gic_cpuif_deactivate();

	for (;;) {
		tegra_fc_cpu_off(cpu);
		asm volatile("dsb sy" ::: "memory");
		tegra_cpu_powerdn_wfi();
	}
}

/*
 * CPU_SUSPEND for the DT cpu-sleep state (0x40000007 => single-core power-down).
 * The kernel has already saved its CPU context (cpu_suspend()) and calls this
 * from its suspend finisher with entry_point = cpu_resume. We record that entry
 * for the resume trampoline, arm the flow controller, and power-gate via WFI.
 *
 * On the wake interrupt the flow controller powers the core back up; it comes
 * out of reset at tegra_secondary_entry (the reset vector we (re)install here),
 * which re-does the per-CPU EL3 setup and drops to the recorded entry point at
 * EL2-NS. This call therefore normally does not return. It returns only if the
 * core did not actually power down (a wake event was already pending, so the FC
 * skipped gating and WFI fell through); tegra_cpu_powerdn_wfi() will have
 * restored coherency, and we report the shallow outcome so the kernel unwinds.
 *
 * power_state is not decoded: cpu-sleep is the only idle-state and it always
 * maps to a core power-down.
 */
s32 __secure psci_cpu_suspend_64(u32 __always_unused function_id,
				 u32 __always_unused power_state,
				 u64 entry_point, u64 context_id)
{
	u64 mpidr;
	u32 cpu;

	asm volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
	cpu = mpidr & MPIDR_CPU_MASK;
	if (cpu >= 4)
		return ARM_PSCI_RET_INVAL;

	/* Warm-boot hand-off for the resume trampoline (EL3, caches off). */
	tegra_cpu_entry[cpu] = entry_point;
	tegra_cpu_context[cpu] = context_id;
	asm volatile("mrs %0, cntfrq_el0" : "=r"(tegra_boot_cntfrq));
	tegra_setup_cpu_reset_vector();

	/* Order the hand-off stores before the core is released to power-gate. */
	asm volatile("dsb sy; isb" ::: "memory");

	tegra_fc_cpu_powerdn(cpu);

	/* Flush L1, leave coherency, WFI: the FC power-gates us here. */
	tegra_cpu_powerdn_wfi();

	/*
	 * Reached only if the core did not power down (WFI fell through on an
	 * already-pending wake); coherency is restored. The flow controller is
	 * still armed, so disarm it, and retire the resume entry with it: "entry
	 * is set" must keep meaning "a gate is armed and still pending", which is
	 * the invariant the trampoline's stale-entry trap tests against.
	 */
	tegra_fc_cpu_disarm(cpu);
	tegra_cpu_entry[cpu] = 0;
	asm volatile("dsb sy" ::: "memory");

	return ARM_PSCI_RET_SUCCESS;
}

s32 __secure psci_cpu_suspend(u32 function_id, u32 power_state,
			      u32 entry_point, u32 context_id)
{
	return psci_cpu_suspend_64(function_id, power_state, entry_point,
				   context_id);
}

/*
 * SYSTEM_SUSPEND (SC7 / "deep"). The kernel calls this on the last online CPU
 * with its context already saved and entry_point = cpu_resume; every other core
 * is OFF. We record the resume entry exactly like CPU_SUSPEND does, hand the
 * SC7 entry firmware to the BPMP, and power-gate with the CPU rail turned off.
 *
 * From there the BPMP finishes the job: DRAM to self-refresh, rails off. On a
 * wake the BootROM runs the warm-boot (WB0) code, which brings SDRAM back and
 * jumps to the AArch64 reset vector we install below -- the same
 * tegra_secondary_entry trampoline the CPU_ON/CPU_SUSPEND paths use, which
 * drops to the recorded entry at EL2-NS, by way of tegra_sc7_resume_finish().
 * So this call does not return on a successful suspend.
 *
 * Deliberately not ported from ARM-TF: tegra_se_suspend(), the software
 * Security Engine context save it does here on plain T210
 * (tegra_se_context_save_sw, plat/nvidia/tegra/soc/t210/drivers/se/). The
 * BootROM verifies and restores that context on a warm boot, so it looked like
 * a prerequisite - but the warm boot demonstrably succeeds without it: the
 * warm-boot firmware reaches its "WBD" marker and releases the CPU with
 * PMC_SCRATCH43 holding nothing but leftover garbage. What that costs is the SE
 * key/RSA state, which nothing on this board relies on across a suspend.
 *
 * Also skipped: ARM-TF's defensive reset of every DMA master before handing the
 * SoC to the BPMP (tegra_reset_all_dma_masters()). The kernel has already
 * suspended its drivers, so nothing should be mastering; it is a hardening step
 * rather than a correctness one.
 */
s32 __secure psci_system_suspend_64(u32 __always_unused function_id,
				    u64 entry_point, u64 context_id)
{
	u64 mpidr;
	u32 cpu;

	asm volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
	cpu = mpidr & MPIDR_CPU_MASK;
	if (cpu >= 4)
		return ARM_PSCI_RET_INVAL;

	/* Warm-boot hand-off for the resume trampoline (EL3, caches off). */
	tegra_cpu_entry[cpu] = entry_point;
	tegra_cpu_context[cpu] = context_id;
	asm volatile("mrs %0, cntfrq_el0" : "=r"(tegra_boot_cntfrq));
	tegra_setup_cpu_reset_vector();
	asm volatile("dsb sy; isb" ::: "memory");

	/*
	 * If the BPMP will not take the firmware there is nothing to finish the
	 * sequence, and powering the rail off here would hang the board with no
	 * way back. Report failure instead and let the kernel unwind.
	 */
	if (!tegra_bpmp_sc7_start()) {
		tegra_cpu_entry[cpu] = 0;
		asm volatile("dsb sy" ::: "memory");
		return ARM_PSCI_RET_INTERNAL_FAILURE;
	}

	/*
	 * Tell the warm-boot trampoline that the SoC, and not just this core,
	 * is about to lose power, so that it re-does the SoC-level setup on the
	 * way out (tegra_sc7_resume_finish()), and take the copy of the GIC
	 * state that it will put back.
	 */
	tegra_gic_save();
	tegra_sc7_resume_pending = 1;

	tegra_fc_soc_powerdn(cpu);

	/* Flush L1, leave coherency, WFI: the FC gates us and the rail drops. */
	tegra_cpu_powerdn_wfi();

	/* Only reached if the power-down did not happen. */
	tegra_sc7_resume_pending = 0;
	tegra_fc_cpu_disarm(cpu);
	tegra_cpu_entry[cpu] = 0;
	asm volatile("dsb sy" ::: "memory");

	return ARM_PSCI_RET_SUCCESS;
}

s32 __secure psci_system_suspend(u32 function_id, u32 entry_point,
				 u32 context_id)
{
	return psci_system_suspend_64(function_id, entry_point, context_id);
}

s32 __secure psci_affinity_info_64(u32 __always_unused function_id,
				   u64 target_affinity, u32 lowest_affinity_level)
{
	u32 cpu = target_affinity & MPIDR_CPU_MASK;

	if (lowest_affinity_level > 0)
		return ARM_PSCI_RET_INVAL;
	if (target_affinity & ~((u64)MPIDR_CPU_MASK))
		return ARM_PSCI_RET_INVAL;
	if (cpu >= 4)
		return ARM_PSCI_RET_INVAL;

	return psci_cpu_state[cpu];
}

s32 __secure psci_affinity_info(u32 function_id, u32 target_affinity,
				u32 lowest_affinity_level)
{
	return psci_affinity_info_64(function_id, target_affinity,
				     lowest_affinity_level);
}

u32 __secure psci_migrate_info_type(void)
{
	/* No trusted OS to migrate. */
	return 2;
}

s32 __secure psci_features(u32 __always_unused function_id, u32 psci_fid)
{
	switch (psci_fid) {
	case ARM_PSCI_0_2_FN_CPU_SUSPEND:
	case ARM_PSCI_0_2_FN64_CPU_SUSPEND:
		/* Supported, and using the extended power-state parameter format. */
		return PSCI_CPU_SUSPEND_EXT_POWER_STATE;
	case ARM_PSCI_0_2_FN_PSCI_VERSION:
	case ARM_PSCI_0_2_FN_CPU_ON:
	case ARM_PSCI_0_2_FN_AFFINITY_INFO:
	case ARM_PSCI_0_2_FN_MIGRATE_INFO_TYPE:
	case ARM_PSCI_0_2_FN_SYSTEM_RESET:
	case ARM_PSCI_0_2_FN64_CPU_ON:
	case ARM_PSCI_0_2_FN64_AFFINITY_INFO:
	case ARM_PSCI_1_0_FN_PSCI_FEATURES:
	case ARM_PSCI_1_0_FN_SYSTEM_SUSPEND:
	case ARM_PSCI_1_0_FN64_SYSTEM_SUSPEND:
		return 0x0;
	default:
		return ARM_PSCI_RET_NI;
	}
}

void __secure psci_system_reset(void)
{
	/* Trigger a full SoC reset via PMC_CNTRL.MAIN_RST. */
	u32 val = readl(NV_PA_PMC_BASE + PMC_CNTRL);

	writel(val | PMC_CNTRL_MAIN_RST, NV_PA_PMC_BASE + PMC_CNTRL);

	while (1)
		tegra_wfi();
}
