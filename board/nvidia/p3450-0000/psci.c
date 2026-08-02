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
 */

#include <asm/arch-tegra/tegra.h>
#include <asm/arch-tegra/pmc.h>
#include <asm/io.h>
#include <asm/psci.h>
#include <asm/secure.h>
#include <linux/types.h>

/*
 * Deliberately not including <asm/system.h>: it declares a single-argument
 * psci_features() prototype for the PSCI *client*, which conflicts with the
 * two-argument secure-monitor entry point below (same pattern as the imx8m
 * and stm32mp1 PSCI backends). wfi is issued inline instead.
 */
#define tegra_wfi()	asm volatile("wfi" ::: "memory")

#define MPIDR_CPU_MASK			0xff

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
#define FLOWCTRL_HALT_LIC_IRQ		(1 << 11)	/* HALT: wake on LIC IRQ */
#define FLOWCTRL_HALT_LIC_FIQ		(1 << 10)	/* HALT: wake on LIC FIQ */
#define FLOWCTRL_HALT_GIC_IRQ		(1 << 9)	/* HALT: wake on GIC IRQ */
#define FLOWCTRL_HALT_GIC_FIQ		(1 << 8)	/* HALT: wake on GIC FIQ */
#define FLOWCTRL_CSR_INTR_FLAG		(1 << 15)	/* CSR: W1C interrupt status */
#define FLOWCTRL_CSR_EVENT_FLAG		(1 << 14)	/* CSR: W1C event status */
#define FLOWCTRL_WAIT_WFI_BITMAP	(1 << 8)	/* CSR: gate when CPU<<n in WFI */
#define FLOWCTRL_CSR_ENABLE		(1 << 0)	/* CSR: power-gate enable (HW-cleared) */

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
#define GICD_IGROUPR0			0x080
#define GICD_ISENABLER0			0x100
#define GICC_CTLR			0x000
#define GICC_PMR			0x004

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
 * WFI, waking on any GIC/LIC interrupt. Mirrors ARM-TF t210's
 * tegra_fc_prepare_suspend(cpu, 0) for the core-power-down case. The CSR ENABLE
 * bit is cleared by hardware once the power-gate sequence completes, so no
 * disarm is needed on resume - a subsequent plain kernel WFI will not re-gate.
 */
static void __secure tegra_fc_cpu_powerdn(u32 cpu)
{
	u32 halt = FLOWCTRL_HALT_GIC_IRQ | FLOWCTRL_HALT_GIC_FIQ |
		   FLOWCTRL_HALT_LIC_IRQ | FLOWCTRL_HALT_LIC_FIQ |
		   FLOWCTRL_WAITEVENT;
	u32 csr = FLOWCTRL_CSR_INTR_FLAG | FLOWCTRL_CSR_EVENT_FLAG |
		  FLOWCTRL_CSR_ENABLE | (FLOWCTRL_WAIT_WFI_BITMAP << cpu);

	writel(halt, NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_halt_off[cpu]);
	writel(csr, NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
	readl(NV_PA_FLOW_BASE + flowctrl_csr_off[cpu]);
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

	/* Reached only if the core did not power down; coherency is restored. */
	return ARM_PSCI_RET_SUCCESS;
}

s32 __secure psci_cpu_suspend(u32 function_id, u32 power_state,
			      u32 entry_point, u32 context_id)
{
	return psci_cpu_suspend_64(function_id, power_state, entry_point,
				   context_id);
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
