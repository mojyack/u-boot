// SPDX-License-Identifier: GPL-2.0+
/*
 * Jetson Nano EMC frequency-table injection for the RAM-boot flow.
 *
 * On a normal boot, cboot builds the mainline tegra210-emc driver's binary
 * timing table (an array of `struct tegra210_emc_timing`) into a reserved-memory
 * carveout and adds memory-region{,-names} to the kernel's EMC node.  The
 * RAM-boot recipe (tegra-rcm) bypasses cboot, so the kernel probes with no table
 * and fails: "tegra210-emc ...: failed to get nominal EMC table: -19", leaving
 * EMC stuck at its boot rate (no DFS/reclocking).
 *
 * This reproduces cboot's behaviour inside U-Boot's ft_board_setup():
 *   1. read the emc-table DT subnodes carried in U-Boot's own control DTB
 *      (tegra210-p3450-emc-tables.dtsi, copied verbatim from the stock bl-dtb),
 *   2. select the entry set matching the board's strapped ram-code,
 *   3. serialise each into the mainline `struct tegra210_emc_timing` binary
 *      (4928 bytes) at a fixed carveout in RAM, and
 *   4. add the reserved-memory node(s) + memory-region{,-names} to the kernel FDT.
 *
 * The property->field mapping is a faithful port of the downstream kernel's
 * drivers/memory/tegra/tegra210-dt-parse.c; the target layout is mainline
 * `struct tegra210_emc_timing` (byte-identical to downstream `struct emc_table`).
 */

#include <stdio.h>
#include <cpu_func.h>
#include <fdt_support.h>
#include <fdtdec.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <asm/io.h>
#include <asm/arch/tegra.h>
#include <asm/global_data.h>

DECLARE_GLOBAL_DATA_PTR;

/* sizeof(struct tegra210_emc_timing) from linux drivers/memory/tegra/tegra210-emc.h */
#define EMC_TIMING_SIZE		4928
#define TEGRA_EMC_MAX_FREQS	16

/*
 * Carveout addresses cboot conventionally uses on T210: the free gap between
 * fdt_addr_r (0x83000000, <=1 MiB) and ramdisk_addr_r (0x83420000).  Two 64 KiB
 * regions, nominal then derated, ending exactly at ramdisk_addr_r.
 */
#define EMC_NOMINAL_ADDR	0x83400000
#define EMC_DERATED_ADDR	0x83410000
#define EMC_REGION_SIZE		0x10000

/* PMC APBDEV_PMC_STRAPPING_OPT_A_0 = 0x7000e464, ram-code in bits [7:4]. */
#define PMC_STRAPPING_OPT_A	0x64

/* The rate nvtboot brings DRAM up at on this board (kernel probes at 204 MHz). */
#define EMC_BOOT_RATE_KHZ	204000

/* Minerva trainer (board/nvidia/p3450-0000/minerva/emc-train.c). */
int tegra210_emc_train_all(void *nom, int n_nom, void *der, int n_der,
			   unsigned int cur_khz);

static u32 emc_read_ram_code(void)
{
	return (readl(NV_PA_PMC_BASE + PMC_STRAPPING_OPT_A) >> 4) & 0xf;
}

/* --- little sequential serialiser into a per-entry buffer --- */
struct emc_cursor {
	u8 *buf;
	unsigned int pos;
};

static u32 prop_u32(const void *fdt, int node, const char *name)
{
	const fdt32_t *p;
	int len;

	p = fdt_getprop(fdt, node, name, &len);
	if (!p || len < (int)sizeof(*p))
		return 0;
	return fdt32_to_cpu(*p);
}

static void put_u32(struct emc_cursor *c, u32 v)
{
	/* store native (little-endian) u32, matching the kernel's struct read */
	u8 *d = c->buf + c->pos;

	d[0] = v; d[1] = v >> 8; d[2] = v >> 16; d[3] = v >> 24;
	c->pos += sizeof(u32);
}

static void put_zero(struct emc_cursor *c, unsigned int n)
{
	memset(c->buf + c->pos, 0, n);
	c->pos += n;
}

static void put_str(struct emc_cursor *c, const void *fdt, int node,
		    const char *name, unsigned int field_sz)
{
	const char *s;
	int len;

	memset(c->buf + c->pos, 0, field_sz);
	s = fdt_getprop(fdt, node, name, &len);
	if (s && len > 0) {
		unsigned int l = strnlen(s, field_sz - 1);

		memcpy(c->buf + c->pos, s, l);
	}
	c->pos += field_sz;
}

/*
 * Emit a fixed-length u32 array field.  The NVIDIA parser reads exactly `count`
 * elements via of_property_read_u32_array(), which fails (leaving zeros) if the
 * property is shorter than `count`; replicate that, then zero-fill to `slots`.
 */
static void put_arr(struct emc_cursor *c, const void *fdt, int node,
		    const char *name, unsigned int slots, unsigned int count)
{
	const fdt32_t *p;
	int len;
	unsigned int avail, i;

	p = fdt_getprop(fdt, node, name, &len);
	avail = (p && len > 0) ? (unsigned int)len / sizeof(*p) : 0;
	if (count > avail)	/* -EOVERFLOW in the kernel: nothing written */
		count = 0;
	if (count > slots)
		count = slots;
	for (i = 0; i < slots; i++)
		put_u32(c, i < count ? fdt32_to_cpu(p[i]) : 0);
}

static const char * const clktree[8] = {
	"c0d0u0", "c0d0u1", "c0d1u0", "c0d1u1",
	"c1d0u0", "c1d0u1", "c1d1u0", "c1d1u1",
};

static void put_clktree(struct emc_cursor *c, const void *fdt, int node,
			const char *which)
{
	char name[64];
	unsigned int i;

	for (i = 0; i < 8; i++) {
		snprintf(name, sizeof(name),
			 "nvidia,%s_dram_clktree_%s", which, clktree[i]);
		put_u32(c, prop_u32(fdt, node, name));
	}
}

/* Serialise one emc-table subnode into a 4928-byte struct tegra210_emc_timing. */
static void emc_build_entry(const void *fdt, int node, u8 *out)
{
	struct emc_cursor c = { .buf = out, .pos = 0 };
	u32 rev = prop_u32(fdt, node, "nvidia,revision");
	u32 nb  = prop_u32(fdt, node, "nvidia,burst-regs-num");
	u32 nbpc = prop_u32(fdt, node, "nvidia,burst-regs-per-ch-num");
	u32 nt  = prop_u32(fdt, node, "nvidia,trim-regs-num");
	u32 ntpc = prop_u32(fdt, node, "nvidia,trim-regs-per-ch-num");
	u32 nmc = prop_u32(fdt, node, "nvidia,burst-mc-regs-num");
	u32 nud = prop_u32(fdt, node, "nvidia,la-scale-regs-num");
	u32 nvref = prop_u32(fdt, node, "nvidia,vref-regs-num");
	u32 ndram = prop_u32(fdt, node, "nvidia,dram-timing-regs-num");

	put_u32(&c, rev);					/* revision */
	put_str(&c, fdt, node, "nvidia,dvfs-version", 60);	/* dvfs_ver[60] */
	put_u32(&c, prop_u32(fdt, node, "clock-frequency"));	/* rate */
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-min-mv"));	/* min_volt */
	put_u32(&c, prop_u32(fdt, node, "nvidia,gk20a-min-mv"));	/* gpu_min_volt */
	put_str(&c, fdt, node, "nvidia,source", 32);		/* clock_src[32] */
	put_u32(&c, prop_u32(fdt, node, "nvidia,src-sel-reg"));	/* clk_src_emc */
	put_u32(&c, prop_u32(fdt, node, "nvidia,needs-training")); /* needs_training */
	put_u32(&c, 0);						/* training_pattern (unparsed) */
	put_u32(&c, prop_u32(fdt, node, "nvidia,trained"));	/* trained */

	/* periodic-training block: only present for rev >= 6 */
	if (rev >= 6) {
		put_u32(&c, prop_u32(fdt, node, "nvidia,periodic_training"));
		put_clktree(&c, fdt, node, "trained");		/* [8] */
		put_clktree(&c, fdt, node, "current");		/* [8] */
		put_u32(&c, prop_u32(fdt, node, "nvidia,run_clocks"));
		put_u32(&c, prop_u32(fdt, node, "nvidia,tree_margin"));
	} else {
		put_zero(&c, (1 + 8 + 8 + 1 + 1) * sizeof(u32));
	}

	put_u32(&c, nb);					/* num_burst */
	put_u32(&c, nbpc);					/* num_burst_per_ch */
	put_u32(&c, nt);					/* num_trim */
	put_u32(&c, ntpc);					/* num_trim_per_ch */
	put_u32(&c, nmc);					/* num_mc_regs */
	put_u32(&c, nud);					/* num_up_down */
	put_u32(&c, nvref);					/* vref_num */
	put_u32(&c, 0);						/* training_mod_num (unparsed) */
	put_u32(&c, ndram);					/* dram_timing_num */

	/* ptfv_list[12]: only for rev >= 7 */
	if (rev >= 7)
		put_arr(&c, fdt, node, "nvidia,ptfv", 12, 12);
	else
		put_zero(&c, 12 * sizeof(u32));

	put_arr(&c, fdt, node, "nvidia,emc-registers", 221, nb);
	put_arr(&c, fdt, node, "nvidia,emc-burst-regs-per-ch", 8, nbpc);
	put_arr(&c, fdt, node, "nvidia,emc-shadow-regs-ca-train", 221, nb);
	put_arr(&c, fdt, node, "nvidia,emc-shadow-regs-quse-train", 221, nb);
	put_arr(&c, fdt, node, "nvidia,emc-shadow-regs-rdwr-train", 221, nb);
	put_arr(&c, fdt, node, "nvidia,emc-trim-regs", 138, nt);
	put_arr(&c, fdt, node, "nvidia,emc-trim-regs-per-ch", 10, ntpc);
	put_arr(&c, fdt, node, "nvidia,emc-vref-regs", 4, nvref);
	put_arr(&c, fdt, node, "nvidia,emc-dram-timing-regs", 5, ndram);
	put_zero(&c, 20 * sizeof(u32));	/* training_mod_regs[20]  (unparsed) */
	put_zero(&c, 12 * sizeof(u32));	/* save_restore_mod_regs[12] (unparsed) */
	put_arr(&c, fdt, node, "nvidia,emc-burst-mc-regs", 33, nmc);
	put_arr(&c, fdt, node, "nvidia,emc-la-scale-regs", 24, nud);

	put_u32(&c, prop_u32(fdt, node, "nvidia,min-mrs-wait"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-mrw"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-mrw2"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-mrw3"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-mrw4"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-mrw9"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-mrs"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-emrs"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-emrs2"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-auto-cal-config"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-auto-cal-config2"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-auto-cal-config3"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-auto-cal-config4"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-auto-cal-config5"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-auto-cal-config6"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-auto-cal-config7"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-auto-cal-config8"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-cfg-2"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-sel-dpd-ctrl"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-fdpd-ctrl-cmd-no-ramp"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,dll-clk-src"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,clk-out-enb-x-0-clk-enb-emc-dll"));
	put_u32(&c, prop_u32(fdt, node, "nvidia,emc-clock-latency-change")); /* latency */

	/* Any shortfall/overrun is a mapping bug; guard the carveout. */
	if (c.pos != EMC_TIMING_SIZE)
		printf("emc-table: BUG entry size %u != %u\n", c.pos,
		       EMC_TIMING_SIZE);
}

/*
 * Find /emc-tables in the control DTB, descend into the ram-code-matching child,
 * serialise every subnode with @compat into @dst, return the entry count.
 */
static int emc_parse_tables(const void *fdt, u32 ram_code, const char *compat,
			    u8 *dst, unsigned int max)
{
	int src, rc, sub;
	unsigned int n = 0;

	src = fdt_path_offset(fdt, "/emc-tables");
	if (src < 0)
		return 0;

	rc = -1;
	fdt_for_each_subnode(sub, fdt, src) {
		if (prop_u32(fdt, sub, "nvidia,ram-code") == ram_code) {
			rc = sub;
			break;
		}
	}
	if (rc < 0)
		return 0;

	fdt_for_each_subnode(sub, fdt, rc) {
		if (fdt_node_check_compatible(fdt, sub, compat) != 0)
			continue;
		if (n >= max)
			break;
		emc_build_entry(fdt, sub, dst + n * EMC_TIMING_SIZE);
		n++;
	}
	return n;
}

static int emc_add_carveout(void *blob, u32 addr, uint32_t *phandle)
{
	static const char *compat[] = { "nvidia,tegra210-emc-table" };
	struct fdt_memory mem = {
		.start = addr,
		.end   = addr + EMC_REGION_SIZE - 1,
	};

	return fdtdec_add_reserved_memory(blob, "emc-table", &mem, compat,
					  ARRAY_SIZE(compat), phandle, 0);
}

/*
 * Called from ft_board_setup(): synthesise the EMC tables into RAM and wire the
 * kernel FDT up to them.  Non-fatal on any error -- the kernel then simply keeps
 * its boot EMC rate (the pre-existing behaviour), so we never block the boot.
 */
void tegra210_emc_ft_setup(void *blob)
{
	const void *ctrl = gd->fdt_blob;
	u32 ram_code = emc_read_ram_code();
	uint32_t ph_nom, ph_der;
	int n_nom, n_der, emc, ret;
	fdt32_t regions[2];

	memset((void *)EMC_NOMINAL_ADDR, 0, EMC_REGION_SIZE);
	memset((void *)EMC_DERATED_ADDR, 0, EMC_REGION_SIZE);

	n_nom = emc_parse_tables(ctrl, ram_code, "nvidia,tegra21-emc-table",
				 (u8 *)EMC_NOMINAL_ADDR, TEGRA_EMC_MAX_FREQS);
	if (n_nom <= 0) {
		printf("emc-table: no nominal table for ram-code 0x%x\n",
		       ram_code);
		return;
	}
	n_der = emc_parse_tables(ctrl, ram_code,
				 "nvidia,tegra21-emc-table-derated",
				 (u8 *)EMC_DERATED_ADDR, TEGRA_EMC_MAX_FREQS);

	/*
	 * Train the high (>204 MHz) entries (nominal + derated) so the kernel
	 * will switch to them. Non-fatal: on failure the untrained table is
	 * still injected and the kernel simply keeps 204 MHz (pre-training
	 * behaviour).
	 */
	ret = tegra210_emc_train_all((void *)EMC_NOMINAL_ADDR, n_nom,
				     (void *)EMC_DERATED_ADDR, n_der,
				     EMC_BOOT_RATE_KHZ);
	printf("emc-table: trained %d entries (nominal + derated)\n", ret);
	flush_dcache_all();

	emc = fdt_path_offset(blob, "/external-memory-controller@7001b000");
	if (emc < 0)
		emc = fdt_node_offset_by_compatible(blob, -1,
						    "nvidia,tegra210-emc");
	if (emc < 0) {
		printf("emc-table: no EMC node in kernel DTB\n");
		return;
	}

	ret = emc_add_carveout(blob, EMC_NOMINAL_ADDR, &ph_nom);
	if (ret < 0) {
		printf("emc-table: add nominal carveout failed: %d\n", ret);
		return;
	}
	/* re-lookup: adding nodes can move offsets */
	emc = fdt_path_offset(blob, "/external-memory-controller@7001b000");
	if (emc < 0)
		emc = fdt_node_offset_by_compatible(blob, -1,
						    "nvidia,tegra210-emc");

	if (n_der > 0) {
		ret = emc_add_carveout(blob, EMC_DERATED_ADDR, &ph_der);
		if (ret < 0) {
			printf("emc-table: add derated carveout failed: %d\n",
			       ret);
			n_der = 0;
		}
		emc = fdt_path_offset(blob,
				      "/external-memory-controller@7001b000");
		if (emc < 0)
			emc = fdt_node_offset_by_compatible(blob, -1,
						    "nvidia,tegra210-emc");
	}

	regions[0] = cpu_to_fdt32(ph_nom);
	if (n_der > 0) {
		regions[1] = cpu_to_fdt32(ph_der);
		fdt_setprop(blob, emc, "memory-region", regions,
			    sizeof(regions));
		fdt_setprop(blob, emc, "memory-region-names",
			    "nominal\0derated", sizeof("nominal\0derated"));
	} else {
		fdt_setprop(blob, emc, "memory-region", regions,
			    sizeof(regions[0]));
		fdt_setprop(blob, emc, "memory-region-names", "nominal",
			    sizeof("nominal"));
	}

	printf("emc-table: injected %d nominal + %d derated entries (ram-code 0x%x)\n",
	       n_nom, n_der, ram_code);
}
