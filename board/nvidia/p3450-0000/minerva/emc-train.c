// SPDX-License-Identifier: GPL-2.0
/*
 * Glue between U-Boot's EMC-table injection and the Minerva Training Cell.
 *
 * The mainline tegra210-emc driver refuses to switch to any >204 MHz timing
 * whose `trained` flag is 0 and never trains itself (NVIDIA delegates one-time
 * LPDDR4 DFS training to cboot). Under the RAM-boot flow cboot is bypassed, so
 * the timing table we synthesise from the bl-dtb is untrained and EMC is stuck
 * at 204 MHz. Minerva (CTCaer) is an open reimplementation of exactly that T210
 * training; here we run it in U-Boot to train the high entries in place, so the
 * table we hand the kernel has trained=1 + the full trained calibration
 * (trained_dram_clktree[] plus the DQ/DQS/QUSE/CA-vref trim results).
 *
 * This unit is compiled against Minerva's own headers/types only (no U-Boot
 * headers) to avoid u32/bool typedef clashes; it exposes a single plain-C entry
 * to board/nvidia/p3450-0000/emc-table.c.  emc_table_t is byte-identical to the
 * mainline struct tegra210_emc_timing that emc-table.c builds.
 */

#include "mtc.h"

_Static_assert(sizeof(emc_table_t) == EMC_TABLE_ENTRY_SIZE_R7,
	       "emc_table_t must match the 4928-byte kernel timing struct");

/*
 * Scratch used to train a derated entry: Minerva looks entries up by rate, and
 * both the nominal and derated tables contain a 1600 MHz entry, so a derated
 * entry can't be trained in an array that also holds the nominal one. We build
 * a private [boot-rate src, derated dst] pair here instead.
 */
static emc_table_t train_scratch[2];

static emc_table_t *find_rate(emc_table_t *t, int n, unsigned int khz)
{
	int i;

	for (i = 0; i < n; i++)
		if ((unsigned int)t[i].rate_khz == khz)
			return &t[i];
	return 0;
}

/*
 * Train every >boot-rate entry in the nominal and derated tables, in a single
 * Minerva session so its FSP / PLL-select / RAM-pattern state stays coherent.
 * Derated entries are trained (not copied) so their DQ/DQS/QUSE/CA-vref trim
 * results are the calibration measured for the derated timing, not a partial
 * copy. Every OP_TRAIN trains in place and returns to the boot rate.
 *
 * @nom/@n_nom, @der/@n_der: the nominal and derated tables (built in place)
 * @cur_khz: the rate the board booted at (must match a nominal entry)
 * Returns the number of entries trained, or -1 if the boot entry wasn't found
 * (caller then injects the untrained table -> 204 MHz only).
 */
int tegra210_emc_train_all(void *nom_v, int n_nom, void *der_v, int n_der,
			   unsigned int cur_khz)
{
	emc_table_t *nom = (emc_table_t *)nom_v;
	emc_table_t *der = (emc_table_t *)der_v;
	emc_table_t *src;
	mtc_config_t cfg;
	int i, trained = 0;

	src = find_rate(nom, n_nom, cur_khz);
	if (!src)
		return -1;

	/* Minerva driver state, initialised as Hekate does for first training. */
	cfg.sdram_id = 0;			/* unused by the training path */
	cfg.prev_temp = 0;
	cfg.emc_2X_clk_src_is_pllmb = false;
	cfg.fsp_for_src_freq = false;
	cfg.train_ram_patterns = true;
	cfg.current_emc_table = src;

	/* 1) nominal entries: train in place (src and dst share the array) */
	cfg.mtc_table = nom;
	cfg.table_entries = n_nom;
	for (i = 0; i < n_nom; i++) {
		if ((unsigned int)nom[i].rate_khz <= cur_khz || nom[i].trained)
			continue;
		cfg.train_mode = OP_TRAIN;
		cfg.rate_from = cur_khz;
		cfg.rate_to = nom[i].rate_khz;
		minerva_main(&cfg);
		if (nom[i].trained)
			trained++;
	}

	/* 2) derated entries: train each via [boot-rate src, derated dst] */
	train_scratch[0] = *src;		/* boot-rate config == live HW */
	cfg.mtc_table = train_scratch;
	cfg.table_entries = 2;
	cfg.current_emc_table = &train_scratch[0];
	for (i = 0; i < n_der; i++) {
		if ((unsigned int)der[i].rate_khz <= cur_khz || der[i].trained)
			continue;
		train_scratch[1] = der[i];
		cfg.train_mode = OP_TRAIN;
		cfg.rate_from = cur_khz;
		cfg.rate_to = der[i].rate_khz;
		minerva_main(&cfg);
		der[i] = train_scratch[1];	/* copy trained result back */
		if (der[i].trained)
			trained++;
	}

	return trained;
}
