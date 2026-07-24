# SPDX-License-Identifier: GPL-2.0
#
# Minerva Training Cell (CTCaer) - imported T210 LPDDR4 DFS training, plus the
# U-Boot glue. Built as an isolated unit against Minerva's own headers/types.

obj-y += mtc.o
obj-y += emc-train.o

# Minerva is third-party and intentionally casts fixed MMIO addresses through
# u32; silence the expected warnings rather than patch upstream code.
ccflags-y += -Wno-int-to-pointer-cast -Wno-unused -Wno-sign-compare
ccflags-y += -Wno-strict-prototypes
