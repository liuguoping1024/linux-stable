// SPDX-License-Identifier: GPL-2.0-only
/*
 * xsave/xrstor support.
 *
 * Author: Suresh Siddha <suresh.b.siddha@intel.com>
 */
#include <linux/bitops.h>
#include <linux/compat.h>
#include <linux/cpu.h>
#include <linux/mman.h>
#include <linux/nospec.h>
#include <linux/pkeys.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/coredump.h>
#include <linux/sort.h>

#include <asm/fpu/api.h>
#include <asm/fpu/regset.h>
#include <asm/fpu/signal.h>
#include <asm/fpu/xcr.h>

#include <asm/cpuid/api.h>
#include <asm/msr.h>
#include <asm/tlbflush.h>
#include <asm/prctl.h>
#include <asm/elf.h>

#include <uapi/asm/elf.h>

#include "context.h"
#include "internal.h"
#include "legacy.h"
#include "xstate.h"

#define for_each_extended_xfeature(bit, mask)				\
	(bit) = FIRST_EXTENDED_XFEATURE;				\
	for_each_set_bit_from(bit, (unsigned long *)&(mask), 8 * sizeof(mask))

/*
 * Although we spell it out in here, the Processor Trace
 * xfeature is completely unused.  We use other mechanisms
 * to save/restore PT state in Linux.
 */
static const char *xfeature_names[] =
{
	"x87 floating point registers",
	"SSE registers",
	"AVX registers",
	"MPX bounds registers",
	"MPX CSR",
	"AVX-512 opmask",
	"AVX-512 Hi256",
	"AVX-512 ZMM_Hi256",
	"Processor Trace (unused)",
	"Protection Keys User registers",
	"PASID state",
	"Control-flow User registers",
	"Control-flow Kernel registers (KVM only)",
	"unknown xstate feature",
	"unknown xstate feature",
	"unknown xstate feature",
	"unknown xstate feature",
	"AMX Tile config",
	"AMX Tile data",
	"APX registers",
	"unknown xstate feature",
};

static unsigned short xsave_cpuid_features[] __initdata = {
	[XFEATURE_FP]				= X86_FEATURE_FPU,
	[XFEATURE_SSE]				= X86_FEATURE_XMM,
	[XFEATURE_YMM]				= X86_FEATURE_AVX,
	[XFEATURE_BNDREGS]			= X86_FEATURE_MPX,
	[XFEATURE_BNDCSR]			= X86_FEATURE_MPX,
	[XFEATURE_OPMASK]			= X86_FEATURE_AVX512F,
	[XFEATURE_ZMM_Hi256]			= X86_FEATURE_AVX512F,
	[XFEATURE_Hi16_ZMM]			= X86_FEATURE_AVX512F,
	[XFEATURE_PT_UNIMPLEMENTED_SO_FAR]	= X86_FEATURE_INTEL_PT,
	[XFEATURE_PKRU]				= X86_FEATURE_OSPKE,
	[XFEATURE_PASID]			= X86_FEATURE_ENQCMD,
	[XFEATURE_CET_USER]			= X86_FEATURE_SHSTK,
	[XFEATURE_CET_KERNEL]			= X86_FEATURE_SHSTK,
	[XFEATURE_XTILE_CFG]			= X86_FEATURE_AMX_TILE,
	[XFEATURE_XTILE_DATA]			= X86_FEATURE_AMX_TILE,
	[XFEATURE_APX]				= X86_FEATURE_APX,
};

static unsigned int xstate_offsets[XFEATURE_MAX] __ro_after_init =
	{ [ 0 ... XFEATURE_MAX - 1] = -1};
static unsigned int xstate_sizes[XFEATURE_MAX] __ro_after_init =
	{ [ 0 ... XFEATURE_MAX - 1] = -1};
static unsigned int xstate_flags[XFEATURE_MAX] __ro_after_init;

/*
 * Ordering of xstate components in uncompacted format:  The xfeature
 * number does not necessarily indicate its position in the XSAVE buffer.
 * This array defines the traversal order of xstate features.
 */
static unsigned int xfeature_uncompact_order[XFEATURE_MAX] __ro_after_init =
	{ [ 0 ... XFEATURE_MAX - 1] = -1};

static inline unsigned int next_xfeature_order(unsigned int i, u64 mask)
{
	for (; xfeature_uncompact_order[i] != -1; i++) {
		if (mask & BIT_ULL(xfeature_uncompact_order[i]))
			break;
	}

	return i;
}

/* Iterate xstate features in uncompacted order: */
#define for_each_extended_xfeature_in_order(i, mask)	\
	for (i = 0;					\
	     i = next_xfeature_order(i, mask),		\
	     xfeature_uncompact_order[i] != -1;		\
	     i++)

#define XSTATE_FLAG_SUPERVISOR	BIT(0)
#define XSTATE_FLAG_ALIGNED64	BIT(1)

/*
 * Return whether the system supports a given xfeature.
 *
 * Also return the name of the (most advanced) feature that the caller requested:
 */
int cpu_has_xfeatures(u64 xfeatures_needed, const char **feature_name)
{
	u64 xfeatures_missing = xfeatures_needed & ~fpu_kernel_cfg.max_features;

	if (unlikely(feature_name)) {
		long xfeature_idx, max_idx;
		u64 xfeatures_print;
		/*
		 * So we use FLS here to be able to print the most advanced
		 * feature that was requested but is missing. So if a driver
		 * asks about "XFEATURE_MASK_SSE | XFEATURE_MASK_YMM" we'll print the
		 * missing AVX feature - this is the most informative message
		 * to users:
		 */
		if (xfeatures_missing)
			xfeatures_print = xfeatures_missing;
		else
			xfeatures_print = xfeatures_needed;

		xfeature_idx = fls64(xfeatures_print)-1;
		max_idx = ARRAY_SIZE(xfeature_names)-1;
		xfeature_idx = min(xfeature_idx, max_idx);

		*feature_name = xfeature_names[xfeature_idx];
	}

	if (xfeatures_missing)
		return 0;

	return 1;
}
EXPORT_SYMBOL_GPL(cpu_has_xfeatures);

static bool xfeature_is_aligned64(int xfeature_nr)
{
	return xstate_flags[xfeature_nr] & XSTATE_FLAG_ALIGNED64;
}

static bool xfeature_is_supervisor(int xfeature_nr)
{
	return xstate_flags[xfeature_nr] & XSTATE_FLAG_SUPERVISOR;
}

static unsigned int xfeature_get_offset(u64 xcomp_bv, int xfeature)
{
	unsigned int offs, i;

	/*
	 * Non-compacted format and legacy features use the cached fixed
	 * offsets.
	 */
	if (!cpu_feature_enabled(X86_FEATURE_XCOMPACTED) ||
	    xfeature <= XFEATURE_SSE)
		return xstate_offsets[xfeature];

	/*
	 * Compacted format offsets depend on the actual content of the
	 * compacted xsave area which is determined by the xcomp_bv header
	 * field.
	 */
	offs = FXSAVE_SIZE + XSAVE_HDR_SIZE;
	for_each_extended_xfeature(i, xcomp_bv) {
		if (xfeature_is_aligned64(i))
			offs = ALIGN(offs, 64);
		if (i == xfeature)
			break;
		offs += xstate_sizes[i];
	}
	return offs;
}

/*
 * Enable the extended processor state save/restore feature.
 * Called once per CPU onlining.
 */
void fpu__init_cpu_xstate(void)
{
	if (!boot_cpu_has(X86_FEATURE_XSAVE) || !fpu_kernel_cfg.max_features)
		return;

	cr4_set_bits(X86_CR4_OSXSAVE);

	/*
	 * Must happen after CR4 setup and before xsetbv() to allow KVM
	 * lazy passthrough.  Write independent of the dynamic state static
	 * key as that does not work on the boot CPU. This also ensures
	 * that any stale state is wiped out from XFD. Reset the per CPU
	 * xfd cache too.
	 */
	if (cpu_feature_enabled(X86_FEATURE_XFD))
		xfd_set_state(init_fpstate.xfd);

	/*
	 * XCR_XFEATURE_ENABLED_MASK (aka. XCR0) sets user features
	 * managed by XSAVE{C, OPT, S} and XRSTOR{S}.  Only XSAVE user
	 * states can be set here.
	 */
	xsetbv(XCR_XFEATURE_ENABLED_MASK, fpu_user_cfg.max_features);

	/*
	 * MSR_IA32_XSS sets supervisor states managed by XSAVES.
	 */
	if (boot_cpu_has(X86_FEATURE_XSAVES)) {
		wrmsrq(MSR_IA32_XSS, xfeatures_mask_supervisor() |
				     xfeatures_mask_independent());
	}
}

static bool xfeature_enabled(enum xfeature xfeature)
{
	return fpu_kernel_cfg.max_features & BIT_ULL(xfeature);
}

static int compare_xstate_offsets(const void *xfeature1, const void *xfeature2)
{
	return  xstate_offsets[*(unsigned int *)xfeature1] -
		xstate_offsets[*(unsigned int *)xfeature2];
}

/*
 * Record the offsets and sizes of various xstates contained
 * in the XSAVE state memory layout. Also, create an ordered
 * list of xfeatures for handling out-of-order offsets.
 */
static void __init setup_xstate_cache(void)
{
	u32 eax, ebx, ecx, edx, xfeature, i = 0;
	/*
	 * The FP xstates and SSE xstates are legacy states. They are always
	 * in the fixed offsets in the xsave area in either compacted form
	 * or standard form.
	 */
	xstate_offsets[XFEATURE_FP]	= 0;
	xstate_sizes[XFEATURE_FP]	= offsetof(struct fxregs_state,
						   xmm_space);

	xstate_offsets[XFEATURE_SSE]	= xstate_sizes[XFEATURE_FP];
	xstate_sizes[XFEATURE_SSE]	= sizeof_field(struct fxregs_state,
						       xmm_space);

	for_each_extended_xfeature(xfeature, fpu_kernel_cfg.max_features) {
		cpuid_count(CPUID_LEAF_XSTATE, xfeature, &eax, &ebx, &ecx, &edx);

		xstate_sizes[xfeature] = eax;
		xstate_flags[xfeature] = ecx;

		/*
		 * If an xfeature is supervisor state, the offset in EBX is
		 * invalid, leave it to -1.
		 */
		if (xfeature_is_supervisor(xfeature))
			continue;

		xstate_offsets[xfeature] = ebx;

		/* Populate the list of xfeatures before sorting */
		xfeature_uncompact_order[i++] = xfeature;
	}

	/*
	 * Sort xfeatures by their offsets to support out-of-order
	 * offsets in the uncompacted format.
	 */
	sort(xfeature_uncompact_order, i, sizeof(unsigned int), compare_xstate_offsets, NULL);
}

/*
 * Print out all the supported xstate features:
 */
static void __init print_xstate_features(void)
{
	int i;

	for (i = 0; i < XFEATURE_MAX; i++) {
		u64 mask = BIT_ULL(i);
		const char *name;

		if (cpu_has_xfeatures(mask, &name))
			pr_info("x86/fpu: Supporting XSAVE feature 0x%03Lx: '%s'\n", mask, name);
	}
}

/*
 * This check is important because it is easy to get XSTATE_*
 * confused with XSTATE_BIT_*.
 */
#define CHECK_XFEATURE(nr) do {		\
	WARN_ON(nr < FIRST_EXTENDED_XFEATURE);	\
	WARN_ON(nr >= XFEATURE_MAX);	\
} while (0)

/*
 * Print out xstate component offsets and sizes
 */
static void __init print_xstate_offset_size(void)
{
	int i;

	for_each_extended_xfeature(i, fpu_kernel_cfg.max_features) {
		pr_info("x86/fpu: xstate_offset[%d]: %4d, xstate_sizes[%d]: %4d\n",
			i, xfeature_get_offset(fpu_kernel_cfg.max_features, i),
			i, xstate_sizes[i]);
	}
}

/*
 * This function is called only during boot time when x86 caps are not set
 * up and alternative can not be used yet.
 */
static __init void os_xrstor_booting(struct xregs_state *xstate)
{
	u64 mask = fpu_kernel_cfg.max_features & XFEATURE_MASK_FPSTATE;
	u32 lmask = mask;
	u32 hmask = mask >> 32;
	int err;

	if (cpu_feature_enabled(X86_FEATURE_XSAVES))
		XSTATE_OP(XRSTORS, xstate, lmask, hmask, err);
	else
		XSTATE_OP(XRSTOR, xstate, lmask, hmask, err);

	/*
	 * We should never fault when copying from a kernel buffer, and the FPU
	 * state we set at boot time should be valid.
	 */
	WARN_ON_FPU(err);
}

/*
 * All supported features have either init state all zeros or are
 * handled in setup_init_fpu() individually. This is an explicit
 * feature list and does not use XFEATURE_MASK*SUPPORTED to catch
 * newly added supported features at build time and make people
 * actually look at the init state for the new feature.
 */
#define XFEATURES_INIT_FPSTATE_HANDLED		\
	(XFEATURE_MASK_FP |			\
	 XFEATURE_MASK_SSE |			\
	 XFEATURE_MASK_YMM |			\
	 XFEATURE_MASK_OPMASK |			\
	 XFEATURE_MASK_ZMM_Hi256 |		\
	 XFEATURE_MASK_Hi16_ZMM	 |		\
	 XFEATURE_MASK_PKRU |			\
	 XFEATURE_MASK_BNDREGS |		\
	 XFEATURE_MASK_BNDCSR |			\
	 XFEATURE_MASK_PASID |			\
	 XFEATURE_MASK_CET_USER |		\
	 XFEATURE_MASK_CET_KERNEL |		\
	 XFEATURE_MASK_XTILE |			\
	 XFEATURE_MASK_APX)

/*
 * setup the xstate image representing the init state
 */
static void __init setup_init_fpu_buf(void)
{
	BUILD_BUG_ON((XFEATURE_MASK_USER_SUPPORTED |
		      XFEATURE_MASK_SUPERVISOR_SUPPORTED) !=
		     XFEATURES_INIT_FPSTATE_HANDLED);

	if (!boot_cpu_has(X86_FEATURE_XSAVE))
		return;

	print_xstate_features();

	xstate_init_xcomp_bv(&init_fpstate.regs.xsave, init_fpstate.xfeatures);

	/*
	 * Init all the features state with header.xfeatures being 0x0
	 */
	os_xrstor_booting(&init_fpstate.regs.xsave);

	/*
	 * All components are now in init state. Read the state back so
	 * that init_fpstate contains all non-zero init state. This only
	 * works with XSAVE, but not with XSAVEOPT and XSAVEC/S because
	 * those use the init optimization which skips writing data for
	 * components in init state.
	 *
	 * XSAVE could be used, but that would require to reshuffle the
	 * data when XSAVEC/S is available because XSAVEC/S uses xstate
	 * compaction. But doing so is a pointless exercise because most
	 * components have an all zeros init state except for the legacy
	 * ones (FP and SSE). Those can be saved with FXSAVE into the
	 * legacy area. Adding new features requires to ensure that init
	 * state is all zeroes or if not to add the necessary handling
	 * here.
	 */
	fxsave(&init_fpstate.regs.fxsave);
}

int xfeature_size(int xfeature_nr)
{
	u32 eax, ebx, ecx, edx;

	CHECK_XFEATURE(xfeature_nr);
	cpuid_count(CPUID_LEAF_XSTATE, xfeature_nr, &eax, &ebx, &ecx, &edx);
	return eax;
}

/* Validate an xstate header supplied by userspace (ptrace or sigreturn) */
static int validate_user_xstate_header(const struct xstate_header *hdr,
				       struct fpstate *fpstate)
{
	/* No unknown or supervisor features may be set */
	if (hdr->xfeatures & ~fpstate->user_xfeatures)
		return -EINVAL;

	/* Userspace must use the uncompacted format */
	if (hdr->xcomp_bv)
		return -EINVAL;

	/*
	 * If 'reserved' is shrunken to add a new field, make sure to validate
	 * that new field here!
	 */
	BUILD_BUG_ON(sizeof(hdr->reserved) != 48);

	/* No reserved bits may be set */
	if (memchr_inv(hdr->reserved, 0, sizeof(hdr->reserved)))
		return -EINVAL;

	return 0;
}

static void __init __xstate_dump_leaves(void)
{
	int i;
	u32 eax, ebx, ecx, edx;
	static int should_dump = 1;

	if (!should_dump)
		return;
	should_dump = 0;
	/*
	 * Dump out a few leaves past the ones that we support
	 * just in case there are some goodies up there
	 */
	for (i = 0; i < XFEATURE_MAX + 10; i++) {
		cpuid_count(CPUID_LEAF_XSTATE, i, &eax, &ebx, &ecx, &edx);
		pr_warn("CPUID[%02x, %02x]: eax=%08x ebx=%08x ecx=%08x edx=%08x\n",
			CPUID_LEAF_XSTATE, i, eax, ebx, ecx, edx);
	}
}

#define XSTATE_WARN_ON(x, fmt, ...) do {					\
	if (WARN_ONCE(x, "XSAVE consistency problem: " fmt, ##__VA_ARGS__)) {	\
		__xstate_dump_leaves();						\
	}									\
} while (0)

#define XCHECK_SZ(sz, nr, __struct) ({					\
	if (WARN_ONCE(sz != sizeof(__struct),				\
	    "[%s]: struct is %zu bytes, cpu state %d bytes\n",		\
	    xfeature_names[nr], sizeof(__struct), sz)) {		\
		__xstate_dump_leaves();					\
	}								\
	true;								\
})


/**
 * check_xtile_data_against_struct - Check tile data state size.
 *
 * Calculate the state size by multiplying the single tile size which is
 * recorded in a C struct, and the number of tiles that the CPU informs.
 * Compare the provided size with the calculation.
 *
 * @size:	The tile data state size
 *
 * Returns:	0 on success, -EINVAL on mismatch.
 */
static int __init check_xtile_data_against_struct(int size)
{
	u32 max_palid, palid, state_size;
	u32 eax, ebx, ecx, edx;
	u16 max_tile;

	/*
	 * Check the maximum palette id:
	 *   eax: the highest numbered palette subleaf.
	 */
	cpuid_count(CPUID_LEAF_TILE, 0, &max_palid, &ebx, &ecx, &edx);

	/*
	 * Cross-check each tile size and find the maximum number of
	 * supported tiles.
	 */
	for (palid = 1, max_tile = 0; palid <= max_palid; palid++) {
		u16 tile_size, max;

		/*
		 * Check the tile size info:
		 *   eax[31:16]:  bytes per title
		 *   ebx[31:16]:  the max names (or max number of tiles)
		 */
		cpuid_count(CPUID_LEAF_TILE, palid, &eax, &ebx, &edx, &edx);
		tile_size = eax >> 16;
		max = ebx >> 16;

		if (tile_size != sizeof(struct xtile_data)) {
			pr_err("%s: struct is %zu bytes, cpu xtile %d bytes\n",
			       __stringify(XFEATURE_XTILE_DATA),
			       sizeof(struct xtile_data), tile_size);
			__xstate_dump_leaves();
			return -EINVAL;
		}

		if (max > max_tile)
			max_tile = max;
	}

	state_size = sizeof(struct xtile_data) * max_tile;
	if (size != state_size) {
		pr_err("%s: calculated size is %u bytes, cpu state %d bytes\n",
		       __stringify(XFEATURE_XTILE_DATA), state_size, size);
		__xstate_dump_leaves();
		return -EINVAL;
	}
	return 0;
}

/*
 * We have a C struct for each 'xstate'.  We need to ensure
 * that our software representation matches what the CPU
 * tells us about the state's size.
 */
static bool __init check_xstate_against_struct(int nr)
{
	/*
	 * Ask the CPU for the size of the state.
	 */
	int sz = xfeature_size(nr);

	/*
	 * Match each CPU state with the corresponding software
	 * structure.
	 */
	switch (nr) {
	case XFEATURE_YMM:	  return XCHECK_SZ(sz, nr, struct ymmh_struct);
	case XFEATURE_BNDREGS:	  return XCHECK_SZ(sz, nr, struct mpx_bndreg_state);
	case XFEATURE_BNDCSR:	  return XCHECK_SZ(sz, nr, struct mpx_bndcsr_state);
	case XFEATURE_OPMASK:	  return XCHECK_SZ(sz, nr, struct avx_512_opmask_state);
	case XFEATURE_ZMM_Hi256:  return XCHECK_SZ(sz, nr, struct avx_512_zmm_uppers_state);
	case XFEATURE_Hi16_ZMM:	  return XCHECK_SZ(sz, nr, struct avx_512_hi16_state);
	case XFEATURE_PKRU:	  return XCHECK_SZ(sz, nr, struct pkru_state);
	case XFEATURE_PASID:	  return XCHECK_SZ(sz, nr, struct ia32_pasid_state);
	case XFEATURE_XTILE_CFG:  return XCHECK_SZ(sz, nr, struct xtile_cfg);
	case XFEATURE_CET_USER:	  return XCHECK_SZ(sz, nr, struct cet_user_state);
	case XFEATURE_CET_KERNEL: return XCHECK_SZ(sz, nr, struct cet_supervisor_state);
	case XFEATURE_APX:        return XCHECK_SZ(sz, nr, struct apx_state);
	case XFEATURE_XTILE_DATA: check_xtile_data_against_struct(sz); return true;
	default:
		XSTATE_WARN_ON(1, "No structure for xstate: %d\n", nr);
		return false;
	}

	return true;
}

static unsigned int xstate_calculate_size(u64 xfeatures, bool compacted)
{
	unsigned int topmost = fls64(xfeatures) -  1;
	unsigned int offset, i;

	if (topmost <= XFEATURE_SSE)
		return sizeof(struct xregs_state);

	if (compacted) {
		offset = xfeature_get_offset(xfeatures, topmost);
	} else {
		/* Walk through the xfeature order to pick the last */
		for_each_extended_xfeature_in_order(i, xfeatures)
			topmost = xfeature_uncompact_order[i];
		offset = xstate_offsets[topmost];
	}

	return offset + xstate_sizes[topmost];
}

/*
 * This essentially double-checks what the cpu told us about
 * how large the XSAVE buffer needs to be.  We are recalculating
 * it to be safe.
 *
 * Independent XSAVE features allocate their own buffers and are not
 * covered by these checks. Only the size of the buffer for task->fpu
 * is checked here.
 */
static bool __init paranoid_xstate_size_valid(unsigned int kernel_size)
{
	bool compacted = cpu_feature_enabled(X86_FEATURE_XCOMPACTED);
	bool xsaves = cpu_feature_enabled(X86_FEATURE_XSAVES);
	unsigned int size = FXSAVE_SIZE + XSAVE_HDR_SIZE;
	int i;

	for_each_extended_xfeature(i, fpu_kernel_cfg.max_features) {
		if (!check_xstate_against_struct(i))
			return false;
		/*
		 * Supervisor state components can be managed only by
		 * XSAVES.
		 */
		if (!xsaves && xfeature_is_supervisor(i)) {
			XSTATE_WARN_ON(1, "Got supervisor feature %d, but XSAVES not advertised\n", i);
			return false;
		}
	}
	size = xstate_calculate_size(fpu_kernel_cfg.max_features, compacted);
	XSTATE_WARN_ON(size != kernel_size,
		       "size %u != kernel_size %u\n", size, kernel_size);
	return size == kernel_size;
}

/*
 * Get total size of enabled xstates in XCR0 | IA32_XSS.
 *
 * Note the SDM's wording here.  "sub-function 0" only enumerates
 * the size of the *user* states.  If we use it to size a buffer
 * that we use 'XSAVES' on, we could potentially overflow the
 * buffer because 'XSAVES' saves system states too.
 *
 * This also takes compaction into account. So this works for
 * XSAVEC as well.
 */
static unsigned int __init get_compacted_size(void)
{
	unsigned int eax, ebx, ecx, edx;
	/*
	 * - CPUID function 0DH, sub-function 1:
	 *    EBX enumerates the size (in bytes) required by
	 *    the XSAVES instruction for an XSAVE area
	 *    containing all the state components
	 *    corresponding to bits currently set in
	 *    XCR0 | IA32_XSS.
	 *
	 * When XSAVES is not available but XSAVEC is (virt), then there
	 * are no supervisor states, but XSAVEC still uses compacted
	 * format.
	 */
	cpuid_count(CPUID_LEAF_XSTATE, 1, &eax, &ebx, &ecx, &edx);
	return ebx;
}

/*
 * Get the total size of the enabled xstates without the independent supervisor
 * features.
 */
static unsigned int __init get_xsave_compacted_size(void)
{
	u64 mask = xfeatures_mask_independent();
	unsigned int size;

	if (!mask)
		return get_compacted_size();

	/* Disable independent features. */
	wrmsrq(MSR_IA32_XSS, xfeatures_mask_supervisor());

	/*
	 * Ask the hardware what size is required of the buffer.
	 * This is the size required for the task->fpu buffer.
	 */
	size = get_compacted_size();

	/* Re-enable independent features so XSAVES will work on them again. */
	wrmsrq(MSR_IA32_XSS, xfeatures_mask_supervisor() | mask);

	return size;
}

static unsigned int __init get_xsave_size_user(void)
{
	unsigned int eax, ebx, ecx, edx;
	/*
	 * - CPUID function 0DH, sub-function 0:
	 *    EBX enumerates the size (in bytes) required by
	 *    the XSAVE instruction for an XSAVE area
	 *    containing all the *user* state components
	 *    corresponding to bits currently set in XCR0.
	 */
	cpuid_count(CPUID_LEAF_XSTATE, 0, &eax, &ebx, &ecx, &edx);
	return ebx;
}

static int __init init_xstate_size(void)
{
	/* Recompute the context size for enabled features: */
	unsigned int user_size, kernel_size, kernel_default_size;
	bool compacted = cpu_feature_enabled(X86_FEATURE_XCOMPACTED);

	/* Uncompacted user space size */
	user_size = get_xsave_size_user();

	/*
	 * XSAVES kernel size includes supervisor states and uses compacted
	 * format. XSAVEC uses compacted format, but does not save
	 * supervisor states.
	 *
	 * XSAVE[OPT] do not support supervisor states so kernel and user
	 * size is identical.
	 */
	if (compacted)
		kernel_size = get_xsave_compacted_size();
	else
		kernel_size = user_size;

	kernel_default_size =
		xstate_calculate_size(fpu_kernel_cfg.default_features, compacted);

	if (!paranoid_xstate_size_valid(kernel_size))
		return -EINVAL;

	fpu_kernel_cfg.max_size = kernel_size;
	fpu_user_cfg.max_size = user_size;

	fpu_kernel_cfg.default_size = kernel_default_size;
	fpu_user_cfg.default_size =
		xstate_calculate_size(fpu_user_cfg.default_features, false);

	guest_default_cfg.size =
		xstate_calculate_size(guest_default_cfg.features, compacted);

	return 0;
}

/*
 * We enabled the XSAVE hardware, but something went wrong and
 * we can not use it.  Disable it.
 */
static void __init fpu__init_disable_system_xstate(unsigned int legacy_size)
{
	pr_info("x86/fpu: XSAVE disabled\n");

	fpu_kernel_cfg.max_features = 0;
	cr4_clear_bits(X86_CR4_OSXSAVE);
	setup_clear_cpu_cap(X86_FEATURE_XSAVE);

	/* Restore the legacy size.*/
	fpu_kernel_cfg.max_size = legacy_size;
	fpu_kernel_cfg.default_size = legacy_size;
	fpu_user_cfg.max_size = legacy_size;
	fpu_user_cfg.default_size = legacy_size;
	guest_default_cfg.size = legacy_size;

	/*
	 * Prevent enabling the static branch which enables writes to the
	 * XFD MSR.
	 */
	init_fpstate.xfd = 0;

	fpstate_reset(x86_task_fpu(current));
}

static u64 __init host_default_mask(void)
{
	/*
	 * Exclude dynamic features (require userspace opt-in) and features
	 * that are supported only for KVM guests.
	 */
	return ~((u64)XFEATURE_MASK_USER_DYNAMIC | XFEATURE_MASK_GUEST_SUPERVISOR);
}

static u64 __init guest_default_mask(void)
{
	/*
	 * Exclude dynamic features, which require userspace opt-in even
	 * for KVM guests.
	 */
	return ~(u64)XFEATURE_MASK_USER_DYNAMIC;
}

/*
 * Enable and initialize the xsave feature.
 * Called once per system bootup.
 */
void __init fpu__init_system_xstate(unsigned int legacy_size)
{
	unsigned int eax, ebx, ecx, edx;
	u64 xfeatures;
	int err;
	int i;

	if (!boot_cpu_has(X86_FEATURE_FPU)) {
		pr_info("x86/fpu: No FPU detected\n");
		return;
	}

	if (!boot_cpu_has(X86_FEATURE_XSAVE)) {
		pr_info("x86/fpu: x87 FPU will use %s\n",
			boot_cpu_has(X86_FEATURE_FXSR) ? "FXSAVE" : "FSAVE");
		return;
	}

	/*
	 * Find user xstates supported by the processor.
	 */
	cpuid_count(CPUID_LEAF_XSTATE, 0, &eax, &ebx, &ecx, &edx);
	fpu_kernel_cfg.max_features = eax + ((u64)edx << 32);

	/*
	 * Find supervisor xstates supported by the processor.
	 */
	cpuid_count(CPUID_LEAF_XSTATE, 1, &eax, &ebx, &ecx, &edx);
	fpu_kernel_cfg.max_features |= ecx + ((u64)edx << 32);

	if ((fpu_kernel_cfg.max_features & XFEATURE_MASK_FPSSE) != XFEATURE_MASK_FPSSE) {
		/*
		 * This indicates that something really unexpected happened
		 * with the enumeration.  Disable XSAVE and try to continue
		 * booting without it.  This is too early to BUG().
		 */
		pr_err("x86/fpu: FP/SSE not present amongst the CPU's xstate features: 0x%llx.\n",
		       fpu_kernel_cfg.max_features);
		goto out_disable;
	}

	if (fpu_kernel_cfg.max_features & XFEATURE_MASK_APX &&
	    fpu_kernel_cfg.max_features & (XFEATURE_MASK_BNDREGS | XFEATURE_MASK_BNDCSR)) {
		/*
		 * This is a problematic CPU configuration where two
		 * conflicting state components are both enumerated.
		 */
		pr_err("x86/fpu: Both APX/MPX present in the CPU's xstate features: 0x%llx.\n",
		       fpu_kernel_cfg.max_features);
		goto out_disable;
	}

	fpu_kernel_cfg.independent_features = fpu_kernel_cfg.max_features &
					      XFEATURE_MASK_INDEPENDENT;

	/*
	 * Clear XSAVE features that are disabled in the normal CPUID.
	 */
	for (i = 0; i < ARRAY_SIZE(xsave_cpuid_features); i++) {
		unsigned short cid = xsave_cpuid_features[i];

		/* Careful: X86_FEATURE_FPU is 0! */
		if ((i != XFEATURE_FP && !cid) || !boot_cpu_has(cid))
			fpu_kernel_cfg.max_features &= ~BIT_ULL(i);
	}

	if (!cpu_feature_enabled(X86_FEATURE_XFD))
		fpu_kernel_cfg.max_features &= ~XFEATURE_MASK_USER_DYNAMIC;

	if (!cpu_feature_enabled(X86_FEATURE_XSAVES))
		fpu_kernel_cfg.max_features &= XFEATURE_MASK_USER_SUPPORTED;
	else
		fpu_kernel_cfg.max_features &= XFEATURE_MASK_USER_SUPPORTED |
					XFEATURE_MASK_SUPERVISOR_SUPPORTED;

	fpu_user_cfg.max_features = fpu_kernel_cfg.max_features;
	fpu_user_cfg.max_features &= XFEATURE_MASK_USER_SUPPORTED;

	/*
	 * Now, given maximum feature set, determine default values by
	 * applying default masks.
	 */
	fpu_kernel_cfg.default_features = fpu_kernel_cfg.max_features & host_default_mask();
	fpu_user_cfg.default_features   = fpu_user_cfg.max_features & host_default_mask();
	guest_default_cfg.features      = fpu_kernel_cfg.max_features & guest_default_mask();

	/* Store it for paranoia check at the end */
	xfeatures = fpu_kernel_cfg.max_features;

	/*
	 * Initialize the default XFD state in initfp_state and enable the
	 * dynamic sizing mechanism if dynamic states are available.  The
	 * static key cannot be enabled here because this runs before
	 * jump_label_init(). This is delayed to an initcall.
	 */
	init_fpstate.xfd = fpu_user_cfg.max_features & XFEATURE_MASK_USER_DYNAMIC;

	/* Set up compaction feature bit */
	if (cpu_feature_enabled(X86_FEATURE_XSAVEC) ||
	    cpu_feature_enabled(X86_FEATURE_XSAVES))
		setup_force_cpu_cap(X86_FEATURE_XCOMPACTED);

	/* Enable xstate instructions to be able to continue with initialization: */
	fpu__init_cpu_xstate();

	/* Cache size, offset and flags for initialization */
	setup_xstate_cache();

	err = init_xstate_size();
	if (err)
		goto out_disable;

	/*
	 * Update info used for ptrace frames; use standard-format size and no
	 * supervisor xstates:
	 */
	update_regset_xstate_info(fpu_user_cfg.max_size,
				  fpu_user_cfg.max_features);

	/*
	 * init_fpstate excludes dynamic states as they are large but init
	 * state is zero.
	 */
	init_fpstate.size		= fpu_kernel_cfg.default_size;
	init_fpstate.xfeatures		= fpu_kernel_cfg.default_features;

	if (init_fpstate.size > sizeof(init_fpstate.regs)) {
		pr_warn("x86/fpu: init_fpstate buffer too small (%zu < %d)\n",
			sizeof(init_fpstate.regs), init_fpstate.size);
		goto out_disable;
	}

	setup_init_fpu_buf();

	/*
	 * Paranoia check whether something in the setup modified the
	 * xfeatures mask.
	 */
	if (xfeatures != fpu_kernel_cfg.max_features) {
		pr_err("x86/fpu: xfeatures modified from 0x%016llx to 0x%016llx during init\n",
		       xfeatures, fpu_kernel_cfg.max_features);
		goto out_disable;
	}

	/*
	 * CPU capabilities initialization runs before FPU init. So
	 * X86_FEATURE_OSXSAVE is not set. Now that XSAVE is completely
	 * functional, set the feature bit so depending code works.
	 */
	setup_force_cpu_cap(X86_FEATURE_OSXSAVE);

	print_xstate_offset_size();
	pr_info("x86/fpu: Enabled xstate features 0x%llx, context size is %d bytes, using '%s' format.\n",
		fpu_kernel_cfg.max_features,
		fpu_kernel_cfg.max_size,
		boot_cpu_has(X86_FEATURE_XCOMPACTED) ? "compacted" : "standard");
	return;

out_disable:
	/* something went wrong, try to boot without any XSAVE support */
	fpu__init_disable_system_xstate(legacy_size);
}

/*
 * Restore minimal FPU state after suspend:
 */
void fpu__resume_cpu(void)
{
	/*
	 * Restore XCR0 on xsave capable CPUs:
	 */
	if (cpu_feature_enabled(X86_FEATURE_XSAVE))
		xsetbv(XCR_XFEATURE_ENABLED_MASK, fpu_user_cfg.max_features);

	/*
	 * Restore IA32_XSS. The same CPUID bit enumerates support
	 * of XSAVES and MSR_IA32_XSS.
	 */
	if (cpu_feature_enabled(X86_FEATURE_XSAVES)) {
		wrmsrq(MSR_IA32_XSS, xfeatures_mask_supervisor()  |
				     xfeatures_mask_independent());
	}

	if (fpu_state_size_dynamic())
		wrmsrq(MSR_IA32_XFD, x86_task_fpu(current)->fpstate->xfd);
}

/*
 * Given an xstate feature nr, calculate where in the xsave
 * buffer the state is.  Callers should ensure that the buffer
 * is valid.
 */
static void *__raw_xsave_addr(struct xregs_state *xsave, int xfeature_nr)
{
	u64 xcomp_bv = xsave->header.xcomp_bv;

	if (WARN_ON_ONCE(!xfeature_enabled(xfeature_nr)))
		return NULL;

	if (cpu_feature_enabled(X86_FEATURE_XCOMPACTED)) {
		if (WARN_ON_ONCE(!(xcomp_bv & BIT_ULL(xfeature_nr))))
			return NULL;
	}

	return (void *)xsave + xfeature_get_offset(xcomp_bv, xfeature_nr);
}

/*
 * Given the xsave area and a state inside, this function returns the
 * address of the state.
 *
 * This is the API that is called to get xstate address in either
 * standard format or compacted format of xsave area.
 *
 * Note that if there is no data for the field in the xsave buffer
 * this will return NULL.
 *
 * Inputs:
 *	xstate: the thread's storage area for all FPU data
 *	xfeature_nr: state which is defined in xsave.h (e.g. XFEATURE_FP,
 *	XFEATURE_SSE, etc...)
 * Output:
 *	address of the state in the xsave area, or NULL if the
 *	field is not present in the xsave buffer.
 */
void *get_xsave_addr(struct xregs_state *xsave, int xfeature_nr)
{
	/*
	 * Do we even *have* xsave state?
	 */
	if (!boot_cpu_has(X86_FEATURE_XSAVE))
		return NULL;

	/*
	 * We should not ever be requesting features that we
	 * have not enabled.
	 */
	if (WARN_ON_ONCE(!xfeature_enabled(xfeature_nr)))
		return NULL;

	/*
	 * This assumes the last 'xsave*' instruction to
	 * have requested that 'xfeature_nr' be saved.
	 * If it did not, we might be seeing and old value
	 * of the field in the buffer.
	 *
	 * This can happen because the last 'xsave' did not
	 * request that this feature be saved (unlikely)
	 * or because the "init optimization" caused it
	 * to not be saved.
	 */
	if (!(xsave->header.xfeatures & BIT_ULL(xfeature_nr)))
		return NULL;

	return __raw_xsave_addr(xsave, xfeature_nr);
}
EXPORT_SYMBOL_GPL(get_xsave_addr);

/*
 * Given an xstate feature nr, calculate where in the xsave buffer the state is.
 * The xsave buffer should be in standard format, not compacted (e.g. user mode
 * signal frames).
 */
void __user *get_xsave_addr_user(struct xregs_state __user *xsave, int xfeature_nr)
{
	if (WARN_ON_ONCE(!xfeature_enabled(xfeature_nr)))
		return NULL;

	return (void __user *)xsave + xstate_offsets[xfeature_nr];
}

#ifdef CONFIG_ARCH_HAS_PKEYS

/*
 * This will go out and modify PKRU register to set the access
 * rights for @pkey to @init_val.
 */
int arch_set_user_pkey_access(struct task_struct *tsk, int pkey,
			      unsigned long init_val)
{
	u32 old_pkru, new_pkru_bits = 0;
	int pkey_shift;

	/*
	 * This check implies XSAVE support.  OSPKE only gets
	 * set if we enable XSAVE and we enable PKU in XCR0.
	 */
	if (!cpu_feature_enabled(X86_FEATURE_OSPKE))
		return -EINVAL;

	/*
	 * This code should only be called with valid 'pkey'
	 * values originating from in-kernel users.  Complain
	 * if a bad value is observed.
	 */
	if (WARN_ON_ONCE(pkey >= arch_max_pkey()))
		return -EINVAL;

	/* Set the bits we need in PKRU:  */
	if (init_val & PKEY_DISABLE_ACCESS)
		new_pkru_bits |= PKRU_AD_BIT;
	if (init_val & PKEY_DISABLE_WRITE)
		new_pkru_bits |= PKRU_WD_BIT;

	/* Shift the bits in to the correct place in PKRU for pkey: */
	pkey_shift = pkey * PKRU_BITS_PER_PKEY;
	new_pkru_bits <<= pkey_shift;

	/* Get old PKRU and mask off any old bits in place: */
	old_pkru = read_pkru();
	old_pkru &= ~((PKRU_AD_BIT|PKRU_WD_BIT) << pkey_shift);

	/* Write old part along with new part: */
	write_pkru(old_pkru | new_pkru_bits);

	return 0;
}
#endif /* ! CONFIG_ARCH_HAS_PKEYS */

static void copy_feature(bool from_xstate, struct membuf *to, void *xstate,
			 void *init_xstate, unsigned int size)
{
	membuf_write(to, from_xstate ? xstate : init_xstate, size);
}

/**
 * __copy_xstate_to_uabi_buf - Copy kernel saved xstate to a UABI buffer
 * @to:		membuf descriptor
 * @fpstate:	The fpstate buffer from which to copy
 * @xfeatures:	The mask of xfeatures to save (XSAVE mode only)
 * @pkru_val:	The PKRU value to store in the PKRU component
 * @copy_mode:	The requested copy mode
 *
 * Converts from kernel XSAVE or XSAVES compacted format to UABI conforming
 * format, i.e. from the kernel internal hardware dependent storage format
 * to the requested @mode. UABI XSTATE is always uncompacted!
 *
 * It supports partial copy but @to.pos always starts from zero.
 */
void __copy_xstate_to_uabi_buf(struct membuf to, struct fpstate *fpstate,
			       u64 xfeatures, u32 pkru_val,
			       enum xstate_copy_mode copy_mode)
{
	const unsigned int off_mxcsr = offsetof(struct fxregs_state, mxcsr);
	struct xregs_state *xinit = &init_fpstate.regs.xsave;
	struct xregs_state *xsave = &fpstate->regs.xsave;
	unsigned int zerofrom, i, xfeature;
	struct xstate_header header;
	u64 mask;

	memset(&header, 0, sizeof(header));
	header.xfeatures = xsave->header.xfeatures;

	/* Mask out the feature bits depending on copy mode */
	switch (copy_mode) {
	case XSTATE_COPY_FP:
		header.xfeatures &= XFEATURE_MASK_FP;
		break;

	case XSTATE_COPY_FX:
		header.xfeatures &= XFEATURE_MASK_FP | XFEATURE_MASK_SSE;
		break;

	case XSTATE_COPY_XSAVE:
		header.xfeatures &= fpstate->user_xfeatures & xfeatures;
		break;
	}

	/* Copy FP state up to MXCSR */
	copy_feature(header.xfeatures & XFEATURE_MASK_FP, &to, &xsave->i387,
		     &xinit->i387, off_mxcsr);

	/* Copy MXCSR when SSE or YMM are set in the feature mask */
	copy_feature(header.xfeatures & (XFEATURE_MASK_SSE | XFEATURE_MASK_YMM),
		     &to, &xsave->i387.mxcsr, &xinit->i387.mxcsr,
		     MXCSR_AND_FLAGS_SIZE);

	/* Copy the remaining FP state */
	copy_feature(header.xfeatures & XFEATURE_MASK_FP,
		     &to, &xsave->i387.st_space, &xinit->i387.st_space,
		     sizeof(xsave->i387.st_space));

	/* Copy the SSE state - shared with YMM, but independently managed */
	copy_feature(header.xfeatures & XFEATURE_MASK_SSE,
		     &to, &xsave->i387.xmm_space, &xinit->i387.xmm_space,
		     sizeof(xsave->i387.xmm_space));

	if (copy_mode != XSTATE_COPY_XSAVE)
		goto out;

	/* Zero the padding area */
	membuf_zero(&to, sizeof(xsave->i387.padding));

	/* Copy xsave->i387.sw_reserved */
	membuf_write(&to, xstate_fx_sw_bytes, sizeof(xsave->i387.sw_reserved));

	/* Copy the user space relevant state of @xsave->header */
	membuf_write(&to, &header, sizeof(header));

	zerofrom = offsetof(struct xregs_state, extended_state_area);

	/*
	 * This 'mask' indicates which states to copy from fpstate.
	 * Those extended states that are not present in fpstate are
	 * either disabled or initialized:
	 *
	 * In non-compacted format, disabled features still occupy
	 * state space but there is no state to copy from in the
	 * compacted init_fpstate. The gap tracking will zero these
	 * states.
	 *
	 * The extended features have an all zeroes init state. Thus,
	 * remove them from 'mask' to zero those features in the user
	 * buffer instead of retrieving them from init_fpstate.
	 */
	mask = header.xfeatures;

	for_each_extended_xfeature_in_order(i, mask) {
		xfeature = xfeature_uncompact_order[i];
		/*
		 * If there was a feature or alignment gap, zero the space
		 * in the destination buffer.
		 */
		if (zerofrom < xstate_offsets[xfeature])
			membuf_zero(&to, xstate_offsets[xfeature] - zerofrom);

		if (xfeature == XFEATURE_PKRU) {
			struct pkru_state pkru = {0};
			/*
			 * PKRU is not necessarily up to date in the
			 * XSAVE buffer. Use the provided value.
			 */
			pkru.pkru = pkru_val;
			membuf_write(&to, &pkru, sizeof(pkru));
		} else {
			membuf_write(&to,
				     __raw_xsave_addr(xsave, xfeature),
				     xstate_sizes[xfeature]);
		}
		/*
		 * Keep track of the last copied state in the non-compacted
		 * target buffer for gap zeroing.
		 */
		zerofrom = xstate_offsets[xfeature] + xstate_sizes[xfeature];
	}

out:
	if (to.left)
		membuf_zero(&to, to.left);
}

/**
 * copy_xstate_to_uabi_buf - Copy kernel saved xstate to a UABI buffer
 * @to:		membuf descriptor
 * @tsk:	The task from which to copy the saved xstate
 * @copy_mode:	The requested copy mode
 *
 * Converts from kernel XSAVE or XSAVES compacted format to UABI conforming
 * format, i.e. from the kernel internal hardware dependent storage format
 * to the requested @mode. UABI XSTATE is always uncompacted!
 *
 * It supports partial copy but @to.pos always starts from zero.
 */
void copy_xstate_to_uabi_buf(struct membuf to, struct task_struct *tsk,
			     enum xstate_copy_mode copy_mode)
{
	__copy_xstate_to_uabi_buf(to, x86_task_fpu(tsk)->fpstate,
				  x86_task_fpu(tsk)->fpstate->user_xfeatures,
				  tsk->thread.pkru, copy_mode);
}

static int copy_from_buffer(void *dst, unsigned int offset, unsigned int size,
			    const void *kbuf, const void __user *ubuf)
{
	if (kbuf) {
		memcpy(dst, kbuf + offset, size);
	} else {
		if (copy_from_user(dst, ubuf + offset, size))
			return -EFAULT;
	}
	return 0;
}


/**
 * copy_uabi_to_xstate - Copy a UABI format buffer to the kernel xstate
 * @fpstate:	The fpstate buffer to copy to
 * @kbuf:	The UABI format buffer, if it comes from the kernel
 * @ubuf:	The UABI format buffer, if it comes from userspace
 * @pkru:	The location to write the PKRU value to
 *
 * Converts from the UABI format into the kernel internal hardware
 * dependent format.
 *
 * This function ultimately has three different callers with distinct PKRU
 * behavior.
 * 1.	When called from sigreturn the PKRU register will be restored from
 *	@fpstate via an XRSTOR. Correctly copying the UABI format buffer to
 *	@fpstate is sufficient to cover this case, but the caller will also
 *	pass a pointer to the thread_struct's pkru field in @pkru and updating
 *	it is harmless.
 * 2.	When called from ptrace the PKRU register will be restored from the
 *	thread_struct's pkru field. A pointer to that is passed in @pkru.
 *	The kernel will restore it manually, so the XRSTOR behavior that resets
 *	the PKRU register to the hardware init value (0) if the corresponding
 *	xfeatures bit is not set is emulated here.
 * 3.	When called from KVM the PKRU register will be restored from the vcpu's
 *	pkru field. A pointer to that is passed in @pkru. KVM hasn't used
 *	XRSTOR and hasn't had the PKRU resetting behavior described above. To
 *	preserve that KVM behavior, it passes NULL for @pkru if the xfeatures
 *	bit is not set.
 */
static int copy_uabi_to_xstate(struct fpstate *fpstate, const void *kbuf,
			       const void __user *ubuf, u32 *pkru)
{
	struct xregs_state *xsave = &fpstate->regs.xsave;
	unsigned int offset, size;
	struct xstate_header hdr;
	u64 mask;
	int i;

	offset = offsetof(struct xregs_state, header);
	if (copy_from_buffer(&hdr, offset, sizeof(hdr), kbuf, ubuf))
		return -EFAULT;

	if (validate_user_xstate_header(&hdr, fpstate))
		return -EINVAL;

	/* Validate MXCSR when any of the related features is in use */
	mask = XFEATURE_MASK_FP | XFEATURE_MASK_SSE | XFEATURE_MASK_YMM;
	if (hdr.xfeatures & mask) {
		u32 mxcsr[2];

		offset = offsetof(struct fxregs_state, mxcsr);
		if (copy_from_buffer(mxcsr, offset, sizeof(mxcsr), kbuf, ubuf))
			return -EFAULT;

		/* Reserved bits in MXCSR must be zero. */
		if (mxcsr[0] & ~mxcsr_feature_mask)
			return -EINVAL;

		/* SSE and YMM require MXCSR even when FP is not in use. */
		if (!(hdr.xfeatures & XFEATURE_MASK_FP)) {
			xsave->i387.mxcsr = mxcsr[0];
			xsave->i387.mxcsr_mask = mxcsr[1];
		}
	}

	for (i = 0; i < XFEATURE_MAX; i++) {
		mask = BIT_ULL(i);

		if (hdr.xfeatures & mask) {
			void *dst = __raw_xsave_addr(xsave, i);

			offset = xstate_offsets[i];
			size = xstate_sizes[i];

			if (copy_from_buffer(dst, offset, size, kbuf, ubuf))
				return -EFAULT;
		}
	}

	if (hdr.xfeatures & XFEATURE_MASK_PKRU) {
		struct pkru_state *xpkru;

		xpkru = __raw_xsave_addr(xsave, XFEATURE_PKRU);
		*pkru = xpkru->pkru;
	} else {
		/*
		 * KVM may pass NULL here to indicate that it does not need
		 * PKRU updated.
		 */
		if (pkru)
			*pkru = 0;
	}

	/*
	 * The state that came in from userspace was user-state only.
	 * Mask all the user states out of 'xfeatures':
	 */
	xsave->header.xfeatures &= XFEATURE_MASK_SUPERVISOR_ALL;

	/*
	 * Add back in the features that came in from userspace:
	 */
	xsave->header.xfeatures |= hdr.xfeatures;

	return 0;
}

/*
 * Convert from a ptrace standard-format kernel buffer to kernel XSAVE[S]
 * format and copy to the target thread. Used by ptrace and KVM.
 */
int copy_uabi_from_kernel_to_xstate(struct fpstate *fpstate, const void *kbuf, u32 *pkru)
{
	return copy_uabi_to_xstate(fpstate, kbuf, NULL, pkru);
}

/*
 * Convert from a sigreturn standard-format user-space buffer to kernel
 * XSAVE[S] format and copy to the target thread. This is called from the
 * sigreturn() and rt_sigreturn() system calls.
 */
int copy_sigframe_from_user_to_xstate(struct task_struct *tsk,
				      const void __user *ubuf)
{
	return copy_uabi_to_xstate(x86_task_fpu(tsk)->fpstate, NULL, ubuf, &tsk->thread.pkru);
}

static bool validate_independent_components(u64 mask)
{
	u64 xchk;

	if (WARN_ON_FPU(!cpu_feature_enabled(X86_FEATURE_XSAVES)))
		return false;

	xchk = ~xfeatures_mask_independent();

	if (WARN_ON_ONCE(!mask || mask & xchk))
		return false;

	return true;
}

/**
 * xsaves - Save selected components to a kernel xstate buffer
 * @xstate:	Pointer to the buffer
 * @mask:	Feature mask to select the components to save
 *
 * The @xstate buffer must be 64 byte aligned and correctly initialized as
 * XSAVES does not write the full xstate header. Before first use the
 * buffer should be zeroed otherwise a consecutive XRSTORS from that buffer
 * can #GP.
 *
 * The feature mask must be a subset of the independent features.
 */
void xsaves(struct xregs_state *xstate, u64 mask)
{
	int err;

	if (!validate_independent_components(mask))
		return;

	XSTATE_OP(XSAVES, xstate, (u32)mask, (u32)(mask >> 32), err);
	WARN_ON_ONCE(err);
}

/**
 * xrstors - Restore selected components from a kernel xstate buffer
 * @xstate:	Pointer to the buffer
 * @mask:	Feature mask to select the components to restore
 *
 * The @xstate buffer must be 64 byte aligned and correctly initialized
 * otherwise XRSTORS from that buffer can #GP.
 *
 * Proper usage is to restore the state which was saved with
 * xsaves() into @xstate.
 *
 * The feature mask must be a subset of the independent features.
 */
void xrstors(struct xregs_state *xstate, u64 mask)
{
	int err;

	if (!validate_independent_components(mask))
		return;

	XSTATE_OP(XRSTORS, xstate, (u32)mask, (u32)(mask >> 32), err);
	WARN_ON_ONCE(err);
}

#if IS_ENABLED(CONFIG_KVM)
void fpstate_clear_xstate_component(struct fpstate *fpstate, unsigned int xfeature)
{
	void *addr = get_xsave_addr(&fpstate->regs.xsave, xfeature);

	if (addr)
		memset(addr, 0, xstate_sizes[xfeature]);
}
EXPORT_SYMBOL_GPL(fpstate_clear_xstate_component);
#endif

#ifdef CONFIG_X86_64

#ifdef CONFIG_X86_DEBUG_FPU
/*
 * Ensure that a subsequent XSAVE* or XRSTOR* instruction with RFBM=@mask
 * can safely operate on the @fpstate buffer.
 */
static bool xstate_op_valid(struct fpstate *fpstate, u64 mask, bool rstor)
{
	u64 xfd = __this_cpu_read(xfd_state);

	if (fpstate->xfd == xfd)
		return true;

	 /*
	  * The XFD MSR does not match fpstate->xfd. That's invalid when
	  * the passed in fpstate is current's fpstate.
	  */
	if (fpstate->xfd == x86_task_fpu(current)->fpstate->xfd)
		return false;

	/*
	 * XRSTOR(S) from init_fpstate are always correct as it will just
	 * bring all components into init state and not read from the
	 * buffer. XSAVE(S) raises #PF after init.
	 */
	if (fpstate == &init_fpstate)
		return rstor;

	/*
	 * XSAVE(S): clone(), fpu_swap_kvm_fpstate()
	 * XRSTORS(S): fpu_swap_kvm_fpstate()
	 */

	/*
	 * No XSAVE/XRSTOR instructions (except XSAVE itself) touch
	 * the buffer area for XFD-disabled state components.
	 */
	mask &= ~xfd;

	/*
	 * Remove features which are valid in fpstate. They
	 * have space allocated in fpstate.
	 */
	mask &= ~fpstate->xfeatures;

	/*
	 * Any remaining state components in 'mask' might be written
	 * by XSAVE/XRSTOR. Fail validation it found.
	 */
	return !mask;
}

void xfd_validate_state(struct fpstate *fpstate, u64 mask, bool rstor)
{
	WARN_ON_ONCE(!xstate_op_valid(fpstate, mask, rstor));
}
#endif /* CONFIG_X86_DEBUG_FPU */

static int __init xfd_update_static_branch(void)
{
	/*
	 * If init_fpstate.xfd has bits set then dynamic features are
	 * available and the dynamic sizing must be enabled.
	 */
	if (init_fpstate.xfd)
		static_branch_enable(&__fpu_state_size_dynamic);
	return 0;
}
arch_initcall(xfd_update_static_branch)

void fpstate_free(struct fpu *fpu)
{
	if (fpu->fpstate && fpu->fpstate != &fpu->__fpstate)
		vfree(fpu->fpstate);
}

/**
 * fpstate_realloc - Reallocate struct fpstate for the requested new features
 *
 * @xfeatures:	A bitmap of xstate features which extend the enabled features
 *		of that task
 * @ksize:	The required size for the kernel buffer
 * @usize:	The required size for user space buffers
 * @guest_fpu:	Pointer to a guest FPU container. NULL for host allocations
 *
 * Note vs. vmalloc(): If the task with a vzalloc()-allocated buffer
 * terminates quickly, vfree()-induced IPIs may be a concern, but tasks
 * with large states are likely to live longer.
 *
 * Returns: 0 on success, -ENOMEM on allocation error.
 */
static int fpstate_realloc(u64 xfeatures, unsigned int ksize,
			   unsigned int usize, struct fpu_guest *guest_fpu)
{
	struct fpu *fpu = x86_task_fpu(current);
	struct fpstate *curfps, *newfps = NULL;
	unsigned int fpsize;
	bool in_use;

	fpsize = ksize + ALIGN(offsetof(struct fpstate, regs), 64);

	newfps = vzalloc(fpsize);
	if (!newfps)
		return -ENOMEM;
	newfps->size = ksize;
	newfps->user_size = usize;
	newfps->is_valloc = true;

	/*
	 * When a guest FPU is supplied, use @guest_fpu->fpstate
	 * as reference independent whether it is in use or not.
	 */
	curfps = guest_fpu ? guest_fpu->fpstate : fpu->fpstate;

	/* Determine whether @curfps is the active fpstate */
	in_use = fpu->fpstate == curfps;

	if (guest_fpu) {
		newfps->is_guest = true;
		newfps->is_confidential = curfps->is_confidential;
		newfps->in_use = curfps->in_use;
		guest_fpu->xfeatures |= xfeatures;
		guest_fpu->uabi_size = usize;
	}

	fpregs_lock();
	/*
	 * If @curfps is in use, ensure that the current state is in the
	 * registers before swapping fpstate as that might invalidate it
	 * due to layout changes.
	 */
	if (in_use && test_thread_flag(TIF_NEED_FPU_LOAD))
		fpregs_restore_userregs();

	newfps->xfeatures = curfps->xfeatures | xfeatures;
	newfps->user_xfeatures = curfps->user_xfeatures | xfeatures;
	newfps->xfd = curfps->xfd & ~xfeatures;

	/* Do the final updates within the locked region */
	xstate_init_xcomp_bv(&newfps->regs.xsave, newfps->xfeatures);

	if (guest_fpu) {
		guest_fpu->fpstate = newfps;
		/* If curfps is active, update the FPU fpstate pointer */
		if (in_use)
			fpu->fpstate = newfps;
	} else {
		fpu->fpstate = newfps;
	}

	if (in_use)
		xfd_update_state(fpu->fpstate);
	fpregs_unlock();

	/* Only free valloc'ed state */
	if (curfps && curfps->is_valloc)
		vfree(curfps);

	return 0;
}

static int validate_sigaltstack(unsigned int usize)
{
	struct task_struct *thread, *leader = current->group_leader;
	unsigned long framesize = get_sigframe_size();

	lockdep_assert_held(&current->sighand->siglock);

	/* get_sigframe_size() is based on fpu_user_cfg.max_size */
	framesize -= fpu_user_cfg.max_size;
	framesize += usize;
	for_each_thread(leader, thread) {
		if (thread->sas_ss_size && thread->sas_ss_size < framesize)
			return -ENOSPC;
	}
	return 0;
}

static int __xstate_request_perm(u64 permitted, u64 requested, bool guest)
{
	/*
	 * This deliberately does not exclude !XSAVES as we still might
	 * decide to optionally context switch XCR0 or talk the silicon
	 * vendors into extending XFD for the pre AMX states, especially
	 * AVX512.
	 */
	bool compacted = cpu_feature_enabled(X86_FEATURE_XCOMPACTED);
	struct fpu *fpu = x86_task_fpu(current->group_leader);
	struct fpu_state_perm *perm;
	unsigned int ksize, usize;
	u64 mask;
	int ret = 0;

	/* Check whether fully enabled */
	if ((permitted & requested) == requested)
		return 0;

	/*
	 * Calculate the resulting kernel state size.  Note, @permitted also
	 * contains supervisor xfeatures even though supervisor are always
	 * permitted for kernel and guest FPUs, and never permitted for user
	 * FPUs.
	 */
	mask = permitted | requested;
	ksize = xstate_calculate_size(mask, compacted);

	/*
	 * Calculate the resulting user state size.  Take care not to clobber
	 * the supervisor xfeatures in the new mask!
	 */
	usize = xstate_calculate_size(mask & XFEATURE_MASK_USER_SUPPORTED, false);

	if (!guest) {
		ret = validate_sigaltstack(usize);
		if (ret)
			return ret;
	}

	perm = guest ? &fpu->guest_perm : &fpu->perm;
	/* Pairs with the READ_ONCE() in xstate_get_group_perm() */
	WRITE_ONCE(perm->__state_perm, mask);
	/* Protected by sighand lock */
	perm->__state_size = ksize;
	perm->__user_state_size = usize;
	return ret;
}

/*
 * Permissions array to map facilities with more than one component
 */
static const u64 xstate_prctl_req[XFEATURE_MAX] = {
	[XFEATURE_XTILE_DATA] = XFEATURE_MASK_XTILE_DATA,
};

static int xstate_request_perm(unsigned long idx, bool guest)
{
	u64 permitted, requested;
	int ret;

	if (idx >= XFEATURE_MAX)
		return -EINVAL;

	/*
	 * Look up the facility mask which can require more than
	 * one xstate component.
	 */
	idx = array_index_nospec(idx, ARRAY_SIZE(xstate_prctl_req));
	requested = xstate_prctl_req[idx];
	if (!requested)
		return -EOPNOTSUPP;

	if ((fpu_user_cfg.max_features & requested) != requested)
		return -EOPNOTSUPP;

	/* Lockless quick check */
	permitted = xstate_get_group_perm(guest);
	if ((permitted & requested) == requested)
		return 0;

	/* Protect against concurrent modifications */
	spin_lock_irq(&current->sighand->siglock);
	permitted = xstate_get_group_perm(guest);

	/* First vCPU allocation locks the permissions. */
	if (guest && (permitted & FPU_GUEST_PERM_LOCKED))
		ret = -EBUSY;
	else
		ret = __xstate_request_perm(permitted, requested, guest);
	spin_unlock_irq(&current->sighand->siglock);
	return ret;
}

int __xfd_enable_feature(u64 xfd_err, struct fpu_guest *guest_fpu)
{
	u64 xfd_event = xfd_err & XFEATURE_MASK_USER_DYNAMIC;
	struct fpu_state_perm *perm;
	unsigned int ksize, usize;
	struct fpu *fpu;

	if (!xfd_event) {
		if (!guest_fpu)
			pr_err_once("XFD: Invalid xfd error: %016llx\n", xfd_err);
		return 0;
	}

	/* Protect against concurrent modifications */
	spin_lock_irq(&current->sighand->siglock);

	/* If not permitted let it die */
	if ((xstate_get_group_perm(!!guest_fpu) & xfd_event) != xfd_event) {
		spin_unlock_irq(&current->sighand->siglock);
		return -EPERM;
	}

	fpu = x86_task_fpu(current->group_leader);
	perm = guest_fpu ? &fpu->guest_perm : &fpu->perm;
	ksize = perm->__state_size;
	usize = perm->__user_state_size;

	/*
	 * The feature is permitted. State size is sufficient.  Dropping
	 * the lock is safe here even if more features are added from
	 * another task, the retrieved buffer sizes are valid for the
	 * currently requested feature(s).
	 */
	spin_unlock_irq(&current->sighand->siglock);

	/*
	 * Try to allocate a new fpstate. If that fails there is no way
	 * out.
	 */
	if (fpstate_realloc(xfd_event, ksize, usize, guest_fpu))
		return -EFAULT;
	return 0;
}

int xfd_enable_feature(u64 xfd_err)
{
	return __xfd_enable_feature(xfd_err, NULL);
}

#else /* CONFIG_X86_64 */
static inline int xstate_request_perm(unsigned long idx, bool guest)
{
	return -EPERM;
}
#endif  /* !CONFIG_X86_64 */

u64 xstate_get_guest_group_perm(void)
{
	return xstate_get_group_perm(true);
}
EXPORT_SYMBOL_GPL(xstate_get_guest_group_perm);

/**
 * fpu_xstate_prctl - xstate permission operations
 * @option:	A subfunction of arch_prctl()
 * @arg2:	option argument
 * Return:	0 if successful; otherwise, an error code
 *
 * Option arguments:
 *
 * ARCH_GET_XCOMP_SUPP: Pointer to user space u64 to store the info
 * ARCH_GET_XCOMP_PERM: Pointer to user space u64 to store the info
 * ARCH_REQ_XCOMP_PERM: Facility number requested
 *
 * For facilities which require more than one XSTATE component, the request
 * must be the highest state component number related to that facility,
 * e.g. for AMX which requires XFEATURE_XTILE_CFG(17) and
 * XFEATURE_XTILE_DATA(18) this would be XFEATURE_XTILE_DATA(18).
 */
long fpu_xstate_prctl(int option, unsigned long arg2)
{
	u64 __user *uptr = (u64 __user *)arg2;
	u64 permitted, supported;
	unsigned long idx = arg2;
	bool guest = false;

	switch (option) {
	case ARCH_GET_XCOMP_SUPP:
		supported = fpu_user_cfg.max_features |	fpu_user_cfg.legacy_features;
		return put_user(supported, uptr);

	case ARCH_GET_XCOMP_PERM:
		/*
		 * Lockless snapshot as it can also change right after the
		 * dropping the lock.
		 */
		permitted = xstate_get_host_group_perm();
		permitted &= XFEATURE_MASK_USER_SUPPORTED;
		return put_user(permitted, uptr);

	case ARCH_GET_XCOMP_GUEST_PERM:
		permitted = xstate_get_guest_group_perm();
		permitted &= XFEATURE_MASK_USER_SUPPORTED;
		return put_user(permitted, uptr);

	case ARCH_REQ_XCOMP_GUEST_PERM:
		guest = true;
		fallthrough;

	case ARCH_REQ_XCOMP_PERM:
		if (!IS_ENABLED(CONFIG_X86_64))
			return -EOPNOTSUPP;

		return xstate_request_perm(idx, guest);

	default:
		return -EINVAL;
	}
}

#ifdef CONFIG_PROC_PID_ARCH_STATUS
/*
 * Report the amount of time elapsed in millisecond since last AVX512
 * use in the task.
 */
static void avx512_status(struct seq_file *m, struct task_struct *task)
{
	unsigned long timestamp = READ_ONCE(x86_task_fpu(task)->avx512_timestamp);
	long delta;

	if (!timestamp) {
		/*
		 * Report -1 if no AVX512 usage
		 */
		delta = -1;
	} else {
		delta = (long)(jiffies - timestamp);
		/*
		 * Cap to LONG_MAX if time difference > LONG_MAX
		 */
		if (delta < 0)
			delta = LONG_MAX;
		delta = jiffies_to_msecs(delta);
	}

	seq_put_decimal_ll(m, "AVX512_elapsed_ms:\t", delta);
	seq_putc(m, '\n');
}

/*
 * Report architecture specific information
 */
int proc_pid_arch_status(struct seq_file *m, struct pid_namespace *ns,
			struct pid *pid, struct task_struct *task)
{
	/*
	 * Report AVX512 state if the processor and build option supported.
	 */
	if (cpu_feature_enabled(X86_FEATURE_AVX512F))
		avx512_status(m, task);

	return 0;
}
#endif /* CONFIG_PROC_PID_ARCH_STATUS */

#ifdef CONFIG_COREDUMP
static const char owner_name[] = "LINUX";

/*
 * Dump type, size, offset and flag values for every xfeature that is present.
 */
static int dump_xsave_layout_desc(struct coredump_params *cprm)
{
	int num_records = 0;
	int i;

	for_each_extended_xfeature(i, fpu_user_cfg.max_features) {
		struct x86_xfeat_component xc = {
			.type   = i,
			.size   = xstate_sizes[i],
			.offset = xstate_offsets[i],
			/* reserved for future use */
			.flags  = 0,
		};

		if (!dump_emit(cprm, &xc, sizeof(xc)))
			return 0;

		num_records++;
	}
	return num_records;
}

static u32 get_xsave_desc_size(void)
{
	u32 cnt = 0;
	u32 i;

	for_each_extended_xfeature(i, fpu_user_cfg.max_features)
		cnt++;

	return cnt * (sizeof(struct x86_xfeat_component));
}

int elf_coredump_extra_notes_write(struct coredump_params *cprm)
{
	int num_records = 0;
	struct elf_note en;

	if (!fpu_user_cfg.max_features)
		return 0;

	en.n_namesz = sizeof(owner_name);
	en.n_descsz = get_xsave_desc_size();
	en.n_type = NT_X86_XSAVE_LAYOUT;

	if (!dump_emit(cprm, &en, sizeof(en)))
		return 1;
	if (!dump_emit(cprm, owner_name, en.n_namesz))
		return 1;
	if (!dump_align(cprm, 4))
		return 1;

	num_records = dump_xsave_layout_desc(cprm);
	if (!num_records)
		return 1;

	/* Total size should be equal to the number of records */
	if ((sizeof(struct x86_xfeat_component) * num_records) != en.n_descsz)
		return 1;

	return 0;
}

int elf_coredump_extra_notes_size(void)
{
	int size;

	if (!fpu_user_cfg.max_features)
		return 0;

	/* .note header */
	size  = sizeof(struct elf_note);
	/*  Name plus alignment to 4 bytes */
	size += roundup(sizeof(owner_name), 4);
	size += get_xsave_desc_size();

	return size;
}
#endif /* CONFIG_COREDUMP */
