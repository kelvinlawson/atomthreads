#ifndef __ATOMPORT_ASM_MACROS_H_
#define __ATOMPORT_ASM_MACROS_H_

#include "regs.h"

#ifdef __ASSEMBLY__ /* to be called only from assembly */

#define LEAF(fn)				\
        .globl fn;				\
        .ent fn;				\
fn:

#define END(fn)					\
        .size fn,.-fn;				\
        .end fn

#define tlbp_write_hazard 			\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;

#define tlbp_read_hazard 			\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;

#define tlbw_write_hazard 			\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;					\
        nop;

#define enable_global_interrupts		ei
#define disable_global_interrupts		di $0

#define EXCEPTION_VECTOR(_name, _offset, _where)\
	. = _offset;				\
	.set noreorder;				\
_name:						\
	b _where;				\
	nop;

#define SAVE_REG(reg, treg)			\
        sw reg, ((reg ## _IDX) * 4)(treg)

#define LOAD_REG(reg, treg)			\
        lw reg, ((reg ## _IDX) * 4)(treg)

#define SAVE_INT_CONTEXT(treg)				\
        mfc0 k1, CP0_EPC;				\
	SAVE_REG(t0,treg);				\
	SAVE_REG(t1,treg);				\
	SAVE_REG(t2,treg);				\
	SAVE_REG(t3,treg);				\
	SAVE_REG(t4,treg);				\
	SAVE_REG(t5,treg);				\
	SAVE_REG(t6,treg);				\
	SAVE_REG(t7,treg);				\
	SAVE_REG(t8,treg);				\
	SAVE_REG(t9,treg);				\
	SAVE_REG(v0,treg);				\
	SAVE_REG(v1,treg);				\
	SAVE_REG(a0,treg);				\
	SAVE_REG(a1,treg);				\
	SAVE_REG(a2,treg);				\
	SAVE_REG(a3,treg);				\
	SAVE_REG(s0,treg);				\
	SAVE_REG(s1,treg);				\
	SAVE_REG(s2,treg);				\
	SAVE_REG(s3,treg);				\
	SAVE_REG(s4,treg);				\
	SAVE_REG(s5,treg);				\
	SAVE_REG(s6,treg);				\
	SAVE_REG(s7,treg);				\
	SAVE_REG(gp,treg);				\
	SAVE_REG(s8,treg);				\
	SAVE_REG(ra,treg);				\
        sw k0, (sp_IDX * 4)(treg);			\
        sw k1, (cp0_epc_IDX * 4)(treg);

#define RESTORE_INT_CONTEXT(treg)			\
        lw k1, (cp0_epc_IDX * 4)(treg);			\
        mtc0 k1, CP0_EPC;				\
	LOAD_REG(s0,treg);				\
	LOAD_REG(s1,treg);				\
	LOAD_REG(s2,treg);				\
	LOAD_REG(s3,treg);				\
	LOAD_REG(s4,treg);				\
	LOAD_REG(s5,treg);				\
	LOAD_REG(s6,treg);				\
	LOAD_REG(s7,treg);				\
	LOAD_REG(v0,treg);				\
	LOAD_REG(v1,treg);				\
	LOAD_REG(a0,treg);				\
	LOAD_REG(a1,treg);				\
	LOAD_REG(a2,treg);				\
	LOAD_REG(a3,treg);				\
	LOAD_REG(t0,treg);				\
	LOAD_REG(t1,treg);				\
	LOAD_REG(t2,treg);				\
	LOAD_REG(t3,treg);				\
	LOAD_REG(t4,treg);				\
	LOAD_REG(t5,treg);				\
	LOAD_REG(t6,treg);				\
	LOAD_REG(t7,treg);				\
	LOAD_REG(t8,treg);				\
	LOAD_REG(t9,treg);				\
	LOAD_REG(gp,treg);				\
	LOAD_REG(ra,treg);				\
	LOAD_REG(s8,treg);				\
	lw sp, (sp_IDX * 4)(treg);

#endif /* __ASSEMBLY__ */

#define num_to_string(s)	to_string(s)
#define to_string(s)		#s

#define IASM_SAVE_REG(reg, here)				\
	"sw " to_string(reg) " , " num_to_string(reg ## _IDX)	\
			     " * 4(" num_to_string(here)" )\n\t"

#define IASM_LOAD_REG(reg, here)				\
	"lw " to_string(reg) " , " num_to_string(reg ## _IDX)	\
			     " * 4(" num_to_string(here)" )\n\t"


/*
 * Macros to be used with C code.
 */
#define __read_32bit_c0_register(source, sel)				\
({ int __res;								\
	if (sel == 0)							\
		__asm__ __volatile__(					\
			"mfc0\t%0, " #source "\n\t"			\
			: "=r" (__res));				\
	else								\
		__asm__ __volatile__(					\
			".set\tmips32\n\t"				\
			"mfc0\t%0, " #source ", " #sel "\n\t"		\
			".set\tmips0\n\t"				\
			: "=r" (__res));				\
	__res;								\
})

#define __write_32bit_c0_register(register, sel, value)			\
do {									\
	if (sel == 0)							\
		__asm__ __volatile__(					\
			"mtc0\t%z0, " #register "\n\t"			\
			: : "Jr" ((unsigned int)(value)));		\
	else								\
		__asm__ __volatile__(					\
			".set\tmips32\n\t"				\
			"mtc0\t%z0, " #register ", " #sel "\n\t"	\
			".set\tmips0"					\
			: : "Jr" ((unsigned int)(value)));		\
} while (0)

#define __read_ulong_c0_register(reg, sel)				\
	(unsigned long) __read_32bit_c0_register(reg, sel)

#define __write_ulong_c0_register(reg, sel, val)			\
do {									\
		__write_32bit_c0_register(reg, sel, val);		\
} while (0)

#define read_c0_index()		__read_32bit_c0_register($0, 0)
#define write_c0_index(val)	__write_32bit_c0_register($0, 0, val)

#define read_c0_entrylo0()	__read_ulong_c0_register($2, 0)
#define write_c0_entrylo0(val)	__write_ulong_c0_register($2, 0, val)

#define read_c0_entrylo1()	__read_ulong_c0_register($3, 0)
#define write_c0_entrylo1(val)	__write_ulong_c0_register($3, 0, val)

#define read_c0_conf()		__read_32bit_c0_register($3, 0)
#define write_c0_conf(val)	__write_32bit_c0_register($3, 0, val)

#define read_c0_context()	__read_ulong_c0_register($4, 0)
#define write_c0_context(val)	__write_ulong_c0_register($4, 0, val)

#define read_c0_userlocal()	__read_ulong_c0_register($4, 2)
#define write_c0_userlocal(val)	__write_ulong_c0_register($4, 2, val)

#define read_c0_pagemask()	__read_32bit_c0_register($5, 0)
#define write_c0_pagemask(val)	__write_32bit_c0_register($5, 0, val)

#define read_c0_wired()		__read_32bit_c0_register($6, 0)
#define write_c0_wired(val)	__write_32bit_c0_register($6, 0, val)

#define read_c0_info()		__read_32bit_c0_register($7, 0)

#define read_c0_badvaddr()	__read_ulong_c0_register($8, 0)
#define write_c0_badvaddr(val)	__write_ulong_c0_register($8, 0, val)

#define read_c0_count()		__read_32bit_c0_register($9, 0)
#define write_c0_count(val)	__write_32bit_c0_register($9, 0, val)

#define read_c0_entryhi()	__read_ulong_c0_register($10, 0)
#define write_c0_entryhi(val)	__write_ulong_c0_register($10, 0, val)

#define read_c0_compare()	__read_32bit_c0_register($11, 0)
#define write_c0_compare(val)	__write_32bit_c0_register($11, 0, val)

#define read_c0_status()	__read_32bit_c0_register($12, 0)
#define write_c0_status(val)	__write_32bit_c0_register($12, 0, val)

#define read_c0_cause()		__read_32bit_c0_register($13, 0)
#define write_c0_cause(val)	__write_32bit_c0_register($13, 0, val)

#define read_c0_epc()		__read_ulong_c0_register($14, 0)
#define write_c0_epc(val)	__write_ulong_c0_register($14, 0, val)

#define read_c0_prid()		__read_32bit_c0_register($15, 0)

#define read_c0_config()	__read_32bit_c0_register($16, 0)
#define read_c0_config1()	__read_32bit_c0_register($16, 1)
#define read_c0_config2()	__read_32bit_c0_register($16, 2)
#define write_c0_config(val)	__write_32bit_c0_register($16, 0, val)
#define write_c0_config1(val)	__write_32bit_c0_register($16, 1, val)
#define write_c0_config2(val)	__write_32bit_c0_register($16, 2, val)

#define read_c0_xcontext()	__read_ulong_c0_register($20, 0)
#define write_c0_xcontext(val)	__write_ulong_c0_register($20, 0, val)

#define read_c0_intcontrol()	__read_32bit_c0_ctrl_register($20)
#define write_c0_intcontrol(val) __write_32bit_c0_ctrl_register($20, val)

#define read_c0_framemask()	__read_32bit_c0_register($21, 0)
#define write_c0_framemask(val)	__write_32bit_c0_register($21, 0, val)

/*
 * MIPS32 / MIPS64 performance counters
 */
#define read_c0_cacheerr()	__read_32bit_c0_register($27, 0)

#define read_c0_taglo()		__read_32bit_c0_register($28, 0)
#define write_c0_taglo(val)	__write_32bit_c0_register($28, 0, val)

#define read_c0_dtaglo()	__read_32bit_c0_register($28, 2)
#define write_c0_dtaglo(val)	__write_32bit_c0_register($28, 2, val)

#define read_c0_taghi()		__read_32bit_c0_register($29, 0)
#define write_c0_taghi(val)	__write_32bit_c0_register($29, 0, val)

#define read_c0_errorepc()	__read_ulong_c0_register($30, 0)
#define write_c0_errorepc(val)	__write_ulong_c0_register($30, 0, val)

/* MIPSR2 */
#define read_c0_hwrena()	__read_32bit_c0_register($7, 0)
#define write_c0_hwrena(val)	__write_32bit_c0_register($7, 0, val)

#define read_c0_intctl()	__read_32bit_c0_register($12, 1)
#define write_c0_intctl(val)	__write_32bit_c0_register($12, 1, val)

#define read_c0_srsctl()	__read_32bit_c0_register($12, 2)
#define write_c0_srsctl(val)	__write_32bit_c0_register($12, 2, val)

#define read_c0_srsmap()	__read_32bit_c0_register($12, 3)
#define write_c0_srsmap(val)	__write_32bit_c0_register($12, 3, val)

#define read_c0_ebase()		__read_32bit_c0_register($15, 1)
#define write_c0_ebase(val)	__write_32bit_c0_register($15, 1, val)

#endif /* __ATOMPORT_ASM_MACROS_H_ */
