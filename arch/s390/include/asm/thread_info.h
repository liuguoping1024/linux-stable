/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  S390 version
 *    Copyright IBM Corp. 2002, 2006
 *    Author(s): Martin Schwidefsky (schwidefsky@de.ibm.com)
 */

#ifndef _ASM_THREAD_INFO_H
#define _ASM_THREAD_INFO_H

#include <linux/bits.h>
#include <vdso/page.h>

/*
 * General size of kernel stacks
 */
#if defined(CONFIG_KASAN) || defined(CONFIG_KMSAN)
#define THREAD_SIZE_ORDER 4
#else
#define THREAD_SIZE_ORDER 2
#endif
#define BOOT_STACK_SIZE (PAGE_SIZE << 2)
#define THREAD_SIZE (PAGE_SIZE << THREAD_SIZE_ORDER)

#define STACK_INIT_OFFSET (THREAD_SIZE - STACK_FRAME_OVERHEAD - __PT_SIZE)

#ifndef __ASSEMBLER__

/*
 * low level task data that entry.S needs immediate access to
 * - this struct should fit entirely inside of one cache line
 * - this struct shares the supervisor stack pages
 * - if the contents of this structure are changed, the assembly constants must also be changed
 */
struct thread_info {
	unsigned long		flags;		/* low level flags */
	unsigned long		syscall_work;	/* SYSCALL_WORK_ flags */
	unsigned int		cpu;		/* current CPU */
	unsigned char		sie;		/* running in SIE context */
};

/*
 * macros/functions for gaining access to the thread information structure
 */
#define INIT_THREAD_INFO(tsk)			\
{						\
	.flags		= 0,			\
}

struct task_struct;

void arch_setup_new_exec(void);
#define arch_setup_new_exec arch_setup_new_exec

#endif

/*
 * thread information flags bit numbers
 */
#define TIF_NOTIFY_RESUME	0	/* callback before returning to user */
#define TIF_SIGPENDING		1	/* signal pending */
#define TIF_NEED_RESCHED	2	/* rescheduling necessary */
#define TIF_NEED_RESCHED_LAZY	3	/* lazy rescheduling needed */
#define TIF_UPROBE		4	/* breakpointed or single-stepping */
#define TIF_PATCH_PENDING	5	/* pending live patching update */
#define TIF_ASCE_PRIMARY	6	/* primary asce is kernel asce */
#define TIF_NOTIFY_SIGNAL	7	/* signal notifications exist */
#define TIF_GUARDED_STORAGE	8	/* load guarded storage control block */
#define TIF_ISOLATE_BP_GUEST	9	/* Run KVM guests with isolated BP */
#define TIF_PER_TRAP		10	/* Need to handle PER trap on exit to usermode */
#define TIF_31BIT		16	/* 32bit process */
#define TIF_MEMDIE		17	/* is terminating due to OOM killer */
#define TIF_RESTORE_SIGMASK	18	/* restore signal mask in do_signal() */
#define TIF_SINGLE_STEP		19	/* This task is single stepped */
#define TIF_BLOCK_STEP		20	/* This task is block stepped */
#define TIF_UPROBE_SINGLESTEP	21	/* This task is uprobe single stepped */
#define TIF_SYSCALL_TRACE	24	/* syscall trace active */
#define TIF_SYSCALL_AUDIT	25	/* syscall auditing active */
#define TIF_SECCOMP		26	/* secure computing */
#define TIF_SYSCALL_TRACEPOINT	27	/* syscall tracepoint instrumentation */

#define _TIF_NOTIFY_RESUME	BIT(TIF_NOTIFY_RESUME)
#define _TIF_SIGPENDING		BIT(TIF_SIGPENDING)
#define _TIF_NEED_RESCHED	BIT(TIF_NEED_RESCHED)
#define _TIF_NEED_RESCHED_LAZY	BIT(TIF_NEED_RESCHED_LAZY)
#define _TIF_UPROBE		BIT(TIF_UPROBE)
#define _TIF_PATCH_PENDING	BIT(TIF_PATCH_PENDING)
#define _TIF_ASCE_PRIMARY	BIT(TIF_ASCE_PRIMARY)
#define _TIF_NOTIFY_SIGNAL	BIT(TIF_NOTIFY_SIGNAL)
#define _TIF_GUARDED_STORAGE	BIT(TIF_GUARDED_STORAGE)
#define _TIF_ISOLATE_BP_GUEST	BIT(TIF_ISOLATE_BP_GUEST)
#define _TIF_PER_TRAP		BIT(TIF_PER_TRAP)
#define _TIF_31BIT		BIT(TIF_31BIT)
#define _TIF_MEMDIE		BIT(TIF_MEMDIE)
#define _TIF_RESTORE_SIGMASK	BIT(TIF_RESTORE_SIGMASK)
#define _TIF_SINGLE_STEP	BIT(TIF_SINGLE_STEP)
#define _TIF_BLOCK_STEP		BIT(TIF_BLOCK_STEP)
#define _TIF_UPROBE_SINGLESTEP	BIT(TIF_UPROBE_SINGLESTEP)
#define _TIF_SYSCALL_TRACE	BIT(TIF_SYSCALL_TRACE)
#define _TIF_SYSCALL_AUDIT	BIT(TIF_SYSCALL_AUDIT)
#define _TIF_SECCOMP		BIT(TIF_SECCOMP)
#define _TIF_SYSCALL_TRACEPOINT	BIT(TIF_SYSCALL_TRACEPOINT)

#endif /* _ASM_THREAD_INFO_H */
