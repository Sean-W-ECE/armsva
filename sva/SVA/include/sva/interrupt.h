/*===- interrupt.h - SVA Interrupts   -------------------------------------===
 * 
 *                        Secure Virtual Architecture
 *
 * This file was developed by the LLVM research group and is distributed under
 * the University of Illinois Open Source License. See LICENSE.TXT for details.
 * 
 *===----------------------------------------------------------------------===
 *
 * This header files defines functions and macros used by the SVA Execution
 * Engine for handling interrupts.
 *
 *===----------------------------------------------------------------------===
 */

#ifndef _SVA_INTERRUPT_H
#define _SVA_INTERRUPT_H

#if 0
#include <sva/config.h>
#include <sva/exceptions.h>
#endif
#include <sva/state.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void * sva_getCPUState (tss_t * tssp);

void sva_icontext_setretval (unsigned long, unsigned long, unsigned char error);
void sva_icontext_restart (unsigned long, unsigned long);

/* Types for handlers */
typedef void (*genfault_handler_t)(sva_icontext_t * icontext);
typedef void (*memfault_handler_t)(sva_icontext_t * icontext, void * mp);
typedef void (*interrupt_handler_t)(unsigned int num, sva_icontext_t * icontext);
typedef void * syscall_t;

/* Prototypes for Execution Engine Functions */
extern unsigned char
sva_register_general_exception (unsigned char, genfault_handler_t);

extern unsigned char
sva_register_memory_exception (unsigned char, memfault_handler_t);

extern unsigned char
sva_register_interrupt (unsigned char, interrupt_handler_t);

extern unsigned char
sva_register_syscall (unsigned char, syscall_t);

#if 0
extern void sva_register_old_interrupt (int number, void *interrupt);
extern void sva_register_old_trap      (int number, void *interrupt);
#endif

/**************************** Inline Functions *******************************/

/*
 * DEBUG: copy of set_cpsr for testing only
 */
static __inline uint32_t
sva_set_cpsr(uint32_t bic, uint32_t eor)
{
	uint32_t	tmp, ret;

	__asm __volatile(
		"mrs     %0, cpsr\n"		/* Get the CPSR */
		"bic	 %1, %0, %2\n"		/* Clear bits */
		"eor	 %1, %1, %3\n"		/* XOR bits */
		"msr     cpsr_xc, %1\n"		/* Set the CPSR */
	: "=&r" (ret), "=&r" (tmp)
	: "r" (bic), "r" (eor) : "memory");

	return ret;
}

#define PSR_A 0x00000100
#define PSR_I 0x00000080
#define PSR_F 0x00000040
  
/*
 * Intrinsic: sva_load_lif()
 *
 * Description:
 *  Enables or disables local processor interrupts, depending upon the flag.
 *  Enable is a 9-bit value for ARM port. 3 MSB are A,I,F modify status.
 *  3 LSB are new values for A,I,F.
 *
 * Inputs:
 *  enable - 9b: [A change][I change][F change][3 unused][new A][new I][new F]
 */
static inline uint32_t
sva_load_lif (unsigned int enable)
{
  uint32_t ret, bic, eor, tmp;

  bic = 0;
  eor = 0;
  
  //if [A change] set, read from new A, set bic and eor
  if((enable & PSR_A) >> 8) {
    bic |= PSR_A;
    if((enable & 0x4) >> 2) {
      eor |= PSR_A;
    }
  }
  //if [I change] set, read from new I, set bic and eor
  if((enable & PSR_I) >> 7) {
    bic |= PSR_I;
    if((enable & 0x2) >> 1) {
      eor |= PSR_I;
    }
  }
  //if [F change] set, read from new F, set bic and eor
  if((enable & PSR_F) >> 6) {
    bic |= PSR_F;
    if((enable & 0x1)) {
      eor |= PSR_F;
    }
  }
  //execute assembly
  __asm __volatile(
		"mrs     %0, cpsr\n"		/* Get the CPSR */
		"bic	 %1, %0, %2\n"		/* Clear bits */
		"eor	 %1, %1, %3\n"		/* XOR bits */
		"msr     cpsr_xc, %1\n"		/* Set the CPSR */
	: "=&r" (ret), "=&r" (tmp)
	: "r" (bic), "r" (eor) : "memory");
  
  //return old value of CPSR
  return ret;
}

/*
 * Intrinsic: sva_save_lif()
 *
 * Description:
 *  Return whether interrupts are currently enabled or disabled on the
 *  local processor.
 */
static inline uint32_t
sva_save_lif (void)
{
  uint32_t eflags;

  /*
   * Get the CPSR register and then mask out the IRQ/FIQ disable bits
   */
  __asm__ __volatile__ ("MRS %[reg], %%cpsr" : [reg] "=r" (eflags));
  eflags = eflags & 0x000000C0;
  return (eflags >> 6);
}

#if 0
static inline unsigned int
sva_icontext_lif (void * icontextp)
{
  sva_icontext_t * p = (sva_icontext_t *)icontextp;
  return (p->eflags & 0x00000200);
}
#endif

/*
 * Intrinsic: sva_nop()
 *
 * Description:
 *  Provides a volatile operation that does nothing.  This is useful if you
 *  want to wait for an interrupt but don't want to actually do anything.  In
 *  such a case, you need a "filler" instruction that can be interrupted.
 *
 * TODO:
 *  Currently, we're going to use this as an optimization barrier.  Do not move
 *  loads and stores around this.  This is okay, since LLVM will enforce the
 *  same restriction on the LLVM level.
 */
static inline void
sva_nop (void)
{
  __asm__ __volatile__ ("mov r0, r0" ::: "memory");
}

#ifdef __cplusplus
}
#endif

#endif
