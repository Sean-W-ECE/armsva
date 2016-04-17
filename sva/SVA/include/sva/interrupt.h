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
 * Intrinsic: sva_load_lif()
 *
 * Description:
 *  Enables or disables local processor interrupts, depending upon the flag.
 *  Enable is a 2-bit value for ARM port. Value is bit-wise NOT of I and F
 *  in CPSR
 *
 * Inputs:
 *  0  - Disable ALL local processor interrupts (CPSR.IF = 11)
 *  1  - Disable ONLY IRQ on ARM (CPSR.IF = 10)
 *  2  - Disable ONLY FIQ on ARM (CPSR.IF = 01)
 *  3  - Enable ALL local processor interrupts (CPSR.IF = 00)
 */
static inline uint32_t
sva_load_lif (unsigned int enable)
{
  uint32_t ret;
  //save the current program status register (would be eflags on x86)
  __asm__ __volatile__ ("MRS %0, cpsr\n" : "=r" (ret) : : "memory");
  
  //enable/disable interrupts (IRQ and FIQ)
  if (enable == 3)
    __asm__ __volatile__ ("CPSIE if\n" : : : "memory");
  else if (enable == 2) //enable IRQ, disable FIQ
    __asm__ __volatile__ ("CPSIE i\n"
			  "CPSID f\n" : : : "memory");
  else if (enable == 1) //enable FIQ, disable IRQ
    __asm__ __volatile__ ("CPSIE f\n"
			  "CPSID i\n" : : : "memory");
  else
    __asm__ __volatile__ ("CPSID if\n" : : : "memory");

  //return old value of CPSR
  return ret;
}

/*
 * Intrinsic: sva_set_async_abort()
 * 
 * Description:
 *   ARM-specific function to set the async abort bit in the CPSR
 *   Handles scenario where enable_interrupts called with A bit
 * 
 * Inputs:
 * 0 - Async Abort disabled (CPSR.A = 1)
 * 1 - Async Abort enabled (CPSR.A = 0)
 */
static inline void
sva_set_async_abort (unsigned int enable)
{
  //set the A flag
  if(enable)
    __asm__ __volatile__ ("CPSIE a\n" : : : "memory");
  else
    __asm__ __volatile__ ("CPSID a\n" : : : "memory");
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
