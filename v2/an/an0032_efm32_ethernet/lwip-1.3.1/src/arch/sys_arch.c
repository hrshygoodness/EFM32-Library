/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
//#include <intrinsics.h>
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "arch/sys_arch.h"

#if SYS_LIGHTWEIGHT_PROT
/*
 * This function is used to lock access to critical sections when lwipopt.h
 * defines SYS_LIGHTWEIGHT_PROT. It disables interrupts and returns a value
 * indicating the interrupt enable state when the function entered. This
 * value must be passed back on the matching call to sys_arch_unprotect().
 *
 * @return the interrupt level when the function was entered.
 *
 */
sys_prot_t
sys_arch_protect(void)
{
sys_prot_t Ret = __get_interrupt_state();
  __disable_interrupt();
  return(Ret);
}

/*
 * This function is used to unlock access to critical sections when lwipopt.h
 * defines SYS_LIGHTWEIGHT_PROT. It enables interrupts if the value of the lev
 * parameter indicates that they were enabled when the matching call to
 * sys_arch_protect() was made.
 *
 * @param lev is the interrupt level when the matching protect function was
 * called
 *
 */
void
sys_arch_unprotect(sys_prot_t lev)
{
  __set_interrupt_state(lev);
}
#endif /* SYS_LIGHTWEIGHT_PROT */


