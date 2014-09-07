/*
    FreeRTOS V7.5.2 - Copyright (C) 2013 Real Time Engineers Ltd.

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>! NOTE: The modification to the GPL is included to allow you to distribute
    >>! a combined work that includes FreeRTOS without being obliged to provide
    >>! the source code for proprietary components outside of the FreeRTOS
    >>! kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM0 port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"


#ifndef configSYSTICK_CLOCK_HZ
	#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
#endif
/* Constants required to manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL		( *( ( volatile unsigned long *) 0xe000e010 ))
#define portNVIC_SYSTICK_LOAD		( *( ( volatile unsigned long *) 0xe000e014 ))
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile unsigned long * ) 0xe000e018 ) )
#define portNVIC_INT_CTRL			( *( ( volatile unsigned long *) 0xe000ed04 ))
#define portNVIC_SYSPRI2			( *( ( volatile unsigned long *) 0xe000ed20 ))
#define portNVIC_SYSTICK_CLK		0x00000004
#define portNVIC_SYSTICK_INT		0x00000002
#define portNVIC_SYSTICK_ENABLE		0x00000001
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVSET			0x10000000

#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )
#define portMIN_INTERRUPT_PRIORITY	( 255UL )
#define portNVIC_PENDSV_PRI			( portMIN_INTERRUPT_PRIORITY << 16UL )
#define portNVIC_SYSTICK_PRI		( portMIN_INTERRUPT_PRIORITY << 24UL )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR			( 0x01000000 )

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;

/*
 * Setup the timer to generate the tick interrupts.
 */
 void vPortSetupTimerInterrupt( void );

/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void ) __attribute__ (( naked ));

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void vPortStartFirstTask( void ) __attribute__ (( naked ));

/*-----------------------------------------------------------*/

/*
 * The number of SysTick increments that make up one tick period.
 */
#if configUSE_TICKLESS_IDLE == 1
	static unsigned long ulTimerReloadValueForOneTick = 0;
#endif

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if configUSE_TICKLESS_IDLE == 1
	static unsigned long xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if configUSE_TICKLESS_IDLE == 1
	static unsigned long ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */
	pxTopOfStack--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) pxCode;	/* PC */
	pxTopOfStack -= 6;	/* LR, R12, R3..R1 */
	*pxTopOfStack = ( portSTACK_TYPE ) pvParameters;	/* R0 */
	pxTopOfStack -= 8; /* R11..R4. */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
	__asm volatile (
					"	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */
					"	ldr r1, [r3]					\n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
					"	ldr r0, [r1]					\n" /* The first item in pxCurrentTCB is the task top of stack. */
					"	add r0, r0, #16					\n" /* Move to the high registers. */
					"	ldmia r0!, {r4-r7}				\n" /* Pop the high registers. */
					" 	mov r8, r4						\n"
					" 	mov r9, r5						\n"
					" 	mov r10, r6						\n"
					" 	mov r11, r7						\n"
					"									\n"
					"	msr psp, r0						\n" /* Remember the new top of stack for the task. */
					"									\n"
					"	sub r0, r0, #32					\n" /* Go back for the low registers that are not automatically restored. */
					" 	ldmia r0!, {r4-r7}              \n" /* Pop low registers.  */
					"	mov r1, r14						\n" /* OR R14 with 0x0d. */
					"	movs r0, #0x0d					\n"
					"	orr r1, r0						\n"
					"	bx r1							\n"
					"									\n"
					"	.align 2						\n"
					"pxCurrentTCBConst2: .word pxCurrentTCB	\n"
				);
}
/*-----------------------------------------------------------*/

void vPortStartFirstTask( void )
{
	__asm volatile(
					" movs r0, #0x00 	\n" /* Locate the top of stack. */
					" ldr r0, [r0] 		\n"
					" msr msp, r0		\n" /* Set the msp back to the start of the stack. */
					" cpsie i			\n" /* Globally enable interrupts. */
					" svc 0				\n" /* System call to start first task. */
					" nop				\n"
				);
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portBASE_TYPE xPortStartScheduler( void )
{
	/* Make PendSV, CallSV and SysTick the same priroity as the kernel. */
	(portNVIC_SYSPRI2) |= portNVIC_PENDSV_PRI;
	(portNVIC_SYSPRI2) |= portNVIC_SYSTICK_PRI;

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	vPortSetupTimerInterrupt();

	/* Initialise the critical nesting count ready for the first task. */
	uxCriticalNesting = 0;

	/* Start the first task. */
	vPortStartFirstTask();

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
  /* It is unlikely that the CM0 port will require this function as there
    is nothing to return to.  */
}
/*-----------------------------------------------------------*/

void vPortYield( void )
{
	/* Set a PendSV to request a context switch. */
( portNVIC_INT_CTRL ) = portNVIC_PENDSVSET;

	/* Barriers are normally not required but do ensure the code is completely
	within the specified behaviour for the architecture. */
	__asm volatile( "dsb" );
	__asm volatile( "isb" );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
	__asm volatile( "dsb" );
	__asm volatile( "isb" );
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
    uxCriticalNesting--;
    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/*-----------------------------------------------------------*/

void xPortPendSVHandler( void )
{
	/* This is a naked function. */

	__asm volatile
	(
	"	mrs r0, psp							\n"
	"										\n"
	"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
	"	ldr	r2, [r3]						\n"
	"										\n"
	"	sub r0, r0, #32						\n" /* Make space for the remaining low registers. */
	"	str r0, [r2]						\n" /* Save the new top of stack. */
	"	stmia r0!, {r4-r7}					\n" /* Store the low registers that are not saved automatically. */
	" 	mov r4, r8							\n" /* Store the high registers. */
	" 	mov r5, r9							\n"
	" 	mov r6, r10							\n"
	" 	mov r7, r11							\n"
	" 	stmia r0!, {r4-r7}              	\n"
	"										\n"
	"	push {r3, r14}						\n"
	"	cpsid i								\n"
	"	bl vTaskSwitchContext				\n"
	"	cpsie i								\n"
	"	pop {r2, r3}						\n" /* lr goes in r3. r2 now holds tcb pointer. */
	"										\n"
	"	ldr r1, [r2]						\n"
	"	ldr r0, [r1]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
	"	add r0, r0, #16						\n" /* Move to the high registers. */
	"	ldmia r0!, {r4-r7}					\n" /* Pop the high registers. */
	" 	mov r8, r4							\n"
	" 	mov r9, r5							\n"
	" 	mov r10, r6							\n"
	" 	mov r11, r7							\n"
	"										\n"
	"	msr psp, r0							\n" /* Remember the new top of stack for the task. */
	"										\n"
	"	sub r0, r0, #32						\n" /* Go back for the low registers that are not automatically restored. */
	" 	ldmia r0!, {r4-r7}              	\n" /* Pop low registers.  */
	"										\n"
	"	bx r3								\n"
	"										\n"
	"	.align 2							\n"
	"pxCurrentTCBConst: .word pxCurrentTCB	  "
	);
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
unsigned long ulDummy;

	/* If using preemption, also force a context switch. */
	#if configUSE_PREEMPTION == 1
		(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
	#endif

	/* Only reset the systick load register if configUSE_TICKLESS_IDLE is set to
	1.  If it is set to 0 tickless idle is not being used.  If it is set to a
	value other than 0 or 1 then a timer other than the SysTick is being used
	to generate the tick interrupt. */
	#if configUSE_TICKLESS_IDLE == 1
		portNVIC_SYSTICK_LOAD = ulTimerReloadValueForOneTick;
	#endif
	ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		/* Increment the RTOS tick. */
		if( xTaskIncrementTick() != pdFALSE )
		{
			/* Pend a context switch. */
			(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );
}
/*-----------------------------------------------------------*/
#if configUSE_TICKLESS_IDLE == 1

	__attribute__((weak)) void vPortSuppressTicksAndSleep( portTickType xExpectedIdleTime )
	{
	unsigned long ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickIncrements;
	portTickType xModifiableIdleTime;

		/* Make sure the SysTick reload value does not overflow the counter. */
		if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
		{
			xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
		}

		/* Calculate the reload value required to wait xExpectedIdleTime
		tick periods.  -1 is used because this code will execute part way
		through one of the tick periods, and the fraction of a tick period is
		accounted for later. */
		ulReloadValue = ( ulTimerReloadValueForOneTick * ( xExpectedIdleTime - 1UL ) );
		if( ulReloadValue > ulStoppedTimerCompensation )
		{
			ulReloadValue -= ulStoppedTimerCompensation;
		}

		/* Stop the SysTick momentarily.  The time the SysTick is stopped for
		is accounted for as best it can be, but using the tickless mode will
		inevitably result in some tiny drift of the time maintained by the
		kernel with respect to calendar time. */
		portNVIC_SYSTICK_CTRL = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT;

		/* Adjust the reload value to take into account that the current time
		slice is already partially complete. */
		ulReloadValue += ( portNVIC_SYSTICK_LOAD - ( portNVIC_SYSTICK_LOAD - portNVIC_SYSTICK_CURRENT_VALUE_REG ) );

		/* Enter a critical section but don't use the taskENTER_CRITICAL()
		method as that will mask interrupts that should exit sleep mode. */
		__asm volatile( "cpsid i" );

		/* If a context switch is pending or a task is waiting for the scheduler
		to be unsuspended then abandon the low power entry. */
		if( eTaskConfirmSleepModeStatus() == eAbortSleep )
		{
			/* Restart SysTick. */
			portNVIC_SYSTICK_CTRL = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;

			/* Re-enable interrupts - see comments above the cpsid instruction()
			above. */
			__asm volatile( "cpsie i" );
		}
		else
		{
			/* Set the new reload value. */
			portNVIC_SYSTICK_LOAD = ulReloadValue;

			/* Clear the SysTick count flag and set the count value back to
			zero. */
			portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

			/* Restart SysTick. */
			portNVIC_SYSTICK_CTRL = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;

			/* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
			set its parameter to 0 to indicate that its implementation contains
			its own wait for interrupt or wait for event instruction, and so wfi
			should not be executed again.  However, the original expected idle
			time variable must remain unmodified, so a copy is taken. */
			xModifiableIdleTime = xExpectedIdleTime;
			configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
			if( xModifiableIdleTime > 0 )
			{
				__asm volatile( "wfi" );
				__asm volatile( "dsb" );
				__asm volatile( "isb" );
			}
			configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

			/* Stop SysTick.  Again, the time the SysTick is stopped for is
			accounted for as best it can be, but using the tickless mode will
			inevitably result in some tiny drift of the time maintained by the
			kernel with respect to calendar time. */
			portNVIC_SYSTICK_CTRL = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT;

			/* Re-enable interrupts - see comments above the cpsid instruction()
			above. */
			__asm volatile( "cpsie i" );

			if( ( portNVIC_SYSTICK_CTRL & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
			{
				/* The tick interrupt has already executed, and the SysTick
				count reloaded with the portNVIC_SYSTICK_LOAD value.
				Reset the portNVIC_SYSTICK_LOAD with whatever remains of
				this tick period. */
				portNVIC_SYSTICK_LOAD = ulTimerReloadValueForOneTick - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

				/* The tick interrupt handler will already have pended the tick
				processing in the kernel.  As the pending tick will be
				processed as soon as this function exits, the tick value
				maintained by the tick is stepped forward by one less than the
				time spent waiting. */
				ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
			}
			else
			{
				/* Something other than the tick interrupt ended the sleep.
				Work out how long the sleep lasted. */
				ulCompletedSysTickIncrements = ( xExpectedIdleTime * ulTimerReloadValueForOneTick ) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

				/* How many complete tick periods passed while the processor
				was waiting? */
				ulCompleteTickPeriods = ulCompletedSysTickIncrements / ulTimerReloadValueForOneTick;

				/* The reload value is set to whatever fraction of a single tick
				period remains. */
				portNVIC_SYSTICK_LOAD = ( ( ulCompleteTickPeriods + 1 ) * ulTimerReloadValueForOneTick ) - ulCompletedSysTickIncrements;
			}

			/* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD
			again, then set portNVIC_SYSTICK_LOAD back to its standard
			value. */
			portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
			portNVIC_SYSTICK_CTRL = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;

			vTaskStepTick( ulCompleteTickPeriods );
		}
	}

#endif /* #if configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
__attribute__(( weak )) void vPortSetupTimerInterrupt( void )
{
	/* Calculate the constants required to configure the tick interrupt. */
	#if configUSE_TICKLESS_IDLE == 1
	{
		ulTimerReloadValueForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
		xMaximumPossibleSuppressedTicks = 0xffffffUL / ( ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL );
		ulStoppedTimerCompensation = 45UL / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
	}
	#endif /* configUSE_TICKLESS_IDLE */

	/* Configure SysTick to interrupt at the requested rate. */
	(portNVIC_SYSTICK_LOAD) = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	(portNVIC_SYSTICK_CTRL) = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;
}
/*-----------------------------------------------------------*/

