/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: MIT AND BSD-3-Clause
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*-----------------------------------------------------------
* Implementation of functions defined in portable.h for the RISC-V port.
*----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

/* Standard includes. */
#include "string.h"


/* Let the user override the pre-loading of the initial RA. */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS    0
#endif

#ifndef configUSE_ISR_STACK
#define configUSE_ISR_STACK 1
#endif

/* The stack used by interrupt service routines.  Set configISR_STACK_SIZE_WORDS
 * to use a statically allocated array as the interrupt stack.  Alternative leave
 * configISR_STACK_SIZE_WORDS undefined and update the linker script so that a
 * linker variable names __freertos_irq_stack_top has the same value as the top
 * of the stack used by main.  Using the linker script method will repurpose the
 * stack that was used by main before the scheduler was started for use as the
 * interrupt stack after the scheduler has started. */
#ifdef configISR_STACK_SIZE_WORDS
#error configISR_STACK_SIZE_WORDS not yet supported; we need per core ones
static __attribute__( ( aligned( 16 ) ) ) StackType_t xISRStack[ configISR_STACK_SIZE_WORDS ] = { 0 };
const StackType_t xISRStackTop = ( StackType_t ) &( xISRStack[ configISR_STACK_SIZE_WORDS & ~portBYTE_ALIGNMENT_MASK ] );

/* Don't use 0xa5 as the stack fill bytes as that is used by the kernel for
 * the task stacks, and so will legitimately appear in many positions within
 * the ISR stack. */
    #define portISR_STACK_FILL_BYTE    0xee
#else
    /*
     * If these are 0, then no stack switch is performed.
     *
     * Note: if configUSE_ISR_STACK == 0, then this is never initialized,
     * but we keep this array around, because portASM.S / portContext.h does not
     * have access to config variables
     */
    StackType_t xISRStackTops[configNUMBER_OF_CORES] = {0};
#endif

/* Note this is an SDK IRQ priority which use the ARM convention */
#define portMIN_INTERRUPT_PRIORITY            ( 255UL )

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void ) __attribute__( ( weak ) );

/*-----------------------------------------------------------*/
volatile uint32_t * timerPendingRegister = NULL;

/* Holds the critical nesting value - deliberately non-zero at start up to
 * ensure interrupts are not accidentally enabled before the scheduler starts. */
typedef struct tskTaskControlBlock TCB_t;
#if configNUMBER_OF_CORES == 1
size_t xCriticalNesting = ( size_t ) 0xaaaaaaaa;
size_t *xCriticalNestingArray = &xCriticalNesting;
extern portDONT_DISCARD PRIVILEGED_DATA TCB_t * volatile pxCurrentTCB;
TCB_t * volatile * pxCurrentTCBArray = &pxCurrentTCB;
static uint32_t zero = 0; /* todo we could use a pre-existing zero elsewhere! */
uint32_t * pxCurrentCoreID = &zero;
#else
size_t xCriticalNestings[ configNUMBER_OF_CORES ] = { 0 };
size_t *xCriticalNestingArray = &xCriticalNestings[0];
extern portDONT_DISCARD PRIVILEGED_DATA TCB_t * volatile pxCurrentTCBs[ configNUMBER_OF_CORES ];
TCB_t * volatile * pxCurrentTCBArray = pxCurrentTCBs;
const volatile uint32_t * pxCurrentCoreID = &sio_hw->cpuid;
#endif

/* Used to catch tasks that attempt to return from their implementing function. */
size_t xTaskReturnAddress = ( size_t ) portTASK_RETURN_ADDRESS;

/* Set configCHECK_FOR_STACK_OVERFLOW to 3 to add ISR stack checking to task
 * stack checking.  A problem in the ISR stack will trigger an assert, not call
 * the stack overflow hook function (because the stack overflow hook is specific
 * to a task stack, not the ISR stack). */
#if defined( configISR_STACK_SIZE_WORDS ) && ( configCHECK_FOR_STACK_OVERFLOW > 2 )
    #warning "This path not tested, or even compiled yet."

    static const uint8_t ucExpectedStackBytes[] =
    {
        portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, \
        portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, \
        portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, \
        portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, \
        portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE
    }; \

    #define portCHECK_ISR_STACK()    configASSERT( ( memcmp( ( void * ) xISRStack, ( void * ) ucExpectedStackBytes, sizeof( ucExpectedStackBytes ) ) == 0 ) )
#else /* if defined( configISR_STACK_SIZE_WORDS ) && ( configCHECK_FOR_STACK_OVERFLOW > 2 ) */
    /* Define the function away. */
    #define portCHECK_ISR_STACK()
#endif /* configCHECK_FOR_STACK_OVERFLOW > 2 */

#define INVALID_PRIMARY_CORE_NUM    0xffu
/* The primary core number (the own which has the SysTick handler) */
static uint8_t ucPrimaryCoreNum = INVALID_PRIMARY_CORE_NUM;
/* Initialize to -1 so that when using INTEROP with SDK and running on one core only, we can tell if this is initialized yet */
static int8_t cDoorbellNum = -1;

/* Note: portIS_FREE_RTOS_CORE() also returns false until the scheduler is started */
#if ( configNUMBER_OF_CORES != 1 )
#define portIS_FREE_RTOS_CORE()    ( ucPrimaryCoreNum != INVALID_PRIMARY_CORE_NUM )
#else
#define portIS_FREE_RTOS_CORE()    ( ucPrimaryCoreNum == 0 )
#endif

/*-----------------------------------------------------------*/

#include "timer_drv.h"
#include "hazard3_irq.h"

#define TIMER0_IRQ_PRIORITY 10

void vPortSetupTimerInterrupt( void )
{
    timerPendingRegister = ( volatile uint32_t * ) &TIMER0->PENDING;

    extern void freertos_risc_v_mtimer_interrupt_handler(void);

    external_irq_enable(true);

    h3irq_enable(TIMER0_IRQ_NUM, true);
    h3irq_set_priority(TIMER0_IRQ_NUM, TIMER0_IRQ_PRIORITY);
    h3irq_set_external_irq_handler(TIMER0_IRQ_NUM, freertos_risc_v_mtimer_interrupt_handler);

    timer_int_enable(TIMER0, true);
    timer_set_mode(TIMER0, TIMER_MODE_RELOAD);
    timer_set_div(TIMER0, 12);
    timer_set_expire_count(TIMER0, 1000000 / configTICK_RATE_HZ);
    timer_enable(TIMER0, true);
}

void vPortInstallVectorTableHandlers(void) {
    extern void freertos_risc_v_trap_handler(void);
    extern void freertos_risc_v_interrupt_handler(void);
    irq_set_riscv_vector_handler(RISCV_VEC_MACHINE_EXCEPTION, freertos_risc_v_trap_handler);
    irq_set_riscv_vector_handler(RISCV_VEC_MACHINE_EXTERNAL_IRQ, freertos_risc_v_interrupt_handler);
}

StackType_t xPortInitISRStack(void) {
    StackType_t xISRStackTop;
    //( StackType_t ) /*__freertos_irq_stack_top*/__StackTop;
    // todo fixup for allocated stacks
    extern const uint32_t _stack_top[];
    xISRStackTop = ( StackType_t ) _stack_top;

    xISRStackTops[0] = xISRStackTop;

    // disable sp overflow exception
    asm volatile("li a0, 0");
    asm volatile("csrw 0xbea, a0");

    return xISRStackTop;
}

/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
    extern void xPortStartFirstTask( void );

    vPortInstallVectorTableHandlers();

    #if ( configUSE_ISR_STACK == 1)
        StackType_t xISRStackTop = xPortInitISRStack();

        #if ( configASSERT_DEFINED == 1 )
        {
            /* Check alignment of the interrupt stack - which is the same as the
             * stack that was being used by main() prior to the scheduler being
             * started. */
            configASSERT( ( xISRStackTop & portBYTE_ALIGNMENT_MASK ) == 0 );

            #ifdef configISR_STACK_SIZE_WORDS
            {
                memset( ( void * ) xISRStack, portISR_STACK_FILL_BYTE, sizeof( xISRStack ) );
            }
            #endif /* configISR_STACK_SIZE_WORDS */
        }
        #endif /* configASSERT_DEFINED */
    #endif /* configUSE_ISR_STACK */

    /* If there is a CLINT then it is ok to use the default implementation
     * in this file, otherwise vPortSetupTimerInterrupt() must be implemented to
     * configure whichever clock is to be used to generate the tick interrupt. */
    vPortSetupTimerInterrupt();

    ucPrimaryCoreNum = 0;

    xPortStartFirstTask();

    /* Should not get here as after calling xPortStartFirstTask() only tasks
     * should be executing. */
    return pdFAIL;
}

/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not implemented. */
    for( ; ; )
    {
    }
}

/*-----------------------------------------------------------*/

/* todo need to move portASM.s back into portasm.c so we can  inline this */
UBaseType_t callTaskEnterCriticalFromISR(void) {
    /** todo note on ARM port we call this anyway for single core, but it seems unnecessary */
    #if ( configNUMBER_OF_CORES > 1 )
        return taskENTER_CRITICAL_FROM_ISR();
    #else
        return 0;
    #endif
}

/* todo need to move portASM.s back into portasm.c so we can  inline this */
void callTaskExitCriticalFromISR(UBaseType_t uxSavedInterruptStatus) {
    /** todo note on ARM port we call this anyway for single core, but it seems unnecessary */
    #if ( configNUMBER_OF_CORES > 1 )
        taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
    #endif
}