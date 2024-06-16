/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (C) 2012 - 2020 Xilinx, Inc. All rights reserved.
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
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Xilinx includes. */
#include "xil_types.h"
#include "xscugic.h"

/*#include "xscutimer.h" */
#include "xtime_l.h"

/*#define XSCUTIMER_CLOCK_HZ      ( XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2UL ) */
/*#define XSCUTIMER_LOAD_VALUE    ( XSCUTIMER_CLOCK_HZ / configTICK_RATE_HZ ) */

#define GTIMER_LOAD_VALUE    ( COUNTS_PER_SECOND / configTICK_RATE_HZ )

/*
 * Some FreeRTOSConfig.h settings require the application writer to provide the
 * implementation of a callback function that has a specific name, and a linker
 * error will result if the application does not provide the required function.
 * To avoid the risk of a configuration file setting resulting in a linker error
 * this file provides default implementations of each callback that might be
 * required.  The default implementations are declared as weak symbols to allow
 * the application writer to override the default implementation by providing
 * their own implementation in the application itself.
 */
void vApplicationAssert( const char * pcFileName,
                         uint32_t ulLine ) __attribute__( ( weak ) );
void vApplicationTickHook( void ) __attribute__( ( weak ) );
void vApplicationIdleHook( void ) __attribute__( ( weak ) );
void vApplicationMallocFailedHook( void ) __attribute( ( weak ) );
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char * pcTaskName ) __attribute__( ( weak ) );

/*-----------------------------------------------------------*/

/*
 * Installs pxHandler as the interrupt handler for the peripheral specified by
 * the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have pxHandler assigned as its interrupt
 * handler.  Peripheral IDs are defined in the xparameters.h header file, which
 * is itself part of the BSP project.
 *
 * pxHandler:
 *
 * A pointer to the interrupt handler function itself.  This must be a void
 * function that takes a (void *) parameter.
 *
 * pvCallBackRef:
 *
 * The parameter passed into the handler function.  In many cases this will not
 * be used and can be NULL.  Some times it is used to pass in a reference to
 * the peripheral instance variable, so it can be accessed from inside the
 * handler function.
 *
 * pdPASS is returned if the function executes successfully.  Any other value
 * being returned indicates that the function did not execute correctly.
 */
BaseType_t xPortInstallInterruptHandler( uint8_t ucInterruptID,
                                         XInterruptHandler pxHandler,
                                         void * pvCallBackRef );

/*
 * Enables the interrupt, within the interrupt controller, for the peripheral
 * specified by the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have its interrupt enabled in the
 * interrupt controller.  Peripheral IDs are defined in the xparameters.h header
 * file, which is itself part of the BSP project.
 */
void vPortEnableInterrupt( uint8_t ucInterruptID );

/*
 * Disables the interrupt, within the interrupt controller, for the peripheral
 * specified by the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have its interrupt disabled in the
 * interrupt controller.  Peripheral IDs are defined in the xparameters.h header
 * file, which is itself part of the BSP project.
 */
void vPortDisableInterrupt( uint8_t ucInterruptID );
/*-----------------------------------------------------------*/

/* Timer used to generate the tick interrupt. */
/* static XScuTimer xTimer; */

XScuGic xInterruptController; /* Interrupt controller instance */

/*
 * Global counter used for calculation of run time statistics of tasks.
 * Defined only when the relevant option is turned on
 */
/* volatile uint32_t ulTaskTicks = 0; */
/* volatile uint32_t ulHighFrequencyTimerTicks = 0; */
volatile uint64_t uxHighResTicks = 0;
/*-----------------------------------------------------------*/

#define GTIMER_CONTROL_INTERRUPT_ENABLE_MASK    0x00000004U;
#define GTIMER_ISR_OFFSET                       0x0CU
#define GTIMER_ISR_EVENT_FLAG_MASK              0x00000001U
#define GTIMER_COMP_LOWER_OFFSET                0X10U
#define GTIMER_COMP_UPPER_OFFSET                0X14U
#define GTIMER_COMP_AUTO_INCR_VAL               0x18U

typedef struct xSubtickIntrData SubtickIntrData_t;

struct xSubtickIntrData
{
    TaskHandle_t xTask;
    configRUN_TIME_COUNTER_TYPE xNextTriggerTime;
    SubtickIntrData_t * pxNext;
};

static SubtickIntrData_t * pxPendingSubtickInterrupts = NULL;

static configRUN_TIME_COUNTER_TYPE uxUSleepOverhead = 0;

void prvGlobalTimerDisable( void )
{
    /* Disable Global Timer */
    Xil_Out32( ( u32 ) GLOBAL_TMR_BASEADDR + ( u32 ) GTIMER_CONTROL_OFFSET, ( u32 ) 0x0 );
}

void prvGlobalTimerSetComp( uint64_t comp )
{
    /* Updating Global Timer Counter Register */
    Xil_Out32( ( u32 ) GLOBAL_TMR_BASEADDR + ( u32 ) GTIMER_COMP_UPPER_OFFSET, ( u32 ) ( comp >> 32U ) );
    Xil_Out32( ( u32 ) GLOBAL_TMR_BASEADDR + ( u32 ) GTIMER_COMP_LOWER_OFFSET, ( u32 ) comp );
}

uint64_t prvGlobalTimerGetComp( void )
{
    /* Get Global Timer Counter Register */
    uint32_t high = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COMP_UPPER_OFFSET );
    uint32_t low = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COMP_LOWER_OFFSET );

    return ( ( uint64_t ) high << 32 ) | low;
}

void prvGlobalTimerEnable( void )
{
    /* assume timer is stopped. copy timer value + auto incr to the comparator */
    uint32_t high = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COUNTER_UPPER_OFFSET );
    uint32_t low = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COUNTER_LOWER_OFFSET );
    uint32_t incr = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COMP_AUTO_INCR_VAL );
    uint64_t tStart = ( ( ( uint64_t ) high << 32U ) | low );
    uint64_t next_comp = tStart + incr;

    prvGlobalTimerSetComp( next_comp );
    /* Enable Global Timer: prescaler 0, auto-incr, intr disable, comp enable, global enable */
    Xil_Out32( ( u32 ) GLOBAL_TMR_BASEADDR + ( u32 ) GTIMER_CONTROL_OFFSET, ( u32 ) 0xB );
    /*printf("[%llu]\tGTimer enabled at tick %llu. Next interrupt trigger at %llu.\r\n", xGET_RUN_TIME_COUNTER_VALUE(), tStart, prvGlobalTimerGetComp()); */
}

void prvGlobalTimerSetAutoIncr( uint32_t interval )
{
    /*printf("GTimer auto increment register set to %lu.\r\n", interval); */
    Xil_Out32( ( u32 ) GLOBAL_TMR_BASEADDR + ( u32 ) GTIMER_COMP_AUTO_INCR_VAL, interval );
}

void prvGlobalTimerEnableInterrupt( void )
{
    uint32_t reg = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_CONTROL_OFFSET );

    reg |= GTIMER_CONTROL_INTERRUPT_ENABLE_MASK;
    Xil_Out32( ( u32 ) GLOBAL_TMR_BASEADDR + ( u32 ) GTIMER_CONTROL_OFFSET, reg );
    /*printf("[%llu]\tGTimer control register value: %lu.\r\n", xGET_RUN_TIME_COUNTER_VALUE(), reg); */
}

void prvRegisterSubtickSleep( SubtickIntrData_t * node,
                              TaskHandle_t task,
                              configRUN_TIME_COUNTER_TYPE uxWakeupTime )
{
    node->xTask = task;
    /* FIXME: disable auto-increment? */
    uint64_t uxGTimerCompVal = prvGlobalTimerGetComp();

    if( uxGTimerCompVal >= uxWakeupTime )
    {
        /* first entry */
        taskENTER_CRITICAL();
        node->xNextTriggerTime = uxGTimerCompVal;
        prvGlobalTimerSetComp( uxWakeupTime );
        node->pxNext = pxPendingSubtickInterrupts;
        pxPendingSubtickInterrupts = node;
        taskEXIT_CRITICAL();
    }
    else
    {
        configASSERT( pxPendingSubtickInterrupts != NULL );
        /* insert in the middle */
        taskENTER_CRITICAL();
        SubtickIntrData_t * insertionPoint = pxPendingSubtickInterrupts;

        while( insertionPoint->pxNext != NULL )
        {
            if( insertionPoint->xNextTriggerTime >= uxWakeupTime )
            {
                break;
            }

            insertionPoint = insertionPoint->pxNext;
        }

        node->pxNext = insertionPoint->pxNext;
        insertionPoint->xNextTriggerTime = uxWakeupTime;
        insertionPoint->pxNext = node;
        taskEXIT_CRITICAL();
    }
}

void FreeRTOS_Tick_And_SubtickSleep_Handler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /*taskENTER_CRITICAL_FROM_ISR(); */
    __asm volatile ( "CPSID i   \n"
                     "DSB       \n"
                     "ISB       \n" ::: "memory" );

    portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
    __asm volatile ( "DSB       \n"
                     "ISB       \n"
                     "CPSIE i   \n"
                     "DSB       \n"
                     "ISB       \n" ::: "memory" );

    /* end of taskENTER_CRITICAL_FROM_ISR */
    if( pxPendingSubtickInterrupts == NULL )
    {
        /* Increment the RTOS tick. */
        xHigherPriorityTaskWoken |= xTaskIncrementTick();
    }
    else
    {
        while( pdTRUE )
        {
            /* NOTE: this wakes up the scheduler prvSchedulerFunction() at signal id 0 */
            BaseType_t perSleepingTask;
            vTaskNotifyGiveIndexedFromISR( pxPendingSubtickInterrupts->xTask, 0, &perSleepingTask );
            xHigherPriorityTaskWoken |= perSleepingTask;
            /* load comparator to the next value in the queue */
            configRUN_TIME_COUNTER_TYPE nextTriggerTime = pxPendingSubtickInterrupts->xNextTriggerTime;
            pxPendingSubtickInterrupts = pxPendingSubtickInterrupts->pxNext;

            if( xGET_RUN_TIME_COUNTER_VALUE() < nextTriggerTime )
            {
                /* we are still a while away from the next wake up event. */
                prvGlobalTimerSetComp( nextTriggerTime );
                break;
                /* else, we are missing the next wake up event. Proceed with the while loop to release next node */
            }
            else if( pxPendingSubtickInterrupts == NULL )
            {
                /* we are waking up the scheduler */
                /* Increment the RTOS tick. */
                xHigherPriorityTaskWoken |= xTaskIncrementTick();
                break;
            }
        }
    }

    /* yield all tasks with higher priority */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    /*taskEXIT_CRITICAL_FROM_ISR( pdFALSE ); */
    __asm volatile ( "CPSID i   \n"
                     "DSB       \n"
                     "ISB       \n" ::: "memory" );

    portICCPMR_PRIORITY_MASK_REGISTER = ( 0xFFUL );
    __asm volatile ( "DSB       \n"
                     "ISB       \n"
                     "CPSIE i   \n"
                     "DSB       \n"
                     "ISB       \n" ::: "memory" );

    /* end of taskEXIT_CRITICAL_FROM_ISR */
    configCLEAR_TICK_INTERRUPT();
    return;
}

void FreeRTOS_SetupTickInterrupt( void )
{
    BaseType_t xStatus;
    XScuGic_Config * pxGICConfig;
    const uint8_t ucRisingEdge = 3;

    /* This function is called with the IRQ interrupt disabled, and the IRQ
     * interrupt should be left disabled.  It is enabled automatically when the
     * scheduler is started. */

    /* Ensure XScuGic_CfgInitialize() has been called.  In this demo it has
     * already been called from prvSetupHardware() in main(). */
    pxGICConfig = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );
    xStatus = XScuGic_CfgInitialize( &xInterruptController, pxGICConfig, pxGICConfig->CpuBaseAddress );

    configASSERT( xStatus == XST_SUCCESS );
    ( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

    /* The priority must be the lowest possible. */
    XScuGic_SetPriorityTriggerType( &xInterruptController, XPAR_GLOBAL_TMR_INTR, /*XPAR_SCUTIMER_INTR, */
                                    portLOWEST_USABLE_INTERRUPT_PRIORITY << portPRIORITY_SHIFT,
                                    ucRisingEdge );

    /* Install the FreeRTOS tick handler. */
    /* sub-tick sleep should intercept before FreeRTOS_Tick_Handler */

    /*
     * extern void FreeRTOS_Tick_Handler (void);
     * xStatus = XScuGic_Connect( &xInterruptController, XPAR_SCUTIMER_INTR,
     *                         ( Xil_ExceptionHandler ) FreeRTOS_TickHandler, NULL );
     */
    xStatus = XScuGic_Connect( &xInterruptController, XPAR_GLOBAL_TMR_INTR,
                               ( Xil_ExceptionHandler ) FreeRTOS_Tick_And_SubtickSleep_Handler, NULL );
    configASSERT( xStatus == XST_SUCCESS );
    ( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

    /*
     * XScuTimer_Config * pxTimerConfig;
     * // Initialize the timer.
     * pxTimerConfig = XScuTimer_LookupConfig( XPAR_SCUTIMER_DEVICE_ID );
     * xStatus = XScuTimer_CfgInitialize( &xTimer, pxTimerConfig, pxTimerConfig->BaseAddr );
     * configASSERT( xStatus == XST_SUCCESS );
     * // Remove compiler warning if configASSERT() is not defined.
     * ( void ) xStatus;
     * // Enable Auto reload mode.
     * XScuTimer_EnableAutoReload( &xTimer );
     * // Ensure there is no prescale.
     * XScuTimer_SetPrescaler( &xTimer, 0 );
     * XScuTimer_LoadTimer( &xTimer, XSCUTIMER_LOAD_VALUE );
     * // Start the timer counter and then wait for it to timeout a number of times.
     * XScuTimer_Start( &xTimer );
     */
    prvGlobalTimerDisable();
    prvGlobalTimerSetAutoIncr( GTIMER_LOAD_VALUE );
    prvGlobalTimerEnable();

    /* Enable the interrupt for the xTimer in the interrupt controller. */
    /*XScuGic_Enable( &xInterruptController, XPAR_SCUTIMER_INTR ); */
    XScuGic_Enable( &xInterruptController, XPAR_GLOBAL_TMR_INTR );

    /* Enable the interrupt in the xTimer itself. */
    FreeRTOS_ClearTickInterrupt();

    /*XScuTimer_EnableInterrupt( &xTimer ); */
    prvGlobalTimerEnableInterrupt();
}
/*-----------------------------------------------------------*/

void FreeRTOS_ClearTickInterrupt( void )
{
    /*XScuTimer_ClearInterruptStatus( &xTimer ); */
    Xil_Out32( ( u32 ) GLOBAL_TMR_BASEADDR + ( u32 ) GTIMER_ISR_OFFSET, ( u32 ) GTIMER_ISR_EVENT_FLAG_MASK );
}
/*-----------------------------------------------------------*/

void vApplicationIRQHandler( uint32_t ulICCIAR )
{
    extern const XScuGic_Config XScuGic_ConfigTable[];
    static const XScuGic_VectorTableEntry * pxVectorTable =
        XScuGic_ConfigTable[ XPAR_SCUGIC_SINGLE_DEVICE_ID ].HandlerTable;
    uint32_t ulInterruptID;
    const XScuGic_VectorTableEntry * pxVectorEntry;

    /* The ID of the interrupt is obtained by bitwise anding the ICCIAR value
     * with 0x3FF. */
    ulInterruptID = ulICCIAR & 0x3FFUL;

    if( ulInterruptID < XSCUGIC_MAX_NUM_INTR_INPUTS )
    {
        /* Call the function installed in the array of installed handler functions. */
        pxVectorEntry = &( pxVectorTable[ ulInterruptID ] );
        pxVectorEntry->Handler( pxVectorEntry->CallBackRef );
    }
}
/*-----------------------------------------------------------*/

/* This version of vApplicationAssert() is declared as a weak symbol to allow it
 * to be overridden by a version implemented within the application that is using
 * this BSP. */
void vApplicationAssert( const char * pcFileName,
                         uint32_t ulLine )
{
    volatile uint32_t ul = 0;
    volatile const char * pcLocalFileName =
        pcFileName;                         /* To prevent pcFileName being optimized away. */
    volatile uint32_t ulLocalLine = ulLine; /* To prevent ulLine being optimized away. */

    /* Prevent compile warnings about the following two variables being set but
     * not referenced.  They are intended for viewing in the debugger. */

    ( void ) pcLocalFileName;
    ( void ) ulLocalLine;

    xil_printf( "Assert failed in file %s, line %lu\r\n", pcLocalFileName, ulLocalLine );

    /* If this function is entered then a call to configASSERT() failed in the
     * FreeRTOS code because of a fatal error.  The pcFileName and ulLine
     * parameters hold the file name and line number in that file of the assert
     * that failed.  Additionally, if using the debugger, the function call stack
     * can be viewed to find which line failed its configASSERT() test.  Finally,
     * the debugger can be used to set ul to a non-zero value, then step out of
     * this function to find where the assert function was entered. */
    taskENTER_CRITICAL();
    {
        while( ul == 0 )
        {
            __asm volatile ( "NOP" );
        }
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/* This default tick hook does nothing and is declared as a weak symbol to allow
 * the application writer to override this default by providing their own
 * implementation in the application code. */
void vApplicationTickHook( void )
{
}
/*-----------------------------------------------------------*/

/* This default idle hook does nothing and is declared as a weak symbol to allow
 * the application writer to override this default by providing their own
 * implementation in the application code. */
void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

/* This default malloc failed hook does nothing and is declared as a weak symbol
 * to allow the application writer to override this default by providing their own
 * implementation in the application code. */
void vApplicationMallocFailedHook( void )
{
    xil_printf( "vApplicationMallocFailedHook() called\n" );
}
/*-----------------------------------------------------------*/

/* This default stack overflow hook will stop the application for executing.  It
 * is declared as a weak symbol to allow the application writer to override this
 * default by providing their own implementation in the application code. */
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char * pcTaskName )
{
    /* Attempt to prevent the handle and name of the task that overflowed its stack
     * from being optimised away because they are not used. */
    volatile TaskHandle_t xOverflowingTaskHandle = xTask;
    volatile char * pcOverflowingTaskName = pcTaskName;

    ( void ) xOverflowingTaskHandle;
    ( void ) pcOverflowingTaskName;

    xil_printf( "HALT: Task %s overflowed its stack.", pcOverflowingTaskName );
    portDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

/*
 * Initialise the interrupt controller instance.
 */
static int32_t prvInitialiseInterruptController( void )
{
    BaseType_t xStatus;
    XScuGic_Config * pxGICConfig;

    /* Initialize the interrupt controller driver. */
    pxGICConfig = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );
    xStatus = XScuGic_CfgInitialize( &xInterruptController, pxGICConfig, pxGICConfig->CpuBaseAddress );

    if( xStatus == XST_SUCCESS )
    {
        xStatus = pdPASS;
    }
    else
    {
        xStatus = pdFAIL;
    }

    configASSERT( xStatus == pdPASS );

    return xStatus;
}
/*-----------------------------------------------------------*/

/* Ensure the interrupt controller instance variable is initialised before it is
 * used, and that the initialisation only happens once.
 */
static int32_t prvEnsureInterruptControllerIsInitialised( void )
{
    static int32_t lInterruptControllerInitialised = pdFALSE;
    int32_t lReturn;

    /* Ensure the interrupt controller instance variable is initialised before
     * it is used, and that the initialisation only happens once. */
    if( lInterruptControllerInitialised != pdTRUE )
    {
        lReturn = prvInitialiseInterruptController();

        if( lReturn == pdPASS )
        {
            lInterruptControllerInitialised = pdTRUE;
        }
    }
    else
    {
        lReturn = pdPASS;
    }

    return lReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xPortInstallInterruptHandler( uint8_t ucInterruptID,
                                         XInterruptHandler pxHandler,
                                         void * pvCallBackRef )
{
    int32_t lReturn;

    /* An API function is provided to install an interrupt handler */
    lReturn = prvEnsureInterruptControllerIsInitialised();

    if( lReturn == pdPASS )
    {
        lReturn = XScuGic_Connect( &xInterruptController, ucInterruptID, pxHandler, pvCallBackRef );
    }

    if( lReturn == XST_SUCCESS )
    {
        lReturn = pdPASS;
    }

    configASSERT( lReturn == pdPASS );

    return lReturn;
}
/*-----------------------------------------------------------*/

void vPortEnableInterrupt( uint8_t ucInterruptID )
{
    int32_t lReturn;

    /* An API function is provided to enable an interrupt in the interrupt
     * controller. */
    lReturn = prvEnsureInterruptControllerIsInitialised();

    if( lReturn == pdPASS )
    {
        XScuGic_Enable( &xInterruptController, ucInterruptID );
    }

    configASSERT( lReturn );
}
/*-----------------------------------------------------------*/

void vPortDisableInterrupt( uint8_t ucInterruptID )
{
    int32_t lReturn;

    /* An API function is provided to disable an interrupt in the interrupt
     * controller. */
    lReturn = prvEnsureInterruptControllerIsInitialised();

    if( lReturn == pdPASS )
    {
        XScuGic_Disable( &xInterruptController, ucInterruptID );
    }

    configASSERT( lReturn );
}

#if ( configGENERATE_RUN_TIME_STATS == 1 )

/*
 * For Xilinx implementation this is a dummy function that does a redundant operation
 * of zeroing out the global counter.
 * It is called by FreeRTOS kernel.
 */
    void vCONFIGURE_TIMER_FOR_RUN_TIME_STATS( void )
    {
        /* ulTaskTicks = 0; */
        /* ulHighFrequencyTimerTicks = 0; */
    }
#endif /* if ( configGENERATE_RUN_TIME_STATS == 1 ) */

/*
 * For Xilinx implementation this function returns the global counter used for
 * run time task stats calculation.
 * It is called by FreeRTOS kernel task handling logic.
 */
configRUN_TIME_COUNTER_TYPE xGET_RUN_TIME_COUNTER_VALUE( void )
{
    /*
     * // ulHighFrequencyTimerTicks rolls over every 12 seconds
     * uint32_t taskTicks = xTaskGetTickCount();
     * uint32_t timerTicks = XSCUTIMER_LOAD_VALUE - XScuTimer_GetCounterValue( &xTimer );
     * // we entered this function before ISR handled the timer interrupt.
     * if( ( taskTicks == ulTaskTicks ) && ( timerTicks <= ulHighFrequencyTimerTicks ) )
     * {
     *  taskTicks++;
     * }
     * ulTaskTicks = taskTicks;
     * ulHighFrequencyTimerTicks = timerTicks;
     * uxHighResTicks = (uint64_t)taskTicks * XSCUTIMER_LOAD_VALUE + timerTicks;
     */
    uint32_t high = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COUNTER_UPPER_OFFSET );
    uint32_t low = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COUNTER_LOWER_OFFSET );
    uint32_t high2 = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COUNTER_UPPER_OFFSET );

    if( high2 != high ) /* lower 32 bits have rolled over to upper 32 bits */
    {
        low = Xil_In32( GLOBAL_TMR_BASEADDR + GTIMER_COUNTER_LOWER_OFFSET );
    }

    uxHighResTicks = ( ( ( uint64_t ) high2 << 32 ) | low );
    return ( configRUN_TIME_COUNTER_TYPE ) uxHighResTicks;
}

configRUN_TIME_COUNTER_TYPE xGET_NEXT_TICK_TIME_COUNTER_VALUE( void )
{
    configRUN_TIME_COUNTER_TYPE ret;

    if( pxPendingSubtickInterrupts == NULL )
    {
        ret = prvGlobalTimerGetComp();
    }
    else
    {
        SubtickIntrData_t * pxNode = pxPendingSubtickInterrupts;

        while( pxNode->pxNext != NULL )
        {
            pxNode = pxNode->pxNext;
        }

        ret = pxNode->xNextTriggerTime;
    }

    return ret;
}

void vPortUSleep( uint64_t uxMicrosec )
{
    /* first, determine how many integer ticks we need to sleep. If larger than zero, use vTaskDelay */
    const configRUN_TIME_COUNTER_TYPE sleepStart = xGET_RUN_TIME_COUNTER_VALUE();
    const configRUN_TIME_COUNTER_TYPE sleepEnd = sleepStart + uxMicrosec * COUNTS_PER_SECOND / 1000000UL;
    configRUN_TIME_COUNTER_TYPE nextTick = xGET_NEXT_TICK_TIME_COUNTER_VALUE();

    if( sleepEnd - sleepStart <= uxUSleepOverhead )
    {
        while( sleepEnd > xGET_RUN_TIME_COUNTER_VALUE() ) /* poll timer until time has elapsed */
        {
        }

        return;
    }
    else if( sleepEnd > nextTick )
    {
        const TickType_t sleepTicks = uxMicrosec * ( uint64_t ) configTICK_RATE_HZ / 1000000U;

        /* full ticks */
        if( sleepTicks > 0 )
        {
            vTaskDelay( sleepTicks );
            nextTick = xGET_NEXT_TICK_TIME_COUNTER_VALUE();
        }

        /* last full tick to bring end of sleep within this tick cycle */
        while( sleepEnd > nextTick )
        {
            vTaskDelay( 1 );
            nextTick += COUNTS_PER_SECOND / configTICK_RATE_HZ;
        }

        /* already exceeded */
        if( sleepEnd <= xGET_RUN_TIME_COUNTER_VALUE() )
        {
            return;
        }
    }

    /* sub-tick sleep */
    SubtickIntrData_t node;

    prvRegisterSubtickSleep( &node, xTaskGetCurrentTaskHandle(), sleepEnd - uxUSleepOverhead );
    ulTaskNotifyTakeIndexed( 0, pdTRUE, portMAX_DELAY );
    /* subtick interrupt data node gets automatically destroyed upon exiting sleep */
}

void vPortUSleepCalibrate( void )
{
    printf( "[%llu]\tvPortUSleep(us) function calibration enter...\r\n", xGET_RUN_TIME_COUNTER_VALUE() );
    const int repeat = 100;
    const uint64_t testDelay = 1000;
    uint64_t subtickTiming = 0;

    for( int n = 0; n < repeat; n++ )
    {
        configRUN_TIME_COUNTER_TYPE tStart = xGET_RUN_TIME_COUNTER_VALUE();
        vPortUSleep( testDelay );
        configRUN_TIME_COUNTER_TYPE tEnd = xGET_RUN_TIME_COUNTER_VALUE();
        subtickTiming += tEnd - tStart - testDelay * COUNTS_PER_SECOND / 1000000UL;
    }

    uxUSleepOverhead = subtickTiming / repeat;
    printf( "[%llu]\tvPortUSleep(us) function calibrated. Overhead = %lu ticks.\r\n", xGET_RUN_TIME_COUNTER_VALUE(), uxUSleepOverhead );
}
