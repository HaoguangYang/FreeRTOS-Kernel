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
#include "xscutimer.h"

#define XSCUTIMER_CLOCK_HZ      ( XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2UL )
#define XSCUTIMER_LOAD_VALUE    ( XSCUTIMER_CLOCK_HZ / configTICK_RATE_HZ )

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
static XScuTimer xTimer;
XScuGic xInterruptController; /* Interrupt controller instance */

/*
 * Global counter used for calculation of run time statistics of tasks.
 * Defined only when the relevant option is turned on
 */
#if ( configGENERATE_RUN_TIME_STATS == 1 )
    volatile uint32_t ulTaskTicks;
    volatile uint32_t ulHighFrequencyTimerTicks;
#endif
/*-----------------------------------------------------------*/

void FreeRTOS_SetupTickInterrupt( void )
{
    BaseType_t xStatus;
    extern void FreeRTOS_Tick_Handler( void );
    XScuTimer_Config * pxTimerConfig;
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
    XScuGic_SetPriorityTriggerType( &xInterruptController, XPAR_SCUTIMER_INTR,
                                    portLOWEST_USABLE_INTERRUPT_PRIORITY << portPRIORITY_SHIFT,
                                    ucRisingEdge );

    /* Install the FreeRTOS tick handler. */
    xStatus = XScuGic_Connect( &xInterruptController, XPAR_SCUTIMER_INTR,
                               ( Xil_ExceptionHandler ) FreeRTOS_Tick_Handler, ( void * ) &xTimer );
    configASSERT( xStatus == XST_SUCCESS );
    ( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

    /* Initialise the timer. */
    pxTimerConfig = XScuTimer_LookupConfig( XPAR_SCUTIMER_DEVICE_ID );
    xStatus = XScuTimer_CfgInitialize( &xTimer, pxTimerConfig, pxTimerConfig->BaseAddr );
    configASSERT( xStatus == XST_SUCCESS );
    ( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

    /* Enable Auto reload mode. */
    XScuTimer_EnableAutoReload( &xTimer );

    /* Ensure there is no prescale. */
    XScuTimer_SetPrescaler( &xTimer, 0 );

    XScuTimer_LoadTimer( &xTimer, XSCUTIMER_LOAD_VALUE );

    /* Start the timer counter and then wait for it to timeout a number of
     * times. */
    XScuTimer_Start( &xTimer );

    /* Enable the interrupt for the xTimer in the interrupt controller. */
    XScuGic_Enable( &xInterruptController, XPAR_SCUTIMER_INTR );

    /* Enable the interrupt in the xTimer itself. */
    FreeRTOS_ClearTickInterrupt();
    XScuTimer_EnableInterrupt( &xTimer );
}
/*-----------------------------------------------------------*/

void FreeRTOS_ClearTickInterrupt( void )
{
    XScuTimer_ClearInterruptStatus( &xTimer );
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
        ulTaskTicks = 0;
        ulHighFrequencyTimerTicks = 0;
    }

/*
 * For Xilinx implementation this function returns the global counter used for
 * run time task stats calculation.
 * It is called by FreeRTOS kernel task handling logic.
 */
    configRUN_TIME_COUNTER_TYPE xGET_RUN_TIME_COUNTER_VALUE( void )
    {
        /* ulHighFrequencyTimerTicks rolls over every 12 seconds */
        uint32_t taskTicks = xTaskGetTickCount();
        uint32_t timerTicks = XSCUTIMER_LOAD_VALUE - XScuTimer_GetCounterValue( &xTimer );

        /* we entered this function before ISR handled the timer interrupt. */
        if( ( taskTicks == ulTaskTicks ) && ( timerTicks <= ulHighFrequencyTimerTicks ) )
        {
            taskTicks++;
        }

        ulTaskTicks = taskTicks;
        ulHighFrequencyTimerTicks = timerTicks;
        return taskTicks * XSCUTIMER_LOAD_VALUE + timerTicks;
    }
#endif /* if ( configGENERATE_RUN_TIME_STATS == 1 ) */
