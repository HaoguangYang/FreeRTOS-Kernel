/*
 *  ESFree V1.0 - Copyright (C) 2016 Robin Kase
 *  All rights reserved
 *
 *  This file is part of ESFree.
 *
 *  ESFree is free software; you can redistribute it and/or modify it under
 *  the terms of the GNU General Public licence (version 2) as published by the
 *  Free Software Foundation AND MODIFIED BY one exception.
 *
 ***************************************************************************
 *  >>!   NOTE: The modification to the GPL is included to allow you to     !<<
 *  >>!   distribute a combined work that includes ESFree without being     !<<
 *  >>!   obliged to provide the source code for proprietary components     !<<
 *  >>!   outside of ESFree.                                                !<<
 ***************************************************************************
 *
 *  ESFree is distributed in the hope that it will be useful, but WITHOUT ANY
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 *  FOR A PARTICULAR PURPOSE. Full license text can be found on license.txt.
 */

#include "scheduler.h"
#include <stdio.h>
#include <string.h>

#if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
    #include "list.h"
#endif /* schedSCHEDULING_POLICY_EDF */

#define schedTHREAD_LOCAL_STORAGE_POINTER_INDEX    0

#if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
    #define schedUSE_TCB_SORTED_LIST               1
    #if ( schedEDF_EFFICIENT == 1 )
        #define schedPRIORITY_RUNNING              ( schedSCHEDULER_PRIORITY - 1 )
        #define schedPRIORITY_NOT_RUNNING          tskIDLE_PRIORITY
    #endif /* schedEDF_EFFICIENT */
#else
    #define schedUSE_TCB_SORTED_LIST               1
#endif /* schedSCHEDULING_POLICY_EDF */

/* This typedef extends eTaskState in task.h */
typedef enum
{
    eTaskRunning = 0, /* A task is querying the state of itself, so must be running. */
    eTaskReady,       /* The task being queried is in a ready or pending ready list. */
    eTaskBlocked,     /* The task being queried is in the Blocked state -- for a periodic task this means it has finished a cycle. */
    eTaskSuspended,   /* The task being queried is in the Suspended state, or is in the Blocked state with an infinite time out. */
    eTaskDeleted,     /* The task being queried has been deleted, but its TCB has not yet been freed. */
    eTaskInvalid,     /* Used as an 'invalid state' value. */
    eTaskDeferred,
    eTaskWCETExceeded,
    eTaskDeadlineMissed
} eSchedPeriodicTaskState;

/* Extended Task control block for managing periodic tasks within this library. */
typedef struct xExtended_TCB
{
    TaskFunction_t pvTaskCode; /* Function pointer to the code that will be run periodically. */
    #if ( schedENABLE_SELF_EVALUATION == 1 )
        TaskFunction_t pvEvaluatorCode;
    #endif
    #if ( schedENABLE_SELF_ESTIMATION == 1 )
        TaskFunction_t pvEstimatorCode;
    #endif
    const char * pcName;          /* Name of the task. */
    UBaseType_t uxStackDepth;     /* Stack size of the task. */
    void * pvParameters;          /* Parameters to the task function. */
    UBaseType_t uxPriority;       /* Priority of the task. */
    TaskHandle_t * pxTaskHandle;  /* Task handle for the task. */
    TickType_t xReleaseTime;      /* Release time of the task. */
    TickType_t xRelativeDeadline; /* Relative deadline of the task. */
    TickType_t xAbsoluteDeadline; /* Absolute deadline of the task. */
    TickType_t xPeriod;           /* Task period. */
    TickType_t xLastWakeTime;     /* Last time stamp when the task was running. */
    TickType_t xMaxExecTime;      /* Worst-case execution time of the task. */

    /* NOTE: xExecTime is in timer ticks, used to detect deadline misses.
     * It may not have the highest resolution.
     * For max timing resolution, use ulTaskGetRunTimeCounter(TaskHandle_t) */
    TickType_t xExecTime; /* Current execution time of the task. */

    #if ( ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS ||   \
            schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS ) && \
    schedUSE_TCB_ARRAY )
        BaseType_t xPriorityIsSet; /* pdTRUE if the priority is assigned. */
    #endif
    #if ( schedUSE_TCB_ARRAY == 1 )
        BaseType_t xInUse; /* pdFALSE if this extended TCB is empty. */
    #endif
    #if ( schedUSE_TCB_SORTED_LIST == 1 )
        ListItem_t xTCBListItem;        /* Used to reference TCB from the TCB list. */
        #if ( schedEDF_EFFICIENT == 1 )
            ListItem_t xTCBAllListItem; /* Extra list item for xTCBAllList. */
        #endif /* schedEDF_EFFICIENT */
    #endif /* schedUSE_TCB_SORTED_LIST */

    #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
        UBaseType_t uxExecutedCycles; /* >0 if the task has executed once. */
        UBaseType_t uxMissed;
    #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */

    #if ( schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 || schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
    #endif /* schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME || schedUSE_TIMING_ERROR_DETECTION_DEADLINE */

    volatile eSchedPeriodicTaskState eState;
    #if ( schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 )
        /* The task will be unblocked at this time if it is blocked by the scheduler task. */
        TickType_t xAbsoluteUnblockTime;
    #endif /* schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME */

    #if ( schedUSE_POLLING_SERVER == 1 )
        BaseType_t xIsPeriodicServer; /* pdTRUE if the task is a polling server. */
    #endif /* schedUSE_POLLING_SERVER */
} SchedTCB_t;

#if ( schedUSE_APERIODIC_JOBS == 1 )
    /* Control block for managing Aperiodic jobs. */
    typedef struct xAperiodicJobControlBlock
    {
        TaskFunction_t pvTaskCode; /* Function pointer to the code of the aperiodic job. */
        const char * pcName;       /* Name of the aperiodic job. */
        void * pvParameters;       /* Parameters to the job function. */
        TickType_t xMaxExecTime;   /* Worst-case execution time of the aperiodic job. */
        TickType_t xExecTime;      /* Current execution time of the aperiodic job. */
    } AJCB_t;
#endif /* schedUSE_APERIODIC_JOBS */

#if ( schedUSE_SPORADIC_JOBS == 1 )
    /* Control block for managing sporadic jobs. */
    typedef struct xSporadicJobControlBlock
    {
        TaskFunction_t pvTaskCode;    /* Function pointer to the code of the sporadic job. */
        const char * pcName;          /* Name of the job. */
        void * pvParameters;          /* Pararmeters to the job function. */
        TickType_t xRelativeDeadline; /* Relative deadline of the sporadic job. */
        TickType_t xMaxExecTime;      /* Worst-case execution time of the sporadic job. */
        TickType_t xExecTime;         /* Current execution time of the sporadic job. */

        #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
            TickType_t xAbsoluteDeadline; /* Absolute deadline of the sporadic job. */
        #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */
    } SJCB_t;
#endif /* schedUSE_SPORADIC_JOBS */

#if ( schedUSE_TCB_ARRAY == 1 )
    static BaseType_t prvGetTCBIndexFromHandle( TaskHandle_t xTaskHandle );
    static void prvInitTCBArray( void );
    /* Find index for an empty entry in xTCBArray. Return -1 if there is no empty entry. */
    static BaseType_t prvFindEmptyElementIndexTCB( void );
    /* Remove a pointer to extended TCB from xTCBArray. */
    static void prvDeleteTCBFromArray( BaseType_t xIndex );
#elif ( schedUSE_TCB_SORTED_LIST == 1 )
    static void prvAddTCBToList( SchedTCB_t * pxTCB );
    static void prvDeleteTCBFromList( SchedTCB_t * pxTCB );
#endif /* schedUSE_TCB_ARRAY */
static SchedTCB_t * prvGetTCBFromHandle( TaskHandle_t taskHandle );

static TickType_t xSystemStartTime = 0;

static void prvPeriodicTaskCode( void * pvParameters );
/* static void prvCreateAllTasks( void ); */


#if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS || \
      schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS )
    static void prvSetFixedPriorities( void );
#elif ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
    static void prvInitEDF( void );
    #if ( schedEDF_NAIVE == 1 )
        static void prvUpdatePrioritiesEDF( void );
    #elif ( schedEDF_EFFICIENT == 1 )
        static void prvInsertTCBToReadyList( SchedTCB_t * pxTCB );
    #endif /* schedEDF_NAIVE */
    #if ( schedUSE_TCB_SORTED_LIST == 1 )
        static void prvSwapList( List_t ** ppxList1,
                                 List_t ** ppxList2 );
    #endif /* schedUSE_TCB_SORTED_LIST */
#endif /* schedSCHEDULING_POLICY_EDF */

#if ( schedUSE_SCHEDULER_TASK == 1 )
    static void prvSchedulerCheckTimingError( TickType_t xTickCount,
                                              SchedTCB_t * pxTCB );
    static void prvSchedulerFunction( void );
    static void prvCreateSchedulerTask( void );
    static void prvWakeScheduler( void );

    #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
        static void prvPeriodicTaskRecreate( SchedTCB_t * pxTCB );
        static void prvDeadlineMissedHook( SchedTCB_t * pxTCB,
                                           TickType_t xTickCount );
        static void prvCheckDeadline( SchedTCB_t * pxTCB,
                                      TickType_t xTickCount );
        #if ( schedUSE_SPORADIC_JOBS == 1 )
            static void prvDeadlineMissedHookSporadicJob( BaseType_t xIndex );
            static void prvCheckSporadicJobDeadline( TickType_t xTickCount );
        #endif /* schedUSE_SPORADIC_JOBS */
    #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */

    #if ( schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 )
        static void prvExecTimeExceedHook( TickType_t xTickCount,
                                           SchedTCB_t * pxCurrentTask );
    #endif /* schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME */

#endif /* schedUSE_SCHEDULER_TASK */


#if ( schedUSE_POLLING_SERVER == 1 )
    static void prvPollingServerFunction( void );
    void prvPollingServerCreate( void );
#endif /* schedUSE_POLLING_SERVER */


#if ( schedUSE_APERIODIC_JOBS == 1 )
    static AJCB_t * prvGetNextAperiodicJob( void );
    static BaseType_t prvFindEmptyElementIndexAJCB( void );
#endif /* schedUSE_APERIODIC_JOBS */


#if ( schedUSE_SPORADIC_JOBS == 1 )
    static SJCB_t * prvGetNextSporadicJob( void );
    static BaseType_t prvFindEmptyElementIndexSJCB( void );
    static BaseType_t prvAnalyzeSporadicJobSchedulability( SJCB_t * pxSporadicJob,
                                                           TickType_t xTickCount );
#endif /* schedUSE_SPORADIC_JOBS */


#if ( schedUSE_TCB_ARRAY == 1 )
/* Array for extended TCBs. */
static SchedTCB_t xTCBArray[ schedMAX_NUMBER_OF_PERIODIC_TASKS ] = { 0 };
/* Counter for number of periodic tasks. */
static BaseType_t xTaskCounter = 0;
#elif ( schedUSE_TCB_SORTED_LIST == 1 )
    static List_t xTCBList;           /* Sorted linked list for all periodic tasks. */
    static List_t * pxTCBList = NULL; /* Pointer to xTCBList. */
    #if ( schedEDF_NAIVE == 1 )
        /* static List_t xTCBList;      / * Sorted linked list for all periodic tasks. * / */
        static List_t xTCBTempList;                 /* A temporary list used for switching lists. */
        static List_t xTCBOverflowedList;           /* Sorted linked list for periodic tasks that have overflowed deadline. */
        /* static List_t * pxTCBList = NULL;                / * Pointer to xTCBList. * / */
        static List_t * pxTCBTempList = NULL;       /* Pointer to xTCBTempList. */
        static List_t * pxTCBOverflowedList = NULL; /* Pointer to xTCBOverflowedList. */
    #elif ( schedEDF_EFFICIENT == 1 )
        /* static List_t xTCBListAll;   / * Sorted linked list for all periodic tasks. * / */
        static List_t xTCBBlockedList;                           /* Linked list for blocked periodic tasks. */
        static List_t xTCBReadyList;                             /* Sorted linked list for all ready periodic tasks. */
        static List_t xTCBOverflowedReadyList;                   /* Sorted linked list for all ready periodic tasks with overflowed deadline. */
        /* static List_t * pxTCBListAll;                    / * Pointer to xTCBListAll. * / */
        static List_t * pxTCBBlockedList = NULL;                 /* Pointer to xTCBBlockedList. */
        static List_t * pxTCBReadyList = NULL;                   /* Pointer to xTCBReadyList. */
        static List_t * pxTCBOverflowedReadyList = NULL;         /* Pointer to xTCBOverflowedReadyList. */

        static SchedTCB_t * pxCurrentTCB = NULL;                 /* Pointer to current executing periodic task. */
        static SchedTCB_t * pxPreviousTCB = NULL;                /* Pointer to previous executing periodic task. */
        static BaseType_t xSwitchToSchedulerOnSuspend = pdFALSE; /* pdTRUE if context switch to scheduler task occur since a task is suspended. */
        static BaseType_t xSwitchToSchedulerOnBlock = pdFALSE;   /* pdTRUE if context switch to scheduler task occur since a task is blocked. */
        static BaseType_t xSwitchToSchedulerOnReady = pdFALSE;   /* pdTRUE if context switch to scheduler task occur since a task becomes ready. */
    #endif /* schedEDF_NAIVE */
#endif /* schedUSE_TCB_ARRAY */

#if ( schedUSE_SCHEDULER_TASK )
    #ifdef schedSCHEDULER_TASK_PERIOD
        static TickType_t xSchedulerWakeCounter = 0;
    #endif
    static TaskHandle_t xSchedulerHandle = NULL;
#endif /* schedUSE_SCHEDULER_TASK */

#if ( schedUSE_APERIODIC_JOBS == 1 )
/* Array for extended AJCBs (Aperiodic Job Control Block). */
static AJCB_t xAJCBFifo[ schedMAX_NUMBER_OF_APERIODIC_JOBS ] = { 0 };
static BaseType_t xAJCBFifoHead = 0;
static BaseType_t xAJCBFifoTail = 0;
static UBaseType_t uxAperiodicJobCounter = 0;
#endif /* schedUSE_APERIODIC_JOBS */

#if ( schedUSE_SPORADIC_JOBS == 1 )
/* Array for extended SJCBs (Sporadic Job Control Block). */
static SJCB_t xSJCBFifo[ schedMAX_NUMBER_OF_SPORADIC_JOBS ] = { 0 };
static BaseType_t xSJCBFifoHead = 0;
static BaseType_t xSJCBFifoTail = 0;

static UBaseType_t uxSporadicJobCounter = 0;
static TickType_t xAbsolutePreviousMaxResponseTime = 0;
#endif /* schedUSE_SPORADIC_JOBS */


#if ( schedUSE_POLLING_SERVER == 1 )
    static TaskHandle_t xPollingServerHandle = NULL;
    #if ( schedUSE_APERIODIC_JOBS == 1 )
        static AJCB_t * pxCurrentAperiodicJob;
    #endif /* schedUSE_APERIODIC_JOBS */
    #if ( schedUSE_SPORADIC_JOBS == 1 )
        static SJCB_t * pxCurrentSporadicJob;
    #endif /* schedUSE_SPORADIC_JOBS */
#endif /* schedUSE_POLLING_SERVER */

void vSchedSpinAlong() __attribute__( ( weak ) );
void vSchedSpinAlong()
{
}

#if ( schedUSE_TCB_ARRAY == 1 )
    /* Returns index position in xTCBArray of TCB with same task handle as parameter. */
    static BaseType_t prvGetTCBIndexFromHandle( TaskHandle_t xTaskHandle )
    {
        static BaseType_t xIndex = 0;
        BaseType_t xIterator;

        for( xIterator = 0; xIterator < schedMAX_NUMBER_OF_PERIODIC_TASKS; xIterator++ )
        {
            if( ( pdTRUE == xTCBArray[ xIndex ].xInUse ) &&
                ( *xTCBArray[ xIndex ].pxTaskHandle == xTaskHandle ) )
            {
                return xIndex;
            }

            xIndex++;

            if( schedMAX_NUMBER_OF_PERIODIC_TASKS == xIndex )
            {
                xIndex = 0;
            }
        }

        return -1;
    }

    /* Initializes xTCBArray. */
    static void prvInitTCBArray( void )
    {
        UBaseType_t uxIndex;

        for( uxIndex = 0; uxIndex < schedMAX_NUMBER_OF_PERIODIC_TASKS; uxIndex++ )
        {
            xTCBArray[ uxIndex ].xInUse = pdFALSE;
        }
    }

    /* Find index for an empty entry in xTCBArray. Returns -1 if there is no empty entry. */
    static BaseType_t prvFindEmptyElementIndexTCB( void )
    {
        BaseType_t xIndex;

        for( xIndex = 0; xIndex < schedMAX_NUMBER_OF_PERIODIC_TASKS; xIndex++ )
        {
            if( pdFALSE == xTCBArray[ xIndex ].xInUse )
            {
                return xIndex;
            }
        }

        return -1;
    }

    /* Remove a pointer to extended TCB from xTCBArray. */
    static void prvDeleteTCBFromArray( BaseType_t xIndex )
    {
        configASSERT( xIndex >= 0 && xIndex < schedMAX_NUMBER_OF_PERIODIC_TASKS );
        configASSERT( pdTRUE == xTCBArray[ xIndex ].xInUse );

        if( xTCBArray[ pdTRUE == xIndex ].xInUse )
        {
            xTCBArray[ xIndex ].xInUse = pdFALSE;
            xTaskCounter--;
        }
    }

#elif ( schedUSE_TCB_SORTED_LIST == 1 )
    /* Add an extended TCB to sorted linked list. */
    static void prvAddTCBToList( SchedTCB_t * pxTCB )
    {
        /* Initialise TCB list item. */
        vListInitialiseItem( &pxTCB->xTCBListItem );
        /* Set owner of list item to the TCB. */
        listSET_LIST_ITEM_OWNER( &pxTCB->xTCBListItem, pxTCB );

        #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF || schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS )
            /* List is sorted by absolute deadline value. */
            listSET_LIST_ITEM_VALUE( &pxTCB->xTCBListItem, pxTCB->xAbsoluteDeadline );
            #if ( schedEDF_EFFICIENT == 1 )
                /* Initialise list item for xTCBListAll. */
                vListInitialiseItem( &pxTCB->xTCBAllListItem );
                /* Set owner of list item to the TCB. */
                listSET_LIST_ITEM_OWNER( &pxTCB->xTCBAllListItem, pxTCB );
                /* There is no need to sort the list. */
            #endif /* schedEDF_EFFICIENT */
        #elif ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS )
            /* List is sorted by period. */
            listSET_LIST_ITEM_VALUE( &pxTCB->xTCBListItem, pxTCB->xPeriod );
        #else  /* if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF || schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS ) */
            /* List is sorted by user-defined priority. */
            listSET_LIST_ITEM_VALUE( &pxTCB->xTCBListItem, pxTCB->uxPriority );
        #endif /* if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF || schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS ) */

        #if ( schedSCHEDULING_POLICY != schedSCHEDULING_POLICY_EDF || schedEDF_NAIVE == 1 )
            /* Insert TCB into list. */
            vListInsert( pxTCBList, &pxTCB->xTCBListItem );
        #else
            /* schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF && schedEDF_EFFICIENT == 1 */
            /* Insert TCB into ready list. */
            vListInsert( pxTCBReadyList, &pxTCB->xTCBListItem );
            /* Insert TCB into list containing tasks in any state. */
            vListInsert( pxTCBList, &pxTCB->xTCBAllListItem );
        #endif
    }

    /* Delete an extended TCB from sorted linked list. */
    static void prvDeleteTCBFromList( SchedTCB_t * pxTCB )
    {
        #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF && schedEDF_EFFICIENT == 1 )
            uxListRemove( &pxTCB->xTCBAllListItem );
        #endif /* schedEDF_EFFICIENT */
        uxListRemove( &pxTCB->xTCBListItem );
        vPortFree( pxTCB );
    }
#endif /* schedUSE_TCB_ARRAY */

SchedTCB_t * prvGetTCBFromHandle( TaskHandle_t taskHandle )
{
    return ( SchedTCB_t * ) pvTaskGetThreadLocalStoragePointer(
        taskHandle, schedTHREAD_LOCAL_STORAGE_POINTER_INDEX );
}

#if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
    #if ( schedUSE_TCB_SORTED_LIST == 1 )
        /* Swap content of two lists. */
        static void prvSwapList( List_t ** ppxList1,
                                 List_t ** ppxList2 )
        {
            List_t * pxTemp;

            pxTemp = *ppxList1;
            *ppxList1 = *ppxList2;
            *ppxList2 = pxTemp;
        }
    #endif /* schedUSE_TCB_SORTED_LIST */

    #if ( schedEDF_NAIVE == 1 )
        /* Update priorities of all periodic tasks with respect to EDF policy. */
        static void prvUpdatePrioritiesEDF( void )
        {
            SchedTCB_t * pxTCB;

            #if ( schedUSE_TCB_SORTED_LIST == 1 )
                ListItem_t * pxTCBListItem;
                ListItem_t * pxTCBListItemTemp;

                if( listLIST_IS_EMPTY( pxTCBList ) &&
                    !listLIST_IS_EMPTY( pxTCBOverflowedList ) )
                {
                    prvSwapList( &pxTCBList, &pxTCBOverflowedList );
                }

                const ListItem_t * pxTCBListEndMarker = listGET_END_MARKER( pxTCBList );
                pxTCBListItem = listGET_HEAD_ENTRY( pxTCBList );

                while( pxTCBListItem != pxTCBListEndMarker )
                {
                    pxTCB = listGET_LIST_ITEM_OWNER( pxTCBListItem );

                    /* Update priority in the SchedTCB list. */
                    listSET_LIST_ITEM_VALUE( pxTCBListItem, pxTCB->xAbsoluteDeadline );

                    pxTCBListItemTemp = pxTCBListItem;
                    pxTCBListItem = listGET_NEXT( pxTCBListItem );
                    uxListRemove( pxTCBListItem->pxPrevious );

                    /* If absolute deadline overflowed, insert TCB to overflowed list. */
                    if( pxTCB->xAbsoluteDeadline < pxTCB->xLastWakeTime )
                    {
                        vListInsert( pxTCBOverflowedList, pxTCBListItemTemp );
                    }
                    else /* Insert TCB into temp list in usual case. */
                    {
                        vListInsert( pxTCBTempList, pxTCBListItemTemp );
                    }
                }

                /* Swap list with temp list. */
                prvSwapList( &pxTCBList, &pxTCBTempList );

                #if ( schedUSE_SCHEDULER_TASK == 1 )
                    BaseType_t xHighestPriority = schedSCHEDULER_PRIORITY - 1;
                #else
                    BaseType_t xHighestPriority = configMAX_PRIORITIES - 1;
                #endif /* schedUSE_SCHEDULER_TASK */

                const ListItem_t * pxTCBListEndMarkerAfterSwap =
                    listGET_END_MARKER( pxTCBList );
                pxTCBListItem = listGET_HEAD_ENTRY( pxTCBList );

                while( pxTCBListItem != pxTCBListEndMarkerAfterSwap )
                {
                    pxTCB = listGET_LIST_ITEM_OWNER( pxTCBListItem );
                    configASSERT( -1 <= xHighestPriority );
                    pxTCB->uxPriority = xHighestPriority;
                    vTaskPrioritySet( *pxTCB->pxTaskHandle, pxTCB->uxPriority );

                    xHighestPriority--;
                    pxTCBListItem = listGET_NEXT( pxTCBListItem );
                }
            #endif /* schedUSE_TCB_SORTED_LIST */
        }
    #elif ( schedEDF_EFFICIENT == 1 )
        /* Removes given SchedTCB from blocked list and inserts it to ready list. */
        static void prvInsertTCBToReadyList( SchedTCB_t * pxTCB )
        {
            /* Remove ready task from blocked list. */
            uxListRemove( &pxTCB->xTCBListItem );

            /* Check whether absolute deadline has overflowed or not. */
            if( pxTCB->xAbsoluteDeadline < pxTCB->xLastWakeTime )
            {
                /* Absolute deadline has overflowed. */
                vListInsert( pxTCBOverflowedReadyList, &pxTCB->xTCBListItem );
            }
            else
            {
                vListInsert( pxTCBReadyList, &pxTCB->xTCBListItem );
            }

            /* If ready list is empty and overflowed ready list is not, swap those. */
            if( listLIST_IS_EMPTY( pxTCBReadyList ) &&
                !listLIST_IS_EMPTY( pxTCBOverflowedReadyList ) )
            {
                prvSwapList( &pxTCBReadyList, &pxTCBOverflowedReadyList );
            }

/*
 *          ListItem_t *pxListItem = listGET_HEAD_ENTRY( pxTCBReadyList );
 *          SchedTCB_t *pxHighestPriorityTCB = listGET_LIST_ITEM_OWNER( pxListItem );
 *          SchedTCB_t *pxNextTCB = listGET_LIST_ITEM_OWNER( pxListItem->pxNext );
 *          if( pxCurrentTCB != pxHighestPriorityTCB )
 *          {
 *              if( NULL != pxCurrentTCB )
 *              {
 *                  pxPreviousTCB = pxCurrentTCB;
 *              }
 *              pxCurrentTCB = pxHighestPriorityTCB;
 *              xSwitchToSchedulerOnReady = pdTRUE;
 *              prvWakeScheduler();
 *          }
 */

            if( ( NULL == pxCurrentTCB ) || ( ( signed ) (
                                                  pxCurrentTCB->xAbsoluteDeadline > pxTCB->xAbsoluteDeadline ) > 0 ) )
            {
                if( NULL != pxCurrentTCB )
                {
                    pxPreviousTCB = pxCurrentTCB;
                }

                pxCurrentTCB = pxTCB;
                xSwitchToSchedulerOnReady = pdTRUE;
                prvWakeScheduler();
            }
        }

        /* Called when a task is blocked. */
        void vSchedulerBlockTrace( void )
        {
            if( ( NULL != pxCurrentTCB ) &&
                ( xTaskGetCurrentTaskHandle() == *pxCurrentTCB->pxTaskHandle ) )
            {
                /* Remove current task from ready list. */
                uxListRemove( &pxCurrentTCB->xTCBListItem );
                /* Insert current task to blocked list. */
                vListInsert( pxTCBBlockedList, &pxCurrentTCB->xTCBListItem );

                pxPreviousTCB = pxCurrentTCB;
                pxCurrentTCB = NULL;
                xSwitchToSchedulerOnBlock = pdTRUE;
                prvWakeScheduler();
            }
        }

        /* Called when a task is suspended. */
        void vSchedulerSuspendTrace( TaskHandle_t taskHandle )
        {
            SchedTCB_t * pxTCBToSuspend = prvGetTCBFromHandle( taskHandle );

            if( NULL != pxTCBToSuspend )
            {
                /* Remove suspended task from ready list. */
                uxListRemove( &pxTCBToSuspend->xTCBListItem );
                /* Insert suspended task to blocked list. */
                vListInsert( pxTCBBlockedList, &pxTCBToSuspend->xTCBListItem );

                if( *pxCurrentTCB->pxTaskHandle == *pxTCBToSuspend->pxTaskHandle )
                {
                    pxPreviousTCB = pxCurrentTCB;
                    pxCurrentTCB = NULL;
                    xSwitchToSchedulerOnSuspend = pdTRUE;
                    prvWakeScheduler();
                }
            }
        }

        /* Called when a task becomes ready. */
        void vSchedulerReadyTrace( TaskHandle_t taskHandle )
        {
            if( ( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED ) ||
                ( taskHandle == xSchedulerHandle ) )
            {
                return;
            }

            #if ( configUSE_TIMERS == 1 )
                /* Check if the unblocked task was the timer task. */
                if( xTimerGetTimerDaemonTaskHandle() == xTaskHandle )
                {
                    return;
                }
            #endif /* configUSE_TIMERS */

            SchedTCB_t * pxTCBReady = prvGetTCBFromHandle( taskHandle );

            if( NULL != pxTCBReady )
            {
                prvInsertTCBToReadyList( pxTCBReady );
            }
        }
    #endif /* schedEDF_EFFICIENT */
#endif /* schedSCHEDULING_POLICY_EDF */

/* The whole function code that is executed by every periodic task.
 * This function wraps the task code specified by the user. */
static void prvPeriodicTaskCode( void * pvParameters )
{
    SchedTCB_t * pxThisTask = prvGetTCBFromHandle( xTaskGetCurrentTaskHandle() );

    configASSERT( NULL != pxThisTask );

    if( 0 != pxThisTask->xReleaseTime )
    {
        ( void ) xTaskDelayUntil( &pxThisTask->xLastWakeTime, pxThisTask->xReleaseTime );
    }
    else
    {
        pxThisTask->xLastWakeTime = xSystemStartTime;
    }

    for( ; ; )
    {
        pxThisTask->xExecTime = 0;
        pxThisTask->eState = eTaskReady;
        #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
            pxThisTask->uxExecutedCycles++;
        #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */
        #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
            #if ( schedEDF_NAIVE == 1 )
                /* Wake up the scheduler task to update priorities of all periodic tasks. */
                prvWakeScheduler();
            #endif /* schedEDF_NAIVE */
        #endif /* schedSCHEDULING_POLICY_EDF */

        /*
         * taskENTER_CRITICAL();
         * printf( "tickcount %d Task %s Abs deadline %d lastWakeTime %d prio %d Handle %x\r\n",
         *      xTaskGetTickCount(), pxThisTask->pcName, pxThisTask->xAbsoluteDeadline,
         *      pxThisTask->xLastWakeTime, uxTaskPriorityGet( NULL ), *pxThisTask->pxTaskHandle );
         * taskEXIT_CRITICAL();
         */

        /* NOTE: Execute the task function specified by the user. */
        pxThisTask->eState = eTaskRunning;
        pxThisTask->pvTaskCode( pvParameters );

        #if ( schedENABLE_SELF_EVALUATION == 1 )
            if( pxThisTask->pvEvaluatorCode )
            {
                pxThisTask->pvEvaluatorCode( pvParameters );
            }
        #endif
        pxThisTask->eState = eTaskBlocked;

        /*
         * taskENTER_CRITICAL();
         * printf( "execution time %d Task %s\r\n", pxThisTask->xExecTime, pxThisTask->pcName );
         * taskEXIT_CRITICAL();
         */

        #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
            pxThisTask->xAbsoluteDeadline = pxThisTask->xLastWakeTime +
                                            pxThisTask->xPeriod + pxThisTask->xRelativeDeadline;
            #if ( schedEDF_EFFICIENT == 1 )
                listSET_LIST_ITEM_VALUE(
                    &pxThisTask->xTCBListItem, pxThisTask->xAbsoluteDeadline );
            #endif /* schedEDF_EFFICIENT */

            #if ( schedEDF_NAIVE == 1 )
                /* Wake up the scheduler task to update priorities of all periodic tasks. */
                prvWakeScheduler();
            #endif /* schedEDF_NAIVE */
        #endif /* schedSCHEDULING_POLICY_EDF */

        ( void ) xTaskDelayUntil( &pxThisTask->xLastWakeTime, pxThisTask->xPeriod );
    }
}

#if 0
/* Creates all periodic tasks stored in TCB array, or TCB list. */
    static void prvCreateAllTasks( void )
    {
        SchedTCB_t * pxTCB;

        #if ( schedUSE_TCB_ARRAY == 1 )
            BaseType_t xIndex;

            for( xIndex = 0; xIndex < xTaskCounter; xIndex++ )
            {
                configASSERT( pdTRUE == xTCBArray[ xIndex ].xInUse );
                pxTCB = &xTCBArray[ xIndex ];

                BaseType_t xReturnValue = xTaskCreate(
                    prvPeriodicTaskCode,
                    pxTCB->pcName,
                    pxTCB->uxStackDepth,
                    pxTCB->pvParameters,
                    pxTCB->uxPriority,
                    pxTCB->pxTaskHandle );

                configASSERT( pdPASS == xReturnValue );
                vTaskSetThreadLocalStoragePointer( *pxTCB->pxTaskHandle,
                                                   schedTHREAD_LOCAL_STORAGE_POINTER_INDEX, pxTCB );
                #if ( schedPERIODIC_TASK_AUTOTART == 0 )
                    /* all tasks are created as suspended */
                    vTaskSuspend( pxTCB->pxTaskHandle );
                    pxTCB->eState = eTaskSuspended;
                #else
                    pxTCB->eState = eTaskReady;
                #endif
            }
        #elif ( schedUSE_TCB_SORTED_LIST == 1 )
            const ListItem_t * pxTCBListEndMarker = listGET_END_MARKER( pxTCBList );
            ListItem_t * pxTCBListItem = listGET_HEAD_ENTRY( pxTCBList );

            while( pxTCBListItem != pxTCBListEndMarker )
            {
                pxTCB = listGET_LIST_ITEM_OWNER( pxTCBListItem );
                configASSERT( NULL != pxTCB );

                /* NOTE: creation of the task inside the RTOS */

                /* FIXME: configASSERT(scheduler is not running || this task priority >= pxTCB->uxPriority)
                 * NOTE: newly created task gets pushed to the ready queue directly.
                 * If the round-robin sched (kernel sched) is running,
                 * the newly created task will run its first cycle immediately if it has the highest priority.
                 * because it is by default added to the ready queue (task.c, line 2087).
                 * However, its local storage pointer is still unset (NULL), causing the first cycle to fail. */
                BaseType_t xReturnValue = xTaskCreate(
                    prvPeriodicTaskCode,
                    pxTCB->pcName,
                    pxTCB->uxStackDepth,
                    pxTCB->pvParameters,
                    pxTCB->uxPriority,
                    pxTCB->pxTaskHandle );

                configASSERT( pdPASS == xReturnValue );
                /* the tcb is stored with the task handle, providing */
                /* the associated function pointers and memory space */
                vTaskSetThreadLocalStoragePointer( *( pxTCB->pxTaskHandle ),
                                                   schedTHREAD_LOCAL_STORAGE_POINTER_INDEX, pxTCB );
                pxTCBListItem = listGET_NEXT( pxTCBListItem );

                #if ( schedPERIODIC_TASK_AUTOTART == 0 )
                    /* all tasks are created as suspended */
                    vTaskSuspend( *( pxTCB->pxTaskHandle ) );
                    pxTCB->eState = eTaskSuspended;
                #else
                    pxTCB->eState = eTaskReady;
                #endif
            }
        #endif /* schedUSE_TCB_ARRAY */
    }
#endif /* if 0 */

static void prvCreateTask( SchedTCB_t * pxTCB )
{
    #if ( schedUSE_TCB_ARRAY == 1 )
        configASSERT( pdTRUE == pxTCB->xInUse );
    #elif ( schedUSE_TCB_SORTED_LIST == 1 )
        configASSERT( NULL != pxTCB );
    #endif
    /* NOTE: creation of the task inside the RTOS */

    /* FIXME: configASSERT(scheduler is not running || this task priority >= pxTCB->uxPriority)
     * NOTE: newly created task gets pushed to the ready queue directly.
     * If the round-robin sched (kernel sched) is running,
     * the newly created task will run its first cycle immediately if it has the highest priority.
     * because it is by default added to the ready queue (task.c, line 2087).
     * However, its local storage pointer is still unset (NULL), causing the first cycle to fail. */
    /* prevents task switching at this point */
    UBaseType_t uxSchedRunning = ( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING );

    if( uxSchedRunning )
    {
        vTaskSuspendAll();
    }

    BaseType_t xReturnValue = xTaskCreate(
        prvPeriodicTaskCode,
        pxTCB->pcName,
        pxTCB->uxStackDepth,
        pxTCB->pvParameters,
        pxTCB->uxPriority,
        pxTCB->pxTaskHandle );

    configASSERT( pdPASS == xReturnValue );
    /* the tcb is stored with the task handle, providing */
    /* the associated function pointers and memory space */
    vTaskSetThreadLocalStoragePointer( *( pxTCB->pxTaskHandle ),
                                       schedTHREAD_LOCAL_STORAGE_POINTER_INDEX, pxTCB );
    #if ( schedPERIODIC_TASK_AUTOTART == 0 )
        /* all tasks are created as suspended */
        vTaskSuspend( *( pxTCB->pxTaskHandle ) );
        pxTCB->eState = eTaskSuspended;
    #else
        pxTCB->eState = eTaskReady;
    #endif

    /* resumes scheduler */
    if( uxSchedRunning )
    {
        ( void ) xTaskResumeAll();
    }
}

/* Creates a periodic task. */
void vSchedulerPeriodicTaskCreate( TaskFunction_t pvTaskCode,
                                   const char * pcName,
                                   UBaseType_t uxStackDepth,
                                   void * pvParameters,
                                   UBaseType_t uxPriority,
                                   TaskHandle_t * pxCreatedTask,
                                   TickType_t xPhaseTick,
                                   TickType_t xPeriodTick,
                                   TickType_t xMaxExecTimeTick,
                                   TickType_t xDeadlineTick )
{
    SchedTCB_t * pxNewTCB;

    taskENTER_CRITICAL();
    #if ( schedUSE_TCB_ARRAY == 1 )
        BaseType_t xIndex = prvFindEmptyElementIndexTCB();
        configASSERT( xTaskCounter < schedMAX_NUMBER_OF_PERIODIC_TASKS );
        configASSERT( xIndex != -1 );
        pxNewTCB = &xTCBArray[ xIndex ];
    #else
        pxNewTCB = pvPortMalloc( sizeof( SchedTCB_t ) );
    #endif /* schedUSE_TCB_ARRAY */

    /* Intialize item. */
    *pxNewTCB = ( SchedTCB_t ) {
        .pvTaskCode = pvTaskCode,
        .pcName = pcName,
        .uxStackDepth = uxStackDepth,
        .pvParameters = pvParameters,
        .uxPriority = uxPriority,
        .pxTaskHandle = pxCreatedTask,
        .xReleaseTime = xPhaseTick,
        .xPeriod = xPeriodTick,
        .xMaxExecTime = xMaxExecTimeTick,
        .xRelativeDeadline = xDeadlineTick,
        .xExecTime = 0,
        .eState = eTaskInvalid
    };
    #if ( schedUSE_TCB_ARRAY == 1 )
        pxNewTCB->xInUse = pdTRUE;
    #endif /* schedUSE_TCB_ARRAY */

    #if ( ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS ||   \
            schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS ) && \
    schedUSE_TCB_ARRAY )
        pxNewTCB->xPriorityIsSet = pdFALSE;
    #elif ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
        pxNewTCB->xAbsoluteDeadline = pxNewTCB->xRelativeDeadline +
                                      pxNewTCB->xReleaseTime + xSystemStartTime;
        #if ( schedEDF_EFFICIENT == 1 )
            pxNewTCB->uxPriority = schedPRIORITY_NOT_RUNNING;
        #elif ( schedEDF_NAIVE == 1 ) /* schedEDF_EFFICIENT */
            pxNewTCB->uxPriority = -1;
        #endif /* schedEDF_NAIVE */
    #endif /* schedSCHEDULING_POLICY */
    #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
        pxNewTCB->uxExecutedCycles = 0;
    #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */
    #if ( schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 )
        pxNewTCB->xSuspended = pdFALSE;
        pxNewTCB->xMaxExecTimeExceeded = pdFALSE;
    #endif /* schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME */
    #if ( schedUSE_POLLING_SERVER == 1 )
        pxNewTCB->xIsPeriodicServer = pdFALSE;
    #endif /* schedUSE_POLLING_SERVER */

    #if ( schedUSE_TCB_ARRAY == 1 )
        xTaskCounter++;
    #elif ( schedUSE_TCB_SORTED_LIST == 1 )
        prvAddTCBToList( pxNewTCB );
    #endif /* schedUSE_TCB_SORTED_LIST */
    taskEXIT_CRITICAL();
    #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS || \
          schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS )
        /*taskENTER_CRITICAL(); */
        prvSetFixedPriorities();
        /*taskEXIT_CRITICAL(); */
    #endif
    prvCreateTask( pxNewTCB );
    #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
        if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING )
        {
            /* Wake up the scheduler task to update priorities of all periodic tasks. */
            prvWakeScheduler();
        }
    #endif /* schedSCHEDULING_POLICY_EDF */
}

/* Deletes a periodic task. */
void vSchedulerPeriodicTaskDelete( TaskHandle_t taskHandle )
{
    if( taskHandle != NULL )
    {
        #if ( schedUSE_TCB_ARRAY == 1 )
            prvDeleteTCBFromArray( prvGetTCBIndexFromHandle( taskHandle ) );
        #elif ( schedUSE_TCB_SORTED_LIST == 1 )
            prvDeleteTCBFromList( prvGetTCBFromHandle( taskHandle ) );
        #endif /* schedUSE_TCB_ARRAY */
    }
    else
    {
        #if ( schedUSE_TCB_ARRAY == 1 )
            prvDeleteTCBFromArray(
                prvGetTCBIndexFromHandle( xTaskGetCurrentTaskHandle() ) );
        #elif ( schedUSE_TCB_SORTED_LIST == 1 )
            prvDeleteTCBFromList( prvGetTCBFromHandle(
                                      xTaskGetCurrentTaskHandle() ) );
        #endif /* schedUSE_TCB_ARRAY */
    }

    vTaskDelete( taskHandle );
}

void vSchedulerPeriodicTaskUpdate( TaskHandle_t taskHandle,
                                   TickType_t xPeriodTick,
                                   TickType_t xMaxExecTimeTick,
                                   TickType_t xDeadlineTick )
{
    SchedTCB_t * tcb = prvGetTCBFromHandle( taskHandle );

    configASSERT( NULL != tcb );
    /* ISR should NOT kick in */
    taskENTER_CRITICAL();
    tcb->xPeriod = xPeriodTick;
    tcb->xMaxExecTime = xMaxExecTimeTick;
    tcb->xRelativeDeadline = xDeadlineTick;
    #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS || \
          schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS )
        prvSetFixedPriorities();
    #endif
    taskEXIT_CRITICAL();
}

TickType_t ulSchedulerPeriodicTaskGetPeriod( TaskHandle_t * taskHandle )
{
    SchedTCB_t * tcb = prvGetTCBFromHandle( *taskHandle );

    configASSERT( NULL != tcb );
    return tcb->xPeriod;
}

#if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
    UBaseType_t uxSchedulerPeriodicTaskGetMissed( TaskHandle_t * taskHandle )
    {
        SchedTCB_t * tcb = prvGetTCBFromHandle( *taskHandle );

        configASSERT( NULL != tcb );
        return tcb->uxMissed;
    }
#endif

#if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS || \
      schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS )

    /* Initiazes fixed priorities of all periodic tasks with respect to RMS or
     * DMS policy. */
    static void prvSetFixedPriorities( void )
    {
        SchedTCB_t * pxTCB;

        #if ( schedUSE_SCHEDULER_TASK == 1 )
            BaseType_t xHighestPriority = schedSCHEDULER_PRIORITY;
        #else
            BaseType_t xHighestPriority = configMAX_PRIORITIES;
        #endif /* schedUSE_SCHEDULER_TASK */

        #if ( schedUSE_TCB_ARRAY == 1 )
            TickType_t xShortest, xPreviousShortest = 0;
            SchedTCB_t * pxShortestTaskPointer;
            /* O(N^2) double for-loop across all tasks to find out period ranking for every task */
            BaseType_t xIter, xIndex;

            for( xIter = 0; xIter < xTaskCounter; xIter++ )
            {
                xShortest = portMAX_DELAY;

                /* search for shortest period/deadline */
                for( xIndex = 0; xIndex < xTaskCounter; xIndex++ )
                {
                    pxTCB = &xTCBArray[ xIndex ];
                    configASSERT( pdTRUE == pxTCB->xInUse );

                    if( pdTRUE == pxTCB->xPriorityIsSet )
                    {
                        continue;
                    }

                    #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS )
                        if( pxTCB->xPeriod <= xShortest )
                        {
                            xShortest = pxTCB->xPeriod;
                            pxShortestTaskPointer = pxTCB;
                        }
                    #elif ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS )
                        if( pxTCB->xRelativeDeadline <= xShortest )
                        {
                            xShortest = pxTCB->xRelativeDeadline;
                            pxShortestTaskPointer = pxTCB;
                        }
                    #endif /* schedSCHEDULING_POLICY */
                }

                configASSERT( -1 <= xHighestPriority );

                if( xPreviousShortest != xShortest )
                {
                    /* this if condition is entered at least once for any tasks */
                    xHighestPriority--;
                }

                /* set highest priority to task with xShortest period (the highest priority is configMAX_PRIORITIES-1) */
                pxShortestTaskPointer->uxPriority = xHighestPriority;
                pxShortestTaskPointer->xPriorityIsSet = pdTRUE;

                xPreviousShortest = xShortest;
            }
        #elif ( schedUSE_TCB_SORTED_LIST == 1 )
            /* NOTE: pxTCBList is sorted in increasing order of period (decr order of priority) */
            const ListItem_t * pxTCBListEndMarker = listGET_END_MARKER( pxTCBList );
            UBaseType_t uxLastItemValue = 0;

            for( ListItem_t * rankPtr = listGET_HEAD_ENTRY( pxTCBList );
                 rankPtr != pxTCBListEndMarker;
                 rankPtr = listGET_NEXT( rankPtr ) )
            {
                /* do not decrease priority value if the next task have the same period or deadline */
                UBaseType_t uxThisItemValue = listGET_LIST_ITEM_VALUE( rankPtr );

                if( uxThisItemValue != uxLastItemValue )
                {
                    xHighestPriority--;
                    uxLastItemValue = uxThisItemValue;
                }

                pxTCB = listGET_LIST_ITEM_OWNER( rankPtr );
                configASSERT( NULL != pxTCB );
                configASSERT( 0 <= xHighestPriority );

                if( pxTCB->uxPriority != xHighestPriority )
                {
                    pxTCB->uxPriority = xHighestPriority;

                    if( pxTCB->pxTaskHandle != NULL )
                    {
                        vTaskPrioritySet( *( pxTCB->pxTaskHandle ), pxTCB->uxPriority );
                    }
                }
            }
        #endif /* if ( schedUSE_TCB_ARRAY == 1 ) */
    }

#elif ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
    /* Initializes priorities of all periodic tasks with respect to EDF policy. */
    static void prvInitEDF( void )
    {
        SchedTCB_t * pxTCB;

        #if ( schedEDF_NAIVE == 1 )
            #if ( schedUSE_SCHEDULER_TASK == 1 )
                UBaseType_t uxHighestPriority = schedSCHEDULER_PRIORITY - 1;
            #else
                UBaseType_t uxHighestPriority = configMAX_PRIORITIES - 1;
            #endif /* schedUSE_SCHEDULER_TASK */

            const ListItem_t * pxTCBListEndMarker = listGET_END_MARKER( pxTCBList );
            ListItem_t * pxTCBListItem = listGET_HEAD_ENTRY( pxTCBList );

            while( pxTCBListItem != pxTCBListEndMarker )
            {
                pxTCB = listGET_LIST_ITEM_OWNER( pxTCBListItem );

                pxTCB->uxPriority = uxHighestPriority;
                uxHighestPriority--;

                pxTCBListItem = listGET_NEXT( pxTCBListItem );
            }
        #elif ( schedEDF_EFFICIENT == 1 )
            /* start first task in the list */
            ListItem_t * pxTCBListItem = listGET_HEAD_ENTRY( pxTCBReadyList );
            pxCurrentTCB = listGET_LIST_ITEM_OWNER( pxTCBListItem );
            pxCurrentTCB->uxPriority = schedPRIORITY_RUNNING;
        #endif /* schedEDF_EFFICIENT */
    }
#endif /* schedSCHEDULING_POLICY */


#if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )

    /* Recreates a deleted task that still has its information left in the task array (or list). */
    static void prvPeriodicTaskRecreate( SchedTCB_t * pxTCB )
    {
        BaseType_t xReturnValue = xTaskCreate(
            prvPeriodicTaskCode,
            pxTCB->pcName,
            pxTCB->uxStackDepth,
            pxTCB->pvParameters,
            pxTCB->uxPriority,
            pxTCB->pxTaskHandle );

        if( pdPASS == xReturnValue )
        {
            vTaskSetThreadLocalStoragePointer( *pxTCB->pxTaskHandle,
                                               schedTHREAD_LOCAL_STORAGE_POINTER_INDEX, ( SchedTCB_t * ) pxTCB );

            /* This must be set to false so that the task does not miss the deadline immediately when it is created. */
            pxTCB->uxExecutedCycles = 0;
            #if ( schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 )
                pxTCB->xSuspended = pdFALSE;
                pxTCB->xMaxExecTimeExceeded = pdFALSE;
            #endif /* schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME */

            #if ( schedEDF_EFFICIENT == 1 )
                prvInsertTCBToReadyList( pxTCB );
            #endif /* schedEDF_EFFICIENT */
        }
        else
        {
            /* if task creation failed */
        }
    }

    /* Called when a deadline of a periodic task is missed.
     * Deletes the periodic task that has missed it's deadline and recreate it.
     * The periodic task is released during next period. */
    static void prvDeadlineMissedHook( SchedTCB_t * pxTCB,
                                       TickType_t xTickCount )
    {
        printf( "\r\ndeadline missed! %s tick %ld\r\n", pxTCB->pcName, xTickCount );

        /* Delete the pxTask and recreate it. */
        pxTCB->uxMissed++;
        vTaskDelete( *pxTCB->pxTaskHandle );
        pxTCB->xExecTime = 0;
        prvPeriodicTaskRecreate( pxTCB );

        pxTCB->xReleaseTime = pxTCB->xLastWakeTime + pxTCB->xPeriod;
        /* Need to reset lastWakeTime for correct release. */
        pxTCB->xLastWakeTime = 0;
        pxTCB->xAbsoluteDeadline = pxTCB->xRelativeDeadline + pxTCB->xReleaseTime;
        #if ( schedEDF_EFFICIENT == 1 )
            listSET_LIST_ITEM_VALUE( &pxTCB->xTCBListItem, pxTCB->xAbsoluteDeadline );
        #endif /* schedEDF_EFFICIENT */
    }

    /* Checks whether given task has missed deadline or not. */
    static void prvCheckDeadline( SchedTCB_t * pxTCB,
                                  TickType_t xTickCount )
    {
        if( ( NULL != pxTCB ) && ( pxTCB->eState == eTaskBlocked ) &&
            ( pxTCB->uxExecutedCycles > 0 ) )
        {
            /* Need to update absolute deadline if the scheduling policy is not EDF. */
            #if ( schedSCHEDULING_POLICY != schedSCHEDULING_POLICY_EDF )
                pxTCB->xAbsoluteDeadline = pxTCB->xLastWakeTime + pxTCB->xRelativeDeadline;
            #endif /* schedSCHEDULING_POLICY */

            /* Using ICTOH method proposed by Carlini and Buttazzo, to check whether deadline is missed. */
            if( ( signed ) ( pxTCB->xAbsoluteDeadline - xTickCount ) < 0 )
            {
                /* Deadline is missed. */
                prvDeadlineMissedHook( pxTCB, xTickCount );
            }
        }
    }

    #if ( schedUSE_SPORADIC_JOBS == 1 )
        /* Called when a deadline of a sporadic job is missed. */
        static void prvDeadlineMissedHookSporadicJob( BaseType_t xIndex )
        {
            printf( "\r\ndeadline missed sporadic job! %s\r\n\r\n",
                    xSJCBFifo[ xIndex ].pcName );
        }

        /* Checks if any sporadic job has missed it's deadline. */
        static void prvCheckSporadicJobDeadline( TickType_t xTickCount )
        {
            if( uxSporadicJobCounter > 0 )
            {
                BaseType_t xIndex;

                for( xIndex = xSJCBFifoHead - 1; xIndex < uxSporadicJobCounter;
                     xIndex++ )
                {
                    if( -1 == xIndex )
                    {
                        xIndex = schedMAX_NUMBER_OF_SPORADIC_JOBS - 1;
                    }

                    if( schedMAX_NUMBER_OF_SPORADIC_JOBS == xIndex )
                    {
                        xIndex = 0;
                    }

                    /* Using ICTOH method proposed by Carlini and Buttazzo, to check whether deadline is missed. */
                    if( ( signed ) ( xSJCBFifo[ xIndex ].xAbsoluteDeadline - xTickCount ) < 0 )
                    {
                        prvDeadlineMissedHookSporadicJob( xIndex );
                    }
                }
            }
        }
    #endif /* schedUSE_SPORADIC_JOBS */
#endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */


#if ( schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 )

    /* Called if a periodic task has exceeded it's worst-case execution time.
     * The periodic task is blocked until next period. A context switch to
     * the scheduler task occur to block the periodic task. */
    static void prvExecTimeExceedHook( TickType_t xTickCount,
                                       SchedTCB_t * pxCurrentTask )
    {
        printf( "\r\nworst case execution time exceeded! %s %ld %ld\r\n",
                pxCurrentTask->pcName, pxCurrentTask->xExecTime, xTickCount );

        pxCurrentTask->eState = eTaskWCETExceeded;
        /* NOTE: IMPORTANT -- This function is called from within the ISR, therefore */
        /* suspending a task is unsafe. */
        pxCurrentTask->xMaxExecTimeExceeded = pdTRUE;
        /* Is not suspended yet, but will be suspended by the scheduler later. */
        pxCurrentTask->xAbsoluteUnblockTime = pxCurrentTask->xLastWakeTime +
                                              pxCurrentTask->xPeriod;
        pxCurrentTask->xExecTime = 0;
        #if ( schedUSE_POLLING_SERVER == 1 )
            if( pdTRUE == pxCurrentTask->xIsPeriodicServer )
            {
                pxCurrentTask->xAbsoluteDeadline =
                    pxCurrentTask->xAbsoluteUnblockTime +
                    pxCurrentTask->xRelativeDeadline;
                #if ( schedEDF_EFFICIENT == 1 )
                    listSET_LIST_ITEM_VALUE( &pxCurrentTask->xTCBListItem,
                                             pxCurrentTask->xAbsoluteDeadline );
                #endif /* schedEDF_EFFICIENT */
            }
        #endif /* schedUSE_POLLING_SERVER */
        BaseType_t xHigherPriorityTaskWoken;
        vTaskNotifyGiveIndexedFromISR( xSchedulerHandle, 0, &xHigherPriorityTaskWoken );
        schedYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

#endif /* schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME */


#if ( schedUSE_SCHEDULER_TASK == 1 )

    /* Called by the scheduler task. Checks all tasks for any enabled
     * Timing Error Detection feature. */
    static void prvSchedulerCheckTimingError( TickType_t xTickCount,
                                              SchedTCB_t * pxTCB )
    {
        #if ( schedUSE_TCB_ARRAY == 1 )
            if( pdFALSE == pxTCB->xInUse )
            {
                return;
            }
        #endif

        #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
            #if ( schedUSE_POLLING_SERVER == 1 )
                /* If the task is periodic server, do not check deadline. */
                if( pdTRUE == pxTCB->xIsPeriodicServer )
                {
                    #if ( schedUSE_SPORADIC_JOBS == 1 )
                        prvCheckSporadicJobDeadline( xTickCount );
                    #endif /* schedUSE_SPORADIC_JOBS */
                }
                else
            #endif /* schedUSE_POLLING_SERVER */
            {
                /* Since lastWakeTime is updated to next wake time when the task is delayed, tickCount > lastWakeTime implies that
                 * the task has not finished it's job this period. */

                /* Using ICTOH method proposed by Carlini and Buttazzo, to check the condition unaffected by counter overflows. */
                if( ( signed ) ( xTickCount - pxTCB->xLastWakeTime ) > 0 )
                {
                    pxTCB->eState = eTaskReady;
                }

                prvCheckDeadline( pxTCB, xTickCount );
            }
        #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */


        #if ( schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 )
            if( pxTCB->eState == eTaskWCETExceeded )
            {
                vTaskSuspend( *pxTCB->pxTaskHandle );
                pxTCB->eState = eTaskDeferred;
            }

            if( pxTCB->eState == eTaskDeferred )
            {
                /* Using ICTOH method proposed by Carlini and Buttazzo, to check whether absolute unblock time is reached. */
                if( ( signed ) ( pxTCB->xAbsoluteUnblockTime - xTickCount ) <= 0 )
                {
                    pxTCB->eState = eTaskReady;
                    vTaskResume( *pxTCB->pxTaskHandle );
                }
            }
        #endif /* schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME */

        return;
    }

    static inline void prvEDFSchedCycle( void )
    {
        #if ( schedEDF_NAIVE == 1 )
            prvUpdatePrioritiesEDF();
        #elif ( schedEDF_EFFICIENT == 1 )
            if( pdTRUE == xSwitchToSchedulerOnBlock )
            {
                if( NULL != pxPreviousTCB )
                {
                    vTaskPrioritySet( *pxPreviousTCB->pxTaskHandle,
                                      schedPRIORITY_NOT_RUNNING );
                    pxPreviousTCB->uxPriority = schedPRIORITY_NOT_RUNNING;
                }

                if( !listLIST_IS_EMPTY( pxTCBReadyList ) )
                {
                    ListItem_t * pxListItem = listGET_HEAD_ENTRY( pxTCBReadyList );
                    pxCurrentTCB = listGET_LIST_ITEM_OWNER( pxListItem );

                    if( NULL != *pxCurrentTCB->pxTaskHandle )
                    {
                        vTaskPrioritySet( *pxCurrentTCB->pxTaskHandle,
                                          schedPRIORITY_RUNNING );
                        pxCurrentTCB->uxPriority = schedPRIORITY_RUNNING;
                    }
                }

                xSwitchToSchedulerOnBlock = pdFALSE;
            }
            else if( pdTRUE == xSwitchToSchedulerOnSuspend )
            {
                if( NULL != *pxPreviousTCB->pxTaskHandle )
                {
                    vTaskPrioritySet( *pxPreviousTCB->pxTaskHandle,
                                      schedPRIORITY_NOT_RUNNING );
                    pxPreviousTCB->uxPriority = schedPRIORITY_NOT_RUNNING;
                }

                if( !listLIST_IS_EMPTY( pxTCBReadyList ) )
                {
                    ListItem_t * pxListItem = listGET_HEAD_ENTRY( pxTCBReadyList );
                    pxCurrentTCB = listGET_LIST_ITEM_OWNER( pxListItem );

                    if( NULL != *pxCurrentTCB->pxTaskHandle )
                    {
                        vTaskPrioritySet( *pxCurrentTCB->pxTaskHandle,
                                          schedPRIORITY_RUNNING );
                        pxCurrentTCB->uxPriority = schedPRIORITY_RUNNING;
                    }
                }

                xSwitchToSchedulerOnSuspend = pdFALSE;
            }
            else if( pdTRUE == xSwitchToSchedulerOnReady )
            {
                if( NULL != pxPreviousTCB )
                {
                    vTaskPrioritySet( *pxPreviousTCB->pxTaskHandle,
                                      schedPRIORITY_NOT_RUNNING );
                    pxPreviousTCB->uxPriority = schedPRIORITY_NOT_RUNNING;
                }

                if( NULL != pxCurrentTCB )
                {
                    vTaskPrioritySet( *pxCurrentTCB->pxTaskHandle,
                                      schedPRIORITY_RUNNING );
                    pxCurrentTCB->uxPriority = schedPRIORITY_RUNNING;
                }

                xSwitchToSchedulerOnReady = pdFALSE;
            }
        #endif /* schedEDF_EFFICIENT */
    }

    /* NOTE: Function code for the scheduler task. */
    static void prvSchedulerFunction( void )
    {
        for( ; ; )
        {
            #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
                prvEDFSchedCycle();
            #endif /* schedSCHEDULING_POLICY_EDF */

            /* check timing error */
            #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 || \
                  schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 )
                TickType_t xTickCount = xTaskGetTickCount();
                SchedTCB_t * pxTCB;

                #if ( schedUSE_TCB_ARRAY == 1 )
                    BaseType_t xIndex;

                    for( xIndex = 0; xIndex < schedMAX_NUMBER_OF_PERIODIC_TASKS; xIndex++ )
                    {
                        pxTCB = &xTCBArray[ xIndex ];
                        prvSchedulerCheckTimingError( xTickCount, pxTCB );
                    }
                #elif ( schedUSE_TCB_SORTED_LIST == 1 )
                    const ListItem_t * pxTCBListEndMarker = listGET_END_MARKER( pxTCBList );
                    ListItem_t * pxTCBListItem = listGET_HEAD_ENTRY( pxTCBList );

                    while( pxTCBListItem != pxTCBListEndMarker )
                    {
                        pxTCB = listGET_LIST_ITEM_OWNER( pxTCBListItem );

                        prvSchedulerCheckTimingError( xTickCount, pxTCB );

                        pxTCBListItem = listGET_NEXT( pxTCBListItem );
                    }
                #endif /* schedUSE_TCB_SORTED_LIST */
            #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE || schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME */

            /* spin additional (void*)(void) functions that maintains the system state (e.g. I/O status check) */
            vSchedSpinAlong();

            /* NOTE: scheduler is waiting for a notification at signal id 0 */
            ulTaskNotifyTakeIndexed( 0, pdTRUE, portMAX_DELAY );
        }
    }

    /* Creates the scheduler task. */
    static void prvCreateSchedulerTask( void )
    {
        xTaskCreate(
            ( TaskFunction_t ) prvSchedulerFunction,
            "Scheduler",
            schedSCHEDULER_TASK_STACK_SIZE,
            NULL,
            schedSCHEDULER_PRIORITY,
            &xSchedulerHandle );
    }
#endif /* schedUSE_SCHEDULER_TASK */


#if ( schedUSE_APERIODIC_JOBS == 1 )

    /* Returns AJCB of first aperiodic job stored in FIFO. Returns NULL if
     * the FIFO is empty. */
    static AJCB_t * prvGetNextAperiodicJob( void )
    {
        /* If FIFO is empty. */
        if( 0 == uxAperiodicJobCounter )
        {
            return NULL;
        }

        AJCB_t * pxReturnValue = &xAJCBFifo[ xAJCBFifoHead ];

        /* Move FIFO head to next element in the queue. */
        xAJCBFifoHead++;

        if( schedMAX_NUMBER_OF_APERIODIC_JOBS == xAJCBFifoHead )
        {
            xAJCBFifoHead = 0;
        }

        #if ( schedUSE_SPORADIC_JOBS == 1 )
            /* Update maximum response time which is needed for acceptance test
             * for sporadic jobs. */
            if( schedPOLLING_SERVER_MAX_EXECUTION_TIME >= pxReturnValue->xMaxExecTime )
            {
                xAbsolutePreviousMaxResponseTime = xTaskGetTickCount() +
                                                   schedPOLLING_SERVER_PERIOD * 2;
            }
            else
            {
                xAbsolutePreviousMaxResponseTime = xTaskGetTickCount() +
                                                   schedPOLLING_SERVER_PERIOD +
                                                   ( pxReturnValue->xMaxExecTime / schedPOLLING_SERVER_MAX_EXECUTION_TIME + 1 )
                                                   * schedPOLLING_SERVER_PERIOD;
            }
        #endif /* schedUSE_SPORADIC_JOBS */

        return pxReturnValue;
    }

    /* Find index for an empty entry in xAJCBArray. Returns -1 if there is
     * no empty entry. */
    static BaseType_t prvFindEmptyElementIndexAJCB( void )
    {
        /* If the FIFO is full. */
        if( schedMAX_NUMBER_OF_APERIODIC_JOBS == uxAperiodicJobCounter )
        {
            return -1;
        }

        BaseType_t xReturnValue = xAJCBFifoTail;

        /* Extend the FIFO tail. */
        xAJCBFifoTail++;

        if( schedMAX_NUMBER_OF_APERIODIC_JOBS == xAJCBFifoTail )
        {
            xAJCBFifoTail = 0;
        }

        return xReturnValue;
    }

    /* Creates an aperiodic job. */
    void vSchedulerAperiodicJobCreate( TaskFunction_t pvTaskCode,
                                       const char * pcName,
                                       void * pvParameters,
                                       TickType_t xMaxExecTimeTick )
    {
        taskENTER_CRITICAL();
        BaseType_t xIndex = prvFindEmptyElementIndexAJCB();

        if( -1 == xIndex )
        {
            /* The AJCBFifo is full. */
            taskEXIT_CRITICAL();
            return;
        }

        configASSERT( uxAperiodicJobCounter < schedMAX_NUMBER_OF_APERIODIC_JOBS );
        AJCB_t * pxNewAJCB = &xAJCBFifo[ xIndex ];

        /* Add item to AJCBList. */
        *pxNewAJCB = ( AJCB_t ) {
            .pvTaskCode = pvTaskCode,
            .pcName = pcName,
            .pvParameters = pvParameters,
            .xMaxExecTime = xMaxExecTimeTick,
            .xExecTime = 0,
        };

        uxAperiodicJobCounter++;
        taskEXIT_CRITICAL();
    }
#endif /* schedUSE_APERIODIC_JOBS */

#if ( schedUSE_SPORADIC_JOBS == 1 )

    /* Returns SJCB of first sporadic job stored in FIFO. Returns NULL if
     * the FIFO is empty. */
    static SJCB_t * prvGetNextSporadicJob( void )
    {
        /* If FIFO is empty. */
        if( 0 == uxSporadicJobCounter )
        {
            return NULL;
        }

        SJCB_t * pxReturnValue = &xSJCBFifo[ xSJCBFifoHead ];

        /* Move FIFO head to next element in the queue. */
        xSJCBFifoHead++;

        if( schedMAX_NUMBER_OF_SPORADIC_JOBS == xSJCBFifoHead )
        {
            xSJCBFifoHead = 0;
        }

        return pxReturnValue;
    }

    /* Find index for an empty entry in xSJCBArray. Returns -1 if there is
     * no empty entry. */
    static BaseType_t prvFindEmptyElementIndexSJCB( void )
    {
        /* If the FIFO is full. */
        if( schedMAX_NUMBER_OF_SPORADIC_JOBS == uxSporadicJobCounter )
        {
            return -1;
        }

        BaseType_t xReturnValue = xSJCBFifoTail;

        /* Extend the FIFO tail. */
        xSJCBFifoTail++;

        if( schedMAX_NUMBER_OF_SPORADIC_JOBS == xSJCBFifoTail )
        {
            xSJCBFifoTail = 0;
        }

        return xReturnValue;
    }

    /* Called from xSchedulerSporadicJobCreate. Analyzes if the given sporadic
     * job is (guaranteed) schedulable. Return pdTRUE if the sporadic job can
     * meet it's deadline (with guarantee), otherwise pdFALSE. */
    static BaseType_t prvAnalyzeSporadicJobSchedulability( SJCB_t * pxSporadicJob,
                                                           TickType_t xTickCount )
    {
        TickType_t xRelativeMaxResponseTime;

        if( xAbsolutePreviousMaxResponseTime > xTickCount )
        {
            /* Using ICTOH method to calculate relative max response time. */
            xRelativeMaxResponseTime = xAbsolutePreviousMaxResponseTime - xTickCount;
        }
        else
        {
            xRelativeMaxResponseTime = 0;
            xAbsolutePreviousMaxResponseTime = 0;
        }

        if( schedPOLLING_SERVER_MAX_EXECUTION_TIME >= pxSporadicJob->xMaxExecTime )
        {
            xRelativeMaxResponseTime += schedPOLLING_SERVER_PERIOD * 2;
        }
        else
        {
            xRelativeMaxResponseTime += schedPOLLING_SERVER_PERIOD +
                                        ( pxSporadicJob->xMaxExecTime / schedPOLLING_SERVER_MAX_EXECUTION_TIME + 1 )
                                        * schedPOLLING_SERVER_PERIOD;
        }

        if( xRelativeMaxResponseTime < pxSporadicJob->xRelativeDeadline )
        {
            /* Accept job. */
            xAbsolutePreviousMaxResponseTime = xRelativeMaxResponseTime + xTickCount;
            return pdTRUE;
        }
        else
        {
            /* Do not accept job. */
            return pdFALSE;
        }
    }

    /* Creates a sporadic job if it is schedulable. Returns pdTRUE if the
     * sporadic job can meet it's deadline (with guarantee) and created,
     * otherwise pdFALSE. */
    BaseType_t xSchedulerSporadicJobCreate( TaskFunction_t pvTaskCode,
                                            const char * pcName,
                                            void * pvParameters,
                                            TickType_t xMaxExecTimeTick,
                                            TickType_t xDeadlineTick )
    {
        taskENTER_CRITICAL();
        BaseType_t xAccept = pdFALSE;
        BaseType_t xIndex = prvFindEmptyElementIndexSJCB();
        TickType_t xTickCount = xTaskGetTickCount();

        if( -1 == xIndex )
        {
            /* The SJCBFifo is full. */
            taskEXIT_CRITICAL();
            return xAccept;
        }

        configASSERT( uxSporadicJobCounter < schedMAX_NUMBER_OF_SPORADIC_JOBS );
        SJCB_t * pxNewSJCB = &xSJCBFifo[ xIndex ];

        /* Add item to SJCBList. */
        *pxNewSJCB = ( SJCB_t ) {
            .pvTaskCode = pvTaskCode,
            .pcName = pcName,
            .pvParameters = pvParameters,
            .xRelativeDeadline = xDeadlineTick,
            .xMaxExecTime = xMaxExecTimeTick,
            .xExecTime = 0
        };

        #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
            pxNewSJCB->xAbsoluteDeadline = pxNewSJCB->xRelativeDeadline + xTickCount;
        #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */

        xAccept = prvAnalyzeSporadicJobSchedulability( pxNewSJCB, xTickCount );

        if( pdTRUE == xAccept )
        {
            uxSporadicJobCounter++;
        }
        else
        {
            if( xSJCBFifoTail == 0 )
            {
                xSJCBFifoTail = schedMAX_NUMBER_OF_SPORADIC_JOBS - 1;
            }
            else
            {
                xSJCBFifoTail--;
            }
        }

        taskEXIT_CRITICAL();
        return xAccept;
    }
#endif /* schedUSE_SPORADIC_JOBS */

#if ( schedUSE_POLLING_SERVER == 1 )
    /* Function code for the Polling Server. */
    static void prvPollingServerFunction( void )
    {
        for( ; ; )
        {
            #if ( schedUSE_SPORADIC_JOBS == 1 )
                pxCurrentSporadicJob = prvGetNextSporadicJob();

                if( NULL == pxCurrentSporadicJob )
                {
                    /* No ready sporadic job in the queue. */
                    #if ( schedUSE_APERIODIC_JOBS != 1 )
                        return;
                    #endif /* schedUSE_APERIODIC_JOBS */
                }
                else
                {
                    /* NOTE: Run sporadic job */
                    pxCurrentSporadicJob->pvTaskCode(
                        pxCurrentSporadicJob->pvParameters );

                    SchedTCB_t * pxThisTask = prvGetTCBFromHandle(
                        xTaskGetCurrentTaskHandle() );
                    /* printf( "PS finished sporadic job tick %d abs deadline %d\r\n", xTaskGetTickCount(), pxThisTask->xAbsoluteDeadline, ); */
                    uxSporadicJobCounter--;

                    if( 0 == uxSporadicJobCounter )
                    {
                        xAbsolutePreviousMaxResponseTime = 0;
                    }

                    continue;
                }
            #endif /* schedUSE_SPORADIC_JOBS */

            #if ( schedUSE_APERIODIC_JOBS == 1 )
                pxCurrentAperiodicJob = prvGetNextAperiodicJob();

                if( NULL == pxCurrentAperiodicJob )
                {
                    /* No ready aperiodic job in the queue. */
                    return;
                }
                else
                {
                    /* NOTE: Run aperiodic job */
                    pxCurrentAperiodicJob->pvTaskCode( pxCurrentAperiodicJob->pvParameters );
                    uxAperiodicJobCounter--;
                    #if ( schedUSE_SPORADIC_JOBS == 1 )
                        xAbsolutePreviousMaxResponseTime = 0;
                    #endif /* schedUSE_SPORADIC_JOBS */
                }
            #endif /* schedUSE_APERIODIC_JOBS */
        }
    }

    /* Creates Polling Server as a periodic task. */
    void prvPollingServerCreate( void )
    {
        vSchedulerPeriodicTaskCreate(
            ( TaskFunction_t ) prvPollingServerFunction,
            "PollingServer",
            schedPOLLING_SERVER_STACK_SIZE,
            NULL,
            #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_MANUAL )
                schedPOLLING_SERVER_PRIORITY,
            #else
                0, /* use lowest priority */
            #endif
            &xPollingServerHandle,
            0,
            schedPOLLING_SERVER_PERIOD,
            schedPOLLING_SERVER_MAX_EXECUTION_TIME,
            schedPOLLING_SERVER_DEADLINE
            );
        /* this is the polling server task */
        SchedTCB_t * tcb = prvGetTCBFromHandle( xPollingServerHandle );
        tcb->xIsPeriodicServer = pdTRUE;
        #if ( schedPERIODIC_TASK_AUTOTART == 0 )
            vTaskResume( *( tcb->pxTaskHandle ) );
        #endif
    }

    /*
     * void vPollingServerDelete( void )
     * {
     *  vPeriodicTaskDelete( xPollingServerHandle );
     * }
     */

#endif /* schedUSE_POLLING_SERVER */

#if ( schedUSE_SCHEDULER_TASK == 1 )
    /* Wakes up (context switches to) the scheduler task. */
    static void prvWakeScheduler( void )
    {
        BaseType_t xHigherPriorityTaskWoken;

        /* NOTE: this wakes up the scheduler prvSchedulerFunction() at signal id 0 */
        vTaskNotifyGiveIndexedFromISR( xSchedulerHandle, 0, &xHigherPriorityTaskWoken );
        /* yield all other tasks with higher priority */
        schedYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    /* Called every timer tick (timer interrupt -> xTaskIncrementTick).
     * Requires: configUSE_TICK_HOOK == 1 */
    void vApplicationTickHook( void )
    {
        TaskHandle_t xCurrentTaskHandle = xTaskGetCurrentTaskHandle();

        SchedTCB_t * pxCurrentTask = prvGetTCBFromHandle( xCurrentTaskHandle );

        if( ( NULL != pxCurrentTask ) &&
            ( xCurrentTaskHandle != xSchedulerHandle ) &&
            ( xCurrentTaskHandle != xTaskGetIdleTaskHandle() ) )
        {
            pxCurrentTask->xExecTime++;
            #if ( schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME == 1 )
                if( pxCurrentTask->xMaxExecTime <= pxCurrentTask->xExecTime )
                {
                    if( pdFALSE == pxCurrentTask->xMaxExecTimeExceeded )
                    {
                        if( pdFALSE == pxCurrentTask->xSuspended )
                        {
                            prvExecTimeExceedHook( xTaskGetTickCountFromISR(),
                                                   pxCurrentTask );
                        }
                    }
                }
            #endif /* schedUSE_TIMING_ERROR_DETECTION_EXECUTION_TIME */
        }

        #if ( schedUSE_TIMING_ERROR_DETECTION_DEADLINE == 1 )
            /* unless specified, scheduler task is woken up every timer tick. */
            #ifdef schedSCHEDULER_TASK_PERIOD
                xSchedulerWakeCounter = ( xSchedulerWakeCounter + 1 ) % schedSCHEDULER_TASK_PERIOD;

                if( xSchedulerWakeCounter == 0 )
            #endif
            {
                prvWakeScheduler();
            }
        #endif /* schedUSE_TIMING_ERROR_DETECTION_DEADLINE */
    }
#endif /* schedUSE_SCHEDULER_TASK */

/* This function must be called before any other function call from this module. */
void vSchedulerInit( void )
{
    #if ( schedUSE_TCB_ARRAY == 1 )
        prvInitTCBArray();
    #elif ( schedUSE_TCB_SORTED_LIST == 1 )
        vListInitialise( &xTCBList );
        pxTCBList = &xTCBList;
        #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
            #if ( schedEDF_NAIVE == 1 )
                /* vListInitialise( &xTCBList ); */
                vListInitialise( &xTCBTempList );
                vListInitialise( &xTCBOverflowedList );
                /* pxTCBList = &xTCBList; */
                pxTCBTempList = &xTCBTempList;
                pxTCBOverflowedList = &xTCBOverflowedList;
            #elif ( schedEDF_EFFICIENT == 1 )
                /* vListInitialise( &xTCBListAll ); */
                vListInitialise( &xTCBBlockedList );
                vListInitialise( &xTCBReadyList );
                vListInitialise( &xTCBOverflowedReadyList );
                /* pxTCBListAll = &xTCBListAll; */
                pxTCBBlockedList = &xTCBBlockedList;
                pxTCBReadyList = &xTCBReadyList;
                pxTCBOverflowedReadyList = &xTCBOverflowedReadyList;
            #endif /* schedEDF_NAIVE */
        #endif /* if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF ) */
    #endif /* schedUSE_TCB_ARRAY */
}

/* Starts scheduling tasks. All periodic tasks (including polling server) must
 * have been created with API function before calling this function. */
void vSchedulerStart( void )
{
    #if ( schedUSE_SCHEDULER_TASK == 1 )
        prvCreateSchedulerTask();
    #endif /* schedUSE_SCHEDULER_TASK */

    /*
     #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_RMS || \
     *      schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_DMS )
     *  prvSetFixedPriorities();
     #elif ...
     */
    #if ( schedSCHEDULING_POLICY == schedSCHEDULING_POLICY_EDF )
        prvInitEDF();
    #endif /* schedSCHEDULING_POLICY */

    /* prvCreateAllTasks(); */

    xSystemStartTime = xTaskGetTickCount();
    vTaskStartScheduler();

    #if ( schedUSE_POLLING_SERVER == 1 )
        prvPollingServerCreate();
    #endif /* schedUSE_POLLING_SERVER */

    vUSleepCalibrate();
}

TaskHandle_t pxGetTaskHandleByName( const char * name )
{
    TaskHandle_t task = NULL;
    SchedTCB_t * tcb;

    #if ( schedUSE_TCB_ARRAY == 1 )
        static BaseType_t xIndex = 0;
        BaseType_t xIterator;

        for( xIterator = 0; xIterator < schedMAX_NUMBER_OF_PERIODIC_TASKS;
             xIterator++ )
        {
            tcb = &( xTCBArray[ xIndex ] );

            if( ( pdTRUE == tcb->xInUse ) &&
                ( strcmp( tcb->pcName, name ) == 0 ) )
            {
                task = *( tcb->pxTaskHandle );
                break;
            }

            xIndex++;

            if( schedMAX_NUMBER_OF_PERIODIC_TASKS == xIndex )
            {
                xIndex = 0;
            }
        }
    #elif ( schedUSE_TCB_SORTED_LIST == 1 )
        const ListItem_t * pxTCBListEndMarker = listGET_END_MARKER( pxTCBList );
        ListItem_t * pxTCBListItem;

        for( pxTCBListItem = listGET_HEAD_ENTRY( pxTCBList );
             pxTCBListItem != pxTCBListEndMarker;
             pxTCBListItem = listGET_NEXT( pxTCBListItem ) )
        {
            tcb = listGET_LIST_ITEM_OWNER( pxTCBListItem );

            if( strcmp( tcb->pcName, name ) == 0 )
            {
                task = *( tcb->pxTaskHandle );
                break;
            }
        }
    #endif /* if ( schedUSE_TCB_ARRAY == 1 ) */

    return task;
}

void vPrintAllTaskNames()
{
}
