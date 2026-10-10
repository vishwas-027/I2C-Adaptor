#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* Cortex-M4 implements 4 priority bits */
#define __NVIC_PRIO_BITS                     4U

/*-----------------------------------------------------------*/
/* Clock configuration */

#define configCPU_CLOCK_HZ                   ( ( uint32_t ) 84000000 )
#define configTICK_RATE_HZ                   ( ( TickType_t ) 1000 )

/*-----------------------------------------------------------*/
/* Scheduler */

#define configUSE_PREEMPTION                 1
#define configUSE_TIME_SLICING               1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

#define configMAX_PRIORITIES                 5

#define configMINIMAL_STACK_SIZE             128
#define configTOTAL_HEAP_SIZE                (20 * 1024)

#define configMAX_TASK_NAME_LEN              16

/*-----------------------------------------------------------*/
/* Memory */

#define configSUPPORT_STATIC_ALLOCATION      0
#define configSUPPORT_DYNAMIC_ALLOCATION     1

/*-----------------------------------------------------------*/
/* Tick */

#define configTICK_TYPE_WIDTH_IN_BITS        TICK_TYPE_WIDTH_32_BITS

/*-----------------------------------------------------------*/
/* Hooks */

#define configUSE_IDLE_HOOK                  0
#define configUSE_TICK_HOOK                  0

/*-----------------------------------------------------------*/
/* Synchronization */

#define configUSE_MUTEXES                    1
#define configUSE_COUNTING_SEMAPHORES        1
#define configUSE_RECURSIVE_MUTEXES          1

/*-----------------------------------------------------------*/
/* Timers */

#define configUSE_TIMERS                     0

/*-----------------------------------------------------------*/
/* Interrupt priorities */

#define configKERNEL_INTERRUPT_PRIORITY \
    (15 << (8 - __NVIC_PRIO_BITS))

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (5 << (8 - __NVIC_PRIO_BITS))

/*-----------------------------------------------------------*/
/* API */

#define INCLUDE_vTaskDelay                   1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskSuspend                 1
#define INCLUDE_xTaskGetSchedulerState       1


#define configASSERT(x)                     \
    if ((x) == 0)                           \
    {                                       \
        taskDISABLE_INTERRUPTS();           \
        for (;;)                            \
        {                                   \
        }                                   \
    }
#endif
