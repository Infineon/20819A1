/***************************************************************************//**
 * \file <wiced_bt_rtos.h>
 *
 * Provides application-level access to RTOS functionality contained in ROM.
 *
 *//*****************************************************************************
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

#ifndef WICED_BT_RTOS_H__
#define WICED_BT_RTOS_H__

#include "wiced_result.h"
#include "wiced_bt_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup  rtos RTOS
 * \ingroup     rtos
 * \{
 *
 * \defgroup group_rtos_macros Macros
 * \defgroup group_rtos_globals Global Variables
 * \defgroup group_rtos_data_structures Data Structures
 * \defgroup group_rtos_enums Enumerated Types
 * \defgroup group_rtos_functions Functions
 * \{
 *     \defgroup group_rtos_functions_thread Thread
 *     \defgroup group_rtos_functions_semaphore Semaphore
 *     \defgroup group_rtos_functions_mutex Mutex
 *     \defgroup group_rtos_functions_queue Queue
 *     \defgroup group_rtos_functions_worker Worker Thread
 *     \defgroup group_rtos_functions_evtflag Event Flag
 * \}
 *
 */

/******************************************************************************
 * Macro definitions                                                          *
 ******************************************************************************/

/**
 * \addtogroup group_rtos_macros
 * \{
 */

#define WICED_NO_WAIT           0 /**< Do not wait if queue/semaphore empty */
#define WICED_WAIT_FOREVER      ((uint32_t) 0xFFFFFFFF) /**< Wait infinitely */

#define THREAD_PRIORITY_MIN     0 /**< Minimum value for the thread priority */
#define THREAD_PRIORITY_MAX     7 /**< Maximum value for the thread priority */

/** \} group_rtos_macros */


/******************************************************************************
 * Global Enumerations definitions                                            *
 ******************************************************************************/

/**
 * \addtogroup group_rtos_enums
 * \{
 */

/**
 * Indicates under which scenario to trigger event flag callback. Used by
 * \ref wiced_rtos_wait_for_event_flags.
 */
typedef enum
{
    WAIT_FOR_ANY_EVENT,    /**< Trigger callback if any OR'd event occurs */
    WAIT_FOR_ALL_EVENTS,   /**< Trigger callback only if exact flag match */
} wiced_event_flags_wait_option_t;

/** Sleep thread or keep active when using \ref wiced_rtos_thread_delay */
typedef enum
{
    ALLOW_THREAD_TO_SLEEP, /**<  Allow the current running thread to sleep */
    KEEP_THREAD_ACTIVE,    /**<  Keep the current running thread active */
} wiced_delay_type_t;

/** \} group_rtos_enums */

/******************************************************************************
 * Global Data Structure definitions                                          *
 ******************************************************************************/

/**
 * \addtogroup group_rtos_data_structures
 * \{
 */

typedef struct _wiced_thread_t        wiced_thread_t;
typedef struct _wiced_queue_t         wiced_queue_t;
typedef struct _wiced_semaphore_t     wiced_semaphore_t;
typedef struct _wiced_mutex_t         wiced_mutex_t;
typedef struct _wiced_event_flags_t   wiced_event_flags_t;
typedef struct _wiced_worker_thread_t wiced_worker_thread_t;

/*******************************************************************************
 * Callback Name: wiced_thread_function_t
 ****************************************************************************//**
 *
 * Callback registered using \ref wiced_rtos_init_thread and used as the main
 * function of the newly created thread.
 *
 * \param arg                         argument passed when registering the thread
 *
 * \return void
 *
 *******************************************************************************/
typedef void ( *wiced_thread_function_t )(uint32_t arg);

/*******************************************************************************
 * Callback Name: event_handler_t
 ****************************************************************************//**
 *
 * Used to enqueue a task on a worker thread using the API \ref
 * wiced_rtos_send_asynchronous_event. This callback will be executed once the
 * worker thread is done executing the preceding tasks and pops it off the queue.
 *
 * \param arg                   void pointer passed when registering the callback
 *
 * \return wiced_result_t
 *
 *******************************************************************************/
typedef wiced_result_t ( *event_handler_t )(void *arg);

/** \} group_rtos_data_structures */


/******************************************************************************
 * Global variables                                                          *
 ******************************************************************************/

/**
 * \addtogroup group_rtos_globals
 * \{
 */

extern wiced_worker_thread_t wiced_hardware_io_worker_thread;
extern wiced_worker_thread_t wiced_networking_worker_thread;

/** \} group_rtos_globals */


/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/**
 * \addtogroup group_rtos_functions
 * \{
 */

/**
 * \addtogroup group_rtos_functions_thread
 * \{
 *
 * Defines a group of APIs, which interface to RTOS multithreading
 * functionality contained in ROM.
 *
 */

/*******************************************************************************
 * Function Name: wiced_rtos_create_thread
 ****************************************************************************//**
 *
 * Allocates memory for a new thread instance and returns the pointer.
 *
 * \param void
 *
 * \return
 *  - NULL an error occured and the instance was not allocated, do not init
 *  - valid pointer to thread instance, which can now we initialized
 *
 *******************************************************************************/
wiced_thread_t *wiced_rtos_create_thread(void);

/*******************************************************************************
 * Function Name: wiced_rtos_init_thread
 ****************************************************************************//**
 *
 * Initializes the thread instance created by \ref wiced_rtos_create_thread .
 * Thread is then started in callback 'function' using given parameters.
 *
 * \param[in] thread         pointer to thread instance to wait for
 * \param[in] priority       THREAD_PRIORITY_MIN=0 to THREAD_PRIORITY_MAX=7
 * \param[in] name           ASCII name for the thread (NULL is allowed)
 * \param[in] function       main thread function
 * \param[in] stack_size     num bytes allocated for thread stack
 * \param[in] arg            argument which will be passed to thread function
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_init_thread(wiced_thread_t *thread,
                                      uint8_t priority,
                                      const char *name,
                                      wiced_thread_function_t function,
                                      uint32_t stack_size,
                                      void *arg);

/*******************************************************************************
 * Function Name: wiced_rtos_delay_milliseconds
 ****************************************************************************//**
 *
 * Causes the current thread to block for AT LEAST the specified number of
 * milliseconds. If the processor is heavily loaded with higher priority tasks,
 * the delay may be much longer than requested.
 *
 * \param[in] milliseconds   milliseconds to delay current thread
 * \param[in] delay_type     ALLOW_THREAD_TO_SLEEP or KEEP_THREAD_ACTIVE
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_delay_milliseconds(uint32_t milliseconds, wiced_delay_type_t delay_type);

/*******************************************************************************
 * Function Name: wiced_rtos_delay_microseconds
 ****************************************************************************//**
 *
 * Causes the current thread to block for AT LEAST the specified number of
 * microseconds. If the processor is heavily loaded with higher priority tasks,
 * the delay may be much longer than requested.
 *
 * \param[in] microseconds   microseconds to delay current thread
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 * \note
 * All threads with equal or lower priority than the current thread
 * will not be able to run while the delay is occurring.
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_delay_microseconds(uint32_t microseconds);

/*******************************************************************************
 * Function Name: wiced_rtos_thread_join
 ****************************************************************************//**
 *
 * Causes the current thread to sleep until the specified other thread
 * has terminated. If the processor is heavily loaded with higher priority
 * tasks, this thread may not wake until significantly after the thread
 * termination.
 *
 * \param[in] thread         pointer to thread instance to wait for
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_thread_join(wiced_thread_t *thread);

/*******************************************************************************
 * Function Name: wiced_rtos_thread_force_awake
 ****************************************************************************//**
 *
 * Causes the specified thread to wake from suspension. This will usually
 * cause an error or timeout in that thread, since the task it was waiting on
 * is not complete.
 *
 * \param[in] thread         pointer to thread instance
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_thread_force_awake(wiced_thread_t *thread);

/*******************************************************************************
 * Function Name: wiced_rtos_is_current_thread
 ****************************************************************************//**
 *
 * Checks if a specified thread is the currently running thread.
 *
 * \param[in] thread         pointer to thread instance
 *
 * \return
 *  - WICED_SUCCESS indicates that the current thread is that passed as param
 *  - WICED_ERROR indicates that the current thread is not that passed as param
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_is_current_thread(wiced_thread_t *thread);

/*******************************************************************************
 * Function Name: wiced_rtos_check_for_stack_overflow
 ****************************************************************************//**
 *
 * Verifies if the stack of current thread is corrupted.
 *
 * \param void
 *
 * \return
 *  - WICED_TRUE
 *  - WICED_FALSE
 *
 *******************************************************************************/
wiced_bool_t wiced_rtos_check_for_stack_overflow(void);

/*******************************************************************************
 * Function Name: wiced_rtos_thread_stack_size
 ****************************************************************************//**
 *
 * Returns stack size if thread signature is valid, else returns 0.
 *
 * \param wiced_thread_t *
 *
 * \return
 *  - size as uint32_t
 *  - 0 if thread pointer is invalid
 *
 *******************************************************************************/
uint32_t wiced_rtos_thread_stack_size(wiced_thread_t *thread);

/*******************************************************************************
 * Function Name: wiced_bt_rtos_max_stack_use
 ****************************************************************************//**
 *
 * Returns maximum stack usage size if thread signature is valid, else returns 0
 *
 * @param[in]       thread   : point to the thread context
 *                             Note: thread is a return value from wiced_rtos_create_thread()
 *
 * @return          maximum stack usage
 ********************************************************************************/
uint32_t wiced_bt_rtos_max_stack_use(wiced_thread_t *thread);

/** \} group_rtos_functions_thread */


/**
 * \addtogroup group_rtos_functions_semaphore
 * \{
 *
 * Defines a group of APIs, which interface to RTOS semaphore functionality
 * contained in ROM.
 *
 * Semaphores are primarily utilized for synchronization between threads.
 * Asynchronous events can be triggered across threads because a semaphore
 * can be incremented or decremented by any thread.
 *
 * \note
 * This is set of APIs implements a counting semaphore.
 *
 */

/*******************************************************************************
 * Function Name: wiced_rtos_create_semaphore
 ****************************************************************************//**
 *
 * Allocates memory for a new semaphore instance and returns the pointer. Once
 * created, the semaphore must be initialized before use by calling
 * \ref wiced_rtos_init_semaphore .
 *
 * \param void
 *
 * \return
 *  - NULL an error occurred and the instance was not allocated, do not init
 *  - valid pointer to samphore instance, which can now we initialized
 *
 *******************************************************************************/
wiced_semaphore_t *wiced_rtos_create_semaphore(void);

/*******************************************************************************
 * Function Name: wiced_rtos_init_semaphore
 ****************************************************************************//**
 *
 * Initializes a counting semaphore created by \ref wiced_rtos_create_semaphore.
 *
 * \param[in] semaphore      pointer to semaphore instance
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_init_semaphore(wiced_semaphore_t *semaphore);

/*******************************************************************************
 * Function Name: wiced_rtos_set_semaphore
 ****************************************************************************//**
 *
 * Set (post/put/increment) a semaphore
 *
 * \param[in] semaphore      pointer to semaphore instance
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_set_semaphore(wiced_semaphore_t *semaphore);

/*******************************************************************************
 * Function Name: wiced_rtos_get_semaphore
 ****************************************************************************//**
 *
 * Attempts to get (wait/decrement) a semaphore. If semaphore is at zero already,
 * then the calling thread will be suspended until another thread sets the
 * semaphore with \ref wiced_rtos_set_semaphore
 *
 * \param[in] semaphore      pointer to semaphore instance
 * \param[in] timeout_ms     milliseconds to wait for semaphore
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_get_semaphore(wiced_semaphore_t *semaphore, uint32_t timeout_ms);

/** \} group_rtos_functions_semaphore */


/**
 * \addtogroup group_rtos_functions_mutex
 * \{
 *
 * Defines a group of APIs, which interface to RTOS mutex functionality.
 *
 */

/*******************************************************************************
 * Function Name: wiced_rtos_create_mutex
 ****************************************************************************//**
 *
 * Allocates memory for a new mutex instance and returns the pointer. Once
 * created, the mutex must be initialized before use by calling
 * \ref wiced_rtos_init_mutex .
 *
 * \param void
 *
 * \return
 *  - NULL an error occurred and the instance was not allocated, do not init
 *  - valid pointer to mutex instance, which can now we initialized
 *
 *******************************************************************************/
wiced_mutex_t *wiced_rtos_create_mutex(void);

/*******************************************************************************
 * Function Name: wiced_rtos_init_mutex
 ****************************************************************************//**
 *
 * Initialize the mutex created by \ref wiced_rtos_create_mutex .
 *
 * \param[in] mutex      pointer to mutex instance
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_init_mutex(wiced_mutex_t *mutex);

/*******************************************************************************
 * Function Name: wiced_rtos_lock_mutex
 ****************************************************************************//**
 *
 * Lock the given mutex. Blocks other threads from proceeding until the
 * current thread releases the lock. If another thread already holds the
 * lock, then the current thread will be suspended until released.
 *
 * \param[in] mutex      pointer to mutex instance
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_lock_mutex(wiced_mutex_t *mutex);

/*******************************************************************************
 * Function Name: wiced_rtos_unlock_mutex
 ****************************************************************************//**
 *
 * Release lock on mutex. Other threads waiting for the mutex can now
 * enter the critical section.
 *
 * \param[in] mutex      pointer to mutex instance
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_unlock_mutex(wiced_mutex_t *mutex);

/** \} group_rtos_functions_mutex */


/**
 * \addtogroup group_rtos_functions_queue
 * \{
 *
 * Defines a group of APIs, which interface to RTOS queues functionality
 * contained in ROM.
 *
 * \note
 * The queue APIs will not function until \ref wiced_bt_stack_init is called
 * so that the proper buffer space is allocated at runtime.
 *
 */

/*******************************************************************************
 * Function Name: wiced_rtos_create_queue
 ****************************************************************************//**
 *
 * Allocates memory for a new queue instance and returns the pointer
 *
 * \param void
 *
 * \return
 *  - NULL an error occurred and the instance was not allocated, do not init
 *  - valid pointer to queue instance, which can now we initialized
 *
 *******************************************************************************/
wiced_queue_t *wiced_rtos_create_queue(void);

/*******************************************************************************
 * Function Name: wiced_rtos_init_queue
 ****************************************************************************//**
 *
 * Initializes queue instance created by \ref wiced_rtos_create_queue
 *
 * \param[in] queue                  pointer to queue instance
 * \param[in] name                   ASCII name for the queue (NULL is allowed)
 * \param[in] message_size           num bytes of each queue element
 * \param[in] number_of_messages     max number of elements in the queue
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 * Note : This API will create separate private pool so please increase max_number_of_buffer_pools (in wiced_bt_cfg_settings_t structure) by number of rtos queue created
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_init_queue(wiced_queue_t *queue,
                                     const char *name,
                                     uint32_t message_size,
                                     uint32_t number_of_messages);

/*******************************************************************************
 * Function Name: wiced_rtos_push_to_queue
 ****************************************************************************//**
 *
 * Enqueues an element. Memcpy's from the message pointer into the queue.
 *
 * \param[in] queue          pointer to queue instance
 * \param[out] message       pointer to queue element
 * \param[in] timeout_ms     Deprecated
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 * \note
 * Message parameter must have enough allocated memory to hold the length of
 * a queue element, which is specified in \ref wiced_rtos_init_queue .
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_push_to_queue(wiced_queue_t *queue, void *message, uint32_t timeout_ms /*DEPRECATED ARG*/);

/*******************************************************************************
 * Function Name: wiced_rtos_pop_from_queue
 ****************************************************************************//**
 *
 * Dequeues element and memcpy's it to given pointer. Can block for specified
 * period of time if queue is empty.
 *
 * \param[in] queue       pointer to queue instance
 * \param[out] message    pointer to which popped element will be memcpy'd
 * \param[in] timeout_ms  milliseconds to wait for queue to fill if empty
 *                        (this parameter is deprecated on 20819)
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 * \note
 * Message parameter must have enough allocated memory to hold the length of
 * popped element, which is specified in \ref wiced_rtos_init_queue .
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_pop_from_queue(wiced_queue_t *queue, void *message, uint32_t timeout_ms);

/*******************************************************************************
 * Function Name: wiced_rtos_is_queue_empty
 ****************************************************************************//**
 *
 * Returns boolean value indicating whether queue is empty
 *
 * \param[in] queue      pointer to queue instance
 *
 * \return
 *  - WICED_TRUE
 *  - WICED_FALSE
 *
 *******************************************************************************/
wiced_bool_t wiced_rtos_is_queue_empty(wiced_queue_t *queue);

/*******************************************************************************
 * Function Name: wiced_rtos_is_queue_full
 ****************************************************************************//**
 *
 * Returns boolean value indicating whether queue is full
 *
 * \param[in] queue      pointer to queue instance
 *
 * \return
 *  - WICED_TRUE
 *  - WICED_FALSE
 *
 *******************************************************************************/
wiced_bool_t wiced_rtos_is_queue_full(wiced_queue_t *queue);

/*******************************************************************************
 * Function Name: wiced_rtos_get_queue_occupancy
 ****************************************************************************//**
 *
 * Read number of elements enqueued
 *
 * \param[in] queue      pointer to queue instance
 * \param[out] count     pointer to integer for storing occupancy count

 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_get_queue_occupancy(wiced_queue_t *queue, uint32_t *count);

/** \} group_rtos_functions_queue */


/**
 * \addtogroup group_rtos_functions_worker
 * \{
 *
 * Defines a group of APIs which enable the use of worker threads. Each thread
 * can be created with its own stack size, priority, and task queue size. In
 * the work thread, the only way to execute code is to enqueue a new task and
 * have it execute a callback.
 *
 * \note
 * The worker thread APIs will not function until \ref wiced_bt_stack_init is
 * called so that the proper buffer space is allocated at runtime.
 *
 */

/*******************************************************************************
 * Function Name: wiced_rtos_create_worker_thread
 ****************************************************************************//**
 *
 * Allocates memory for a new worker thread instance and returns the pointer.
 * The created instance must be initialized using
 * \ref wiced_rtos_init_worker_thread before use.
 *
 * \param void
 *
 * \return
 *  - NULL an error occurred and the instance was not allocated, do not init
 *  - valid pointer to worker thread instance, which can now we initialized
 *
 *******************************************************************************/
wiced_worker_thread_t *wiced_rtos_create_worker_thread(void);

/*******************************************************************************
 * Function Name: wiced_rtos_init_worker_thread
 ****************************************************************************//**
 *
 * Initializes the worker thread instance that was allocated in
 * wiced_rtos_create_worker_thread. The worker thread creates a regular RTOS
 * thread with the given stack size and priority, as well as an RTOS queue.
 * The created thread blocks indefinitely until a task in enqueued. The task
 * is dequeued and executed in the worker thread context, then the thread loops
 * back around to wait for the next task.
 *
 * \param[in] worker_thread      worker thread instance to init
 * \param[in] priority           thread priority
 * \param[in] stack_size         thread's stack size in number of bytes
 * \param[in] event_queue_size   number of events the task queue can hold

 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 * \note
 * The number of buffers in \ref wiced_bt_cfg_settings_t.max_number_of_buffer_pools
 * must be increased for each worker thread the application will utilize.
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_init_worker_thread(wiced_worker_thread_t *worker_thread,
                                             uint8_t priority,
                                             uint32_t stack_size,
                                             uint32_t event_queue_size);

/*******************************************************************************
 * Function Name: wiced_rtos_send_asynchronous_event
 ****************************************************************************//**
 *
 * Enqueue an asynchronous event to be triggered in the given worker thread
 *
 * \param[in] worker_thread  worker thread instance to which the event is sent
 * \param[in] function       callback function executed in worker thread
 * \param[in] arg            argument passed to the callback function
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 * \note
 * Be sure to only pass a pointer to 'arg' which points to a global or static
 * variable. The pointer will be referenced in a different context after the
 * current function returns and will not be able to access local variables.
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_send_asynchronous_event(wiced_worker_thread_t *worker_thread,
                                                  event_handler_t function,
                                                  void *arg);

/** \} group_rtos_functions_worker */


/**
 * \addtogroup group_rtos_functions_evtflag
 * \{
 *
 * Defines a group of APIs to expose the event flags RTOS functionality.
 *
 */

/*******************************************************************************
 * Function Name: wiced_rtos_create_event_flags
 ****************************************************************************//**
 *
 * Allocates memory for a new event flags instance and returns the pointer
 *
 * \param void
 *
 * \return
 *  - NULL an error occurred and the instance was not allocated, do not init
 *  - valid pointer to event flags instance, which can now we initialized
 *
 *******************************************************************************/
wiced_event_flags_t *wiced_rtos_create_event_flags(void);

/*******************************************************************************
 * Function Name: wiced_rtos_init_event_flags
 ****************************************************************************//**
 *
 * Initializes the event flags instance (malloc'd by wiced_rtos_create_event_flags)
 *
 * \param[in] event_flags        a pointer to the event flags handle
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_init_event_flags(wiced_event_flags_t *event_flags);

/*******************************************************************************
 * Function Name: wiced_rtos_wait_for_event_flags
 ****************************************************************************//**
 *
 * Blocking wait for specific bit patter of event flags to be set elsewhere
 *
 * \param[in] event_flags        a pointer to the event flags handle
 * \param[in] flags_to_wait_for  bit pattern of event flags to wait for
 * \param[in] flags_set          event flag(s) set
 * \param[in] clear_set_flags    option to clear set flag(s)
 * \param[in] wait_option        WAIT_FOR_ANY_EVENT or WAIT_FOR_ALL_EVENTS
 * \param[in] timeout_ms         milliseconds to block
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_wait_for_event_flags(wiced_event_flags_t *event_flags,
                                               uint32_t flags_to_wait_for,
                                               uint32_t *flags_set,
                                               wiced_bool_t clear_set_flags,
                                               wiced_event_flags_wait_option_t wait_option,
                                               uint32_t timeout_ms);

/*******************************************************************************
 * Function Name: wiced_rtos_set_event_flags
 ****************************************************************************//**
 *
 * Set the event flags in logical OR operation
 *
 * \param[in] event_flags    a pointer to the event flags handle
 * \param[in] flags_to_set   a group of event flags (ORed bit-fields) to set
 *
 * \return
 *  - WICED_SUCCESS
 *  - WICED_ERROR
 *
 *******************************************************************************/
wiced_result_t wiced_rtos_set_event_flags(wiced_event_flags_t *event_flags, uint32_t flags_to_set);

/** \} group_rtos_functions_evtflag */

/** \} group_rtos_functions */

/** \} rtos */

#ifdef __cplusplus
}
#endif

#endif // WICED_RTOS_H__
