// Kernel functions
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#ifndef KERNEL_H_
#define KERNEL_H_

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();


//State Machine
#define RTOS_ENTRY 0

// mutex
#define MAX_MUTEXES             2
#define MAX_MUTEX_QUEUE_SIZE    1

#define mutex_bus_i2c1          0
#define mutex_bus_spi1          1

#define semaphore_mpu_data_ready 0

// semaphore
#define MAX_SEMAPHORES              1
#define MAX_SEMAPHORE_QUEUE_SIZE    1


// tasks
#define MAX_TASKS 12

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


bool initMutex(uint8_t mutex);
bool initSemaphore(uint8_t semaphore, uint8_t count);
void initRtos(void);
void startRtos(void);
bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes);

void _setupStack(uint8_t task);
uint8_t getTaskCurrent();
uint64_t* getCurrentSrd();
uint32_t _getCurrent(void);

void yield(void);
void sleep(uint32_t tick);
void wait(int8_t semaphore);
void post(int8_t semaphore);
void lock(int8_t mutex);
void unlock(int8_t mutex);
void atomic_read(void*src, void* dst, uint32_t size);
void atomic_write(void* dst, void* src, uint32_t size);

void systickIsr(void);
void pendSvIsr(void);
void svCallIsr(void);

//SVC Functions
void _kill(uint8_t pid);
void _lockMutex(uint8_t mutex);
void _unlockMutex(uint8_t mutex, uint8_t task);
void _postSemaphore(uint8_t semaphore);
void _waitSemaphore(uint8_t semaphore);

#endif
