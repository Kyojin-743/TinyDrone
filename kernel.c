// Kernel functions
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"

#include "asm.h"
#include "shell.h"
#include "mm.h"
#include "kernel.h"
#include "tasks.h"
#include "nvic.h"

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// mutex
typedef struct _mutex
{
    bool lock;
    uint8_t queueSize;
    uint8_t processQueue[MAX_MUTEX_QUEUE_SIZE];
    uint8_t lockedBy;
} mutex;
mutex mutexes[MAX_MUTEXES];

// semaphore
typedef struct _semaphore
{
    uint8_t count;
    uint8_t queueSize;
    uint8_t processQueue[MAX_SEMAPHORE_QUEUE_SIZE];
} semaphore;
semaphore semaphores[MAX_SEMAPHORES];

// task states
#define STATE_INVALID           0 // no task
#define STATE_UNRUN             1 // task has never been run
#define STATE_READY             2 // has run, can resume at any time
#define STATE_DELAYED           3 // has run, but now awaiting timer
#define STATE_BLOCKED_SEMAPHORE 4 // has run, but now blocked by semaphore
#define STATE_BLOCKED_MUTEX     5 // has run, but now blocked by mutex

// task
uint8_t taskCurrent = 0;          // index of last dispatched task
uint8_t taskCount = 0;            // total number of valid tasks

// control
bool priorityScheduler = true;    // priority (true) or round-robin (false)
bool priorityInheritance = false; // priority inheritance for mutexes
bool preemption = true;          // preemption (true) or cooperative (false)


// tcb
#define NUM_PRIORITIES   8
struct _tcb
{
    uint8_t state;                  // see STATE_ values above
    void *pid;                      // used to uniquely identify thread (add of task fn)
    void *sp;                       // current stack pointer
    uint32_t size;                  // useful for freeing, and restarting threads
    uint8_t priority;               // 0=highest
    uint8_t currentPriority;        // 0=highest (needed for pi)
    uint32_t ticks;                 // ticks until sleep complete
    uint8_t srd[NUM_SRAM_REGIONS];  // MPU subregion disable bits
    char name[16];                  // name of task used in ps command
    uint8_t mutex;                  // index of the mutex in use or blocking the thread
    uint8_t semaphore;              // index of the semaphore that is blocking the thread
} tcb[MAX_TASKS];

// svc numbers
#define SVC_START           0x0     //Entry point to Rtos
#define SVC_YIELD           0x1     //Yields current task
#define SVC_SLEEP           0x2     //Yields current task for x milliseconds
#define SVC_LOCK            0x3     //Locks a Mutex (Yields if already locked)s
#define SVC_UNLOCK          0x4     //Unlocks Owned Mutex
#define SVC_WAIT            0x5     //Waits for a sempahore (Yields if none)
#define SVC_POST            0x6     //Posts a new semaphore
#define SVC_REBOOT          0x7     //Reboots the device
#define SVC_KILL            0x8     //Kills a task
#define SVC_MALLOC          0x9     //Allows a task to request more memorys
#define SVC_ATOMIC_READ     0xA     //Allows a task to read n bytes from global memory
#define SVC_ATOMIC_WRITE    0xB     //Allows a task to write n bytes from global memory

#define SYSTICK_TIME        80e3

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

bool initMutex(uint8_t mutex)
{
    uint8_t w;

    if (!(mutex < MAX_MUTEXES)) return false;

    mutexes[mutex].lock = false;
    mutexes[mutex].lockedBy = 0;
    for(w = 0; w < MAX_MUTEX_QUEUE_SIZE; ++w)
        mutexes[mutex].processQueue[w] = MAX_TASKS;
    return true;
}

bool initSemaphore(uint8_t semaphore, uint8_t count)
{
    uint8_t s;

    if(!(semaphore < MAX_SEMAPHORES)) return false;

    semaphores[semaphore].count = count;
    for(s = 0; s < MAX_SEMAPHORE_QUEUE_SIZE; ++s)
        semaphores[semaphore].processQueue[s] = MAX_TASKS;
    return true;
}

void initRtos(void)
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }

    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN;
    NVIC_ST_RELOAD_R = SYSTICK_TIME - 1;
}

#define CAN_RUN(x)\
    (tcb[(x)].state == STATE_READY || tcb[(x)].state == STATE_UNRUN)
#define SAME_PRIO(x, p)\
    (tcb[(x)].currentPriority == (p))

uint8_t rtosScheduler(void)
{
    static uint8_t rrTaskIdx = 0;
    static uint8_t prioTaskIdx[NUM_PRIORITIES] = {0};
    uint8_t t;
    uint8_t minPrio = 0xFF;
    bool ok = false;

    if(priorityScheduler)
        for(t = 0; t < MAX_TASKS; t++)
            if((minPrio > tcb[t].currentPriority) && CAN_RUN(t))
                minPrio = tcb[t].currentPriority;
    while (!ok)
    {
        t = priorityScheduler?
                ((prioTaskIdx[minPrio] = (prioTaskIdx[minPrio]+1)%MAX_TASKS)):
                ((rrTaskIdx = (rrTaskIdx+1)%MAX_TASKS));
        ok = priorityScheduler?(CAN_RUN(t) && SAME_PRIO(t, minPrio)):(CAN_RUN(t));
    }
    return t;
}

void startRtos(void)
{
    taskCurrent = 0;
    applySramAccessMask(*(uint64_t*)tcb[taskCurrent].srd);
    setPsp(tcb[taskCurrent].sp);
    setAsp();
    tcb[taskCurrent].state = STATE_READY;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
    enableNvicInterrupt(INT_GPIOE);
    enableNvicInterrupt(INT_GPIOB);
    setTMPL();
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
    idle();
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool found = false;
    uint8_t i = 0;
    uint32_t *stackPtr = nullptr;
    uint64_t srdBitMask = createNoSramAccessMask();

    stackBytes = ceil512(stackBytes);

    if (!(taskCount < MAX_TASKS))
        return false;

    // make sure fn not already in list (prevent reentrancy)
    while (!found && (i < MAX_TASKS))
        if((tcb[i++].pid == fn)) return false;

    // find first available tcb record
    i = 0;
    while (tcb[i].state != STATE_INVALID) {i++;}

    tcb[i].state = STATE_READY;
    tcb[i].pid = fn;
    tcb[i].sp = 0;
    *((uint64_t*)tcb[i].srd) = srdBitMask;
    tcb[i].priority = (tcb[i].currentPriority = priority);
    tcb[i].mutex = MAX_MUTEXES;
    tcb[i].semaphore = MAX_SEMAPHORES;

    taskCurrent = taskCount;
    if(!(stackPtr = (uint32_t*)mallocHeap(stackBytes))) return false;
    addSramAccessWindow((uint64_t*)tcb[i].srd, stackPtr, stackBytes);
    tcb[i].size = stackBytes;
    tcb[i].sp = stackPtr + stackBytes/4;
    strcpy(tcb[i].name, name);

    _setupStack(taskCurrent);

    // increment task count
    taskCount++;

    return true;
}

void systickIsr(void)
{
    uint8_t t;
    for(t = 0; t < MAX_TASKS; t++)
    {
        if(tcb[t].state == STATE_DELAYED)
        {
            tcb[t].ticks--;
            if(!tcb[t].ticks)
                tcb[t ].state = STATE_READY;
        }
    }
    if(preemption)
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    return;
}

__attribute__((naked)) void pendSvIsr(void)
{
    pushHwRegs();
    tcb[taskCurrent].sp = getPsp();
    taskCurrent = rtosScheduler();
    setPsp(tcb[taskCurrent].sp);
    applySramAccessMask(*getCurrentSrd());
    popHwRegs();
}

void svCallIsr(void) {
    uint8_t *PC = (uint8_t*)*(getPsp() + 6);
    uint8_t svcNumber = *(PC - 2);

    switch(svcNumber) {
    case(SVC_START): {
        taskCurrent = rtosScheduler();
        setPsp(tcb[taskCurrent].sp);
        applySramAccessMask(*getCurrentSrd());
        tcb[taskCurrent].sp = ((uint32_t*)tcb[taskCurrent].sp) + 9;
        popHwRegs();
    } break;
    case(SVC_YIELD): {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    } break;
    case(SVC_SLEEP): {
        uint32_t ticks = *getPsp();
        tcb[taskCurrent].ticks = ticks;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    } break;
    case(SVC_LOCK):     { _lockMutex(*getPsp());                     } break;
    case(SVC_UNLOCK):   { _unlockMutex(*getPsp(), taskCurrent);      } break;
    case(SVC_WAIT):     { _waitSemaphore(*getPsp());                 } break;
    case(SVC_POST):     { _postSemaphore(*getPsp());                 } break;
    case(SVC_REBOOT):   { NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ; }break;
    case(SVC_ATOMIC_READ): {
        bytecpy((void*)*getPsp(), (void*)*(getPsp()+1), *(getPsp()+2));
    }break;
    case(SVC_ATOMIC_WRITE): {
        bytecpy((void*)*(getPsp()+1), (void*)*getPsp(), *(getPsp()+2));
    }break;
    case(SVC_KILL): {
        uint32_t pid = *getPsp();
        uint8_t i;
        for(i = 0; i < taskCount; i++)
            if((uint32_t)tcb[i].pid == pid) _kill(i);
    } break;
    case(SVC_MALLOC): {
        uint32_t sizeBytes = *getPsp();
        void *addr = mallocHeap(sizeBytes);
        addSramAccessWindow((uint64_t*)tcb[taskCurrent].srd, addr, sizeBytes);
        *getPsp() = (uint32_t)addr;
    } break;
    }
}

void yield(void) {
    __asm(" SVC #0x1");
}

void sleep(uint32_t tick) {
    __asm(" SVC #0x2");
}

void wait(int8_t semaphore) {
    __asm(" SVC #0x5");
}

void post(int8_t semaphore) {
    __asm(" SVC #0x6");
}

void lock(int8_t mutex) {
    __asm(" SVC #0x3");
}

void unlock(int8_t mutex) {
    __asm(" SVC #0x4");
}

void atomic_read(void*src, void* dst, uint32_t size) {
    __asm(" SVC #0xA");
}

void atomic_write(void* dst, void* src, uint32_t size) {
    __asm(" SVC #0xB");
}

uint8_t getTaskCurrent() {
    return taskCurrent;
}

uint64_t* getCurrentSrd() {
    return (uint64_t*)tcb[taskCurrent].srd;
}

void _setupStack(uint8_t task) {
    uint32_t* stackPtr = tcb[task].sp;
    _fn fn = (_fn)tcb[task].pid;
    *(--stackPtr) = 1<<24;
    *(--stackPtr) = (uint32_t)fn;
    stackPtr -= 14;
    *(--stackPtr) = 0xFFFFFFFD;
    stackPtr -= 33;
    tcb[task].sp = stackPtr;
}

void _lockMutex(uint8_t mutex) {
    uint8_t *waiter = mutexes[mutex].processQueue;

    tcb[taskCurrent].mutex = mutex;

    //the mutex is owned by another task
    if(mutexes[mutex].lock) {
        //if pi enabled -> boost priority of the task owning the mutex to the task thats being blocked
        if(priorityInheritance)
            if(tcb[mutexes[mutex].lockedBy].priority > tcb[taskCurrent].priority)
                tcb[mutexes[mutex].lockedBy].currentPriority = tcb[taskCurrent].priority;

        //block requesting task and add to queue
        tcb[taskCurrent].state = STATE_BLOCKED_MUTEX;
        waiter[mutexes[mutex].queueSize] = taskCurrent;
        mutexes[mutex].queueSize++;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
    // the mutex was unlocked and the current task can own the mutex
    else {
        mutexes[mutex].lock = true;
        mutexes[mutex].lockedBy = taskCurrent;
    }
}

void _unlockMutex(uint8_t mutex, uint8_t task) {
    uint8_t m;
    uint8_t *waiter = mutexes[mutex].processQueue;

    //If the task calling unlock actually owns the mutex
    if(mutexes[mutex].lockedBy == task) {
        tcb[task].mutex = MAX_MUTEXES;

        //if the task was temporarily elevated -> revert back to original
        if(tcb[task].priority != tcb[task].currentPriority)
            tcb[task].currentPriority = tcb[task].priority;

        //if there are tasks blocked
        if(mutexes[mutex].queueSize) {
            //swap ownership to the first task in queue
            mutexes[mutex].lockedBy = waiter[0];
            tcb[waiter[0]].state = STATE_READY;

            //age the queue
            for(m = 0; m < mutexes[mutex].queueSize; m++) {
                //last index dont want to reach outside of array
                if(m + 1 >= MAX_MUTEX_QUEUE_SIZE)
                    waiter[m] = MAX_TASKS;
                else
                    waiter[m] = waiter[m + 1];
            }
            mutexes[mutex].queueSize--;
        }
        //no tasks blocked by mutex
        else {
             mutexes[mutex].lock = false;
             mutexes[mutex].lockedBy = MAX_TASKS;
        }
    }
    //task tried unlocking mutex it didnt own
    else for(;;);
}

void _kill(uint8_t pid) {
    //pid = the index in tcb of the task to kill
    uint8_t *waiter, i, m, s;
    uint32_t addr;

    //The task either owns a mutex or is blocked by one
    if(tcb[pid].mutex != MAX_MUTEXES) {
        if(tcb[pid].state == STATE_BLOCKED_MUTEX) {
            //setup
            waiter = mutexes[tcb[pid].mutex].processQueue;

            //find the index in the queue that the pid is at
            for(i = 0; i < MAX_MUTEX_QUEUE_SIZE; i++)
                if(waiter[i] == pid) {
                    //age the queue
                    for(m = i; m < mutexes[tcb[pid].mutex].queueSize; m++) {
                        //last index dont want to reach outside of array
                        if(m + 1 >= MAX_MUTEX_QUEUE_SIZE)
                            waiter[m] = MAX_TASKS;
                        else
                            waiter[m] = waiter[m + 1];
                    }
                }
            mutexes[tcb[pid].mutex].queueSize--;
        }
        //the task owns the mutex and was running
        else
            _unlockMutex(tcb[pid].mutex, pid);
    }

    //waiting on semaphore
    if(tcb[pid].state == STATE_BLOCKED_SEMAPHORE) {
        for(i = 0; i < semaphores[tcb[pid].semaphore].queueSize; i++)
            if(waiter[i] == pid) {
                //age the queue
                for(s = i; s < semaphores[tcb[pid].semaphore].queueSize; s++) {
                    //so last task in queue wont try to access outside bounds of array
                    if(s + 1 >= MAX_SEMAPHORE_QUEUE_SIZE)
                        waiter[s] = MAX_TASKS;
                    else
                        waiter[s] = waiter[s + 1];
                }
            }
        semaphores[tcb[pid].semaphore].queueSize--;
    }

    tcb[pid].state = STATE_INVALID;

    while(findHeapEntry(pid, &addr, nullptr))
        freeHeap((void*)addr);
}

void _postSemaphore(uint8_t semaphore) {
    uint8_t s;
    uint8_t *waiter = semaphores[semaphore].processQueue;

    //if there are tasks waiting for the semaphore
    if(semaphores[semaphore].queueSize) {
        //allow first task in queue to consume the posted semaphore
        tcb[waiter[0]].state = STATE_READY;

        //age the queue
        for(s = 0; s < semaphores[semaphore].queueSize; s++) {
            //so last task in queue wont try to access outside bounds of array
            if(s + 1 >= MAX_SEMAPHORE_QUEUE_SIZE)
                waiter[s] = MAX_TASKS;
            else
                waiter[s] = waiter[s + 1];
        }
        semaphores[semaphore].queueSize--;
    }
    //no tasks to consume the sempaphore -> add to count
    else
        semaphores[semaphore].count++;
}

void _waitSemaphore(uint8_t semaphore) {
    uint8_t *waiter = semaphores[semaphore].processQueue;

    //consume a semaphore and return to continue running
    if(semaphores[semaphore].count)
        semaphores[semaphore].count--;
    //no semaphores posted
    else {
        //block requesting task and add to queue
        waiter[semaphores[semaphore].queueSize] = taskCurrent;
        semaphores[semaphore].queueSize++;
        tcb[taskCurrent].state = STATE_BLOCKED_SEMAPHORE;
        tcb[taskCurrent].semaphore = semaphore;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}
