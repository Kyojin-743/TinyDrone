// RTOS Framework
// Andrew Espinoza

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "wait.h"
#include "mm.h"
#include "kernel.h"
#include "faults.h"
#include "tasks.h"
#include "shell.h"

#define HEARTBEAT PORTF,3

//-----------------------------------------------------------------------------
// Sub-Routine Prototypes
//-----------------------------------------------------------------------------
void initCpuTimer(void);
void initFpu(void);
void initHeartbeat(void);
void heartBeatIsr(void);

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok = true;

    // Initialize hardware
    initSystemClockTo40Mhz();
    initSystemClockTo80Mhz();
    initTaskHw();
    initRtos();
    initHeartbeat();
    initFpu();
    initMpu();
    initHeap();

    // Initialize mutexes and semaphores
    ok &= initMutex(mutex_bus_i2c1);
    ok &= initMutex(mutex_bus_spi1);

    ok &= initSemaphore(semaphore_mpu_data_ready, 0);

    // Add required idle process at lowest priority
    ok &= createThread(idle, "Idle", 7, 512);
    ok &= createThread(task_ahrs_pid,"attitude", 0, ceil512(2000));
    ok &= createThread(task_receive_input, "rc", 2, ceil512(1000));
    ok &= createThread(task_send_telem, "telem", 2, ceil512(500));

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        while(true);
}


//-----------------------------------------------------------------------------
// Sub-Routines
//-----------------------------------------------------------------------------

void initFpu(void) {
    //Allow floating point access to thread mode
    NVIC_CPAC_R = NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL;
    //Turn off autosave of floating point registers
    NVIC_FPCC_R &= ~(NVIC_FPCC_ASPEN | NVIC_FPCC_LSPEN);
    __asm__(" DSB");
    __asm__(" ISB");
}

void initHeartbeat(void)
{
    enablePort(PORTF);
    selectPinPushPullOutput(HEARTBEAT);

    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;
    _delay_cycles(3);

    WTIMER0_CTL_R  &= ~(TIMER_CTL_TBEN);
    WTIMER0_CFG_R   = 0x4;
    WTIMER0_TBMR_R  = TIMER_TBMR_TBMR_PERIOD;
    WTIMER0_TBILR_R = 40e6;
    WTIMER0_IMR_R  |= TIMER_IMR_TBTOIM;
    NVIC_EN2_R |= (uint32_t)1 << (INT_WTIMER0B - 16 - 64);
    WTIMER0_CTL_R  |= TIMER_CTL_TBEN;
}

void heartBeatIsr(void)
{
    WTIMER0_ICR_R |= TIMER_ICR_TBTOCINT;
    togglePinValue(HEARTBEAT);
}

