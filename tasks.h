// Tasks
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#ifndef TASKS_H_
#define TASKS_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initTaskHw(void);

//TODO: Add prototypes for tasks
void task_ahrs_pid(void);
void task_receive_input(void);
void task_send_telem(void);
void idle(void);

#endif
