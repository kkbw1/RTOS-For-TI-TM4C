// RTOS Framework - Spring 2019
// J Losh

// Student Name:
// Kibum Kwon

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Green:  PE2
// Yellow: PE3
// Orange: PE4
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board orange LED

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------
// SVC number
#define SVC_YIELD   100
#define SVC_SLEEP   101
#define SVC_WAIT    102
#define SVC_POST    103

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[16];
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10       // maximum number of valid tasks

uint8_t taskOld = 0;
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

uint16_t i;
uint32_t SYSTEM_SP;

// for the shell task
#define MAX_CHARS   32          // max received characters in a string
uint8_t flag_null = 0;
char str[MAX_CHARS + 1];
uint8_t cnt_str = 0;
uint8_t idx_strEnd;
uint8_t cnt_arg;
uint8_t pos[MAX_CHARS];
char type[MAX_CHARS];
uint8_t cnt_parse;
char type_now, type_old;
char str_parse[4][12];          // max 14chars 4args can be stored
char PS_TITLE[] = "Name             PID      Priority   CPU";

#define INTERVAL_MS_CPU_TIME 1000   // 1000msec
uint16_t cnt_tcb_t;
float alpha = 0.95;
uint8_t cnt_pp;
uint8_t idx_pp_process;
uint8_t idx_pp_display;

// flags for preemption, priority scheduling, priority inheritence
bool preemption;
bool priorityScheduling;
bool priorityInheritence;

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // -8=highest to 7=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint8_t skips;
    uint8_t maxSkip;
    uint8_t t;                     // for cpu usage time measurement
    float t_filt[2];               // pingpong buffer, after filtering of t value
} tcb[MAX_TASKS];

//void yield();
//void sleep(uint32_t tick);
//void wait(struct semaphore *pSemaphore);
//void post(struct semaphore *pSemaphore);

uint32_t getSP();
void setSP(uint32_t sp);
uint16_t getSVC();
uint32_t getR0();
uint32_t getR1();
uint32_t getR2();
void setNewTask(uint32_t pid);
char getcUart0withYield();
void putcUart0(char c);
void putsUart0(char* str);
//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------
void rtosInit()
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
    // REQUIRED: initialize systick for 1ms system timer
    // SystemTick Setting
    NVIC_ST_CTRL_R |= (NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE);
    NVIC_ST_RELOAD_R = 40000 - 1;    // 1msec, max val=2^24-1=16,777,215
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;

    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;

        if(!priorityScheduling)
        {
            // Roundrobin Scheduling
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);    // if task is ready or unrun, it is ok.
        }
        else
        {
            // Priority Scheduling
            if(tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)
            {
                if(tcb[task].skips == 0)    // reloading skips
                {
                    tcb[task].skips = tcb[task].maxSkip;
                    ok = true;
                }
                else
                {
                    tcb[task].skips--;
                    ok = false;
                }
            }
        }
    }

    return task;
}

void rtosStart()
{
    // REQUIRED: add code to call the first task to be run
    _fn fn;
    taskCurrent = rtosScheduler();

    // Add code to initialize the SP with tcb[task_current].sp;
    // Step1
    SYSTEM_SP = getSP();                    // Storing System SP to a global variable
    setSP((uint32_t)tcb[taskCurrent].sp);   // Switching task SP to processor SP

    tcb[taskCurrent].state = STATE_READY;
    fn = (_fn)tcb[taskCurrent].pid;
    (*fn)();
}

bool createThread(_fn fn, char name[], int priority)
{
    bool ok = false;
    uint8_t i = 0;
    uint8_t j = 0;
    bool found = false;

    // REQUIRED: store the thread name
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][255];
            tcb[i].priority = priority;
            tcb[i].currentPriority = tcb[i].priority;
            tcb[i].maxSkip = tcb[i].currentPriority + 8;
            tcb[i].skips = tcb[i].maxSkip;
            tcb[i].ticks = 0;
            tcb[i].t = 0;
            tcb[i].t_filt[0] = 0;
            tcb[i].t_filt[1] = 0;
            // storing the thread name
            for(j = 0; name[j]; j++)
                tcb[i].name[j] = name[j];

            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    // Step 12 deleteThread
    uint8_t i, j = 0;
    bool found = false;
    uint8_t taskIdx = 0;

    i = 0;
    while (i < MAX_TASKS)
    {
        found = (tcb[i].pid ==  fn);    // searching the task that has fn
        if(found)
        {
            // deleting a semaphore record in tcb
            if(tcb[i].semaphore != 0)    // if the task is blocked or using a semaphore now
            {
                // finding the semaphore
                struct semaphore *pSemaphore = (struct semaphore *)tcb[i].semaphore;
                if(tcb[i].state == STATE_BLOCKED)
                {
                    // finding the element that equals the index i
                    for(j=0; j<pSemaphore->queueSize; j++)
                    {
                        if(i == pSemaphore->processQueue[j])
                        {
                            taskIdx = j;
                            break;
                        }
                    }
                    // moving elements after the index i in the array to left once
                    for(j=taskIdx; j<(pSemaphore->queueSize-1); j++)
                    {
                        pSemaphore->processQueue[j] = pSemaphore->processQueue[j+1];
                    }
                    pSemaphore->queueSize--;
                }
                else    // if the task is using the semaphore now, unlock the semaphore before deleting the task
                {
                    if(pSemaphore->queueSize == 0)
                    {
                        pSemaphore->count++;
                    }
                    else    // if one or more tasks are blocked in the queue
                    {
                        tcb[pSemaphore->processQueue[0]].state = STATE_READY;
                        // moving elements after the index i in the array to left once
                        for(j=0; j<(pSemaphore->queueSize-1); j++)
                        {
                            pSemaphore->processQueue[j] = pSemaphore->processQueue[j+1];
                        }
                        pSemaphore->queueSize--;
                    }
                }
            }

            tcb[i].pid = 0;
            tcb[i].state = STATE_INVALID;
            tcb[i].sp = 0;
            memset(&stack[i][0], 0, 256);   // clear the task stack

            taskCount--;
            break;
        }
        i++;
    }
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    // Step 12
    uint8_t i = 0;
    bool found = false;

    i = 0;
    while (i < MAX_TASKS)
    {
        found = (tcb[i].pid ==  fn);
        if(found)
        {
            tcb[i].priority = priority;
            tcb[i].currentPriority = tcb[i].priority;
            tcb[i].maxSkip = tcb[i].currentPriority+8;
            tcb[i].skips = tcb[i].maxSkip;
            break;
        }
        i++;
    }
}

struct semaphore* createSemaphore(uint8_t count, char name[])
{
    struct semaphore *pSemaphore = 0;
    uint8_t i;

    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;

        for(i=0; name[i]; i++)
            pSemaphore->name[i] = name[i];
    }
    return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    // Step3
    __asm("   SVC  #100");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    // Step 6
    __asm("   SVC  #101");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    // Step7
    __asm("   SVC #102");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    // Step8
    __asm("   SVC #103");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i = 0;

    // measure cpu time per task
    tcb[taskCurrent].t++;
    cnt_tcb_t++;
    if(cnt_tcb_t == INTERVAL_MS_CPU_TIME)    // 1sec
    {
        cnt_tcb_t = 0;

        // switching WR buffers
        if(cnt_pp % 2 == 0)
        {
            idx_pp_process = 0;
            idx_pp_display = 1;
        }
        else
        {
            idx_pp_process = 1;
            idx_pp_display = 0;
        }
        cnt_pp++;

        for(i = 0; i < MAX_TASKS; i++)
        {
            tcb[i].t_filt[idx_pp_process] = (1 - alpha) * tcb[i].t_filt[idx_pp_process] + alpha * tcb[i].t;
            tcb[i].t = 0;
        }
    }

    // Step 6
    // Delay
    for(i = 0; i < MAX_TASKS; i++)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            tcb[i].ticks--;
            if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }

            if(tcb[i].ticks > 10000000) // if ticks too high -> sometimes ticks has really high number... why...?
                tcb[i].ticks = 1000;
        }
    }

    // Step 13
    // Preemption
    if(preemption)
    {
        // PendSV Set
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    WTIMER1_TAV_R = 0;
    // Step4
    //automatically push {r0-r3, r12, LR, PC, xPSR, FP} by interrupt
    //automatically push {r3, lr} by compiler
    //manually push reg list  {r4-r11, sp}
    __asm("   push {r4-r11}");
//    __asm("   mov r0, sp");
//    __asm("   push {r0}");

    // Save sp in tcb[currentTask].sp
    tcb[taskCurrent].sp = getSP();
    // change system sp from task sp in sp reg
    setSP(SYSTEM_SP);
    // get taskCurrent => change a task to be executed
    taskOld = taskCurrent;
    taskCurrent = rtosScheduler();

    if(tcb[taskCurrent].state == STATE_READY)
    {
        // set task sp in sp reg
        setSP((uint32_t)tcb[taskCurrent].sp);
    }
    else    // if state == unrun
    {
        // Step5
        tcb[taskCurrent].state = STATE_READY;
        // the task sp to sp reg
        setSP((uint32_t)tcb[taskCurrent].sp - 84);

        // Set LR = 0xFFFFFFF9
        __asm("   mvn lr, #0x6");       // MOV NOT 0x00000006
        __asm("   str lr, [sp, #36]");
        setNewTask((uint32_t)(tcb[taskCurrent].pid) - 1);

//        setSP((uint32_t)tcb[taskCurrent].sp - 52);  // 84-32
//
//        // Set LR = 0xFFFFFFF9
//        __asm("   mvn lr, #0x6");       // MOV NOT 0x00000006
//        __asm("   str lr, [sp, #4]");
//
//        setNewTask((uint32_t)(tcb[taskCurrent].pid) - 1);
    }

    __asm("   pop {r4-r11}");
}   // <---- __asm(" pop {r3, lr});     __asm(" pop {r0-r3, r12, lr, pc, xPSR, FP});

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    // Step6
    uint32_t r0 = getR0();
    uint32_t r1 = getR1();
    uint32_t r2 = getR2();

    // Step3 read SVC number
    uint8_t num_svc = getSVC();
    switch(num_svc)
    {
    case SVC_YIELD:
        // Step4 yield
        // PendSV Set
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    case SVC_SLEEP:
        // Step6 sleep
        // set sleep timeout
        tcb[taskCurrent].ticks = r0;
        tcb[taskCurrent].state = STATE_DELAYED;
        // PendSV Set
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    case SVC_WAIT:
        // Step7 wait
        tcb[taskCurrent].semaphore = r0;    // storing the semaphore in the tcb field if using or waiting
        if(((struct semaphore *)r0)->count > 0)
        {
            ((struct semaphore *)r0)->count--;
        }
        else    // if count == 0, need to wait until post the semaphore so, switch task
        {
            if(((struct semaphore *)r0)->queueSize < MAX_QUEUE_SIZE)
            {
                ((struct semaphore *)r0)->processQueue[((struct semaphore *)r0)->queueSize] = taskCurrent;
                ((struct semaphore *)r0)->queueSize++;
            }
            tcb[taskCurrent].state = STATE_BLOCKED;
            // Step11 pi support
            if(priorityInheritence)
            {
                // 1.Searching who is using the semaphore
                // 2. if the blocked task priority is higher,
                //    the semaphore using task priority will be the blocked task priority temporarily
                uint8_t i=0;
                while(i < MAX_TASKS)
                {
                    if((uint32_t)tcb[i].semaphore == r0 && (tcb[i].state == STATE_READY || tcb[i].state == STATE_DELAYED))
                    {
                        if(tcb[i].priority > tcb[taskCurrent].priority)
                        {
                            tcb[i].currentPriority = tcb[taskCurrent].priority;
                            tcb[i].maxSkip = tcb[i].currentPriority + 8;
                            tcb[i].skips = tcb[i].maxSkip;
                        }
                        break;
                    }
                    i++;
                }
            }
            // PendSV Set
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }
        break;
    case SVC_POST:
        //Step8 post
        tcb[taskCurrent].semaphore = 0;
        ((struct semaphore *)r0)->count++;
        if(((struct semaphore *)r0)->count == 1)    // if the semaphore was locked before
        {
            if(((struct semaphore *)r0)->queueSize > 0) // if other tasks were waiting to use the sempahore
            {
                tcb[((struct semaphore *)r0)->processQueue[0]].state = STATE_READY;
                // Step11 pi support
                if(priorityInheritence)
                {
                    // Making the priority that temporarily was set higher to set lower back
                    tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
                    tcb[taskCurrent].maxSkip = tcb[taskCurrent].currentPriority + 8;
                }
                // moving every elements in the queue to left once
                uint8_t i;
                for(i=0; i<((struct semaphore *)r0)->queueSize-1; i++)
                {
                    ((struct semaphore *)r0)->processQueue[i] = ((struct semaphore *)r0)->processQueue[i+1];
                }
                ((struct semaphore *)r0)->queueSize--;
                ((struct semaphore *)r0)->count--; // b/c it will be used immediately again by the task that was waiting in the queue
            }
        }
        // PendSV Set (Optional)
//        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
  // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
  //           5 pushbuttons, and uart
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, B, C, F peripherals
    SYSCTL_RCGC2_R =  SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    //==================================================================================================//
    // Configure UART0 pins PA[1:0]
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;            // turn-on UART0, leave other UARTs in same status

    GPIO_PORTA_DEN_R |= 0x03;                           // 0b0000 0011, default, added for clarity
    GPIO_PORTA_AFSEL_R |= 0x03;                         // 0b0000 0011, default, added for clarity
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)

    double UBRD = (double)40000000/16.0/(double)115200;
    UART0_IBRD_R = (uint16_t)UBRD;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = (uint16_t)((UBRD-(uint16_t)(UBRD))*64+0.5);      // round(fract(r)*64)=45

    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
    //==================================================================================================//
    // Configure LED and pushbutton pins PF[0:4]
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1,2,3 are outputs, bits 0,4 are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // bits 1,2,3 set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_PUR_R = 0x10;  // bits 4 enable internal pull-up for push button
    GPIO_PORTF_DEN_R = 0x1E;  // bits 1,2,3,4 // enable LEDs and pushbuttons

    // External SW
    GPIO_PORTA_DIR_R &= ~0x7C;
    GPIO_PORTA_PUR_R |= 0x7C;
    GPIO_PORTA_DEN_R |= 0x7C;

    // External LED
    GPIO_PORTE_DIR_R |= 0x1E;
    GPIO_PORTE_DR8R_R |= 0x1E;
    GPIO_PORTE_DEN_R |= 0x1E;
    GPIO_PORTE_DATA_R &= ~0x1E;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                              // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    uint8_t data;
    data = (GPIO_PORTA_DATA_R >> 2);   // ex) XXX1 1110 -> XXX0 0001 -> 0000 0001
    data = (~data)&0x1F;

    return data;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
uint32_t getSP()
{
    //sub.w      sp, sp, #8    // (WORD)sp = (WORD)sp-(WORD)8;
    uint32_t sp = 0;    // assigned to R0

    // Storing stack pointer value in sp c variable ex) sp = sp;
    __asm("   mov  r0, sp");
    __asm("   str  r0, [sp]");

    return sp+8;
    //add        sp, #8     // (32bit)sp = (32bit)sp + (32bit)8;
}

void setSP(uint32_t sp)
{
    //sub.w      sp, sp, #8     // (WORD)sp = (WORD)sp-(WORD)8;
    //str        r0, [sp]
    sp = sp - 8;

    // value in sp is stored in r0
    // then the value will be stored in stack using sp
    __asm(" mov sp, r0");

    //add        sp, #8     // (32bit)sp = (32bit)sp + (32bit)8;
}

uint16_t getSVC()
{
    uint16_t svc = 0;   // assigned to R0

    // GET data in [SP + offset(=40)]
    __asm("   ldr r1, [sp, #64]");
//    __asm("   ldr r1, [sp, #40]");

    // Get data in [r1 - 2] = 0xDFXX
    __asm("   sub r1, #2");
    __asm("   ldr r0, [r1]");

    // Store in stack
    __asm("   str r0, [sp]");

    return svc;
}

uint32_t getR0()
{   // <--- __asm(" sub.w sp, sp, #8");
    __asm("   mov r3, r0");
    uint32_t r0 = 0;
    __asm("   mov r0, r3");
    __asm("   str r0, [sp]");

    return r0;  // <--- __asm(" ldr r0, [sp]");
}   // <--- __asm(" add sp, #8); __asm(" bx lr");

uint32_t getR1()
{   // <--- __asm(" sub.w sp, sp, #8");
    __asm("   mov r4, r1");
    uint32_t r1 = 0;
    __asm("   mov r0, r4");
    __asm("   str r0, [sp]");

    return r1;
}   // <--- __asm(" add sp, #8); __asm(" bx lr");

uint32_t getR2()
{   // <--- __asm(" sub.w sp, sp, #8");
    __asm("   mov r5, r2");
    uint32_t r2 = 0;
    __asm("   mov r0, r5");
    __asm("   str r0, [sp]");

    return r2;
}   // <--- __asm(" add sp, #8); __asm(" bx lr");

void setNewTask(uint32_t pid)
{
    // set PC to tcb[currentTask].pid
    __asm("   str r0, [sp, #72]");      // offset 72bytes
    // set PSR to 0x21000000(default value)
    __asm("   mov r0, #0");
    __asm("   movt r0, #0x2100");
    __asm("   str r0, [sp, #76]");      // offset 76bytes

//    // set PC to tcb[currentTask].pid
//    __asm("   str r0, [sp, #40]");      // offset 72bytes-32 = 40
//    // set PSR to 0x21000000(default value)
//    __asm("   mov r0, #0");
//    __asm("   movt r0, #0x2100");
//    __asm("   str r0, [sp, #44]");      // offset 76bytes-32 = 44
}

char getcUart0withYield()
{
    while (UART0_FR_R & UART_FR_RXFE)   // waiting UART Receive FIFO Empty bit is set, which means waiting the FIFO empty.
    {
        yield();
    }
    return UART0_DR_R & 0xFF;           // bit masking to get only 16bits data cuz 17~32th bits are garbage values.
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

void parsingString()
{
    //--------------------------------------------------------------//
    // Step 3.
    // Parse Strings
    //--------------------------------------------------------------//
    memset(pos, 0, MAX_CHARS*sizeof(pos[0]));           // Make all data in the array pos to NULL
    memset(type, 0, MAX_CHARS*sizeof(type[0]));         // Make all data in the array type to NULL
    memset(str_parse, 0, 40*sizeof(str_parse[0][0]));   // Make all data in the array str_parse to NULL

    type_now = 0;
    type_old = 0;
    cnt_arg = 0;

    // Marking character types according to their types(letters, numbers, other else)
    uint8_t i = 0;
    for(i = 0; i < idx_strEnd; i++)
    {
        type_old = type_now;
        if((str[i] >= 0x41 && str[i] <= 0x5A) || (str[i] >= 0x61 && str[i] <= 0x7A))    // Upper letters and lower letters
        {
            type_now = 'c';
            if((type_now != type_old) || (i == 0))
            {
                pos[cnt_arg] = i;
                type[cnt_arg] = 'c';
                cnt_arg++;
            }
        }
        else if(str[i] >= 0x30 && str[i] <= 0x39)   // Numbers
        {
            type_now = 'n';
            if((type_now != type_old) || (i == 0))
            {
                pos[cnt_arg] = i;
                type[cnt_arg] = 'n';
                cnt_arg++;
            }
        }
        else        // everything else -> delimiters
        {
            type_now = 'd';
            str[i] = 0;     // change it NULL
        }
    }

    for(i = 0; i < cnt_arg; i++)
    {
        uint8_t pos_now = pos[i];
        if(i == (cnt_arg - 1))     // if it's a last arg
        {
            while(str[pos_now] != 0)
            {
                str_parse[i][pos_now - pos[i]] = str[pos_now];
                pos_now++;
                if(pos_now == sizeof(str))
                    break;
            }
        }
        else    // if it's not a last arg
        {
            while(str[pos_now] != 0)
            {
                str_parse[i][pos_now - pos[i]] = str[pos_now];
                pos_now++;
            }
        }
    }
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------
// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(990);  // 1msec
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);   // 0.99msec
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void shell()
{
    char str_send[44];

    while (true)
    {
        // REQUIRED: add processing for the shell commands through the UART here
        // Step 10
        char c = getcUart0withYield();
        putcUart0(c);

        if(c == 0x08)                   // if c == BackSpace
        {
            if(cnt_str > 0)
            {
                cnt_str--;
                str[cnt_str] = ' ';
            }
        }
        else if(c == 0x0D)              // if c == CarriageReturn
        {
            str[cnt_str] = 0;
            flag_null = 1;
            idx_strEnd = cnt_str;
            cnt_str = 0;
        }
        else if(c >= 32 && c <= 126)    // if c == printable data
        {
            str[cnt_str++] = c;
            if(cnt_str == MAX_CHARS)
            {
                str[cnt_str] = 0;
                flag_null = 1;
                idx_strEnd = cnt_str;
                cnt_str = 0;
            }
        }

        if(flag_null == 1)              // if received NULL char as final
        {
            uint8_t i;

            parsingString();            // parsing characters in str buffer
            putsUart0("\r\n");

            if(strcmp(&str_parse[0][0], "pi") == 0)             // priority inherence
            {
                if(strcmp(&str_parse[1][0], "on") == 0)
                {
                    putsUart0("Priority Inheritence On\r\n\r\n");
                    priorityInheritence = true;
                }
                else if(strcmp(&str_parse[1][0], "off") == 0)
                {
                    putsUart0("Priority Inheritence Off\r\n\r\n");

                    i = 0;
                    while(i < MAX_TASKS)
                    {
                        if(tcb[i].currentPriority != tcb[i].priority)
                        {
                            tcb[i].currentPriority = tcb[i].priority;
                            tcb[i].maxSkip = tcb[i].currentPriority + 8;
                        }
                        i++;
                    }
                    priorityInheritence = false;
                }
            }
            else if(strcmp(&str_parse[0][0], "schedule") == 0)  // roundrobin/priority scheduling
            {
                if(strcmp(&str_parse[1][0], "rr") == 0)
                {
                    putsUart0("Round robin Task Scheduling On\r\n\r\n");
                    priorityScheduling = false;
                }
                else if(strcmp(&str_parse[1][0], "priority") == 0)
                {
                    putsUart0("Priority Task Scheduling On\r\n\r\n");
                    priorityScheduling = true;
                }
                else
                {
                    putsUart0("Type schedule <rr/priority>.\r\n");
                }
            }
            else if(strcmp(&str_parse[0][0], "rtos") == 0)      // preemption set
            {
                if(strcmp(&str_parse[1][0], "preemptive") == 0)
                {
                    putsUart0("Preemption Task Switching On\r\n\r\n");
                    preemption = true;
                }
                else if(strcmp(&str_parse[1][0], "coop") == 0)
                {
                    putsUart0("Cooperative Task Switching On\r\n\r\n");
                    preemption = false;
                }
                else
                {
                    putsUart0("Type rtos <preemptive/coop>.\r\n");
                }
            }
            else if(strcmp(&str_parse[0][0], "reboot") == 0)
            {
                putsUart0("Rebooting\r\n\r\n");
                waitMicrosecond(2000);
                NVIC_APINT_R = (0x05FA << 16) | NVIC_APINT_SYSRESETREQ;   // Reset Microcontroller
            }
            else if(strcmp(&str_parse[0][0], "pidof") == 0)     // display pid number
            {
                if(str_parse[1][0] != 0)
                {
                    i = 0;
                    while(i < MAX_TASKS)
                    {
                        if(strcmp(&str_parse[1][0], &tcb[i].name[0]) == 0)  // compare name
                        {
                            ltoa((uint32_t)tcb[i].pid, str_send);
                            putsUart0("PID:");
                            putsUart0(str_send);
                            putsUart0("\r\n");
                            putsUart0("\r\n");
                            break;
                        }

                        i++;
                        if(i == MAX_TASKS)
                        {
                            putsUart0("Can't find.\r\n\r\n");
                        }
                    }
                }
                else
                {
                    putsUart0("Type pidof <TASK NAME>.\r\n");
                }
            }
            else if(strcmp(&str_parse[0][0], "kill") == 0)      // delete threads
            {
                if(cnt_arg == 2 && str_parse[1][0] > '0' && str_parse[1][0] <= '9')
                {
                    // Step 12
                    long temp = atol(&str_parse[1][0]);     // convert pid number
                    destroyThread(temp);

                    putsUart0("Deleted Thread PID=");
                    putsUart0(&str_parse[1][0]);
                    putsUart0("\r\n");
                    putsUart0("\r\n");
                }
                else
                {
                    putsUart0("Type kill <PID>.\r\n");
                }
            }
            else if(strcmp(&str_parse[0][0], "ps") == 0)        // process status
            {
                float t_filt_sum = 0;
                uint16_t usage[10];
                uint16_t usage_sum = 0;

                // Calculating CPU Usage
                for(i=0; i<MAX_TASKS; i++)
                {
                    if(tcb[i].state != STATE_INVALID)
                        t_filt_sum += tcb[i].t_filt[idx_pp_display];
                }
                for(i=0; i<MAX_TASKS; i++)
                {
                    if(tcb[i].state != STATE_INVALID)
                    {
                        usage[i] = ((uint16_t)tcb[i].t_filt[idx_pp_display]*10000) / t_filt_sum;
                        usage_sum += (uint16_t)usage[i];
                    }
                }

                putsUart0("\r\n");
                putsUart0(PS_TITLE);
                putsUart0("\r\n");
                memset(str_send, '=', sizeof(str_send));
                str_send[43] = 0;
                putsUart0(str_send);
                putsUart0("\r\n");
                for(i=0; i<MAX_TASKS; i++)
                {
                    if(tcb[i].state != STATE_INVALID)
                    {
                        uint8_t j=0;
                        char str_temp[11];
                        memset(str_send, ' ', sizeof(str_send));

                        // Storing Name
                        for(j=0; tcb[i].name[j]; j++)
                            str_send[j] = tcb[i].name[j];
                        // Storing PID
                        ltoa((uint32_t)tcb[i].pid, str_temp);
                        for(j=0; str_temp[j]; j++)
                            str_send[j+17] = str_temp[j];
                        // Storing Priority
                        ltoa(tcb[i].priority, str_temp);
                        for(j=0; str_temp[j]; j++)
                            str_send[j+26] = str_temp[j];
                        // Storing CPU Usage
                        uint8_t usage_int = usage[i]/100;
                        ltoa(usage_int, str_temp);
                        for(j=0; str_temp[j]; j++)
                            str_send[j+37] = str_temp[j];
                        str_send[j+37]='.';
                        ltoa(usage[i]%100, str_temp);
                        if(usage_int >= 10)
                        {
                            for(j=0; str_temp[j]; j++)
                                str_send[j+40] = str_temp[j];
                            str_send[j+40] = '%';
                            str_send[j+41] = 0;
                        }
                        else
                        {
                            for(j=0; str_temp[j]; j++)
                                str_send[j+39] = str_temp[j];
                            str_send[j+39] = '%';
                            str_send[j+40] = 0;
                        }
                        // Send task status
                        putsUart0(str_send);
                        putsUart0("\r\n");
                    }
                }
                memset(str_send, '=', sizeof(str_send));
                str_send[43] = 0;
                putsUart0(str_send);
                putsUart0("\r\n\r\n");
            }
            else if(strcmp(&str_parse[0][0], "ipcs") == 0)      // inter-process communications
            {
                // Sending semaphore information
                putsUart0("Semaphore List\r\n");
                putsUart0("====================\r\n");
                for(i=0; i<semaphoreCount; i++)
                {
                    putsUart0("Name:");
                    putsUart0(semaphores[i].name);
                    putsUart0("\r\n");

                    ltoa(semaphores[i].count, str_send);
                    putcUart0(9);   // write tab
                    putsUart0("Count:");
                    putsUart0(str_send);
                    putsUart0("\r\n");

                    ltoa(semaphores[i].queueSize, str_send);
                    putcUart0(9);   // write tab
                    putsUart0("Queue Size:");
                    putsUart0(str_send);
                    putsUart0("\r\n");

                    putcUart0(9);   // write tab
                    putsUart0("In Queue:\r\n");
                    putcUart0(9);
                    if(semaphores[i].queueSize > 0)
                    {
                        uint8_t j = 0;
                        for(j = 0; j<semaphores[i].queueSize; j++)
                        {
                            putcUart0(j+'1');
                            putsUart0("->");
                            putsUart0(tcb[semaphores[i].processQueue[j]].name);
                            putsUart0("\r\n");
                        }
                    }
                    else
                    {
                        putsUart0("Empty\r\n");
                    }
                    putsUart0("\r\n");
                }
                putsUart0("====================\r\n\r\n");
            }
            else if(strcmp(&str_parse[0][0], "setting") == 0)
            {
                // displaying scheduling, preemption, inheritence
                putsUart0("Pri Scheduling:");
                putcUart0(priorityScheduling+'0');
                putsUart0("\r\n");

                putsUart0("Preemption:");
                putcUart0(preemption+'0');
                putsUart0("\r\n");

                putsUart0("Pri Inheritence:");
                putcUart0(priorityInheritence+'0');
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            else
            {
                putsUart0("It's not a cmd.\r\n\r\n");
            }

            memset(str, 0, MAX_CHARS);      // Make all data in the array str to NULL
            flag_null = 0;
        }
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    rtosInit();

    // Configure 32/64-bit Wide Timer 5
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;                    //1. turn-on wide timer1 clock
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                               //2. turn-off counter before reconfiguring
    WTIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                         //3. configure as 64-bit counter for Wide Timer(32/64bit)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;    //4. configure for periodc mode, count up
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                                //8. turn-on counter

    WTIMER1_TAV_R = 0;

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1, "keyPressed");
    keyReleased = createSemaphore(0, "keyReleased");
    flashReq = createSemaphore(5, "flashReq");
    resource = createSemaphore(1, "resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 4);
    ok &= createThread(flash4Hz, "Flash4Hz", 0);
    ok &= createThread(oneshot, "OneShot", -4);
    ok &= createThread(readKeys, "ReadKeys", 4);
    ok &= createThread(debounce, "Debounce", 4);
    ok &= createThread(important, "Important", -8);
    ok &= createThread(uncooperative, "Uncoop", 2);
    ok &= createThread(shell, "Shell", 0);

//    priorityScheduling = true;

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}
