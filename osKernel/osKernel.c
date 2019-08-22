#include "osKernel.h"

#define NVIC_SYSTICKSET_BIT		( 1UL << 26UL )
#define INT_CTRL_REG		( * ( ( volatile uint32_t * ) 0xe000ed04 ) )

#define NUM_OF_THREADS 3
#define STACK_SIZE     100

#define BUS_FREQ 16000000
uint32_t MILLIS_PRESCALER;

struct tcb{
	int32_t *stackpt;
	struct tcb *nextPt;
};

typedef struct tcb tcbType;
tcbType tcbs[NUM_OF_THREADS];
tcbType *currentPt;
int32_t TCB_STACK[NUM_OF_THREADS][STACK_SIZE];
void osSchedulerLaunch(void);

static void osKernelStackInit(task tsk, int i)
{
	tcbs[i].stackpt = &TCB_STACK[i][STACK_SIZE - 16];
	TCB_STACK[i][STACK_SIZE - 1] = 0x01000000;
	TCB_STACK[i][STACK_SIZE - 2] = (int32_t)(tsk);
}

uint8_t osKernelAddThreads(task tsk,
						   task tsk1,
						   task tsk2)
{
	__disable_irq();
	tcbs[0].nextPt = &tcbs[1];
	tcbs[1].nextPt = &tcbs[2];
	tcbs[2].nextPt = &tcbs[0];

	osKernelStackInit( tsk, 0);
	osKernelStackInit( tsk1,1);
	osKernelStackInit( tsk2,2);

	currentPt = &tcbs[0];
	__enable_irq();

	return 1;
}

void osKernelInit(void)
{
	__disable_irq();

	MILLIS_PRESCALER = BUS_FREQ / 1000;
}

void __attribute__ (( naked )) SysTick_Handler(void)
{
				/*as we are in irq handler, meaning cotext switch occured,
				  *meaning r0, r1, r2, r3, r12, lr, pc, psr already saved on stack*/
	__asm volatile
	(
	"CPSID I         \n"
	"PUSH {R4-R11}    \n"/*save the rest of the registers (r4, r5, r6, r7, r8, r9, r10, r11) on stack*/
	"LDR R0,currentPtConst2  \n"
	"LDR R1,[R0]        \n"
	"STR SP,[R1]        \n"

	"LDR R1,[R1, #4]        \n"
	"STR R1,[R0]        \n"
	"LDR SP,[R1]        \n"
	"POP {R4-R11}        \n"
	"CPSIE I        \n"
	"BX LR        \n"
	"currentPtConst2 : .word currentPt \n"
	);
}

void __attribute__ (( naked )) osSchedulerLaunch(void)
{
//	__asm volatile
//	(
//	"LDR R0, currentPtConst \n "
//	"LDR R2,[R0]       \n "
//	"LDR SP,[R2]	   \n "
//	"POP {R4-R11}     \n "
//	"POP {R0-R3}      \n "
//	"POP {R12}         \n "
//	"ADD SP, SP, #4    \n "
//	"POP {LR}          \n "
//	"ADD SP, SP, #4    \n "
//	"CPSIE I           \n "
//	"BX LR        	   \n "
//	"currentPtConst : .word currentPt \n"
//	);
//	__asm volatile
//	(
//	"	mrs r0, psp							\n"
//	"	isb									\n"
//	"										\n"
//	"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
//	"	ldr	r2, [r3]						\n"
//	"										\n"
//	"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, push high vfp registers. */
//	"	it eq								\n"
//	"	vstmdbeq r0!, {s16-s31}				\n"
//	"										\n"
//	"	stmdb r0!, {r4-r11, r14}			\n" /* Save the core registers. */
//	"										\n"
//	"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
//	"										\n"
//	"	stmdb sp!, {r3}						\n"
//	"	mov r0, %0 							\n"
//	"	msr basepri, r0						\n"
//	"	dsb									\n"
//	"	isb									\n"
//	"	bl vTaskSwitchContext				\n"
//	"	mov r0, #0							\n"
//	"	msr basepri, r0						\n"
//	"	ldmia sp!, {r3}						\n"
//	"										\n"
//	"	ldr r1, [r3]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
//	"	ldr r0, [r1]						\n"
//	"										\n"
//	"	ldmia r0!, {r4-r11, r14}			\n" /* Pop the core registers. */
//	"										\n"
//	"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, pop the high vfp registers too. */
//	"	it eq								\n"
//	"	vldmiaeq r0!, {s16-s31}				\n"
//	"										\n"
//	"	msr psp, r0							\n"
//	"	isb									\n"
//	"										\n"
//	#ifdef WORKAROUND_PMU_CM001 /* XMC4000 specific errata workaround. */
//		#if WORKAROUND_PMU_CM001 == 1
//	"			push { r14 }				\n"
//	"			pop { pc }					\n"
//		#endif
//	#endif
//	"										\n"
//	"	bx r14								\n"
//	"										\n"
//	"	.align 4							\n"
//	"pxCurrentTCBConst: .word pxCurrentTCB	\n"
//	::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
//	);

}

void osKernelLaunch(uint32_t quanta)
{
	SysTick->CTRL = 0;
	SysTick->VAL = 0;
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
	SysTick->LOAD = (quanta * MILLIS_PRESCALER) - 1;
	SysTick->CTRL = 0x00000007;

	//osSchedulerLaunch();

	__asm volatile
	(
	"LDR R0, currentPtConst \n "
	"LDR R2,[R0]       \n "
	"LDR SP,[R2]	   \n "
	"POP {R4-R11}     \n "
	"POP {R0-R3}      \n "
	"POP {R12}         \n "
	"ADD SP, SP, #4    \n "
	"POP {LR}          \n "
	"ADD SP, SP, #4    \n "
	"CPSIE I           \n "
	"BX LR        	   \n "
	"currentPtConst : .word currentPt \n"
	);
}

void osThreadYield(void)
{
	INT_CTRL_REG = NVIC_SYSTICKSET_BIT;
}
