/*AREA| .text|,CODE,READONLY,ALIGN=2
THUMB
EXTERN currentPt

SysTick_Handler
	CPSID I
	PUSH {R4-R11}
	LDR R0,=currentPt
	LDR R1,[R0]
	STR SP,[R1]
*/

.syntax unified
.section .text, "ax"
.balign 4

.section	.text.SysTick_Handler
.type	SysTick_Handler, %function
SysTick_Handler: /*as we are in irq handler, meaning cotext switch occured,
				  *meaning r0, r1, r2, r3, r12, lr, pc, psr already saved on stack*/
	CPSID I
	PUSH {R4, R11} /*save the rest of the registers (r4, r5, r6, r7, r8, r9, r10, r11) on stack*/
	LDR R0,=currentPt
	LDR R1,[R0]
	STR SP,[R1]

	LDR R1,[R1, #4]
	STR R1,[R0]
	LDR SP,[R1]
	POP {R4, R11}
	CPSIE I
	BX LR

//.section	.text.osSchedulerLaunch
////.global osSchedulerLaunch
//.type	osSchedulerLaunch, %function
//osSchedulerLaunch:
//	LDR R0,=currentPt
//	LDR R2,[R0]
//	LDR SP,[R2]
//	POP {R4, R11}
//	POP {R0, R3}
//	POP {R12}
//	ADD SP, SP, #4
//	POP {LR}
//	ADD SP, SP, #4
//	CPSIE I
//	BX LR
