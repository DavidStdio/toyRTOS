#ifndef _OS_KERNEL_H
#define _OS_KERNEL_H

#include <stdint.h>
#include "stm32l4xx.h"


typedef void (*task)(void);

void osKernelLaunch(uint32_t quanta);
void osKernelInit(void);
uint8_t osKernelAddThreads(task tsk,
						   task tsk1,
						   task tsk2);
void osThreadYield(void);
#endif
