#ifndef __WATCH_TASK_H
#define __WATCH_TASK_H

#include "main.h"
#include "uart_bsp.h"
#include "chassisR_task.h"
#include "cmsis_os.h"

#define VBAT_WARNNING_VAL_6S 22.0f
#define VBAT_LOW_VAL_6S 21.0f

#define VBAT_WARNNING_VAL_4S 14.7f
#define VBAT_LOW_VAL_4S 14.0f

extern float VBAT_WARNNING_VAL;
extern float VBAT_LOW_VAL;

void Watch_task(void);

#endif
