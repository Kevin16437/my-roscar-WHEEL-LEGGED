/**************************************************************************
Name		: watch task
Input   : none
Output  : none
Auth    : DHY (qq:965849293)
Date		: 2024
**************************************************************************/	
#include "watch_task.h"
#include "tim.h"

extern chassis_t chassis_move;
float VBAT_WARNNING_VAL = 0.0f;
float VBAT_LOW_VAL = 0.0f;

uint32_t WATCH_TIME=2000;//1S

/**************************************************************************
Function: Watch_Task
Des			: Alert voltage,remoter
Input   : none
Output  : none
Auth    : DHY (qq:965849293)
Date		: 2024
**************************************************************************/	
void Watch_task(void)
{
	while(1)
	{
		if(chassis_move.vbus < VBAT_WARNNING_VAL)
		{
			TIM12->CCR2 = 450;
			
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
			osDelay(100);
			
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			osDelay(100);
			
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
			osDelay(100);
			
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			//osDelay(100);	
		}
		
		if(!remoter.online)
		{
			TIM12->CCR2 = 450;
			
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
			osDelay(50);
			
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			osDelay(50);
			
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
			osDelay(50);
			
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			osDelay(50);	
			
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
			osDelay(50);
			
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			//osDelay(50);	
		}
		osDelay(WATCH_TIME);
	}
}
