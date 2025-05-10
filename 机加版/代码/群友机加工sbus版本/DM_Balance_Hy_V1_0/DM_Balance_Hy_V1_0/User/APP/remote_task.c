#include "remote_task.h"
#include "watch_task.h"
#include "cmsis_os.h"


#define REMOTE_OVERTIME	1000

extern chassis_t chassis_move;
extern INS_t INS;
uint32_t REMOTE_TIME=10;//ps2手柄任务周期是10ms

extern vmc_leg_t right;			
extern vmc_leg_t left;

extern uint16_t adc_val[2];


/**************************************************************************
Function: Sbus Remote
Input   : none
Output  : none
Auth    : DHY (qq:965849293)
Date		: 2024
**************************************************************************/	
void Remote_task(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2);
	float dt = REMOTE_TIME / 1000.0f;
	static float last_jump_vrb = 0;
	static uint32_t vbat_low_count = 0;
	
	while(1)
	{
		chassis_move.vbus = (adc_val[0]*3.3f/65535)*11.0f;
		
		if(chassis_move.vbus_mode == 0)
		{
			
			if(chassis_move.vbus > VBAT_LOW_VAL_6S)
			{
				//6s
				VBAT_WARNNING_VAL = VBAT_WARNNING_VAL_6S;
				VBAT_LOW_VAL = VBAT_LOW_VAL_6S;
				chassis_move.vbus_mode = 2;
			}
			else if(chassis_move.vbus > VBAT_LOW_VAL_4S)
			{
				//4s
				VBAT_WARNNING_VAL = VBAT_WARNNING_VAL_4S;
				VBAT_LOW_VAL = VBAT_LOW_VAL_4S;
				chassis_move.vbus_mode = 1;
			}
			else
			{
				chassis_move.vbus_mode = 0;
				VBAT_WARNNING_VAL = 0.0f;
				VBAT_LOW_VAL = 0.0f;
			}
			
		}
		
		if((HAL_GetTick() - remoter.sbus_recever_time) > REMOTE_OVERTIME)
		{
			remoter.online = 0;
		}
		
		if(remoter.online)
		{
			if(remoter.toggle.swd==0)
			{
				//Power Off
				chassis_move.start_flag=0;
				chassis_move.recover_flag=0;
				
				vbat_low_count = 0;
			}
			else if(remoter.toggle.swd==1)
			{
//				if(chassis_move.vbus > VBAT_LOW_VAL)
//				{
//					//Power On
//					chassis_move.start_flag=1;
//				}
//				else
//				{
//					
//					//Power Off
//					chassis_move.start_flag=0;
//					chassis_move.recover_flag=0;
//				}
				
				if((chassis_move.vbus < VBAT_LOW_VAL)&&(chassis_move.start_flag==1))
					vbat_low_count++;
				
				if(vbat_low_count>100)
				{
					chassis_move.start_flag=0;
					chassis_move.recover_flag=0;
				}
				else
				{
					//Power On
					chassis_move.start_flag=1;
				}
				
			}
			
			if(chassis_move.recover_flag==0
					&&((chassis_move.myPithR<((-3.1415926f)/4.0f)&&chassis_move.myPithR>((-3.1415926f)/2.0f))
					||(chassis_move.myPithR>(3.1415926f/4.0f)&&chassis_move.myPithR<(3.1415926f/2.0f))))
			{
				chassis_move.recover_flag=1;//需要自起
				chassis_move.leg_set=0.08f;//原始腿长
			}
			
			if(chassis_move.start_flag==1)
			{
				if(chassis_move.vbus_mode == 1)
				{
					chassis_move.v_set=((float)remoter.joy.right_vert)*(-0.00097f * 0.5f);//往前大于0
				}
				else if(chassis_move.vbus_mode == 2)
				{
					//6s
					chassis_move.v_set=((float)remoter.joy.right_vert)*(-0.00097f * 1.3f);//往前大于0
				}
				chassis_move.x_set=chassis_move.x_set+chassis_move.v_set*dt;
				
				chassis_move.turn_set += ((float)remoter.joy.left_hori)*(-0.0000625f);
				
				if(remoter.toggle.swa == 0)
				{
					chassis_move.roll_set = -0.03f;
				}
				else
				{
					chassis_move.roll_set += ((float)remoter.joy.right_hori)*(0.000016f);
				}
				mySaturate(&chassis_move.roll_set,-0.40f,0.40f);
				
				if(remoter.toggle.swc == 0)
				{
					chassis_move.leg_set = 0.08f;
				}
				else
				{
					chassis_move.leg_set= (remoter.joy.left_vert + 1024)*(0.00007f) + 0.072f;//chassis_move.leg_set+((float)(data->ly-128))*(-0.000016f); 
				}
				
				mySaturate(&chassis_move.leg_set,0.072f,0.21f);
				
				if(fabsf(chassis_move.last_leg_set-chassis_move.leg_set)>0.0001f)
				{//遥控器控制腿长在变化
					right.leg_flag=1;	//为1标志着遥控器在控制腿长伸缩，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判为离地了
					left.leg_flag=1;	 			
				}
				chassis_move.last_leg_set=chassis_move.leg_set;
				
				if(remoter.toggle.swb == 0)
				{
						//chassis_move.jump_flag=0;
						//chassis_move.jump_flag2=0;
				}
				else
				{
					if(remoter.toggle.swc != 0 && remoter.var.b < 500 && last_jump_vrb>=500 && chassis_move.leg_set <= 0.16f)
					{
						if(chassis_move.vbus_mode == 2)
						{
							//6s
							chassis_move.jump_flag=1;
							chassis_move.jump_flag2=1;
					
						}
					}
				}
				
				last_jump_vrb = remoter.var.b;
			}
			
			else
			{
				chassis_move.v_set=0.0f;//清零
				chassis_move.x_set=chassis_move.x_filter;//保存
				chassis_move.turn_set=chassis_move.total_yaw;//保存
				chassis_move.leg_set=0.08f;//原始腿长
				chassis_move.roll_set=-0.03f; 
			}
			
		}
		else
		{
			chassis_move.start_flag=0;
			chassis_move.recover_flag=0;
			vbat_low_count = 0;
			
			chassis_move.v_set=0.0f;//清零
			chassis_move.x_set=chassis_move.x_filter;//保存
			chassis_move.turn_set=chassis_move.total_yaw;//保存
			chassis_move.leg_set=0.08f;//原始腿长
			chassis_move.roll_set=-0.03f; 
		}
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // 接收完毕后重启
		osDelay(REMOTE_TIME);
	}
}


