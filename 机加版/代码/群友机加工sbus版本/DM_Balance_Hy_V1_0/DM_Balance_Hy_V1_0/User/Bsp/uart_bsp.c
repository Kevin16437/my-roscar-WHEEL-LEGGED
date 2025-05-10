#include "uart_bsp.h"
#include "string.h"
#include "usart.h"

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define REMOTE_RC_OFFSET 1024
#define REMOTE_TOGGLE_DUAL_VAL 1024
#define REMOTE_TOGGLE_THRE_VAL_A 600
#define REMOTE_TOGGLE_THRE_VAL_B 1400
#define DEAD_AREA	120


uint8_t rx_buff[BUFF_SIZE];
remoter_t remoter;

void sbus_frame_parse(remoter_t *remoter, uint8_t *buf)
{
    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
        return;

    if (buf[23] == 0x0C)
        remoter->online = 0;
    else
        remoter->online = 1;

    remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
    remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
		
		remoter->joy.left_hori = remoter->rc.ch[4-1] - REMOTE_RC_OFFSET;
		remoter->joy.left_vert = remoter->rc.ch[3-1]- REMOTE_RC_OFFSET;
		remoter->joy.right_hori = remoter->rc.ch[1-1] - REMOTE_RC_OFFSET;
		remoter->joy.right_vert = remoter->rc.ch[2-1] - REMOTE_RC_OFFSET;
		
		if( remoter->joy.left_hori > -DEAD_AREA && remoter->joy.left_hori < DEAD_AREA)
			remoter->joy.left_hori = 0;
		if( remoter->joy.right_hori > -DEAD_AREA && remoter->joy.right_hori < DEAD_AREA)
			remoter->joy.right_hori = 0;
		if( remoter->joy.right_vert > -DEAD_AREA && remoter->joy.right_vert < DEAD_AREA)
			remoter->joy.right_vert = 0;
		
		if(remoter->rc.ch[5-1] < REMOTE_TOGGLE_DUAL_VAL)
		{
			remoter->toggle.swa = 0;
		}
		else if(remoter->rc.ch[5-1] >= REMOTE_TOGGLE_DUAL_VAL)
		{
			remoter->toggle.swa = 1;
		}
		
		if(remoter->rc.ch[6-1] < REMOTE_TOGGLE_THRE_VAL_A)
		{
			remoter->toggle.swb = 0;
		}
		else if(remoter->rc.ch[6-1] >= REMOTE_TOGGLE_THRE_VAL_A && remoter->rc.ch[6-1] <= REMOTE_TOGGLE_THRE_VAL_B)
		{
			remoter->toggle.swb = 1;
		}
		else if(remoter->rc.ch[6-1] >= REMOTE_TOGGLE_THRE_VAL_B)
		{
			remoter->toggle.swb = 2;
		}
		
		if(remoter->rc.ch[7-1] < REMOTE_TOGGLE_THRE_VAL_A)
		{
			remoter->toggle.swc = 0;
		}
		else if(remoter->rc.ch[7-1] >= REMOTE_TOGGLE_THRE_VAL_A && remoter->rc.ch[7-1] <= REMOTE_TOGGLE_THRE_VAL_B)
		{
			remoter->toggle.swc = 1;
		}
		else if(remoter->rc.ch[7-1] >= REMOTE_TOGGLE_THRE_VAL_B)
		{
			remoter->toggle.swc = 2;
		}
		
		if(remoter->rc.ch[8-1] < REMOTE_TOGGLE_DUAL_VAL)
		{
			remoter->toggle.swd = 0;
		}
		else if(remoter->rc.ch[8-1] >= REMOTE_TOGGLE_DUAL_VAL)
		{
			remoter->toggle.swd = 1;
		}
		
		remoter->var.a = remoter->rc.ch[9-1];
		remoter->var.b = remoter->rc.ch[10-1];
		
		remoter->sbus_recever_time = HAL_GetTick();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == UART5)
	{
		if (Size <= BUFF_SIZE)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ������Ϻ�����
			sbus_frame_parse(&remoter, rx_buff);
			
//			memset(rx_buff, 0, BUFF_SIZE);
		}
		else  // �������ݳ��ȴ���BUFF_SIZE��������
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ������Ϻ�����
			memset(rx_buff, 0, BUFF_SIZE);							   
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ���շ������������
		memset(rx_buff, 0, BUFF_SIZE);							   // ������ջ���		
	}
}
