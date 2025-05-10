#ifndef __UART_BSP_H__
#define __UART_BSP_H__

#include "main.h"

#define BUFF_SIZE	25

extern uint8_t rx_buff[BUFF_SIZE];


typedef struct
{
    uint16_t online;
		uint32_t sbus_recever_time;

    struct
    {
        int16_t ch[10];
    } rc;

    struct
    {
        /* STICK VALUE */
        int16_t left_vert;
        int16_t left_hori;
        int16_t right_vert;
        int16_t right_hori;
    } joy;
		
		struct
		{
			//l1 l2 r2 r1
			uint8_t swa;//2-Stop
			uint8_t swb;//3-Stop
			uint8_t swc;//3-Stop
			uint8_t swd;//2-Stop
		} toggle;

    struct
    {
        /* VAR VALUE */
        float a;
        float b;
    } var;

    struct
    {
        /* KEY VALUE */
        uint8_t l;
				uint8_t r;
    } key;
} remoter_t;

extern remoter_t remoter;

#endif /*__UART_BSP_H__ */

