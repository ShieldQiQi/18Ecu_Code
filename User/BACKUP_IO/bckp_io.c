/*
Author:SHIELD_QI
Date:2018-07-17
************************************/

#include "bckp_io.h"

//备用IO口-输出与输入

void BCKP_IO_GPIO_Config(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure_out;
	GPIO_InitTypeDef GPIO_InitStructure_in;

	RCC_AHB1PeriphClockCmd ( BCKP_OUT1_GPIO_CLK|q_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd ( RUN_ENABLE_GPIO_CLK|BCKP_IN2_GPIO_CLK|BCKP_IN3_GPIO_CLK|BCKP_IN4_GPIO_CLK, ENABLE);  	

//	//三路备用输出
	GPIO_InitStructure_out.GPIO_Mode = GPIO_Mode_OUT;   
	GPIO_InitStructure_out.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_out.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure_out.GPIO_Speed = GPIO_Speed_50MHz; 

	GPIO_InitStructure_out.GPIO_Pin = BCKP_OUT1_PIN;
	GPIO_Init(BCKP_OUT1_GPIO_PORT, &GPIO_InitStructure_out);	

	GPIO_ResetBits(BCKP_OUT1_GPIO_PORT, BCKP_OUT1_PIN);

//	GPIO_InitStructure_out.GPIO_Pin = BCKP_OUT2_PIN;
//	GPIO_Init(BCKP_OUT2_GPIO_PORT, &GPIO_InitStructure_out);	

//	GPIO_InitStructure_out.GPIO_Pin = BCKP_OUT3_PIN;
//	GPIO_Init(BCKP_OUT3_GPIO_PORT, &GPIO_InitStructure_out);	
	
	//四路备用输入
	GPIO_InitStructure_in.GPIO_Mode = GPIO_Mode_IN;   
	GPIO_InitStructure_in.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure_in.GPIO_Speed = GPIO_Speed_50MHz; 

	GPIO_InitStructure_in.GPIO_Pin = RUN_ENABLE_PIN;
	GPIO_Init(RUN_ENABLE_GPIO_PORT, &GPIO_InitStructure_in);	
	
	GPIO_InitStructure_in.GPIO_Pin = q_PIN;
	GPIO_Init(q_PORT, &GPIO_InitStructure_in);
//	GPIO_InitStructure_in.GPIO_Pin = BCKP_IN2_PIN;
//	GPIO_Init(BCKP_IN2_GPIO_PORT, &GPIO_InitStructure_in);
//	
//	GPIO_InitStructure_in.GPIO_Pin = BCKP_IN3_PIN;
//	GPIO_Init(BCKP_IN3_GPIO_PORT, &GPIO_InitStructure_in);
//	
//	GPIO_InitStructure_in.GPIO_Pin = BCKP_IN4_PIN;
//	GPIO_Init(BCKP_IN4_GPIO_PORT, &GPIO_InitStructure_in);
	
}




