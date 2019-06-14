#ifndef __CAN_H
#define	__CAN_H

//CAN1 and CAN2 are on APB1 at 42Mhz(max)
//CAN×ÜÏß£ºCAN1 for Motor,CAN2 for computer£¬EBS£¬etc
//CAN1: RX->PA11  TX->PA12
//CAN2: RX->PB12  TX->PB13

#include "stm32f4xx.h"

//----------------------------Defination for CAN1------------------------------

#define CANx1                       CAN1
#define CAN1_CLK                    RCC_APB1Periph_CAN1
#define CAN1_RX_IRQ								  CAN1_RX0_IRQn
#define CAN1_RX_IRQHandler				 	CAN1_RX0_IRQHandler

#define CAN1_RX_PIN                 GPIO_Pin_11
#define CAN1_TX_PIN                 GPIO_Pin_12
#define CAN1_TX_GPIO_PORT           GPIOA
#define CAN1_RX_GPIO_PORT           GPIOA
#define CAN1_TX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define CAN1_RX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define CAN1_AF_PORT                GPIO_AF_CAN1
#define CAN1_RX_SOURCE              GPIO_PinSource11
#define CAN1_TX_SOURCE              GPIO_PinSource12

//----------------------------Defination for CAN2------------------------------

#define CANx2                      	CAN2
#define CAN2_CLK                    RCC_APB1Periph_CAN2
#define CAN2_RX_IRQ								  CAN2_RX0_IRQn
#define CAN2_RX_IRQHandler	   	    CAN2_RX0_IRQHandler

#define CAN2_RX_PIN                 GPIO_Pin_12
#define CAN2_TX_PIN                 GPIO_Pin_13
#define CAN2_TX_GPIO_PORT           GPIOB
#define CAN2_RX_GPIO_PORT           GPIOB
#define CAN2_TX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CAN2_RX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CAN2_AF_PORT                GPIO_AF_CAN2
#define CAN2_RX_SOURCE              GPIO_PinSource12
#define CAN2_TX_SOURCE              GPIO_PinSource13 

//--------------------------Declaration of functions---------------------------

u8 CAN1_Send_DriverMsg(CanTxMsg msg);
u8 CAN1_Receive_Msg(CanRxMsg *msg);

u8 CAN2_Send_DriverMsg(CanTxMsg msg);
u8 CAN2_Receive_Msg(CanRxMsg *msg);

void CAN1_Config(void);
void CAN2_Config(void);
//void CAN_SetMsg(CanTxMsg *TxMessage);
//void Init_RxMes(CanRxMsg *RxMessage);


#endif
