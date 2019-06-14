/*
Author:SHIELD_QI
Date:2018-06-30
************************************/

#include "can.h"

//------------------------------------Defination of Msg Structure------------------------------------/


//---------------CAN2_���߼�ͣ��ת����ơ�����վ---------------/
/*���սṹ��*/
CanRxMsg CAN2_RxMessage=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,3}
};
/*������*/
CanRxMsg Screen_Rx_Message=
{
	0x7FF,0x7FF,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,0}
};
CanTxMsg Screen_EBS_Tx_Message=//0x47C
{
	0x564,0x564,CAN_Id_Standard,CAN_RTR_Data,8,
	{2,0x0a,0x00,0x00,1,0,0,0}
};
/*RES*/
CanRxMsg RES_Rx_Message=
{
	0x336,0x336,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0,0,0,0,0,0,0}
};
/*EBS*/
CanRxMsg EBS_Rx_Message=
{
	0x7FF,0x7FF,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,0}
};
CanTxMsg EBS_Tx_Message=
{
	0x566,0x566,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0x02,0x00,0x00,1,0,0,0}
};
/*ת�������*///data[6]==1������������Ϊ0����
CanRxMsg Steering_Rx_Message=
{
	0x232,0x232,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,0}
};
//ת��ʹ��
CanTxMsg Steering_Tx_Message0=
{
	0x439,0x439,CAN_Id_Standard,CAN_RTR_Data,8,
	{1,0x02,0x79,0x00,0,0,0,0}
};
//ת�����
CanTxMsg Steering_Tx_Message=
{
	0x439,0x439,CAN_Id_Standard,CAN_RTR_Data,8,
	{2,0x03,0x00,0x00,0,0,0,0}
};
/*ESC��*/
CanRxMsg ESC_Rx_Message=
{
	0x7FF,0x7FF,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,0}
};
CanTxMsg ESC_Tx_Message=
{
	0x221,0x221,CAN_Id_Standard,CAN_RTR_Data,8,
	{0,0x02,0x00,0x00,1,0,0,0}
};
//---------------CAN1_������Ϣ�����ͽṹ��-----------------/
/*ROS����*/
CanTxMsg ROS_TxMsg=
{
	0x439,0x439,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x51,0x04,0x00,0,0,0,0,0}
};
/*���սṹ��*/
CanRxMsg CAN1_RxMessage=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,8,
	{0x02,0,0,0,0,0,0,3}
};
/*ʹʧ�ܿ���*/
CanTxMsg Disable_Msg=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,3,
	{0x51,0x04,0x00,0,0,0,0,0}
};
/*�ٶȿ���*/
CanTxMsg Speed_Cntrol_Msg=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,3,
	{0x31,0x00,0x00,0,0,0,0,0}
};
/*Ť�ؿ���*/
CanTxMsg Torque_Cntrol_Msg=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,3,
	{0x90,0xFC,0x3F,0,0,0,0,0}
};
/*λ�ÿ���*/
CanTxMsg Position_Cntrol_Msg=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,5,
	{0x6E,0xC0,0xC6,0x2D,0x00,0,0,0}
};
/*���󷵻��ٶ�ֵ*/
CanTxMsg SpeedGet_Command_Msg=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,3,
	{0x3D,0x30,0x64,0,0,0,0,0}
};
/*���󷵻�״̬��Ϣ*/
CanTxMsg StatuGet_Command_Msg=
{
	0x201,0x201,CAN_Id_Standard,CAN_RTR_Data,3,
	{0x3D,0x40,0x00,0,0,0,0,0}
};
//-----------------CAN1_���״̬��ȡ�����սṹ��-----------------/
/*�����ٶ�ֵ*/
CanRxMsg Speed_Get_Msg=
{
	0x190,0x190,CAN_Id_Standard,CAN_RTR_Data,4,
	{0x30,0xFF,0x7F,0x00,0,0,0,0}
};
/*����״̬��Ϣ*/
CanRxMsg Statu_Get_Msg=
{
	0x181,0x181,CAN_Id_Standard,CAN_RTR_Data,4,
	{0x40,0x81,0x01,0x00,0,0,0,0}
};

//------------------------------------CAN1 Configure------------------------------------/

void CAN1_Config(void)
{
	
	//------------------ GPIO��ʼ�� ------------------/
 	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(CAN1_TX_GPIO_CLK|CAN1_RX_GPIO_CLK, ENABLE);
	
	//���Ÿ���
  GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
  GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT); 

  //Config CAN TX pins
  GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);
	
	//Configure CAN RX  pins
  GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);

  //�����ж�����
	NVIC_InitTypeDef 			NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	//------------------ CAN1��ʼ�� ------------------/
	CAN_InitTypeDef        CAN_InitStructure;
  RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);

	//CAN�Ĵ�����ʼ��
	CAN_DeInit(CANx1);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   		//MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
	CAN_InitStructure.CAN_ABOM=ENABLE;			  	  //MCR-ABOM  �Զ����߹��� 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   		//MCR-AWUM  ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART=DISABLE;			  	  //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM=DISABLE;				    //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStructure.CAN_TXFP=DISABLE;			  	  //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //��������ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		    //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
	 
	// ss=1 bs1=7 bs2=6 λʱ����Ϊ(1+7+6) �����ʼ�Ϊʱ������tq*(1+7+6)
	
	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq;		    //BTR-TS1 ʱ���1 ռ����7��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;		    //BTR-TS1 ʱ���2 ռ����6��ʱ�䵥Ԫ	
	
	// CAN Baudrate = 500 kBps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB 1 = 42 MHz) 
	
	CAN_InitStructure.CAN_Prescaler =6;		        //BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 42/(1+7+6)/6=500 kbps
	CAN_Init(CANx1, &CAN_InitStructure);

  //---------------- ����CAN1ɸѡ�� ----------------/
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	CAN_FilterInitStructure.CAN_FilterNumber=1;												  //ɸѡ����1
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;				//����������ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;			//ɸѡ��λ��Ϊ����32λ��
  
	CAN_FilterInitStructure.CAN_FilterIdHigh= 													//Ҫɸѡ��ID��λ 			
	((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		
	CAN_FilterInitStructure.CAN_FilterIdLow= 														//Ҫɸѡ��ID��λ 
	(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; 
	
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0;										//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0;											//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;	//ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;								//ʹ��ɸѡ��
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CANx1, CAN_IT_FMP0, ENABLE);														//CANͨ���ж�ʹ��
	
	CANx1->RF0R |= CAN_RF0R_RFOM0;
	CANx1->RF0R |= CAN_RF0R_RFOM0;
	CANx1->RF0R |= CAN_RF0R_RFOM0;
	
}

//------------------------------------CAN2 Configure------------------------------------/

void CAN2_Config(void)
{
	//------------------ GPIO��ʼ�� ------------------/
 	GPIO_InitTypeDef			 GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(CAN2_TX_GPIO_CLK|CAN2_RX_GPIO_CLK, ENABLE);
	
	//���Ÿ���
  GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
  GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT); 

	//Config CAN TX pins
  GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);
	
	//Config CAN RX pins
  GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);


	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/*�ж�����*/
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX_IRQ;	   				//CAN RX0�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	  //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			    //�����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CANͨ�Ų�������**********************************/
	/* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);

	/*CAN�Ĵ�����ʼ��*/
	CAN_DeInit(CANx2);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  �Զ����߹��� 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		 //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
	 
	/* ss=1 bs1=7 bs2=6 λʱ����Ϊ(1+7+6) �����ʼ�Ϊʱ������tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq;		 //BTR-TS1 ʱ���1 ռ����4��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;		 //BTR-TS1 ʱ���2 ռ����2��ʱ�䵥Ԫ	
	
	/* CAN Baudrate = 500k Bps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =6;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 42/(1+4+2)/6=1 Mbps
	CAN_Init(CANx2, &CAN_InitStructure);


	CAN_FilterInitTypeDef  CAN_FilterInitStructure;


	CAN_FilterInitStructure.CAN_FilterNumber=14;																//ɸѡ����14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;								//����������ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;							//ɸѡ��λ��Ϊ����32λ��

	CAN_FilterInitStructure.CAN_FilterIdHigh=
	((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;								//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdLow=
	(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;												  //Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0;														//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0;															//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;					//ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;												//ʹ��ɸѡ��
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/*CANͨ���ж�ʹ��*/	
	CANx1->RF0R |= CAN_RF0R_RFOM0;
	
	CAN_ITConfig(CANx2, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CANx2, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CANx2, CAN_IT_FMP0, ENABLE);
}



void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t ubCounter = 0;

	/*�ѽ��սṹ������*/
  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    RxMessage->Data[ubCounter] = 0x00;
  }
}

void CAN_SetMsg(CanTxMsg *TxMessage)
{	  
	uint8_t ubCounter = 0;

  //TxMessage.StdId=0x00;						 
  TxMessage->ExtId=0x1314;						 //ʹ�õ���չID
  TxMessage->IDE=CAN_ID_EXT;					 //��չģʽ
  TxMessage->RTR=CAN_RTR_DATA;				 //���͵�������
  TxMessage->DLC=8;										 //���ݳ���Ϊ8�ֽ�
	
	/*����Ҫ���͵�����0-7*/
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    TxMessage->Data[ubCounter] = ubCounter;
  }
}

/************************END OF FILE***********************/



