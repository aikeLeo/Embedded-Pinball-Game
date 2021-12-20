#ifndef __KEY_H
#define __KEY_H	 
//#include "sys.h" 
#include "stm32f4xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
#define KEY0 		HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)//GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4) //PE4
#define KEY1 		HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)//GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)	//PE3 
#define KEY2 		HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)//GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) //PE2
#define WK_UP 	    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)//GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)	//PA0


/*���淽ʽ��ͨ��λ��������ʽ��ȡIO*/
/*
#define KEY0 		PEin(4)   	//PE4
#define KEY1 		PEin(3)		//PE3 
#define KEY2 		PEin(2)		//P32
#define WK_UP 	PAin(0)		//PA0
*/


#define KEY0_PRES 	1
#define KEY1_PRES	2
#define KEY2_PRES	3
#define WKUP_PRES   4

//void KEY_Init(void);	//IO��ʼ��
unsigned char KEY_Scan(unsigned char mode);  		//����ɨ�躯��	

#endif
