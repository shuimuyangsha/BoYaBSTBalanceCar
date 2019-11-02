/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
 #include "stm32f10x_it.h"
 #include <stdio.h>
 #include "upstandingcar.h"
 #include "outputdata.h"
 #include "mpu6050.h"
 #include "UltrasonicWave.h"
 #include "stm32f10x_exti.h"
 

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

#if 1
void SysTick_Handler(void)				 //5ms��ʱ��
{  
	BST_u8MainEventCount++;				   //��ѭ������ֵ
	BST_u8trig++;
	BST_u8SpeedControlCount++;			  //С���ٶȿ��Ƶ��ü���ֵ

	GetMotorPulse();						//������㺯��

	BST_u8SpeedControlPeriod++;

	BST_u8DirectionControlPeriod++;		   //ת��ƽ������������ֵ
	BST_u8DirectionControlCount++;

	AngleControl();					  //�Ƕ�PD����PWNM���
	MotorOutput();					  //С����PWM���  	

	if(BST_u8trig>=2)
	{
		UltrasonicWave_StartMeasure();	//���ó��������ͳ��� ��Trig�� <10us �ߵ�ƽ		 
		chaoshengbo();			       //���㳬����������
		BST_u8trig=0;
	}
    if(BST_u8SpeedControlCount>=8)       //������ֵ8ʱ������ϵͳ����40msʱ��(ÿ10���Ƕ�PWM���������1���ٶ�PWM����������ܱ����ٶ�PID��������ŽǶ�PID������Ӷ�Ӱ��С��ƽ��)
	{	
		SpeedControl();                     //��ģ�ٶȿ��ƺ���   ÿ40ms����һ��
		BST_u8SpeedControlCount=0;		  //С���ٶȿ��Ƶ��ü���ֵ����
		BST_u8SpeedControlPeriod=0;		  //ƽ���������ֵ����
	}
		


			    
}	   
#endif




#if 0
void SysTick_Handler(void)
{  	
	
	//SampleInputVoltage();
	MPU6050_Pose();
	AngleControl();
	GetMotorPulse();
	SpeedControl();                  
	DirectionControl();
	MotorOutput();
	#if 1

		g_u8LEDCount++;
		if(g_u8LEDCount>=100)
		{
			g_u8LEDCount=0;
			GPIO_ResetBits(GPIOB, GPIO_Pin_3);
		}
		else 
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_3);
		}
#endif         
#if 0  /*������ Ԥ��������*/
   OutData[0] = g_fCarAngle;
   OutData[1] = g_fGravityAngle;
   OutData[2] = g_fGyroAngleSpeed ;
   OutData[3] = g_iAccelInputVoltage_X_Axis;
   
   OutPut_Data();
#endif	  
	
}
#endif
/*
void USART3_IRQHandler(void)
{
	
	u8 ucBluetoothValue;
	if(USART3->SR&(1<<5))//���յ�����
	{	 
		ucBluetoothValue=USART1->DR; 

    }
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
	    ucBluetoothValue = USART_ReceiveData(USART3);
	  	switch (ucBluetoothValue)
	{
	 	  case 0x01 : BST_fCarSpeed_P+=5;break;//BST_fBluetoothSpeed =   210 ; break;	   //��ǰ�ٶ� 250 
	  case 0x02 : BST_fCarSpeed_P-=5;break;//BST_fBluetoothSpeed = (-200);  break;	   //�����ٶ� -250
	  case 0x03 : BST_fCarSpeed_I+=0.05;break;//BST_fBluetoothDirectionSR = 1; break;//��ת
	  case 0x04 : BST_fCarSpeed_I-=0.05;break;//BST_fBluetoothDirectionSL = 1; break;//��ת
	  case 0x05 : BST_fBluetoothDirectionSR = 1; break ;//����
	  case 0x06 : BST_fBluetoothDirectionSL = 1; break ;//����ת
	  case 0x07 : BST_fBluetoothDirectionL  =0; BST_fBluetoothDirectionR = 0; BST_fBluetoothDirectionSL =0; BST_fBluetoothDirectionSR = 0;  break; //ͣ
	  case 0x08 : BST_fBluetoothDirectionSL =0; BST_fBluetoothDirectionSR = 0;directionl=0;directionr=0; break; //ͣ��ת
	  case 0x09 : BST_fBluetoothSpeed =   0 ;  break;
		
// 		case 0x0A : BST_fCarTurn_P +=   1 ;  break;
// 		case 0x0B : BST_fCarTurn_P -=  1 ;  break;	
// 		case 0x0C : BST_fCarTurn_D +=   0.1 ;  break;
// 		case 0x0D : BST_fCarTurn_D -=   0.1 ;  break;
		

		case 0x0A : BST_fCarAngle_P +=   5 ;  break;
    case 0x0B : BST_fCarAngle_P -=   5 ;  break;	
    case 0x0C : BST_fCarAngle_D +=   0.1 ;  break;
    case 0x0D : BST_fCarAngle_D -=   0.1 ;  break;
		
    case 0x0E : BST_fBluetoothSpeed =   0 ;  break;
    case 0x0F : BST_fBluetoothSpeed =   0 ;  break;		
	  default : BST_fBluetoothSpeed = 0; BST_fBluetoothDirectionL=BST_fBluetoothDirectionR = 0;BST_fBluetoothDirectionSR=BST_fBluetoothDirectionSL=0;break;
	
	}
	USART_ClearITPendingBit(USART3, USART_IT_RXNE); //����жϱ�־
	} 
	 
}
*/
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
