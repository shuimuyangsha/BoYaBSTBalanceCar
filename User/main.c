/******************** (C) COPYRIGHT (2015)BST BALANCECAR **************************
 * 文件名  ：main.c
**********************************************************************************/
//#include "stm32f10x.h"
#include "mpu6050.h"
#include "i2c_mpu6050.h"
#include "motor.h"
#include "upstandingcar.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "i2c.h"
//#include "outputdata.h"
#include "timer.h"
#include "UltrasonicWave.h"
//#include "stm32f10x_usart.h"

float gyz;
int acc;
int acc1;

/*协议相关*/
//extern u8 newLineReceived = 0;

/*
 * 函数名：main
 * 描述  ：主函数
 */
int main(void)
{	
       
	
  	SystemInit();                   //=====系统初始化
	Timerx_Init(5000,7199);				   //定时器TIM1
	UltrasonicWave_Configuration(); 	   //超声波初始化设置 IO口及中断设置			    

	USART1_Config();						//串口1初始化 上位机
	USART3_Config();						//串口3初始化 蓝牙与USART3公用相同IO口
	
	TIM2_PWM_Init();					   //PWM输出初始化
	MOTOR_GPIO_Config();				  //电机IO口初始化
  	LED_GPIO_Config();
	
//   TIM3_External_Clock_CountingMode();	   //左电机脉冲输出外部中断口PA7使用TIM3定时器用作为脉冲数计算
//   TIM4_External_Clock_CountingMode();	   //右电机脉冲输出外部中断口PB7使用TIM4定时器用作为脉冲数计算
  	TIM3_Encoder_Init();                       //编码器获取脉冲数 PA6 7 
  	TIM4_Encoder_Init();                       //编码器获取脉冲数 PB6 7	
	////////////////////DMP/////////////////////////////////
	i2cInit();							   //IIC初始化 用于挂靠在总线上的设备使用
	delay_nms(10);						   //延时10ms
	MPU6050_Init();						   //MPU6050 DMP陀螺仪初始化
	
	SysTick_Init();						  //SysTick函数初始化	
	CarUpstandInit();					  //小车直立参数初始化
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	 //使能总算法时钟

	while (1)
	{

// 		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		MPU6050_Pose();						 //获取MPU6050角度状态
// 		gy0=gyro[0];
// 		UltrasonicWave_StartMeasure();	   //调用超声波发送程序 给Trig脚 <10us 高电平		 
// 		chaoshengbo();			       //计算超声波测距距离
//      	printf("%d",ucBluetoothValue);
//		printf("\t");
//		printf("%f",BST_fSpeedControlOutNew);
//		printf("\t");
//		printf("%f",BST_fCarAngle);
//		printf("\t");
// 		printf("%f",BST_fLeftMotorOut);
//		printf("\t");
//		printf("\n");
		
		if (newLineReceived)
	   	{
			ProtocolCpyData();
			Protocol();
		}
		/*通过状态控制小车*/
		CarStateOut();
		
		SendAutoUp();
		
			 
	 }
 								    
}
