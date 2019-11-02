
#include "upstandingcar.h"
#include "I2C_MPU6050.h"
#include "MOTOR.h"
#include "led.h"
#include "USART.H"
#include "MPU6050.H"
#include "UltrasonicWave.h"
#include "stm32f10x_gpio.h"
#include "math.h" 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*****************************************/
u8 newLineReceived = 0;
u8 inputString[80] = {0};

u8 ProtocolString[80] = {0};
/*小车运行状态枚举*/
enum{
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
}enCarState;

#define 	run_car     '1'//按键前
#define 	back_car    '2'//按键后
#define 	left_car    '3'//按键左
#define 	right_car   '4'//按键右
#define 	stop_car    '0'//按键停


int g_newcarstate = enSTOP; //  1前2后3左4右0停止


char returntemp[] = "$0,0,0,0,0,0,0,0,0,0,0,0cm,8.2V#";
char piddisplay[50] ="$AP";
char manydisplay[80] ={0};
char updata[80] ={0};


/*****************多数据************************/

u8 BST_u8MainEventCount;						  //主循环判断计数  在SysTick_Handler(void)中使用 每1ms加1
u8 BST_u8SpeedControlCount;						  //速度控制循环计数  在SysTick_Handler(void)中使用 每5ms加1
u8 BST_u8SpeedControlPeriod;
u8 BST_u8DirectionControlPeriod;
u8 BST_u8DirectionControlCount;					  //转向控制循环计数  在SysTick_Handler(void)中使用 每5ms加1 
u8 BST_u8trig;
u8 ucBluetoothValue;                      //蓝牙接收数据
float volt = 12.0;



/******电机控制参数******/
float BST_fSpeedControlOut;						   //速度控制PWM
float BST_fSpeedControlOutOld;
float BST_fSpeedControlOutNew;
float BST_fAngleControlOut;
float BST_fLeftMotorOut;
float BST_fRightMotorOut;

float BST_fCarAngle;						 //角度控制PWM
float gyro_z;
float gyrx;
float gy0;

/*-----角度环和速度环PID控制参数-----*///以下参考为重点调试参考，同电池电压有关，建议充好电再调试
float  BST_fCarAngle_P =91.3;//	91.3 //调大小时会左右摆，调大时会振动  请调到基本能够站立 P=91.3是用于给小车在运动过程使用
float  BST_fCarAngle_D =0.21;	// 0.001 0.002 0.004 0.008 0.0010 0.011	 调小时反应慢，调大时会干扰

float  BST_fCarSpeed_P=5.1;
float  BST_fCarSpeed_I=0.10;

const double PID_Original[4] ={91.3, 0.21, 5.1, 0.10}; 
char  alldata[80];
char *iap;

/******速度控制参数******/
s16   BST_s16LeftMotorPulse;					  //左电机脉冲数
s16	  BST_s16RightMotorPulse;					   //右电机脉冲数

s32   BST_s32LeftMotorPulseOld;
s32   BST_s32RightMotorPulseOld;
s32   BST_s32LeftMotorPulseSigma;				  //50ms左电机叠加值
s32   BST_s32RightMotorPulseSigma;				 //50ms右电机叠加值

float BST_fCarSpeed;							 //测速码盘得出的车速
float BST_fCarSpeedOld;

float BST_fCarPosition;						   //测速码盘通过计算得到的小车位移

/*-----悬停参数-----*/
int leftstop=0;
int rightstop=0;
int stopflag=0;

/******超声波********/

float fchaoshengbo = 0;							   //超声波输出量
float juli = 0;									 //超声波距离

/******蓝牙控制参数******/																	
float BST_fBluetoothSpeed;						//蓝牙控制车速
float BST_fBluetoothDirectionNew;			    //用于平缓输出车速使用
float BST_fBluetoothDirectionSL;			    //左转标志位  由于PWM转向输出使用判断输出方式 固需要标志位
float BST_fBluetoothDirectionSR;			   //右转标志位	  由于PWM转向输出使用判断输出方式 固需要标志位
int chaoflag=0;
int x,y1,z1,y2,z2,flagbt;
int g_autoup = 0;
int g_uptimes = 5000;  //自动上报间隔
char charkp[10],charkd[10],charksp[10],charksi[10];
char lspeed[10],rspeed[10],daccel[10],dgyro[10],csb[10],vi[10];
char kp,kd,ksp,ksi;
float dac = 0,dgy = 0;


/************旋转*****************/
float BST_fBluetoothDirectionL;				   //左旋转标志位  由于PWM旋转输出使用判断输出方式 固需要标志位
float BST_fBluetoothDirectionR;				   //右旋转标志位  由于PWM旋转输出使用判断输出方式 固需要标志位
int driectionxco=800;


//******卡尔曼参数************
		
float  Q_angle=0.001;  
float  Q_gyro=0.003;
float  R_angle=0.5;
float  dt=0.005;	                  //dt为kalman滤波器采样时间;
char   C_0 = 1;
float  Q_bias, Angle_err;
float  PCt_0=0, PCt_1=0, E=0;
float  K_0=0, K_1=0, t_0=0, t_1=0;
float  Pdot[4] ={0,0,0,0};
float  PP[2][2] = { { 1, 0 },{ 0, 1 } };


void SendAutoUp(void);

/**********延时子函数****************/
void delay_nms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}
/***********************************/


/***************************************************************
** 函数名称: CarUpstandInit
** 功能描述: 全局变量初始化函数

***************************************************************/
void CarUpstandInit(void)
{
	
	
	BST_s16LeftMotorPulse = BST_s16RightMotorPulse = 0;					  //左右脉冲值   初始化
	BST_s32LeftMotorPulseSigma = BST_s32RightMotorPulseSigma = 0;		  //叠加脉冲数	 初始化

	BST_fCarSpeed = BST_fCarSpeedOld = 0;								   //平衡小车车速	初始化
	BST_fCarPosition = 0;												  //平衡小车位移量	初始化
	BST_fCarAngle    = 0;												  //平衡小车车速	初始化

	BST_fAngleControlOut = BST_fSpeedControlOut = BST_fBluetoothDirectionNew = 0;	//角度PWM、车速PWM、蓝牙控制PWM	 初始化
	BST_fLeftMotorOut    = BST_fRightMotorOut   = 0;								//左右车轮PWM输出值 			 初始化
	BST_fBluetoothSpeed  = 0;														//蓝牙控制车速值                 初始化
	BST_fBluetoothDirectionL =BST_fBluetoothDirectionR= 0;						    //蓝牙控制左右旋转标志位         初始化
	BST_fBluetoothDirectionSL =BST_fBluetoothDirectionSR= 0;						//蓝牙控制左右转向标志位         初始化
	
    BST_u8MainEventCount=0;															//用于5ms定时器子程序SysTick_Handler(void)中总中断计数位
	BST_u8SpeedControlCount=0;														//用于5ms定时器子程序SysTick_Handler(void)中50ms速度平衡融入计数位
    BST_u8SpeedControlPeriod=0;														//用于5ms定时器子程序SysTick_Handler(void)中50ms速度平衡融入计数位

	fchaoshengbo=0;											//用于5ms定时器子程序SysTick_Handler(void)中超声波平衡融入计数位
  
	

}

void ResetPID()
{	
	if(BST_fCarAngle_P != PID_Original[0])
	{
		BST_fCarAngle_P = PID_Original[0];
	}
	if(BST_fCarAngle_D != PID_Original[1])
	{
		BST_fCarAngle_D = PID_Original[1];
	}
	if(BST_fCarSpeed_P != PID_Original[2])
	{
		BST_fCarSpeed_P = PID_Original[2];
	}
	if(BST_fCarSpeed_I != PID_Original[3])
	{
		BST_fCarSpeed_I = PID_Original[3];
	}

}	

/***************************************************************
** 函数名称: AngleControl
** 功能描述: 角度环控制函数

***************************************************************/
void AngleControl(void)	 
{
	if(flagbt==1)
	{
		BST_fCarAngle_P=0;
		BST_fCarAngle_P=y1*1.71875;
	}
		if(flagbt==2)
	{
		BST_fCarAngle_D=0;
		BST_fCarAngle_D=(z1-64)*0.15625;
	}
	dac=accel[2];
	dgy=gyro[2];
	
	BST_fCarAngle = Roll - CAR_ZERO_ANGLE;													   //DMP ROLL滚动方向角度与预设小车倾斜角度值的差得出角度   
	BST_fAngleControlOut =  BST_fCarAngle * BST_fCarAngle_P + gyro[0] * BST_fCarAngle_D ;	  //角度PD控制							   
}

/***************************************************************
** 函数名称: SetMotorVoltageAndDirection
** 功能描述: 电机转速             

***************************************************************/
void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	  u16 u16LeftMotorValue;
	  u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)										 //当左电机PWM输出为负时 PB14设为正 PB15设为负 （PB14 15 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
    {	
	    GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    	 //当左电机PWM输出为正时 PB14设为负 PB15设为正 （PB14 15 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {															 //当右电机PWM输出为负时 PB12设为正 PB13设为负 （PB12 13 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
      GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else														//当右电机PWM输出为正时 PB12设为负 PB13设为正 （PB12 13 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
    {
	    GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	
     
      s16RightVoltage = s16RightVoltage;
    }
		
	   u16RightMotorValue= (u16)s16RightVoltage;
	   u16LeftMotorValue = (u16)s16LeftVoltage;


	TIM_SetCompare3(TIM2,u16LeftMotorValue);			  //TIM2与 u16RightMotorValue对比，不相同则翻转波形，调节PWM占空比
	TIM_SetCompare4(TIM2,u16RightMotorValue);			  //TIM3与 u16LeftMotorValue对比，不相同则翻转波形，调节PWM占空比

#if 1	 /*判断车辆 是否悬停或跌倒，调试用*/
		
  if(Pitch>10||Pitch<-10&BST_fBluetoothDirectionSR==0&BST_fBluetoothDirectionSL==0)
	{		
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);
		stopflag=1;		
	}
	else stopflag=0;
	
	if(BST_fCarAngle > 45 || BST_fCarAngle < (-45))
	{
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);  
		stopflag=1;	
	}
	else stopflag=0;

#endif
}

/***************************************************************
** 函数名称: MotorOutput
** 功能描述: 电机输出函数
             将直立控制、速度控制、方向控制的输出量进行叠加,并加
			 入死区常量，对输出饱和作出处理。

***************************************************************/
void MotorOutput(void)																					  //电机PWM输出函数
{	   
			//右电机转向PWM控制融合平衡角度、速度输出

	BST_fLeftMotorOut  = BST_fAngleControlOut +BST_fSpeedControlOutNew + BST_fBluetoothDirectionNew;//+directionl - BST_fBluetoothDirectionNew;			//左电机转向PWM控制融合平衡角度、速度输出	
    BST_fRightMotorOut = BST_fAngleControlOut +BST_fSpeedControlOutNew - BST_fBluetoothDirectionNew;//-directionl+ BST_fBluetoothDirectionNew;			//右电机转向PWM控制融合平衡角度、速度输出

		
	if((s16)BST_fLeftMotorOut  > MOTOR_OUT_MAX)	BST_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)BST_fLeftMotorOut  < MOTOR_OUT_MIN)	BST_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)BST_fRightMotorOut > MOTOR_OUT_MAX)	BST_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)BST_fRightMotorOut < MOTOR_OUT_MIN)	BST_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((s16)BST_fLeftMotorOut,(s16)BST_fRightMotorOut);
    
}

void GetMotorPulse(void)              //采集电机速度脉冲
{ 
	//////////////////////////////////此部分为外部中断计算脉冲/////////////////////////////////////
	uint16_t u16TempLeft;
  uint16_t u16TempRight;
	
  u16TempLeft = TIM_GetCounter(TIM3);   //  TIM3定时器计算调用
  u16TempRight= TIM_GetCounter(TIM4);	//	 TIM4定时器计算调用
	leftstop=u16TempLeft;
	rightstop=u16TempRight;
  TIM_SetCounter(TIM3,0);//TIM3->CNT = 0;
  TIM_SetCounter(TIM4,0);//TIM4->CNT = 0;   //清零
	BST_s16LeftMotorPulse=u16TempLeft;
  BST_s16RightMotorPulse=-u16TempRight;
		
  BST_s32LeftMotorPulseSigma  +=BST_s16LeftMotorPulse;		 //脉冲值叠加 40ms叠加值
  BST_s32RightMotorPulseSigma +=BST_s16RightMotorPulse; 	 //脉冲值叠加 40ms叠加值
}
/***************************************************************
** 函数名称: SpeedControl
** 功能描述: 速度环控制函数
***************************************************************/

void SpeedControl(void)
{
  
 
	BST_fCarSpeed = (BST_s32LeftMotorPulseSigma  + BST_s32RightMotorPulseSigma );// * 0.5 ;		  //左右电机脉冲数平均值作为小车当前车速
	BST_s32LeftMotorPulseSigma =BST_s32RightMotorPulseSigma = 0;	  //全局变量 注意及时清零		
	BST_fCarSpeedOld *= 0.7;
	BST_fCarSpeedOld +=BST_fCarSpeed*0.3;
	
	BST_fCarPosition += BST_fCarSpeedOld; 		 //路程  即速度积分	   1/11 3:03
	BST_fCarPosition += BST_fBluetoothSpeed;   //融合蓝牙给定速度
	BST_fCarPosition +=	fchaoshengbo;		   //融合超声波给定速度
	if(stopflag==1)
	{
		BST_fCarPosition=0;
		
	}
	

	//积分上限设限//
	if((s32)BST_fCarPosition > CAR_POSITION_MAX)    BST_fCarPosition = CAR_POSITION_MAX;
	if((s32)BST_fCarPosition < CAR_POSITION_MIN)    BST_fCarPosition = CAR_POSITION_MIN;
	
		if(flagbt==3)
	{
		BST_fCarSpeed_P=0;
		BST_fCarSpeed_P=(y2-128)*0.46875;
	}
		if(flagbt==4)
	{
		BST_fCarSpeed_I=0;
		BST_fCarSpeed_I=(z2-192)*0.15625;
	}
	

																								  
	BST_fSpeedControlOutNew = (BST_fCarSpeedOld -CAR_SPEED_SET ) * BST_fCarSpeed_P + (BST_fCarPosition - CAR_POSITION_SET ) * BST_fCarSpeed_I; //速度PI算法 速度*P +位移*I=速度PWM输出

	
}



/***************************************************************
** 函数名称: BluetoothControl
** 功能描述: 蓝牙控制函数
             手机发送内容
			 前：0x01    后：0x02
             左：0x04    右：0x03
             停止：0x07
             功能键：（旋转）
             左旋转:0x05      右旋转：0x06
           	 停转：0x07
** 输　入:   
** 输　出:   
	  	switch (x)
		{
			case 0x00 : BST_fCarAngle_P=BST_fCarAngle_D=BST_fCarSpeed_P=BST_fCarSpeed_I=0;
			case 0x01 : BST_fBluetoothSpeed =   3000 ;chaoflag=1; break;	   //向前速度 250 
			case 0x02 : BST_fBluetoothSpeed = (-3000);chaoflag=1;  break;	   //后退速度 -250
			case 0x03 : BST_fBluetoothDirectionNew= -300; chaoflag=1;break ;//左旋
			case 0x04 : BST_fBluetoothDirectionNew= 300; chaoflag=1;break ;//右旋转
			case 0x05 : BST_fBluetoothDirectionNew= driectionxco; chaoflag=1;break ;//左旋
			case 0x06 : BST_fBluetoothDirectionNew= -driectionxco; chaoflag=1;break ;//右旋转
			case 0x07 : BST_fBluetoothDirectionL  =0; BST_fBluetoothDirectionR = 0; BST_fBluetoothDirectionSL =0; BST_fBluetoothDirectionSR = 0;fchaoshengbo=0;BST_fBluetoothDirectionNew=0;chaoflag=0;   break; //停
			case 0x08 : BST_fBluetoothDirectionSL =0; BST_fBluetoothDirectionSR = 0; fchaoshengbo=0;BST_fBluetoothDirectionNew=0;chaoflag=0;break; //停旋转
			case 0x09 : BST_fBluetoothSpeed =   0 ;  break;
	        case 0x0A : flagbt=1;break;
	        case 0x0B : flagbt=2;break;
	        case 0x0C : flagbt=3;break;
	        case 0x0D : flagbt=4;break;				 
			default : BST_fBluetoothSpeed = 0;flagbt=0; BST_fBluetoothDirectionL=BST_fBluetoothDirectionR = 0;BST_fBluetoothDirectionSR=BST_fBluetoothDirectionSL=0;chaoflag=0;break;	
		}

***************************************************************/
int num = 0;
u8 startBit = 0;
int int9num =0;

void USART3_IRQHandler(void)
{  
	u8 uartvalue = 0;

	if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//注意！不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断  
    {  
		USART_ClearFlag(USART3, USART_FLAG_ORE); //读SR其实就是清除标志
       	USART_ReceiveData(USART3);  		
    }
		
	if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE)!=RESET)
	{
    	USART_ClearITPendingBit(USART3, USART_IT_RXNE);

		uartvalue = USART3->DR;
	    if(uartvalue == '$')
	    {
	      	startBit = 1;
		    num = 0;
	    }
	    if(startBit == 1)
	    {
	       	inputString[num] = uartvalue;     
	    }  
	    if (startBit == 1 && uartvalue == '#') 
	    {
	    	
			newLineReceived = 1; 
			startBit = 0;
			int9num = num;	
		
	    }
		num++;
		if(num >= 80)
		{
			num = 0;
			startBit = 0;
			newLineReceived	= 0;
		}	 
	
	}
	
 
}	 


/**********************超声波距离计算***************************/
void chaoshengbo(void)
{  
	if(chaoflag==0)
	{
	
      	juli=TIM_GetCounter(TIM1)*5*34/200.0;
		
	    if(juli <= 4.00)								  //判断若距离小于8cm，小车输出向后PWM值。
		{
	    	fchaoshengbo= (-300);
	    }
		else if(juli >= 5 & juli <= 8)
		{
			fchaoshengbo=500;
		}
	    else 
		{
			fchaoshengbo=0;						 //距离大于8cm ，超声波PWM输出为0
 	    }
  	}
	
	//寄生此上报数据
	//增加自动上报  10ms进一次，故10*50ms = 500ms
    //SendAutoUp();
 }

/*计算上报的数据*/
void CalcUpData()
{
	float ls, rs, sLence;
	short s_Acc, s_Gyro;

	
	if(g_autoup == 1)
	{
		//CLI();
		ls = BST_fLeftMotorOut;
		rs = BST_fRightMotorOut;
		s_Acc = accel[1];
		s_Gyro = gyro[0];
		sLence = juli;
		//SEI();
		
		dac=(s_Acc/16384.0f)*9.8f;
		dgy=((s_Gyro-128.1f)/131.0f);
		
		ls=ls/3.91;
		rs=rs/3.91;
	
		memset(manydisplay, 0x00, 80);
		memcpy(manydisplay, "$LV", 4);
	
		memset(lspeed, 0x00, sizeof(lspeed));
		memset(rspeed, 0x00, sizeof(rspeed));
		memset(daccel, 0x00, sizeof(daccel));
		memset(dgyro, 0x00, sizeof(dgyro));
		memset(csb, 0x00, sizeof(csb));
		memset(vi, 0x00, sizeof(vi));
	
		if((ls <= 1000) && (ls >= -1000))
			sprintf(lspeed,"%3.2f",ls);
		else
		{
			//sprintf(str,"$AutoUpError!ls=%3.2f#",ls);
			//UART3_Send_Char(str); //返回协议数据包	
			return;
		}
			
		
		if((rs <= 1000) && (rs >= -1000))
			sprintf(rspeed,"%3.2f",rs);
		else
		{
			//sprintf(str,"$AutoUpError!rs=%3.2f#",rs);
			//UART3_Send_Char(str); //返回协议数据包	
			return;
		}
		
		if((dac > -20) && (dac < 20))
			sprintf(daccel,"%3.2f",dac);
		else
		{
			//sprintf(str,"$AutoUpError!dac=%3.2f#",dac);
			//UART3_Send_Char(str); //返回协议数据包	
			return;
		}
		
		if((dgy > -3000) && (dgy < 3000))
			sprintf(dgyro,"%3.2f",dgy);
		else
		{
			//sprintf(str,"$AutoUpError!dgy=%3.2f#",dgy);
			//UART3_Send_Char(str); //返回协议数据包	
			return;
		}
	
		if((sLence >= 0) && (sLence < 10000))
			sprintf(csb,"%3.2f",sLence);
		else
		{
			//sprintf(str,"$AutoUpError!juli=%3.2f#",juli);
			//UART3_Send_Char(str); //返回协议数据包	
			return;
		}
	
		if((volt >= 0) && (volt < 20))
			sprintf(vi,"%3.2f",volt);
		else
		{
			//sprintf(str,"$AutoUpError!volt=%3.2f#",volt);
			//UART3_Send_Char(str); //返回协议数据包	
			return;
		}
	
		strcat(manydisplay,lspeed);
		strcat(manydisplay,",RV");
		strcat(manydisplay,rspeed);
		strcat(manydisplay,",AC");
		strcat(manydisplay,daccel);
		strcat(manydisplay,",GY");
		strcat(manydisplay,dgyro);
		strcat(manydisplay,",CSB");
		strcat(manydisplay,csb);
		strcat(manydisplay,",VT");
		strcat(manydisplay,vi);
		strcat(manydisplay,"#");
		memset(updata, 0x00, 80);
		memcpy(updata, manydisplay, 80);
	}
	
}

/*
自动上报
*/
void SendAutoUp(void)
{
	g_uptimes --;
	if ((g_autoup == 1) && (g_uptimes == 0))
	{
		CalcUpData();
		UART3_Send_Char(updata); //返回协议数据包	
	}
	if(g_uptimes == 0)
		 g_uptimes = 5000;

}

int StringFind(const char *pSrc, const char *pDst)  
{  
    int i, j;  
    for (i=0; pSrc[i]!='\0'; i++)  
    {  
        if(pSrc[i]!=pDst[0])  
            continue;         
        j = 0;  
        while(pDst[j]!='\0' && pSrc[i+j]!='\0')  
        {  
            j++;  
            if(pDst[j]!=pSrc[i+j])  
            break;  
        }  
        if(pDst[j]=='\0')  
            return i;  
    }  
    return -1;  
}  

void CarStateOut(void)
{
	switch (g_newcarstate)
	{
		case enSTOP: //停止
		{

			BST_fBluetoothSpeed = 0;
			fchaoshengbo=0;
			BST_fBluetoothDirectionNew=0;
			chaoflag=0;

		} break; 					   

		case enRUN: //向前速度 250  
		{
			BST_fBluetoothDirectionNew= 0; 	
			BST_fBluetoothSpeed =   3000 ;
			chaoflag=1;

		}break;	   

		case enLEFT://左转 
		{

			BST_fBluetoothDirectionNew= -300; 
			chaoflag=1;

		}break;   
		
		case enRIGHT: //右转
		{

			BST_fBluetoothDirectionNew= 300; 
			chaoflag=1;

		}break;	
		
		case enBACK: //后退速度 -250
		{
			BST_fBluetoothDirectionNew= 0; 	
			BST_fBluetoothSpeed = (-3000);
			chaoflag=1;  

		}break;
		
		case enTLEFT: //左旋
		{
			BST_fBluetoothDirectionNew = -driectionxco; 
			chaoflag=1; 

		}break;
		case enTRIGHT: //右旋
		{
			BST_fBluetoothDirectionNew = driectionxco; 
			chaoflag=1;
		}break;
		
		default: BST_fBluetoothSpeed = 0; break; 					   //停止
	}
}

void ProtocolGetPID(void)
{
	memset(piddisplay, 0x00, sizeof(piddisplay));
	memcpy(piddisplay, "$AP", 4);

	if(BST_fCarAngle_P >= 0 && BST_fCarAngle_P <= 100)
	{
		sprintf(charkp,"%3.2f",BST_fCarAngle_P);
	}
	else
	{	
		UART3_Send_Char("$GetPIDError#"); //返回协议数据包 
		return;
	}

	
	if(BST_fCarAngle_D >= 0 && BST_fCarAngle_D <= 100)
	{
		sprintf(charkd,"%3.2f",BST_fCarAngle_D);
	}
	else
	{	
		UART3_Send_Char("$GetPIDError#"); //返回协议数据包 
		return;
	}
	
	if(BST_fCarSpeed_P >= 0 && BST_fCarSpeed_P <= 100)
	{
		sprintf(charksp,"%3.2f",BST_fCarSpeed_P);
	}
	else
	{	
		UART3_Send_Char("$GetPIDError#"); //返回协议数据包 
		return;
	}

	if(BST_fCarSpeed_I >= 0 && BST_fCarSpeed_I <= 100)
	{
		sprintf(charksi,"%3.2f",BST_fCarSpeed_I);
	}
	else
	{	
		UART3_Send_Char("$GetPIDError#"); //返回协议数据包 
		return;
	}
	
	
	strcat(piddisplay,charkp);
	strcat(piddisplay,",AD");
	strcat(piddisplay,charkd);
	strcat(piddisplay,",VP");
	strcat(piddisplay,charksp);
	strcat(piddisplay,",VI");
	strcat(piddisplay,charksi);
	strcat(piddisplay,"#");
	
	UART3_Send_Char(piddisplay); //返回协议数据包 

}

void ProtocolCpyData(void)
{
	memcpy(ProtocolString, inputString, num+1);
	memset(inputString, 0x00, sizeof(inputString));
}
/***************************************************************************
串口协议数据解析
***************************************************************************/ 
void Protocol(void)
{
	//USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//禁能接收中断

	//判断数据包有效性

		
	
	switch (ProtocolString[1])
	{
		case run_car:	 g_newcarstate = enRUN; UART3_Send_Char(returntemp);break;
		case back_car:  g_newcarstate = enBACK; UART3_Send_Char(returntemp);break;
		case left_car:  g_newcarstate = enLEFT; UART3_Send_Char(returntemp);break;
		case right_car: g_newcarstate = enRIGHT; UART3_Send_Char(returntemp);break;
		case stop_car:  g_newcarstate = enSTOP; UART3_Send_Char(returntemp);break;
		default: g_newcarstate = enSTOP; break;
		
	}

	/*防止数据丢包*/
	if(strlen((const char *)ProtocolString)<21)
	{
		newLineReceived = 0;  
		memset(ProtocolString, 0x00, sizeof(ProtocolString));  
		//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//使能接收中断
		UART3_Send_Char("$ReceivePackError#"); //返回协议数据包 
		return;
	}

	if (ProtocolString[3] == '1') //左摇
	{
		g_newcarstate = enTLEFT;
		UART3_Send_Char(returntemp); //返回协议数据包  	
		
	}
	
	
	if (ProtocolString[3] == '2') //右摇
	{
		g_newcarstate = enTRIGHT;
		UART3_Send_Char(returntemp); //返回协议数据包  	
	}

	
	//查询PID
	if(ProtocolString[5]=='1')
	{
		ProtocolGetPID();
	}
	else if(ProtocolString[5]=='2')  //恢复默认PID
	{
		ResetPID();
		UART3_Send_Char("$OK#"); //返回协议数据包  	
	}

	//自动上报
	if(ProtocolString[7]=='1')
	{
		g_autoup = 1;    
      	UART3_Send_Char("$OK#"); //返回协议数据包  	
	}
	else if(ProtocolString[7]=='2')
	{		
		g_autoup = 0;		
		UART3_Send_Char("$OK#"); //返回协议数据包  	
	}
	
	if (ProtocolString[9] == '1') //角度环更新 $0,0,0,0,1,1,AP23.54,AD85.45,VP10.78,VI0.26#
	{
		//$0,0,0,0,1,1,AP23.54,AD85.45,VP10.78,VI0.26#

		int pos,z; 
		char apad[20] = {0},apvalue[8] = {0},advalue[8] = {0};
			
		pos = StringFind((const char *)ProtocolString, (const char *)"AP");
		if(pos == -1) return;
		
		memcpy(apad,ProtocolString+pos,int9num-pos);

		//AP23.54,AD85.45,VP10.78,VI0.26#
		z = StringFind(apad, ",");
		if(z == -1) return;
		memcpy(apvalue, apad+2, z-2);
		
		BST_fCarAngle_P = atof(apvalue);
		
		
		memset(apad, 0x00, sizeof(apad));
		memcpy(apad, ProtocolString + pos + z + 1, int9num - (pos + z)); //存储AD后面的数据
		z = StringFind(apad, ",");
		if(z == -1) return;
		memcpy(advalue,apad+2, z-2);
		
		BST_fCarAngle_D=atof(advalue);

		UART3_Send_Char("$OK#"); //返回协议数据包  				
	}
		
  	if(ProtocolString[11] == '1')
	{
		int pos,z; 
		char vpvi[20] = {0},vpvalue[8] = {0},vivalue[8] = {0};
			
		pos = StringFind((const char *)ProtocolString, (const char *)"VP");
		if(pos == -1) return;
		
		memcpy(vpvi, ProtocolString+pos, int9num-pos);
		//y=strchr(apad,'AP');
		//AP23.54,AD85.45,VP10.78,VI0.26#
		z = StringFind(vpvi, ",");
		if(z == -1) return;
		memcpy(vpvalue, vpvi+2, z-2);
		
		BST_fCarSpeed_P = atof(vpvalue);
		
		
		memset(vpvi, 0x00, sizeof(vpvi));
		memcpy(vpvi, ProtocolString + pos + z + 1, int9num - (pos + z)); //存储AD后面的数据
		z = StringFind(vpvi, "#");
		if(z == -1) return;
		memcpy(vivalue,vpvi+2, z-2);
		
		BST_fCarSpeed_I=atof(vivalue);

		UART3_Send_Char("$OK#"); //返回协议数据包  		
			
			
	}
	
	newLineReceived = 0;  
	memset(ProtocolString, 0x00, sizeof(ProtocolString));  
	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//使能接收中断

}

 
 
 
 
 
 
 
 
 
