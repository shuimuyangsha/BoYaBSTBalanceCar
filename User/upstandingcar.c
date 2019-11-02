
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
/*С������״̬ö��*/
enum{
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
}enCarState;

#define 	run_car     '1'//����ǰ
#define 	back_car    '2'//������
#define 	left_car    '3'//������
#define 	right_car   '4'//������
#define 	stop_car    '0'//����ͣ


int g_newcarstate = enSTOP; //  1ǰ2��3��4��0ֹͣ


char returntemp[] = "$0,0,0,0,0,0,0,0,0,0,0,0cm,8.2V#";
char piddisplay[50] ="$AP";
char manydisplay[80] ={0};
char updata[80] ={0};


/*****************������************************/

u8 BST_u8MainEventCount;						  //��ѭ���жϼ���  ��SysTick_Handler(void)��ʹ�� ÿ1ms��1
u8 BST_u8SpeedControlCount;						  //�ٶȿ���ѭ������  ��SysTick_Handler(void)��ʹ�� ÿ5ms��1
u8 BST_u8SpeedControlPeriod;
u8 BST_u8DirectionControlPeriod;
u8 BST_u8DirectionControlCount;					  //ת�����ѭ������  ��SysTick_Handler(void)��ʹ�� ÿ5ms��1 
u8 BST_u8trig;
u8 ucBluetoothValue;                      //������������
float volt = 12.0;



/******������Ʋ���******/
float BST_fSpeedControlOut;						   //�ٶȿ���PWM
float BST_fSpeedControlOutOld;
float BST_fSpeedControlOutNew;
float BST_fAngleControlOut;
float BST_fLeftMotorOut;
float BST_fRightMotorOut;

float BST_fCarAngle;						 //�Ƕȿ���PWM
float gyro_z;
float gyrx;
float gy0;

/*-----�ǶȻ����ٶȻ�PID���Ʋ���-----*///���²ο�Ϊ�ص���Բο���ͬ��ص�ѹ�йأ������õ��ٵ���
float  BST_fCarAngle_P =91.3;//	91.3 //����Сʱ�����Ұڣ�����ʱ����  ����������ܹ�վ�� P=91.3�����ڸ�С�����˶�����ʹ��
float  BST_fCarAngle_D =0.21;	// 0.001 0.002 0.004 0.008 0.0010 0.011	 ��Сʱ��Ӧ��������ʱ�����

float  BST_fCarSpeed_P=5.1;
float  BST_fCarSpeed_I=0.10;

const double PID_Original[4] ={91.3, 0.21, 5.1, 0.10}; 
char  alldata[80];
char *iap;

/******�ٶȿ��Ʋ���******/
s16   BST_s16LeftMotorPulse;					  //����������
s16	  BST_s16RightMotorPulse;					   //�ҵ��������

s32   BST_s32LeftMotorPulseOld;
s32   BST_s32RightMotorPulseOld;
s32   BST_s32LeftMotorPulseSigma;				  //50ms��������ֵ
s32   BST_s32RightMotorPulseSigma;				 //50ms�ҵ������ֵ

float BST_fCarSpeed;							 //�������̵ó��ĳ���
float BST_fCarSpeedOld;

float BST_fCarPosition;						   //��������ͨ������õ���С��λ��

/*-----��ͣ����-----*/
int leftstop=0;
int rightstop=0;
int stopflag=0;

/******������********/

float fchaoshengbo = 0;							   //�����������
float juli = 0;									 //����������

/******�������Ʋ���******/																	
float BST_fBluetoothSpeed;						//�������Ƴ���
float BST_fBluetoothDirectionNew;			    //����ƽ���������ʹ��
float BST_fBluetoothDirectionSL;			    //��ת��־λ  ����PWMת�����ʹ���ж������ʽ ����Ҫ��־λ
float BST_fBluetoothDirectionSR;			   //��ת��־λ	  ����PWMת�����ʹ���ж������ʽ ����Ҫ��־λ
int chaoflag=0;
int x,y1,z1,y2,z2,flagbt;
int g_autoup = 0;
int g_uptimes = 5000;  //�Զ��ϱ����
char charkp[10],charkd[10],charksp[10],charksi[10];
char lspeed[10],rspeed[10],daccel[10],dgyro[10],csb[10],vi[10];
char kp,kd,ksp,ksi;
float dac = 0,dgy = 0;


/************��ת*****************/
float BST_fBluetoothDirectionL;				   //����ת��־λ  ����PWM��ת���ʹ���ж������ʽ ����Ҫ��־λ
float BST_fBluetoothDirectionR;				   //����ת��־λ  ����PWM��ת���ʹ���ж������ʽ ����Ҫ��־λ
int driectionxco=800;


//******����������************
		
float  Q_angle=0.001;  
float  Q_gyro=0.003;
float  R_angle=0.5;
float  dt=0.005;	                  //dtΪkalman�˲�������ʱ��;
char   C_0 = 1;
float  Q_bias, Angle_err;
float  PCt_0=0, PCt_1=0, E=0;
float  K_0=0, K_1=0, t_0=0, t_1=0;
float  Pdot[4] ={0,0,0,0};
float  PP[2][2] = { { 1, 0 },{ 0, 1 } };


void SendAutoUp(void);

/**********��ʱ�Ӻ���****************/
void delay_nms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  //�Լ�����
      while(i--) ;    
   }
}
/***********************************/


/***************************************************************
** ��������: CarUpstandInit
** ��������: ȫ�ֱ�����ʼ������

***************************************************************/
void CarUpstandInit(void)
{
	
	
	BST_s16LeftMotorPulse = BST_s16RightMotorPulse = 0;					  //��������ֵ   ��ʼ��
	BST_s32LeftMotorPulseSigma = BST_s32RightMotorPulseSigma = 0;		  //����������	 ��ʼ��

	BST_fCarSpeed = BST_fCarSpeedOld = 0;								   //ƽ��С������	��ʼ��
	BST_fCarPosition = 0;												  //ƽ��С��λ����	��ʼ��
	BST_fCarAngle    = 0;												  //ƽ��С������	��ʼ��

	BST_fAngleControlOut = BST_fSpeedControlOut = BST_fBluetoothDirectionNew = 0;	//�Ƕ�PWM������PWM����������PWM	 ��ʼ��
	BST_fLeftMotorOut    = BST_fRightMotorOut   = 0;								//���ҳ���PWM���ֵ 			 ��ʼ��
	BST_fBluetoothSpeed  = 0;														//�������Ƴ���ֵ                 ��ʼ��
	BST_fBluetoothDirectionL =BST_fBluetoothDirectionR= 0;						    //��������������ת��־λ         ��ʼ��
	BST_fBluetoothDirectionSL =BST_fBluetoothDirectionSR= 0;						//������������ת���־λ         ��ʼ��
	
    BST_u8MainEventCount=0;															//����5ms��ʱ���ӳ���SysTick_Handler(void)�����жϼ���λ
	BST_u8SpeedControlCount=0;														//����5ms��ʱ���ӳ���SysTick_Handler(void)��50ms�ٶ�ƽ���������λ
    BST_u8SpeedControlPeriod=0;														//����5ms��ʱ���ӳ���SysTick_Handler(void)��50ms�ٶ�ƽ���������λ

	fchaoshengbo=0;											//����5ms��ʱ���ӳ���SysTick_Handler(void)�г�����ƽ���������λ
  
	

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
** ��������: AngleControl
** ��������: �ǶȻ����ƺ���

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
	
	BST_fCarAngle = Roll - CAR_ZERO_ANGLE;													   //DMP ROLL��������Ƕ���Ԥ��С����б�Ƕ�ֵ�Ĳ�ó��Ƕ�   
	BST_fAngleControlOut =  BST_fCarAngle * BST_fCarAngle_P + gyro[0] * BST_fCarAngle_D ;	  //�Ƕ�PD����							   
}

/***************************************************************
** ��������: SetMotorVoltageAndDirection
** ��������: ���ת��             

***************************************************************/
void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	  u16 u16LeftMotorValue;
	  u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)										 //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ�� ��PB14 15 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
    {	
	    GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    	 //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ�� ��PB14 15 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {															 //���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ�� ��PB12 13 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
      GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else														//���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ�� ��PB12 13 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
    {
	    GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	
     
      s16RightVoltage = s16RightVoltage;
    }
		
	   u16RightMotorValue= (u16)s16RightVoltage;
	   u16LeftMotorValue = (u16)s16LeftVoltage;


	TIM_SetCompare3(TIM2,u16LeftMotorValue);			  //TIM2�� u16RightMotorValue�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
	TIM_SetCompare4(TIM2,u16RightMotorValue);			  //TIM3�� u16LeftMotorValue�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�

#if 1	 /*�жϳ��� �Ƿ���ͣ�������������*/
		
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
** ��������: MotorOutput
** ��������: ����������
             ��ֱ�����ơ��ٶȿ��ơ�������Ƶ���������е���,����
			 �����������������������������

***************************************************************/
void MotorOutput(void)																					  //���PWM�������
{	   
			//�ҵ��ת��PWM�����ں�ƽ��Ƕȡ��ٶ����

	BST_fLeftMotorOut  = BST_fAngleControlOut +BST_fSpeedControlOutNew + BST_fBluetoothDirectionNew;//+directionl - BST_fBluetoothDirectionNew;			//����ת��PWM�����ں�ƽ��Ƕȡ��ٶ����	
    BST_fRightMotorOut = BST_fAngleControlOut +BST_fSpeedControlOutNew - BST_fBluetoothDirectionNew;//-directionl+ BST_fBluetoothDirectionNew;			//�ҵ��ת��PWM�����ں�ƽ��Ƕȡ��ٶ����

		
	if((s16)BST_fLeftMotorOut  > MOTOR_OUT_MAX)	BST_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)BST_fLeftMotorOut  < MOTOR_OUT_MIN)	BST_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)BST_fRightMotorOut > MOTOR_OUT_MAX)	BST_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)BST_fRightMotorOut < MOTOR_OUT_MIN)	BST_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((s16)BST_fLeftMotorOut,(s16)BST_fRightMotorOut);
    
}

void GetMotorPulse(void)              //�ɼ�����ٶ�����
{ 
	//////////////////////////////////�˲���Ϊ�ⲿ�жϼ�������/////////////////////////////////////
	uint16_t u16TempLeft;
  uint16_t u16TempRight;
	
  u16TempLeft = TIM_GetCounter(TIM3);   //  TIM3��ʱ���������
  u16TempRight= TIM_GetCounter(TIM4);	//	 TIM4��ʱ���������
	leftstop=u16TempLeft;
	rightstop=u16TempRight;
  TIM_SetCounter(TIM3,0);//TIM3->CNT = 0;
  TIM_SetCounter(TIM4,0);//TIM4->CNT = 0;   //����
	BST_s16LeftMotorPulse=u16TempLeft;
  BST_s16RightMotorPulse=-u16TempRight;
		
  BST_s32LeftMotorPulseSigma  +=BST_s16LeftMotorPulse;		 //����ֵ���� 40ms����ֵ
  BST_s32RightMotorPulseSigma +=BST_s16RightMotorPulse; 	 //����ֵ���� 40ms����ֵ
}
/***************************************************************
** ��������: SpeedControl
** ��������: �ٶȻ����ƺ���
***************************************************************/

void SpeedControl(void)
{
  
 
	BST_fCarSpeed = (BST_s32LeftMotorPulseSigma  + BST_s32RightMotorPulseSigma );// * 0.5 ;		  //���ҵ��������ƽ��ֵ��ΪС����ǰ����
	BST_s32LeftMotorPulseSigma =BST_s32RightMotorPulseSigma = 0;	  //ȫ�ֱ��� ע�⼰ʱ����		
	BST_fCarSpeedOld *= 0.7;
	BST_fCarSpeedOld +=BST_fCarSpeed*0.3;
	
	BST_fCarPosition += BST_fCarSpeedOld; 		 //·��  ���ٶȻ���	   1/11 3:03
	BST_fCarPosition += BST_fBluetoothSpeed;   //�ں����������ٶ�
	BST_fCarPosition +=	fchaoshengbo;		   //�ںϳ����������ٶ�
	if(stopflag==1)
	{
		BST_fCarPosition=0;
		
	}
	

	//������������//
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
	

																								  
	BST_fSpeedControlOutNew = (BST_fCarSpeedOld -CAR_SPEED_SET ) * BST_fCarSpeed_P + (BST_fCarPosition - CAR_POSITION_SET ) * BST_fCarSpeed_I; //�ٶ�PI�㷨 �ٶ�*P +λ��*I=�ٶ�PWM���

	
}



/***************************************************************
** ��������: BluetoothControl
** ��������: �������ƺ���
             �ֻ���������
			 ǰ��0x01    ��0x02
             ��0x04    �ң�0x03
             ֹͣ��0x07
             ���ܼ�������ת��
             ����ת:0x05      ����ת��0x06
           	 ͣת��0x07
** �䡡��:   
** �䡡��:   
	  	switch (x)
		{
			case 0x00 : BST_fCarAngle_P=BST_fCarAngle_D=BST_fCarSpeed_P=BST_fCarSpeed_I=0;
			case 0x01 : BST_fBluetoothSpeed =   3000 ;chaoflag=1; break;	   //��ǰ�ٶ� 250 
			case 0x02 : BST_fBluetoothSpeed = (-3000);chaoflag=1;  break;	   //�����ٶ� -250
			case 0x03 : BST_fBluetoothDirectionNew= -300; chaoflag=1;break ;//����
			case 0x04 : BST_fBluetoothDirectionNew= 300; chaoflag=1;break ;//����ת
			case 0x05 : BST_fBluetoothDirectionNew= driectionxco; chaoflag=1;break ;//����
			case 0x06 : BST_fBluetoothDirectionNew= -driectionxco; chaoflag=1;break ;//����ת
			case 0x07 : BST_fBluetoothDirectionL  =0; BST_fBluetoothDirectionR = 0; BST_fBluetoothDirectionSL =0; BST_fBluetoothDirectionSR = 0;fchaoshengbo=0;BST_fBluetoothDirectionNew=0;chaoflag=0;   break; //ͣ
			case 0x08 : BST_fBluetoothDirectionSL =0; BST_fBluetoothDirectionSR = 0; fchaoshengbo=0;BST_fBluetoothDirectionNew=0;chaoflag=0;break; //ͣ��ת
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

	if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//ע�⣡����ʹ��if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)���ж�  
    {  
		USART_ClearFlag(USART3, USART_FLAG_ORE); //��SR��ʵ���������־
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


/**********************�������������***************************/
void chaoshengbo(void)
{  
	if(chaoflag==0)
	{
	
      	juli=TIM_GetCounter(TIM1)*5*34/200.0;
		
	    if(juli <= 4.00)								  //�ж�������С��8cm��С��������PWMֵ��
		{
	    	fchaoshengbo= (-300);
	    }
		else if(juli >= 5 & juli <= 8)
		{
			fchaoshengbo=500;
		}
	    else 
		{
			fchaoshengbo=0;						 //�������8cm ��������PWM���Ϊ0
 	    }
  	}
	
	//�������ϱ�����
	//�����Զ��ϱ�  10ms��һ�Σ���10*50ms = 500ms
    //SendAutoUp();
 }

/*�����ϱ�������*/
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
			//UART3_Send_Char(str); //����Э�����ݰ�	
			return;
		}
			
		
		if((rs <= 1000) && (rs >= -1000))
			sprintf(rspeed,"%3.2f",rs);
		else
		{
			//sprintf(str,"$AutoUpError!rs=%3.2f#",rs);
			//UART3_Send_Char(str); //����Э�����ݰ�	
			return;
		}
		
		if((dac > -20) && (dac < 20))
			sprintf(daccel,"%3.2f",dac);
		else
		{
			//sprintf(str,"$AutoUpError!dac=%3.2f#",dac);
			//UART3_Send_Char(str); //����Э�����ݰ�	
			return;
		}
		
		if((dgy > -3000) && (dgy < 3000))
			sprintf(dgyro,"%3.2f",dgy);
		else
		{
			//sprintf(str,"$AutoUpError!dgy=%3.2f#",dgy);
			//UART3_Send_Char(str); //����Э�����ݰ�	
			return;
		}
	
		if((sLence >= 0) && (sLence < 10000))
			sprintf(csb,"%3.2f",sLence);
		else
		{
			//sprintf(str,"$AutoUpError!juli=%3.2f#",juli);
			//UART3_Send_Char(str); //����Э�����ݰ�	
			return;
		}
	
		if((volt >= 0) && (volt < 20))
			sprintf(vi,"%3.2f",volt);
		else
		{
			//sprintf(str,"$AutoUpError!volt=%3.2f#",volt);
			//UART3_Send_Char(str); //����Э�����ݰ�	
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
�Զ��ϱ�
*/
void SendAutoUp(void)
{
	g_uptimes --;
	if ((g_autoup == 1) && (g_uptimes == 0))
	{
		CalcUpData();
		UART3_Send_Char(updata); //����Э�����ݰ�	
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
		case enSTOP: //ֹͣ
		{

			BST_fBluetoothSpeed = 0;
			fchaoshengbo=0;
			BST_fBluetoothDirectionNew=0;
			chaoflag=0;

		} break; 					   

		case enRUN: //��ǰ�ٶ� 250  
		{
			BST_fBluetoothDirectionNew= 0; 	
			BST_fBluetoothSpeed =   3000 ;
			chaoflag=1;

		}break;	   

		case enLEFT://��ת 
		{

			BST_fBluetoothDirectionNew= -300; 
			chaoflag=1;

		}break;   
		
		case enRIGHT: //��ת
		{

			BST_fBluetoothDirectionNew= 300; 
			chaoflag=1;

		}break;	
		
		case enBACK: //�����ٶ� -250
		{
			BST_fBluetoothDirectionNew= 0; 	
			BST_fBluetoothSpeed = (-3000);
			chaoflag=1;  

		}break;
		
		case enTLEFT: //����
		{
			BST_fBluetoothDirectionNew = -driectionxco; 
			chaoflag=1; 

		}break;
		case enTRIGHT: //����
		{
			BST_fBluetoothDirectionNew = driectionxco; 
			chaoflag=1;
		}break;
		
		default: BST_fBluetoothSpeed = 0; break; 					   //ֹͣ
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
		UART3_Send_Char("$GetPIDError#"); //����Э�����ݰ� 
		return;
	}

	
	if(BST_fCarAngle_D >= 0 && BST_fCarAngle_D <= 100)
	{
		sprintf(charkd,"%3.2f",BST_fCarAngle_D);
	}
	else
	{	
		UART3_Send_Char("$GetPIDError#"); //����Э�����ݰ� 
		return;
	}
	
	if(BST_fCarSpeed_P >= 0 && BST_fCarSpeed_P <= 100)
	{
		sprintf(charksp,"%3.2f",BST_fCarSpeed_P);
	}
	else
	{	
		UART3_Send_Char("$GetPIDError#"); //����Э�����ݰ� 
		return;
	}

	if(BST_fCarSpeed_I >= 0 && BST_fCarSpeed_I <= 100)
	{
		sprintf(charksi,"%3.2f",BST_fCarSpeed_I);
	}
	else
	{	
		UART3_Send_Char("$GetPIDError#"); //����Э�����ݰ� 
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
	
	UART3_Send_Char(piddisplay); //����Э�����ݰ� 

}

void ProtocolCpyData(void)
{
	memcpy(ProtocolString, inputString, num+1);
	memset(inputString, 0x00, sizeof(inputString));
}
/***************************************************************************
����Э�����ݽ���
***************************************************************************/ 
void Protocol(void)
{
	//USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//���ܽ����ж�

	//�ж����ݰ���Ч��

		
	
	switch (ProtocolString[1])
	{
		case run_car:	 g_newcarstate = enRUN; UART3_Send_Char(returntemp);break;
		case back_car:  g_newcarstate = enBACK; UART3_Send_Char(returntemp);break;
		case left_car:  g_newcarstate = enLEFT; UART3_Send_Char(returntemp);break;
		case right_car: g_newcarstate = enRIGHT; UART3_Send_Char(returntemp);break;
		case stop_car:  g_newcarstate = enSTOP; UART3_Send_Char(returntemp);break;
		default: g_newcarstate = enSTOP; break;
		
	}

	/*��ֹ���ݶ���*/
	if(strlen((const char *)ProtocolString)<21)
	{
		newLineReceived = 0;  
		memset(ProtocolString, 0x00, sizeof(ProtocolString));  
		//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//ʹ�ܽ����ж�
		UART3_Send_Char("$ReceivePackError#"); //����Э�����ݰ� 
		return;
	}

	if (ProtocolString[3] == '1') //��ҡ
	{
		g_newcarstate = enTLEFT;
		UART3_Send_Char(returntemp); //����Э�����ݰ�  	
		
	}
	
	
	if (ProtocolString[3] == '2') //��ҡ
	{
		g_newcarstate = enTRIGHT;
		UART3_Send_Char(returntemp); //����Э�����ݰ�  	
	}

	
	//��ѯPID
	if(ProtocolString[5]=='1')
	{
		ProtocolGetPID();
	}
	else if(ProtocolString[5]=='2')  //�ָ�Ĭ��PID
	{
		ResetPID();
		UART3_Send_Char("$OK#"); //����Э�����ݰ�  	
	}

	//�Զ��ϱ�
	if(ProtocolString[7]=='1')
	{
		g_autoup = 1;    
      	UART3_Send_Char("$OK#"); //����Э�����ݰ�  	
	}
	else if(ProtocolString[7]=='2')
	{		
		g_autoup = 0;		
		UART3_Send_Char("$OK#"); //����Э�����ݰ�  	
	}
	
	if (ProtocolString[9] == '1') //�ǶȻ����� $0,0,0,0,1,1,AP23.54,AD85.45,VP10.78,VI0.26#
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
		memcpy(apad, ProtocolString + pos + z + 1, int9num - (pos + z)); //�洢AD���������
		z = StringFind(apad, ",");
		if(z == -1) return;
		memcpy(advalue,apad+2, z-2);
		
		BST_fCarAngle_D=atof(advalue);

		UART3_Send_Char("$OK#"); //����Э�����ݰ�  				
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
		memcpy(vpvi, ProtocolString + pos + z + 1, int9num - (pos + z)); //�洢AD���������
		z = StringFind(vpvi, "#");
		if(z == -1) return;
		memcpy(vivalue,vpvi+2, z-2);
		
		BST_fCarSpeed_I=atof(vivalue);

		UART3_Send_Char("$OK#"); //����Э�����ݰ�  		
			
			
	}
	
	newLineReceived = 0;  
	memset(ProtocolString, 0x00, sizeof(ProtocolString));  
	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//ʹ�ܽ����ж�

}

 
 
 
 
 
 
 
 
 
