#ifndef __UPSTANDINGCAR_H
#define __UPSTANDINGCAR_H
#include "stm32f10x.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0) 


/**********�Ƕȿ��ƺ궨��**********/									
#define    CAR_ZERO_ANGLE (1)		 //��ʼС������ṹ����С�����ڴ�ֱ����Ϊ��Ƕȵ����������Ҫ������������ֱ�����Ƕ�ֵ��

/******�ٶȿ�����غ궨��******/
#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         0
#define MOTOR_LEFT_SPEED_POSITIVE  (BST_fLeftMotorOut >0)
#define MOTOR_RIGHT_SPEED_POSITIVE (BST_fRightMotorOut>0)
#define OPTICAL_ENCODE_CONSTANT  13	//������̶̿Ȳ�
#define SPEED_CONTROL_PERIOD	 40	    //�ٶȻ���������
#define CAR_SPEED_CONSTANT		(1000.0/(float)SPEED_CONTROL_PERIOD/(float)OPTICAL_ENCODE_CONSTANT)
// #define CAR_POSITION_MAX	(MOTOR_OUT_MAX*10)//500////20
// #define CAR_POSITION_MIN	(MOTOR_OUT_MIN*10) //-500//

#define CAR_POSITION_MAX	7000       //500////20
#define CAR_POSITION_MIN	(-7000)     //-500//
/******���������غ궨��******/
#define MOTOR_OUT_DEAD_VAL       0	   //����ֵ8
#define MOTOR_OUT_MAX           1000	   //ռ�ձ������ֵ
#define MOTOR_OUT_MIN         (-1000)   //ռ�ձȸ����ֵ

#define	MOTOR_LEFT_AIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_15))  //PB15��Ӧ�������оƬ���ƽŶ��壬AIN1Ϊ��ʱ��PB15��Ϊ0
#define	MOTOR_LEFT_AIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_15))	  //PB15��Ӧ�������оƬ���ƽŶ��壬AIN1Ϊ��ʱ��PB15��Ϊ1
#define	MOTOR_LEFT_AIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_14))  //PB14��Ӧ�������оƬ���ƽŶ��壬AIN2Ϊ��ʱ��PB14��Ϊ0
#define	MOTOR_LEFT_AIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_14))	  //PB14��Ӧ�������оƬ���ƽŶ��壬AIN2Ϊ��ʱ��PB14��Ϊ1

#define	MOTOR_RIGHT_BIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_12))//PB12��Ӧ�������оƬ���ƽŶ��壬BIN1Ϊ��ʱ��PB12��Ϊ0
#define	MOTOR_RIGHT_BIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_12))		//PB12��Ӧ�������оƬ���ƽŶ��壬BIN1Ϊ��ʱ��PB12��Ϊ1
#define	MOTOR_RIGHT_BIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_13))//PB13��Ӧ�������оƬ���ƽŶ��壬BIN2Ϊ��ʱ��PB13��Ϊ0
#define	MOTOR_RIGHT_BIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_13))		//PB13��Ӧ�������оƬ���ƽŶ��壬BIN2Ϊ��ʱ��PB13��Ϊ1

extern float BST_fCarAngle;					//extern���ڱ������ߺ���ǰ���Ա�ʾ�������ߺ����Ķ����ڱ���ļ��У���ʾ�����������˱�������ʱ��������ģ����Ѱ���䶨�塣
extern float BST_fBluetoothSpeed;
extern float BST_fBluetoothDirectionR;
extern float BST_fBluetoothDirectionL;
extern u8 BST_u8MainEventCount;
extern u8 BST_u8SpeedControlCount;
extern float BST_fSpeedControlOut,BST_fCarAngle_P;
extern float  BST_fAngleControlOut;
extern float BST_fSpeedControlOutNew;
extern u8 BST_u8SpeedControlPeriod;
extern u8 BST_u8DirectionControlPeriod;
extern u8 BST_u8DirectionControlCount;
extern u8 BST_u8LEDCount; 
extern u8 BST_u8trig;
extern u8 BST_u8turnPeriod;
extern u8 BST_u8turnCount;
extern u8 ucBluetoothValue;
extern float angle;
extern float anglex;
extern float gyx,gy0;
extern float gyrx;
extern float gyry;
extern float accelx,accely,accelz,gyrx,gyry,gyrz;
extern float BST_fLeftMotorOut,BST_fRightMotorOut,BST_fBluetoothDirectionNew;
extern s16 BST_s16LeftMotorPulse,BST_s16RightMotorPulse;
extern float juli;
extern 	int x,y1,z1,y2,z2,flagbt;
extern float BST_fCarSpeed_I,BST_fCarSpeed_P,BST_fCarAngle_P,BST_fCarAngle_D;

extern void CarStateOut(void);   	//״̬�����Ƴ�״̬
extern void ProtocolCpyData(void); //���ƴ�������
extern void SendAutoUp(void);


void delay_nms(u16 time);
void CarUpstandInit(void);
//void SampleInputVoltage(void);
void AngleControl(void)	 ;
void MotorOutput(void);
void SpeedControl(void);
void BluetoothControl(void)	;
void GetMotorPulse(void);
void SpeedControlOutput(void);
void DirectionControlOutput(void);
void DirectionControl(void); 
void chaoshengbo(void);
void gfcsbOutput(void);
void csbcontrol(void);
void turn(void);
void turnfliteroutput(void);
void InitMPU6050(void);
void kalmanfilter(float Gyro,float Accel);
void kalmanangle(void);

extern u8 newLineReceived;
void Protocol(void);  /*Э�����*/
#endif
