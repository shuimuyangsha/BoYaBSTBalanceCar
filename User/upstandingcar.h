#ifndef __UPSTANDINGCAR_H
#define __UPSTANDINGCAR_H
#include "stm32f10x.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0) 


/**********角度控制宏定义**********/									
#define    CAR_ZERO_ANGLE (1)		 //初始小车车体结构导致小车存在垂直方向不为零角度的情况，固需要消除此误差，增加直立误差角度值。

/******速度控制相关宏定义******/
#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         0
#define MOTOR_LEFT_SPEED_POSITIVE  (BST_fLeftMotorOut >0)
#define MOTOR_RIGHT_SPEED_POSITIVE (BST_fRightMotorOut>0)
#define OPTICAL_ENCODE_CONSTANT  13	//光电码盘刻度槽
#define SPEED_CONTROL_PERIOD	 40	    //速度环控制周期
#define CAR_SPEED_CONSTANT		(1000.0/(float)SPEED_CONTROL_PERIOD/(float)OPTICAL_ENCODE_CONSTANT)
// #define CAR_POSITION_MAX	(MOTOR_OUT_MAX*10)//500////20
// #define CAR_POSITION_MIN	(MOTOR_OUT_MIN*10) //-500//

#define CAR_POSITION_MAX	7000       //500////20
#define CAR_POSITION_MIN	(-7000)     //-500//
/******电机控制相关宏定义******/
#define MOTOR_OUT_DEAD_VAL       0	   //死区值8
#define MOTOR_OUT_MAX           1000	   //占空比正最大值
#define MOTOR_OUT_MIN         (-1000)   //占空比负最大值

#define	MOTOR_LEFT_AIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_15))  //PB15对应电机驱动芯片控制脚定义，AIN1为负时，PB15设为0
#define	MOTOR_LEFT_AIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_15))	  //PB15对应电机驱动芯片控制脚定义，AIN1为正时，PB15设为1
#define	MOTOR_LEFT_AIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_14))  //PB14对应电机驱动芯片控制脚定义，AIN2为负时，PB14设为0
#define	MOTOR_LEFT_AIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_14))	  //PB14对应电机驱动芯片控制脚定义，AIN2为正时，PB14设为1

#define	MOTOR_RIGHT_BIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_12))//PB12对应电机驱动芯片控制脚定义，BIN1为负时，PB12设为0
#define	MOTOR_RIGHT_BIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_12))		//PB12对应电机驱动芯片控制脚定义，BIN1为正时，PB12设为1
#define	MOTOR_RIGHT_BIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_13))//PB13对应电机驱动芯片控制脚定义，BIN2为负时，PB13设为0
#define	MOTOR_RIGHT_BIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_13))		//PB13对应电机驱动芯片控制脚定义，BIN2为正时，PB13设为1

extern float BST_fCarAngle;					//extern置于变量或者函数前，以表示变量或者函数的定义在别的文件中，提示编译器遇到此变量或函数时，在其它模块中寻找其定义。
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

extern void CarStateOut(void);   	//状态机控制车状态
extern void ProtocolCpyData(void); //复制串口数据
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
void Protocol(void);  /*协议相关*/
#endif
