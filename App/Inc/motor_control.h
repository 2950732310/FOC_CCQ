#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "main.h"
#include "BLDCMotor.h"
#include "mt6816_encoder.h"
#include "pid.h"

/* 电机错误状态枚举 */
typedef enum
{
	M_ERR	= -1,
	M_OK 	= 0,
}MOTOR_ERROR;


/* 电机运行模式枚举 */
typedef enum 
{
  STATE_MODE_IDLE = 0,  // 空闲模式
  STATE_MODE_DETECTING, // 检测模式
  STATE_MODE_RUNNING,   // 运行模式
}STATE_MODE;


/* 电机闭环类型枚举 */
typedef enum
{
    CONTROL_MODE_OPEN = 0,          // 开环控制模式
    CONTROL_MODE_TORQUE = 1,        // 力矩闭环控制模式
    CONTROL_MODE_VELOCITY = 2,      // 速度闭环控制模式
    CONTROL_MODE_POSITION = 3,      // 位置闭环控制模式
} CONTROL_MODE;

typedef struct
{
	STATE_MODE 		State_Mode;				// 运行状态
	CONTROL_MODE 	Control_Mode;			// 闭环类型
	FOC_DATA 		  foc;					    // FOC参数结构体
	ENCODER_DATA 	*mt6816;				  // 编码器数据结构体

  PidTypeDef IqPID;               // 控制电流环IQ的PID控制器
  PidTypeDef IdPID;               // 控制电流环ID的PID控制器
  PidTypeDef VelPID;              // 控制电机速度的PID控制器
  PidTypeDef PosPID;              // 控制电机位置的PID控制器

}MOTOR_DATA;

extern MOTOR_DATA motor;

/*
 * @brief  启动电机控制
 * @param  无
 * @return 无
 */
void Motor_StartControl(void);

/*
 * @brief  电流采样
 * @param  无
 * @return 无
 */
void Current_Sampling(void);
#endif