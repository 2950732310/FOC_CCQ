#ifndef FOC_H
#define FOC_H
#include "main.h"
#include "motor_control.h"

/**
 * @brief  更新当前电流采样值
 * @param  motor: 电机数据结构体指针
 */
void FOC_UpdateCurrentSampling(MOTOR_DATA *motor);

/**
 * @brief  电机对齐
 * @param  motor: 电机数据结构体指针
 * @retval true: 对齐成功
 * @retval false: 对齐失败
 */
bool Foc_Align(MOTOR_DATA *motor);


/**
 * @brief  电流偏移校准
 * @param  motor: 电机数据结构体指针
 */
void FOC_CurrentOffsetCalibration(MOTOR_DATA *motor);


/**
 * @brief  开启闭环控制
 * @param  motor: 电机数据结构体指针
 * @retval MOTOR_ERROR_OK: 开启成功
 * @retval MOTOR_ERROR_INVALID_PARAM: 参数错误
 */
MOTOR_ERROR OpenControlMode(MOTOR_DATA *motor);


/**
 * @brief  电流控制
 * @param  motor: 电机数据结构体指针
 * @retval MOTOR_ERROR_OK: 电流控制成功
 * @retval MOTOR_ERROR_INVALID_PARAM: 参数错误
 */
MOTOR_ERROR CurrentControl(MOTOR_DATA *motor);
#endif

