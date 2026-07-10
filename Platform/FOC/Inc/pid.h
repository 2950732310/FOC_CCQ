#ifndef PID_H_
#define PID_H_

#include "general_def.h" // 包含通用定义的头文件

enum PID_MODE
{
  PID_POSITION = 0, // 位置式
  PID_DELTA         // 增量式
};

typedef struct
{
  uint8_t mode;   // PID模式

  float Kp;       // P项系数
  float Ki;       // I项系数
  float Kd;       // D项系数

  float max_out;  // PID最大输出
  float max_iout; // PID最大积分输出

  float set;      // 设定值
  float fdb;      // 反馈值

  float out;      // PID输出值
  float Pout;     // P项输出值
  float Iout;     // I项输出值
  float Dout;     // D项输出值
  float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
  float error[3]; // 误差项 0最新 1上一次 2上上次

} PidTypeDef;

/**
 * @brief          PID结构体初始化
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_POSITION: 位置式PID
 *                 PID_DELTA: 增量式PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: PID最大输出
 * @param[in]      max_iout: PID最大积分输出
 * @retval         无
 */
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
 * @brief          PID计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈值
 * @param[in]      set: 设定值
 * @retval         PID输出值
 */
extern float PID_Calc(PidTypeDef *pid, float ref, float set);

/**
 * @brief          PID输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         无
 */
extern void PID_clear(PidTypeDef *pid);

#endif /* CODE_INCLUDE_PID_H_ */
