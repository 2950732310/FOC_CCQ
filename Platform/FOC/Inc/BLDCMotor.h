#ifndef BLDCMOTOR_H
#define BLDCMOTOR_H
#include "main.h"
#include "tim.h"
#include "general_def.h"
#include "arm_math.h"

// 基础配置
#define PWM_ARR (htim1.Init.Period)  						    // PWM 自动重装载值用来计算占空比

// 定义宏，用于设置FOC电机的PWM
#define set_foc_pwm_a(value)        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value)     // 设置A相PWM占空比
#define set_foc_pwm_b(value)        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value)     // 设置B相PWM占空比
#define set_foc_pwm_c(value)        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value)     // 设置C相PWM占空比


/* 使用电机型号 */
#define MOTOR_2312 0
#define MOTOR_4310 1
/* 通用参数配置 */
#define BATVEL 				12.0f    					       					// 供电电压 V
#define INVBATVEL 		    (1.0f / BATVEL) 		  			                // 供电电压的倒数

#define TIMER1_CLK_MHz 168                                                      // 定时器时钟频率
#define PWM_FREQUENCY 20000                                                     // PWM频率20KHz
#define PWM_MEASURE_PERIOD (float)(1.0f / (float)PWM_FREQUENCY)                 // PWM周期
#define CURRENT_MEASURE_HZ PWM_FREQUENCY                                        // 电流频率
#define CURRENT_MEASURE_PERIOD (float)(1.0f / (float)CURRENT_MEASURE_HZ)        // 电流周期
#define TS  				1.0f

#define V_REG 1.65f                                                             // ADC参考电压
#define CURRENT_SHUNT_RES 0.05f                                                 // 电流采样电阻
#define CURRENT_AMP_GAIN 50.0f                                                  // 电流放大倍数
#define VIN_R1 1000.0f                                                          // 母线电压采样下分压电阻
#define VIN_R2 10000.0f                                                         // 母线电压采样上分压电阻
#define FAC_CURRENT ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN)) // 电流放大倍数
#define VOLTAGE_TO_ADC_FACTOR (((VIN_R2 + VIN_R1) / VIN_R1) * (3.3f / 4095.0f)) // 电压放大倍数


/* 电机实际参数 */
#if MOTOR_2312
#define MPTOR_P             7u                  // 电机极对数
#define MOTOR_RS            0.0995f             // 相电阻（Ω）
#define MOTOR_LS            0.00001822f         // 相电感（H）
#define MOTOR_NOM_CURRENT   1.0f                // 额定电流（A）
#define MOTOR_TORQUE_LIMIT  0.0f                // 额定转矩（Nm）
#define MOTOR_TORQUE_K      0.0f                // 转矩常数（Nm/A）
#define MOTOR_IQ_MAX        1.0f                // 最大电流（A）
#define MOTOR_FLUX          0.0f                // 磁链
#define MOTOR_DIRECTION     CCW                 // 电机方向 (CCW: 逆时针)方向在电角度零点校准之后不可更改如需更改需重新校准电角度
#define MAX_VOLTAGE         0.5f                // 电角度零点对齐的最大电压 V（与电机相电阻有关系）
#define MAX_VEL_LIMIT       50.0f               // 速度环最大限幅 rev/s
#define MAX_POS_LIMIT       360.0f              // 位置环最大限幅 
#define TRAPEZOID_VEL       15000.0f            // 梯型加减速最大速度 (°/s) */
#define TRAPEZOID_ACC       40000.0f            // 梯型加减速最大加速度 (°/s²) */
#define MAX_V_LIMIT		    2.5f  				// 驱动电压最大限幅 
#define MAX_I_LIMIT		    MAX_V_LIMIT * 0.8f  // 驱动电流积分项最大限幅 80% 的最大电压
#define IQ_KP               0.7f                // q轴电流环比例系数
#define IQ_KI               80.0f               // d轴电流环积分系数
#define IQ_KD               0.0f                // d轴电流环微分系数
#define ID_KP               0.1f                // d轴电流环比例系数
#define ID_KI               25.0f               // d轴电流环积分系数
#define ID_KD               0.0f                // d轴电流环微分系数
#define VEL_KP              0.1f                // 速度环比例系数
#define VEL_KI              0.7f                // 速度环积分系数
#define VEL_KD              0.0f                // 速度环微分系数
#define POS_KP              0.8f                // 位置环比例系数
#endif

/* 电机实际参数 */
#if MOTOR_4310
#define MPTOR_P             11u                                 // 电机极对数
#define MOTOR_RS            4.835f                              // 相电阻（Ω）
#define MOTOR_LS            0.00123816666666f                   // 相电感（H）
#define MOTOR_NOM_CURRENT   0.9f                                // 额定电流（A）
#define MOTOR_TORQUE_LIMIT  0.23f                               // 额定转矩（Nm）
#define MOTOR_TORQUE_K      0.132f                              // 转矩常数（Nm/A）
#define MOTOR_IQ_MAX        1.0f                                // 最大电流（A）
#define MOTOR_FLUX          0.0f                                // 磁链
#define MOTOR_DIRECTION     CCW                                 // 电机方向 (CCW: 逆时针)方向在电角度零点校准之后不可更改如需更改需重新校准电角度
#define MAX_VOLTAGE         2.5f                                // 电角度零点对齐的最大电压 V（与电机相电阻有关系）
#define MAX_VEL_LIMIT       20.0f                               // 速度环最大限幅 rev/s
#define MAX_POS_LIMIT       360.0f                              // 位置环最大限幅 °
#define TRAPEZOID_VEL       7000.0f                             // 梯型加减速最大速度 (°/s) */
#define TRAPEZOID_ACC       20000.0f                            // 梯型加减速最大加速度 (°/s²) */
#define MAX_V_LIMIT		    BATVEL / _SQRT3    				    // 驱动电压最大限幅 1/根3 倍的母线电压
#define MAX_I_LIMIT		    MAX_V_LIMIT * 0.8f    				// 驱动电流积分项最大限幅 80% 的最大电压
#define IQ_KP               10.0f                               // q轴电流环比例系数
#define IQ_KI               40.0f                               // d轴电流环积分系数
#define IQ_KD               0.0f                                // d轴电流环微分系数
#define ID_KP               0.1f                                // d轴电流环比例系数
#define ID_KI               25.0f                               // d轴电流环积分系数
#define ID_KD               0.0f                                // d轴电流环微分系数
#define VEL_KP              0.1f                               // 速度环比例系数
#define VEL_KI              0.05f                                // 速度环积分系数
#define VEL_KD              0.0f                                // 速度环微分系数
#define POS_KP              0.95f                                // 位置环比例系数


#endif

// 扇区枚举
typedef enum
{
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} svpwm_sector_t;

// flash 数据结构体
typedef struct
{
  uint32_t first_run;            // 首次运行标志位
  uint32_t version;              // 版本号
  float encoder_offset;          // 编码器偏移量
  float ia_zero;                 // A相电流零点
  float ib_zero;                 // B相电流零点
  float ic_zero;                 // C相电流零点
  float iq_kp;                   // q轴电流环比例系数
  float iq_ki;                   // q轴电流环积分系数
  float iq_kd;                   // q轴电流环微分系数
  float id_kp;                   // d轴电流环比例系数
  float id_ki;                   // d轴电流环积分系数
  float id_kd;                   // d轴电流环微分系数
  float vel_kp;                  // 速度环比例系数
  float vel_ki;                  // 速度环积分系数
  float vel_kd;                  // 速度环微分系数
  float pos_kp;                  // 位置环比例系数
  float pos_ki;                  // 位置环积分系数
  float pos_kd;                  // 位置环微分系数
}flash_data_t;


// FOC 数据结构体
typedef struct
{   
    /* 以下部分需要存储到flash中 */
    flash_data_t flash_data;

    /* 以下部分不需要存储到flash中 */
    float vbus;             // 母线电压
    float inv_vbus;         // 母线电压的倒数
    float theta;            // 电角度

    float sin_val;          // 电角度的正弦值
    float cos_val;          // 电角度的余弦值

    float i_a;              // a相电流
    float i_b;              // b相电流
    float i_c;              // c相电流

    float v_a;              // a相电压
    float v_b;              // b相电压
    float v_c;              // c相电压

    float i_d;              // D 坐标系电流（FOC中的直轴电流）
    float i_q;              // Q 坐标系电流（FOC中的侧轴电流）

    float v_d;              // D 坐标系电压（FOC中的直轴电压）
    float v_q;              // Q 坐标系电压（FOC中的侧轴电压）

    float i_alpha;          // Alpha 坐标系电流（Clarke变换后的电流）
    float i_beta;           // Beta 坐标系电流（Clarke变换后的电流）

    float v_alpha;          // Alpha 坐标系电压（逆Clarke变换后的电压）
    float v_beta;           // Beta 坐标系电压（逆Clarke变换后的电压）

    float dtc_a;            // A 相 PWM 占空比
    float dtc_b;            // B 相 PWM 占空比
    float dtc_c;            // C 相 PWM 占空比

    float vd_set;           // d轴电压设置
    float vq_set;           // q轴电压设置
	
    float id_set;           // d轴电流设置
    float iq_set;           // q轴电流设置

	float Ualpha_norm;		// α轴电压/母线电压
	float Ubeta_norm;		// β轴电压/母线电压

    float vel_set;          // 速度参考值 (rad/s)
    float vel_fb;           // 速度反馈值 (rad/s)

    float pos_set;          // 位置参考值 (°)
    float pos_fb;           // 位置反馈值 (°)
    float pos_ref;          // 梯度控制目标位置
   


}FOC_DATA;


void Foc_Pwm_Start(void);
void Foc_Pwm_Stop(void);
void Foc_Pwm_LowSides(void);
void Foc_Set_Pwm(FOC_DATA *foc);

void Sin_Cos_Val(FOC_DATA *foc);
void Clarke(FOC_DATA *foc);
void Inv_clarke(FOC_DATA *foc);
void Park(FOC_DATA *foc);
void Inv_Park(FOC_DATA *foc);
void Svpwm_Midpoint(FOC_DATA *foc);
void Svpwm_Sector(FOC_DATA *foc);
int Svpwm(FOC_DATA *foc);
#endif
// BLDCMOTOR_H
