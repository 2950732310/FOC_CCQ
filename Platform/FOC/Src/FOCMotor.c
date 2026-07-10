#include "FOCMotor.h"
#include "DWT.h"
#include "adc.h"
#include "general_def.h"


void FOC_UpdateCurrentSampling(MOTOR_DATA *motor)
{
    motor->foc.i_a = ((uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) - motor->foc.flash_data.ia_zero ) * FAC_CURRENT;
    motor->foc.i_b = ((uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) - motor->foc.flash_data.ib_zero ) * FAC_CURRENT;    
    motor->foc.i_c = ((uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - motor->foc.flash_data.ic_zero ) * FAC_CURRENT;
    motor->foc.vbus = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4) * VOLTAGE_TO_ADC_FACTOR;
}


bool Foc_Align(MOTOR_DATA *motor)
{
    const uint8_t sample_count = 50;
    uint32_t sum_count = 0;

    /* 设置固定电角度 */
    motor->foc.theta = 0.0f;
    motor->foc.v_q = 0.0f;
    Sin_Cos_Val(&motor->foc);

    /* 缓慢增加d轴电压 */
    float align_voltage = 0.0f;
    const float max_voltage = MAX_VOLTAGE;
    while (align_voltage < max_voltage)
    {

        align_voltage += 0.01f;
        motor->foc.v_d = align_voltage;
        Inv_Park(&motor->foc);

        motor->foc.Ualpha_norm = motor->foc.v_alpha * INVBATVEL;
        motor->foc.Ubeta_norm = motor->foc.v_beta * INVBATVEL;

        Svpwm(&motor->foc);

        Foc_Set_Pwm(&motor->foc);

        DWT_Delay_ms(5);
    }

    /* 等待转子稳定 */
    DWT_Delay_ms(1000);


    /* 计算平均位置 */
    sum_count = 0;
    for (uint8_t i = 0; i < sample_count; i++)
    {
        GetMotor_Angle(motor->mt6816);
        sum_count += motor->mt6816->count_in_cpr_;
        DWT_Delay_ms(10);
    }
    motor->mt6816->encoder_offset = sum_count / sample_count;

    /* 重新获取编码器位置 */
    GetMotor_Angle(motor->mt6816);

    /* 重置d q 轴电压 */
    motor->foc.v_d = 0.0f;
    motor->foc.v_q = 0.0f;
    motor->foc.dtc_a = 0;
    motor->foc.dtc_b = 0;
    motor->foc.dtc_c = 0;
    Foc_Set_Pwm(&motor->foc);

    /* 数据保存 */
    motor->foc.flash_data.encoder_offset = motor->mt6816->encoder_offset;
    return true;
}

void FOC_CurrentOffsetCalibration(MOTOR_DATA *motor)
{
  uint32_t sum_a = 0;
  uint32_t sum_b = 0;
  uint32_t sum_c = 0;
  const uint16_t samples = 256;
  uint16_t i;
  uint16_t raw_a = 0;
  uint16_t raw_b = 0;
  uint16_t raw_c = 0;

  /* 1. 先停止 PWM 输出并关闭驱动，确保电流回路处于零激励状态 */
  motor->foc.v_d = 0.0f;
  motor->foc.v_q = 0.0f;
  motor->foc.dtc_a = 0;
  motor->foc.dtc_b = 0;
  motor->foc.dtc_c = 0;
  Foc_Set_Pwm(&motor->foc);
  Foc_Pwm_LowSides();

  /* 2. 等待 ADC/电流回路稳定 */
  DWT_Delay_ms(20);

  /* 3. 当前配置是注入通道 + TIM1触发 + 中断回调，不使用 DMA。
   *    因此直接读取注入通道数据寄存器，并多次取平均。 */
  for (i = 0; i < samples; i++)
  {
    DWT_Delay_us(50);
    raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    raw_c = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

    sum_a += raw_a;
    sum_b += raw_b;
    sum_c += raw_c;
  }

  /* 4. 保存零点偏移 */
  motor->foc.flash_data.ia_zero = (float)sum_a / (float)samples;
  motor->foc.flash_data.ib_zero = (float)sum_b / (float)samples;
  motor->foc.flash_data.ic_zero = (float)sum_c / (float)samples;

  /* 5. 恢复输出，避免后续控制被卡住 */
  motor->foc.v_d = 0.0f;
  motor->foc.v_q = 0.0f;
  motor->foc.dtc_a = 0;
  motor->foc.dtc_b = 0;
  motor->foc.dtc_c = 0;
  Foc_Set_Pwm(&motor->foc);
  DWT_Delay_ms(1);
}


MOTOR_ERROR OpenControlMode(MOTOR_DATA *motor)
{
    // float Ts = 0.00005f;
    // if (Ts <= 0 || Ts > 0.5f)
    //     Ts = 1e-3f;
    // float target_velocity = 200.0f;
    // motor->foc.theta = normalize_angle(motor->foc.theta + target_velocity * Ts); // 固定角度、周期

    /* 获取当前电角度 */
    motor->foc.theta = motor->mt6816->phase_ ;

    /* 设置电压 */
    motor->foc.v_d = 0.0f;
    motor->foc.v_q = CLAMP(motor->foc.vq_set, -MAX_V_LIMIT, MAX_V_LIMIT);

    /* 坐标变换 */
    Sin_Cos_Val(&motor->foc);
    Inv_Park(&motor->foc);

    /* svpwm 输出 */
    motor->foc.Ualpha_norm = motor->foc.v_alpha * INVBATVEL;
    motor->foc.Ubeta_norm = motor->foc.v_beta * INVBATVEL;
    if (0 == Svpwm(&motor->foc))
    {
        Foc_Set_Pwm(&motor->foc);
        return M_OK;
    }

    return M_ERR;
}

int a = 0;
MOTOR_ERROR CurrentControl(MOTOR_DATA *motor)
{
    if (a == 0)
    {
        /* 初始化电流环q轴PID */
        PID_Init(&motor->IqPID, PID_POSITION, &motor->foc.flash_data.iq_kp,MAX_V_LIMIT ,MAX_V_LIMIT );
        /* 初始化电流环d轴PID */
        PID_Init(&motor->IdPID, PID_POSITION, &motor->foc.flash_data.id_kp,MAX_V_LIMIT ,MAX_V_LIMIT);
        a = 1;
    }
    
    motor->foc.theta = motor->mt6816->phase_;
    Sin_Cos_Val(&motor->foc);

    Clarke(&motor->foc);
    Park(&motor->foc);

    motor->foc.iq_set = CLAMP(motor->foc.iq_set, -MOTOR_IQ_MAX,MOTOR_IQ_MAX);
    motor->foc.i_q    = CLAMP(motor->foc.i_q, -MOTOR_IQ_MAX,MOTOR_IQ_MAX);

    motor->foc.v_q = PID_Calc(&motor->IqPID, motor->foc.i_q, motor->foc.iq_set);
    motor->foc.v_d = PID_Calc(&motor->IdPID, motor->foc.i_d, motor->foc.id_set);

    Inv_Park(&motor->foc);
    motor->foc.Ualpha_norm = motor->foc.v_alpha * INVBATVEL;
    motor->foc.Ubeta_norm = motor->foc.v_beta * INVBATVEL;
    if (0 == Svpwm(&motor->foc))
    {
        Foc_Set_Pwm(&motor->foc);
        return M_OK;
    }
}