#include "BLDCMotor.h"

/**
 * @brief 启动FOC电机PWM输出
 * @param None
 * @retval None
 */
void Foc_Pwm_Start(void)
{
    // 设置初始占空比为 50% (让半桥输出 VCC/2，电机不转)
    set_foc_pwm_a(PWM_ARR >> 1);
    set_foc_pwm_b(PWM_ARR >> 1);
    set_foc_pwm_c(PWM_ARR >> 1);

    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}


/**
 * @brief 停止FOC电机PWM输出
 * @param None
 * @retval None
 */
void Foc_Pwm_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}


/**
 * @brief 使FOC电机低侧输出低电平 (用于电机刹车)
 * @param None
 * @retval None
 */
void Foc_Pwm_LowSides(void)
{
    // 设置占空比为 0% (让半桥输出 GND)
    set_foc_pwm_a(0);
    set_foc_pwm_b(0);
    set_foc_pwm_c(0);
    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}


/**
 * @brief 设置FOC电机PWM占空比
 * @param a 通道1占空比 (0-1)
 * @param b 通道2占空比 (0-1)
 * @param c 通道3占空比 (0-1)
 * @retval None
 */
void Foc_Set_Pwm(FOC_DATA *foc)
{
    set_foc_pwm_a((uint16_t)(foc->dtc_a *PWM_ARR));  // 设置通道1的占空比
    set_foc_pwm_b((uint16_t)(foc->dtc_b *PWM_ARR));  // 设置通道2的占空比
    set_foc_pwm_c((uint16_t)(foc->dtc_c *PWM_ARR));  // 设置通道3的占空比
}


/**
 * @brief 计算正弦和余弦值
 *
 * 计算电角度 theta 对应的 sin 和 cos 值
 *
 * @param foc 指向电机数据结构的指针
 */
void Sin_Cos_Val(FOC_DATA *foc)
{
    foc->sin_val = arm_sin_f32(foc->theta); // 计算正弦值
    foc->cos_val = arm_cos_f32(foc->theta); // 计算余弦值
}


/**
 * @brief Clarke变换
 *
 * Clarke变换将三相电流转换为 Alpha-Beta 坐标系下的电流
 *
 * @param foc 指向电机数据结构的指针
 */
void Clarke(FOC_DATA *foc)
{
    foc->i_alpha = foc->i_a;                            // α轴电流等于a相电流
    foc->i_beta  = (foc->i_b - foc->i_c) * ONE_BY_SQRT3; // β轴电流计算，使用√3的倒数进行归一化
}


/**
 * @brief 逆Clarke变换
 *
 * 逆Clarke变换将 Alpha-Beta 坐标系下的电压转换为三相电压
 *
 * @param foc 指向电机数据结构的指针
 */
void Inv_clarke(FOC_DATA *foc)
{
    foc->v_a = foc->v_alpha;                                  // a相电压等于α轴电压
    foc->v_b = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta; // b相电压计算
    foc->v_c = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta; // c相电压计算
}


/**
 * @brief Park变换
 *
 * Park变换将 Alpha-Beta 坐标系下的电流转换为 dq 坐标系下的电流，转换成了两个恒定的直流分量
 *
 * @param foc 指向电机数据结构的指针
 */
void Park(FOC_DATA *foc)
{
    foc->i_d = foc->i_alpha * foc->cos_val + foc->i_beta * foc->sin_val; // d轴电流计算
    foc->i_q = foc->i_beta * foc->cos_val - foc->i_alpha * foc->sin_val; // q轴电流计算
}


/**
 * @brief 逆Park变换
 *
 * 反Park变换将 dq 坐标系下的电压转换为 Alpha-Beta 坐标系下的电压
 *
 * @param foc 指向电机数据结构的指针
 */
void Inv_Park(FOC_DATA *foc)
{
    foc->v_alpha = (foc->v_d * foc->cos_val - foc->v_q * foc->sin_val); // α轴电压计算
    foc->v_beta  = (foc->v_d * foc->sin_val + foc->v_q * foc->cos_val);  // β轴电压计算
}


/**
 * @brief 均值零序分量注入
 *
 * 对α-β坐标系的电压进行均值零序分量注入，以便进行SVPWM调制
 * 电压角度是垂直于转子角度的，n代表的是合成电压的扇区，不是转子所在的扇区
 *
 * @param foc 指向电机数据结构的指针
 */
void Svpwm_Midpoint(FOC_DATA *foc)
{
    // 根据母线电压调整α和β轴电压
    foc->v_alpha = foc->inv_vbus * foc->v_alpha;
    foc->v_beta  = foc->inv_vbus * foc->v_beta;

    float va     = foc->v_alpha;                                  // 保存α轴电压
    float vb     = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta; // 计算b相电压
    float vc     = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta; // 计算c相电压

    float vmax   = max(max(va, vb), vc); // 找到最大电压值
    float vmin   = min(min(va, vb), vc); // 找到最小电压值

    float vcom   = (vmax + vmin) * 0.5f; // 计算零序分量

    // 计算各相的占空比调整值，确保在0到1之间
    foc->dtc_a   = ((vcom - va) + 0.5f), 0.0f, 1.0f;
    foc->dtc_b   = ((vcom - vb) + 0.5f), 0.0f, 1.0f;
    foc->dtc_c   = ((vcom - vc) + 0.5f), 0.0f, 1.0f;
}


/**
 * @brief 扇区判断
 *
 * 根据α-β坐标系中的电压值判断当前所在扇区，并计算各相的占空比
 *
 * @param foc 指向电机数据结构的指针
 */
void Svpwm_Sector(FOC_DATA *foc)
{
    float ta    = 0.0f, tb = 0.0f, tc = 0.0f;                    // 各相占空比时间初始化
    float k     = (TS * _SQRT3) * foc->inv_vbus;                 // 根据母线电压计算k值
    float va    = foc->v_beta;                                   // β轴电压
    float vb    = (_SQRT3 * foc->v_alpha - foc->v_beta) * 0.5f;  // b相电压计算
    float vc    = (-_SQRT3 * foc->v_alpha - foc->v_beta) * 0.5f; // c相电压计算
    int a       = (va > 0.0f) ? 1 : 0;                           // 判断a相是否大于零，结果为1或0
    int b       = (vb > 0.0f) ? 1 : 0;                           // 判断b相是否大于零，结果为1或0
    int c       = (vc > 0.0f) ? 1 : 0;                           // 判断c相是否大于零，结果为1或0
    int sextant = (c << 2) + (b << 1) + a;                       // 根据a、b、c相的状态确定扇区

    switch (sextant)
    {
    case SECTOR_3: // 扇区3
    {
        float t4 = k * vb;
        float t6 = k * va;
        float t0 = (TS - t4 - t6) * 0.5f; // 零矢量的作用时间

        ta       = t4 + t6 + t0; // 计算a相占空比时间
        tb       = t6 + t0;      // 计算b相占空比时间
        tc       = t0;           // c相占空比时间为t0
    }
    break;

    case SECTOR_1: // 扇区1
    {
        float t6 = -k * vc;
        float t2 = -k * vb;
        float t0 = (TS - t2 - t6) * 0.5f;

        ta       = t6 + t0;      // a相占空比时间计算
        tb       = t2 + t6 + t0; // b相占空比时间计算
        tc       = t0;           // c相占空比时间为t0
    }
    break;

    case SECTOR_5: // 扇区5
    {
        float t2 = k * va;
        float t3 = k * vc;
        float t0 = (TS - t2 - t3) * 0.5f;

        ta       = t0;           // a相占空比时间为t0
        tb       = t2 + t3 + t0; // b相占空比时间计算
        tc       = t3 + t0;      // c相占空比时间计算
    }
    break;

    case SECTOR_4: // 扇区4
    {
        float t1 = -k * va;
        float t3 = -k * vb;
        float t0 = (TS - t1 - t3) * 0.5f;

        ta       = t0;           // a相占空比时间为t0
        tb       = t3 + t0;      // b相占空比时间计算
        tc       = t1 + t3 + t0; // c相占空比时间计算
    }
    break;

    case SECTOR_6: // 扇区6
    {
        float t1 = k * vc;
        float t5 = k * vb;
        float t0 = (TS - t1 - t5) * 0.5f;

        ta       = t5 + t0;      // a相占空比时间计算
        tb       = t0;           // b相占空比时间为t0
        tc       = t1 + t5 + t0; // c相占空比时间计算
    }
    break;

    case SECTOR_2: // 扇区2
    {
        float t4 = -k * vc;
        float t5 = -k * va;
        float t0 = (TS - t4 - t5) * 0.5f;

        ta       = t4 + t5 + t0; // a相占空比时间计算
        tb       = t0;           // b相占空比时间为t0
        tc       = t5 + t0;      // c相占空比时间计算
    }
    break;

    default:
        break; // 默认情况，不做处理
    }

    // 更新各相的占空比调整值，反转以适应SVPWM调制方式，使其在[0,1]范围内。
    foc->dtc_a   = (1.0f - ta), 0.0f, 1.0f;
    foc->dtc_b   = (1.0f - tb), 0.0f, 1.0f;
    foc->dtc_c   = (1.0f - tc), 0.0f, 1.0f;
}


/**
 * @brief SVPW(矢量合成函数),控制矢量合成函数
 * @param   alpha 反Park变换所得α值
 * @param   beta  反Park变换所得β值
 * @param   foc->dtc_a   A相最终输出电压值
 * @param   foc->dtc_b   B相最终输出电压值
 * @param   foc->dtc_c   C相最终输出电压值
 *
 * @return  是否合成成功  
 */
int Svpwm(FOC_DATA *foc)
{
    int Sextant;

    /* 判断要合成的矢量落在那个扇区 */
    if (foc->Ubeta_norm >= 0.0f)
    {
        if (foc->Ualpha_norm >= 0.0f)
        {
            // quadrant I
            if (ONE_BY_SQRT3 * foc->Ubeta_norm > foc->Ualpha_norm)
                Sextant = 2; // sextant v2-v3
            else
                Sextant = 1; // sextant v1-v2
        }
        else
        {
            // quadrant II
            if (-ONE_BY_SQRT3 * foc->Ubeta_norm > foc->Ualpha_norm)
                Sextant = 3; // sextant v3-v4
            else
                Sextant = 2; // sextant v2-v3
        }
    }
    else
    {
        if (foc->Ualpha_norm >= 0.0f)
        {
            // quadrant IV
            if (-ONE_BY_SQRT3 * foc->Ubeta_norm > foc->Ualpha_norm)
                Sextant = 5; // sextant v5-v6
            else
                Sextant = 6; // sextant v6-v1
        }
        else
        {
            // quadrant III
            if (ONE_BY_SQRT3 * foc->Ubeta_norm > foc->Ualpha_norm)
                Sextant = 4; // sextant v4-v5
            else
                Sextant = 5; // sextant v5-v6
        }
    }

    /* 根据扇区计算所需扇区两边矢量值并合成需要电压矢量 */
    switch (Sextant)
    {
    // sextant v1-v2
    case 1:
    {
        // Vector on-times
        float t1 = foc->Ualpha_norm - ONE_BY_SQRT3 * foc->Ubeta_norm;
        float t2 = TWO_BY_SQRT3 * foc->Ubeta_norm;

        // PWM timings
        foc->dtc_a = (1.0f - t1 - t2) * 0.5f;
        foc->dtc_b = foc->dtc_a + t1;
        foc->dtc_c = foc->dtc_b + t2;
    }
    break;

    // sextant v2-v3
    case 2:
    {
        // Vector on-times
        float t2 = foc->Ualpha_norm + ONE_BY_SQRT3 * foc->Ubeta_norm;
        float t3 = -foc->Ualpha_norm + ONE_BY_SQRT3 * foc->Ubeta_norm;

        // PWM timings
        foc->dtc_b = (1.0f - t2 - t3) * 0.5f;
        foc->dtc_a = foc->dtc_b + t3;
        foc->dtc_c = foc->dtc_a + t2;
    }
    break;

    // sextant v3-v4
    case 3:
    {
        // Vector on-times
        float t3 = TWO_BY_SQRT3 * foc->Ubeta_norm;
        float t4 = -foc->Ualpha_norm - ONE_BY_SQRT3 * foc->Ubeta_norm;

        // PWM timings
        foc->dtc_b = (1.0f - t3 - t4) * 0.5f;
        foc->dtc_c = foc->dtc_b + t3;
        foc->dtc_a = foc->dtc_c + t4;
    }
    break;

    // sextant v4-v5
    case 4:
    {
        // Vector on-times
        float t4 = -foc->Ualpha_norm + ONE_BY_SQRT3 * foc->Ubeta_norm;
        float t5 = -TWO_BY_SQRT3 * foc->Ubeta_norm;

        // PWM timings
        foc->dtc_c = (1.0f - t4 - t5) * 0.5f;
        foc->dtc_b = foc->dtc_c + t5;
        foc->dtc_a = foc->dtc_b + t4;
    }
    break;

    // sextant v5-v6
    case 5:
    {
        // Vector on-times
        float t5 = -foc->Ualpha_norm - ONE_BY_SQRT3 * foc->Ubeta_norm;
        float t6 = foc->Ualpha_norm - ONE_BY_SQRT3 * foc->Ubeta_norm;

        // PWM timings
        foc->dtc_c = (1.0f - t5 - t6) * 0.5f;
        foc->dtc_a = foc->dtc_c + t5;
        foc->dtc_b = foc->dtc_a + t6;
    }
    break;

    // sextant v6-v1
    case 6:
    {
        // Vector on-times
        float t6 = -TWO_BY_SQRT3 * foc->Ubeta_norm;
        float t1 = foc->Ualpha_norm + ONE_BY_SQRT3 * foc->Ubeta_norm;

        // PWM timings
        foc->dtc_a = (1.0f - t6 - t1) * 0.5f;
        foc->dtc_c = foc->dtc_a + t1;
        foc->dtc_b = foc->dtc_c + t6;
    }
    break;
    }

    // 检查结果是否有效（在[0,1]范围内）
    int result_valid =
        foc->dtc_a >= 0.0f && foc->dtc_a <= 1.0f && foc->dtc_b >= 0.0f && foc->dtc_b <= 1.0f && foc->dtc_c >= 0.0f && foc->dtc_c <= 1.0f;

    return result_valid ? 0 : -1;
}