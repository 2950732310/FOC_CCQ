#include "mt6816_encoder.h"
#include <stdbool.h>
#include "general_def.h"
#include "BLDCMotor.h"

/**
 * @brief 编码器数据结构体实例化
 *
 * 定义并初始化全局编码器数据对象，包含 SPI 外设句柄、片选引脚、
 * 电机极对数、方向、角度/速度估计值等所有编码器相关参数。
 * 该结构体供电机控制算法（FOC）调用，获取当前转子位置和速度信息。
 */
ENCODER_DATA encoder_data = {
    .hspi                 = &MT6816_SPI_Get_HSPI, // 指向 SPI 外设的指针
    .CS_Port              = SPI1_CS_GPIO_Port,    // 片选引脚的端口
    .CS_Pin               = SPI1_CS_Pin,          // 片选引脚的引脚号
    .pole_pairs           = MPTOR_P,              // 设置极对数为电机参数
    .encoder_offset       = 0,                    // 设置电角度偏移量
    .dir                  = MOTOR_DIRECTION,      // 设置编码器旋转方向为顺时针
    .raw                  = 0,                    // 初始化原始计数值为0
    .cnt                  = 0,                    // 计数器
    .theta_acc            = 0.01f,                // 设置角度累加步长为0.01弧度
    .count_in_cpr_        = 0,                    // 原始值（整数型）
    .shadow_count_        = 0,                    // 原始值累加计数（整数型）
    .pos_estimate_counts_ = 0.0f,                 // 原始值累加计数（浮点型）
    .vel_estimate_counts_ = 0.0f,                 // 原始值转速
    .pos_cpr_counts_      = 0.0f,                 // 原始值（浮点型）
    .pos_estimate_        = 0.0f,                 // 圈数
    .vel_estimate_        = 0.0f,                 // 加速度 rad/s
    .pos_cpr_             = 0.0f,                 // 0-1
    .phase_               = 0.0f,                 // 电角度
    .interpolation_       = 0.0f,                 // 插值系数
    .elec_angle           = 0.0f,                 // 电角度
    .mec_angle            = 0.0f,                 // 机械角度(弧度)
    .mec_angle_deg        = 0.0f,                 // 机械角度(角度)
    .calib_valid          = 0,                    // 表示校准的数据是否有效

};

/**
 * @brief 将角度归一化到 [0, 2π] 范围内
 *
 * 利用浮点取模运算将任意角度映射到 [0, 2π) 区间。
 * 用于确保电角度和机械角度始终处于标准范围内，避免角度累积溢出。
 *
 * @param angle 要归一化的角度值（弧度）
 * @return float 归一化后的角度值，范围为 [0, 2π)
 */
float normalize_angle(float angle)
{
    float a = fmod(angle, M_2PI);    /* 取余去除整圈部分 */
    return a >= 0 ? a : (a + M_2PI); /* 若为负则加 2π 确保非负 */
}

/**
 * @brief 通过 SPI 向 MT6816 读写一个字节数据
 *
 * 操作流程：拉低片选 → 发送命令/地址字节 → 接收返回数据字节 → 拉高片选。
 * MT6816 为半双工 SPI 从设备，发送和接收需分两次独立调用 HAL 库函数完成。
 *
 * @param txdata 指向待发送数据字节的指针
 * @param rxdata 指向接收数据存放缓冲区的指针
 * @return true  发送和接收均成功（HAL_OK）
 * @return false 发送或接收任一失败
 */
bool MT6816_read_write_byte(uint8_t *txdata, uint8_t *rxdata)
{
    MT6816_SPI_CS_L();                                                          /* 拉低片选，选中 MT6816 */
    bool transmitResult = (HAL_OK == HAL_SPI_Transmit(&MT6816_SPI_Get_HSPI, txdata, 1, MT6816_MAX_DELAY));
    bool receiveResult  = (HAL_OK == HAL_SPI_Receive(&MT6816_SPI_Get_HSPI, rxdata, 1, MT6816_MAX_DELAY));
    MT6816_SPI_CS_H();                                                          /* 拉高片选，释放 SPI 总线 */
    return transmitResult && receiveResult;
}

/**
 * @brief 从 MT6816 编码器读取原始角度数据
 *
 * 通过 SPI 连续读取角度寄存器和警告寄存器各 1 字节，
 * 合并为 16 位原始数据后执行奇偶校验，校验通过后右移 2 位
 * 得到 14 位角度值（MT6816 分辨率为 14 位，即 0~16383）。
 * 通信超时或奇偶校验失败时递增相应错误计数。
 *
 * @param encoder 指向编码器数据结构的指针，读取的角度值存入 encoder->angle
 * @return true  读取成功且奇偶校验通过
 * @return false 通信超时或奇偶校验错误
 */
bool mt6816_read_raw(ENCODER_DATA *encoder)
{
    /* 发送地址：先发角度寄存器地址，再发警告寄存器地址 */
    const uint8_t tx[] = {MT6816_Angle_Reg, MT6816_Warning_Reg};
    uint8_t rx[2];              /* 接收缓冲区，分别存储角度和警告字节 */
    uint8_t h_count;            /* 用于奇偶校验的 1 比特计数 */
    uint16_t rawAngle;          /* 合并后的 16 位原始数据 */

    /* 两次 SPI 通信：第一次获取角度数据，第二次获取警告/校验数据 */
    bool Result_One = MT6816_read_write_byte((uint8_t *)&tx[0], (uint8_t *)&rx[0]);
    bool Result_Two = MT6816_read_write_byte((uint8_t *)&tx[1], (uint8_t *)&rx[1]);

    /* 通信超时处理：任意一次 SPI 通信失败则跳转超时标签 */
    if (!Result_One || !Result_Two)
    {
        goto TIMEOUT;
    }

    /* 通信成功时递减接收错误计数（若有累积） */
    if (encoder->rx_err_count)
    {
        encoder->rx_err_count--;
    }

    /* 将两个字节拼接为 16 位原始数据（大端序：高字节在前） */
    rawAngle = ((rx[0] & 0xFF) << 8) | (rx[1] & 0xFF);

    /*
     * 奇偶校验：统计 rawAngle 16 位中 '1' 的个数。
     * MT6816 使用奇校验，若 '1' 的个数为奇数则校验通过（h_count & 0x01 == 0）。
     * 若为偶数则说明数据传输错误，跳转到校验错误处理。
     */
    h_count = 0;
    for (uint8_t j = 0; j < 16; j++)
    {
        if (rawAngle & (0x01 << j))
        {
            h_count++;
        }
    }
    /* 奇校验要求 1 的个数为奇数，若为偶数则校验失败 */
    if (h_count & 0x01)
    {
        goto CHECK_ERR;
    }

    /*
     * 原始数据格式：[D15:D0]，其中高 14 位 [D15:D2] 为角度值，
     * 低 2 位 [D1:D0] 为校验/状态位，故右移 2 位得到实际角度。
     */
    encoder->angle = rawAngle >> 2;

    /* 奇偶校验成功时递减校验错误计数（若有累积） */
    if (encoder->check_err_count)
    {
        encoder->check_err_count--;
    }

    return true;

/* 奇偶校验错误处理标签 */
CHECK_ERR:
    return false;

/* 通信超时处理标签 */
TIMEOUT:
    MT6816_SPI_CS_H();                  /* 确保片选释放，防止 SPI 总线锁死 */

    /* 递增接收错误计数，上限为 0xFF，用于上层监控通信质量 */
    if (encoder->rx_err_count < 0xFF)
    {
        encoder->rx_err_count++;
    }

    return false;
}

/**
 * @brief 获取电机角度（核心函数）
 *
 * 该函数是编码器数据处理的核心，完成以下流程：
 * 1. 通过 SPI 读取 MT6816 原始角度值
 * 2. 根据旋转方向将原始值映射为正向计数值
 * 3. 利用查找表对磁编码器非线性进行线性化校正
 * 4. 计算编码器计数值增量，更新多圈累积计数
 * 5. 使用二阶锁相环（PLL）对位置和速度进行估计与滤波
 * 6. 执行编码器计数插值（interpolation），提高低速时的角度分辨率
 * 7. 计算机械角度和电角度，供 FOC 控制算法使用
 *
 * @param encoder 指向编码器数据结构的指针，输出更新后的角度/速度值
 */
void GetMotor_Angle(ENCODER_DATA *encoder)
{
    /*
     * PLL（锁相环）系数计算：
     * pll_kp_ — 比例系数，决定位置跟踪带宽
     * pll_ki_ — 积分系数，决定速度估计的收敛速度
     * snap_threshold — 速度零位锁定阈值，低于此值强制归零以防止低速抖动
     */
    static const float pll_kp_ = 2.0f * ENCODER_PLL_BANDWIDTH;
    static const float pll_ki_ = 0.25f * SQ(pll_kp_);
    static const float snap_threshold = 0.5f * CURRENT_MEASURE_PERIOD * pll_ki_;

    /* 第一步：从 MT6816 读取原始角度数据 */
    if (mt6816_read_raw(encoder))
    {
        /*
         * 第二步：根据旋转方向映射计数值
         * 顺时针（CW）：原始值直接使用
         * 逆时针（CCW）：用编码器最大分辨率减去原始值，使逆时针旋转时计数值递减变为递增
         */
        if (encoder->dir == CW)
        {
            encoder->raw = encoder->angle;
        }
        else
        {
            encoder->raw = (ENCODER_CPR - encoder->angle);
        }
    }

    /*
     * 第三步：磁编码器非线性校正（查找表线性化）
     *
     * MT6816 磁编码器因磁场非理想性存在角度非线性误差。
     * 在校准时会生成 128 个点的偏移查找表 offset_lut[128]。
     * 这里将 14 位原始值（0~16383）映射到 128 个区间（每区间 128 个码值），
     * 通过线性插值计算当前角度对应的偏移量，从而修正非线性误差。
     *
     * raw >> 7    : 将 14 位原始值除以 128，得到查找表索引（0~127）
     * ((raw >> 7) + 1) % 128 : 下一个索引，用于线性插值
     * raw - ((raw >> 7) << 7) : 区间内的子码值，用于计算插值权重
     */
    int off_1      = encoder->offset_lut[encoder->raw >> 7];
    int off_2      = encoder->offset_lut[((encoder->raw >> 7) + 1) % 128];
    int off_interp = off_1 + ((off_2 - off_1) * (encoder->raw - ((encoder->raw >> 7) << 7)) >> 7);
    int cnt        = encoder->raw - off_interp;                    /* 校正后的计数值 */
    if (cnt > ENCODER_CPR)
    {
        cnt -= ENCODER_CPR;
    }
    else if (cnt < 0)
    {
        cnt += ENCODER_CPR;
    }
    encoder->cnt   = cnt;                                          /* 保存校正后的单圈计数值 */

    /*
     * 第四步：增量计算与多圈累积
     *
     * delta_enc：当前周期与上一周期之间的编码器计数值变化量。
     * mod() 确保差值在合法范围内，防止过零跳变。
     * 若变化量超过半圈（ENCODER_CPR_DIV），则减去一圈处理绕回。
     */
    int delta_enc  = encoder->cnt - encoder->count_in_cpr_;
    delta_enc      = mod(delta_enc, ENCODER_CPR);
    if (delta_enc > ENCODER_CPR_DIV)
    {
        delta_enc -= ENCODER_CPR;
    }

    /* 更新多圈累积计数值（shadow_count_ 记录总圈数，count_in_cpr_ 记录单圈内位置） */
    encoder->shadow_count_ += delta_enc;
    encoder->count_in_cpr_ += delta_enc;
    encoder->count_in_cpr_  = mod(encoder->count_in_cpr_, ENCODER_CPR);

    /*
     * 第五步：二阶锁相环（PLL）位置/速度估计
     *
     * PLL 通过闭环反馈实时估计转子位置和速度，具有天然的滤波特性，
     * 可以有效抑制编码器量化噪声和测量跳变。
     *
     * 预测阶段：利用上一周期的速度估计值推算当前周期位置
     * 相位检测：计算预测位置与实际测量位置的误差
     * 反馈更新：PI 控制器根据相位误差修正位置和速度估计
     */
    /* 预测当前计数值（基于前一周期的速度） */
    encoder->pos_estimate_counts_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
    encoder->pos_cpr_counts_      += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;

    /* 离散相位检测器：计算预测值与实测值的误差 */
    float delta_pos_counts         = (float)(encoder->shadow_count_ - (int32_t)floor(encoder->pos_estimate_counts_));
    float delta_pos_cpr_counts     = (float)(encoder->count_in_cpr_ - (int32_t)floor(encoder->pos_cpr_counts_));
    delta_pos_cpr_counts           = wrap_pm(delta_pos_cpr_counts, ENCODER_CPR_DIV);

    /* PLL 反馈更新：PI 控制器调节位置和速度估计 */
    encoder->pos_estimate_counts_ += CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_counts;
    encoder->pos_cpr_counts_      += CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_cpr_counts;
    encoder->pos_cpr_counts_       = fmodf_pos(encoder->pos_cpr_counts_, ENCODER_CPR_F);
    encoder->vel_estimate_counts_ += CURRENT_MEASURE_PERIOD * pll_ki_ * delta_pos_cpr_counts;

    /* 速度零位锁定：当速度估计值小于阈值时强制归零，防止零速附近抖动 */
    bool snap_to_zero_vel          = false;
    if (ABS(encoder->vel_estimate_counts_) < snap_threshold)
    {
        encoder->vel_estimate_counts_ = 0.0f;    /* 强制归零，消除 Δ-Σ 调制抖动 */
        snap_to_zero_vel = true;
    }

    /*
     * 第六步：将 PLL 输出转换为控制器可直接使用的物理量
     * pos_estimate_ — 转子位置（圈数，浮点）
     * vel_estimate_ — 转子速度（圈/秒）
     * pos_cpr_      — 单圈内位置归一化值（0~1）
     */
    float pos_cpr_last     = encoder->pos_cpr_;
    encoder->pos_estimate_ = encoder->pos_estimate_counts_ / ENCODER_CPR_F;
    encoder->vel_estimate_ = encoder->vel_estimate_counts_ / ENCODER_CPR_F;
    encoder->pos_cpr_      = encoder->pos_cpr_counts_ / ENCODER_CPR_F;
    float delta_pos_cpr    = wrap_pm(encoder->pos_cpr_ - pos_cpr_last, 0.5f);

    /*
     * 第七步：编码器计数插值
     *
     * 当编码器计数值在两个采样周期之间未变化时（delta_enc == 0），
     * 利用 PLL 估计的速度对位置进行帧间插值，提高低速时的角度分辨率。
     * 当编码器有新的跳变时，直接重置插值系数。
     * 当速度为零锁定时，插值系数固定为 0.5 以防止随机漂移。
     */
    int32_t corrected_enc  = encoder->count_in_cpr_ - encoder->encoder_offset;
    if (snap_to_zero_vel)
    {
        /* 零速锁定：插值系数固定为 0.5，防止位置随机漂移 */
        encoder->interpolation_ = 0.5f;
    }
    else if (delta_enc > 0)
    {
        /* 正方向跳变：插值重置到区间起点 */
        encoder->interpolation_ = 0.0f;
    }
    else if (delta_enc < 0)
    {
        /* 反方向跳变：插值重置到区间终点 */
        encoder->interpolation_ = 1.0f;
    }
    else
    {
        /*
         * 计数值未变化：利用速度估计值进行帧间预测插值，
         * 将插值系数限制在 [0, 1] 范围内，确保插值位置不超出当前编码器区间
         */
        encoder->interpolation_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
        if (encoder->interpolation_ > 1.0f)
            encoder->interpolation_ = 1.0f;
        if (encoder->interpolation_ < 0.0f)
            encoder->interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + encoder->interpolation_;

    /*
     * 第八步：计算机械角度与电角度
     *
     * elec_rad_per_enc — 每个编码器计数对应的电角度弧度值
     * phase_           — 电角度（范围 -π ~ π），用于 FOC 电流环 Park/逆Park变换
     * mec_angle        — 机械角度（弧度，范围 0 ~ 2π）
     * elec_angle       — 电角度（弧度，范围 0 ~ 2π）= 极对数 × 机械角度
     * mec_angle_deg    — 机械角度（度），供调试和上位机显示使用
     */
    float elec_rad_per_enc = encoder->pole_pairs * M_2PI * (1.0f / ENCODER_CPR_F);
    float ph               = elec_rad_per_enc * interpolated_enc;
    encoder->phase_        = wrap_pm_pi(ph);                              /* 电角度（-π ~ π），用于 Park 变换 */
    encoder->mec_angle     = normalize_angle(corrected_enc * (360.0f / ENCODER_CPR_F) * (M_PI / 180.0f)); /* 机械角度（弧度） */
    encoder->elec_angle    = normalize_angle(encoder->pole_pairs * encoder->mec_angle);                   /* 电角度 = 极对数 × 机械角度 */
    encoder->mec_angle_deg = encoder->mec_angle * 180.0f / M_PI;                                         /* 机械角度（度） */
}

/**
 * @brief 电角度增量累加函数
 *
 * 在编码器未就绪或开环控制模式下，按设定的步长 theta_acc 周期性累加电角度，
 * 模拟转子旋转，使电机能够以开环方式旋转（如拖动启动、对齐定位等场景）。
 * 角度超过 [0, 2π] 范围时自动回绕。
 *
 * @param encoder 指向编码器数据结构的指针，更新 encoder->elec_angle
 */
void Theta_ADD(ENCODER_DATA *encoder)
{
    /* 按固定步长累加电角度（步长由 theta_acc 决定，正值为正转，负值为反转） */
    encoder->elec_angle += encoder->theta_acc;

    /* 将电角度约束在 [0, 2π] 范围内，防止溢出 */
    if (encoder->elec_angle > M_2PI)
        encoder->elec_angle = 0.0f;
    else if (encoder->elec_angle < 0)
        encoder->elec_angle = M_2PI;
}
