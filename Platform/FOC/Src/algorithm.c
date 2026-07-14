#include "algorithm.h"


/*========================一阶低通滤波算法===========================*/

/**
 * @brief 一阶低通滤波器初始化
 * @param filter 滤波器结构体指针
 * @param alpha  滤波系数
 */
void LPF_Init(LPF_Filter_t *filter, float alpha) {
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    filter->alpha = alpha;
    filter->out_last = 0.0f;
}

/**
 * @brief 一阶低通滤波器计算
 * @param filter filter 滤波器结构体指针
 * @param input  当前采样的原始值 (如 Park 变换后的 Id 或 Iq)
 * @return float 滤波后的输出值
 */
float LPF_Calc(LPF_Filter_t *filter, float input) {
    // 公式: Y(n) = alpha * X(n) + (1 - alpha) * Y(n-1)
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->out_last;
    
    // 更新历史状态
    filter->out_last = output;
    
    return output;
}