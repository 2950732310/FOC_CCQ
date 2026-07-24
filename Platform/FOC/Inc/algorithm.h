#ifndef ALGORITHM_H
#define ALGORITHM_H
#include "main.h"

typedef struct {
    float alpha;     // 滤波系数 (0.0 < alpha <= 1.0)
    float out_last;  // 上一次的滤波输出值
} LPF_Filter_t;



void LPF_Init(LPF_Filter_t *filter, float alpha);
float LPF_Calc(LPF_Filter_t *filter, float input);

#endif // !ALGORITHM_H