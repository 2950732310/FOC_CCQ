#ifndef LED_H
#define LED_H

#include "tim.h"

/*这里是计算所得CCR的宏定义*/
#define CODE_1 (140) 		// 1码定时器计数次数
#define CODE_0 (70)  		// 0码定时器计数次数

#define LED_MAX_NUM 1 		// LED数量宏定义，这里使用一个LED

/*建立一个定义单个LED三原色值大小的结构体*/
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color_TypeDef;

/* RGB颜色枚举 */
enum RGB_COLOUR{
	RED_F = 0,			// 红色
	GREEN_F,			// 绿色
	BLUE_F,				// 蓝色
	SKY_F,				// 天蓝色
	MAGENTA_F,			// 品红色
	YELLOW_F,			// 黄色
	ORANGE_F,			// 橙色
	BLACK_F,			// 黑色
	WHITE_F,			// 白色
	PURPLE_F,			// 紫色
	BROWN_F,			// 棕色
	GRAY_F,				// 灰色
	PINK_F,				// 粉色
	GOLD_F,				// 金色
	SILVER_F,			// 银色
};

void RGB_SetColor(uint8_t LedId, RGB_Color_TypeDef Color); // 给一个LED装载24个颜色数据码（0码和1码）
void RGB_SetBrightness(uint8_t brightness);                 // 设置亮度，取值 0~255
void RGB_ToggleGreen(void);                                 // 亮灭翻转，亮绿色
void Reset_Load(void);                                     // 该函数用于将数组最后24个数据变为0，代表RESET_code
void RGB_SendArray(void);                                  // 发送LED数据
void RGB_DisplayColor(RGB_Color_TypeDef color);            // 显示指定颜色
void RGB_DisplayColorById(uint8_t color_id);               // 根据颜色编号显示颜色
void RGB_DMA_CompleteCallback(void);                       // DMA传输完成后的回调函数

#endif
