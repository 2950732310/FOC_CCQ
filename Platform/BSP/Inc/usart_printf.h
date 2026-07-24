#ifndef USART_PRINTF_H
#define USART_PRINTF_H

#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/*======================= DMA发送接口 =======================*/

/**
 * @brief  通过DMA发送格式化字符串（printf风格）
 * @param  fmt: 格式化字符串
 * @retval 发送成功返回0，失败返回-1，-2等错误码
 */
int UART_Printf_DMA(const char *fmt, ...);

/**
 * @brief  通过DMA发送原始数据
 * @param  data: 数据缓冲区
 * @param  len:  数据长度
 * @retval None
 */
void UART_SendData_DMA(const uint8_t *data, uint16_t len);

/*===================== 串口协议解析 =======================*/

#define PROTOCOL_RX_BUF_SIZE    128      /* 接收行缓冲区大小 */
#define PROTOCOL_MAX_ARGS       8        /* 命令最大参数个数 */
#define RX_DMA_BUF_SIZE         256      /* DMA循环接收缓冲区大小 */

/** 
 * @brief  解析后的命令结构
 */
typedef struct
{
    char cmd[16];                        /* 命令类型: GET/SET/INFO/HELP */
    char args[PROTOCOL_MAX_ARGS][32];    /* 参数字符串数组 */
    int  argc;                           /* 参数个数 */
} ProtocolCmd_t;

/**
 * @brief  协议命令处理回调函数类型
 * @param  cmd: 解析后的命令结构体指针
 * @retval None
 */
typedef void (*ProtocolCallback_t)(const ProtocolCmd_t *cmd);

/**
 * @brief  启动DMA循环接收
 * @note  DMA CIRCULAR模式 + IDLE中断，自动连续接收
 */
void UART_StartRX_DMA(void);

/**
 * @brief  USART IDLE中断处理函数（由stm32g4xx_it.c调用）
 * @note  在中断中调用，标记新数据到达
 */
void UART_OnIdle(void);

/**
 * @brief  注册协议命令处理回调
 * @param  callback: 回调函数指针
 * @retval None
 */
void UART_RegisterProtocolCallback(ProtocolCallback_t callback);

/**
 * @brief  通过DMA发送协议响应（自动添加\r\n）
 * @param  fmt: 格式化字符串
 * @retval None
 */
void UART_Response_DMA(const char *fmt, ...);

/**
 * @brief  串口协议处理任务（在RTOS线程中周期调用）
 * @note  处理DMA接收到的数据，提取完整行并解析
 */
void UART_ProtocolProcess(void);

#endif
