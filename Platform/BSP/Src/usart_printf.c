#include "usart_printf.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>

/*======================== DMA发送部分 =======================*/

/* 打印缓冲区大小 */
#define PRINTF_BUF_SIZE    256

/* DMA发送忙标志 */
static volatile uint8_t dma_tx_busy = 0;

/**
 * @brief  UART DMA发送完成回调
 * @note   当DMA传输完成时由HAL库调用，清除忙标志
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        dma_tx_busy = 0;
    }
}

/**
 * @brief  等待上一次DMA传输完成
 */
static void UART_WaitDMAReady(void)
{
    while (dma_tx_busy)
    {
        __NOP();
    }
}

/**
 * @brief  通过DMA发送原始数据（非阻塞）
 * @param  data: 数据缓冲区指针
 * @param  len:  要发送的数据长度
 * @retval None
 */
void UART_SendData_DMA(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return;

    UART_WaitDMAReady();

    dma_tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, len);
}

/**
 * @brief  通过DMA发送格式化字符串（printf风格）
 * @param  fmt: 格式化字符串
 * @retval None
 *
 * @note   支持标准的printf格式化方式，内部使用vsnprintf格式化到缓冲区
 *         然后通过DMA发送，最大支持PRINTF_BUF_SIZE-1字节
 */
int UART_Printf_DMA(const char *fmt, ...)
{
    if(dma_tx_busy)
    {
        return -1;   // 直接丢弃
    }

    static char buf[PRINTF_BUF_SIZE];
    va_list args;
    int len;

    /* 【必须先等待】前一次DMA传输完成，再覆写static缓冲区 */
    // UART_WaitDMAReady();

    va_start(args, fmt);
    len = vsnprintf(buf, PRINTF_BUF_SIZE, fmt, args);
    va_end(args);

    if (len > 0)
    {
        /* 限制最大发送长度 */
        if (len >= PRINTF_BUF_SIZE)
        {
            len = PRINTF_BUF_SIZE - 1;
        }
        UART_SendData_DMA((const uint8_t *)buf, (uint16_t)len);
    }
    return 0;
}

/*====================== 串口协议解析 =======================*/

/* --- DMA循环接收 --- */
static uint8_t  dma_rx_buf[RX_DMA_BUF_SIZE];           /* DMA循环缓冲区       */
static volatile uint16_t dma_rx_last_pos = 0;           /* 上次处理到的位置    */
static volatile uint8_t  dma_rx_pending = 0;            /* 有待处理的新数据    */

/* --- 行解析 --- */
static char     rx_line[PROTOCOL_RX_BUF_SIZE];          /* 当前拼装的行       */
static uint16_t rx_line_len = 0;                        /* 行长度             */

/* --- 协议回调 --- */
static ProtocolCallback_t protocol_callback = NULL;

/**
 * @brief  启动DMA循环接收
 * @note   CIRCULAR模式 + IDLE中断，自动连续接收
 */
void UART_StartRX_DMA(void)
{
    dma_rx_last_pos = 0;        // 初始化上次处理位置
    dma_rx_pending = 0;         // 初始化待处理标志
    rx_line_len = 0;            // 初始化行长度

    /* 启动DMA循环接收 */
    HAL_UART_Receive_DMA(&huart1, dma_rx_buf, RX_DMA_BUF_SIZE);

    /* 使能IDLE中断 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/**
 * @brief  USART IDLE中断处理函数（由stm32g4xx_it.c调用）
 * @note   在中断上下文中调用，仅标记数据到达
 */
void UART_OnIdle(void)
{
    dma_rx_pending = 1;
}

/**
 * @brief  注册协议命令处理回调
 * @param  callback: 回调函数指针
 */
void UART_RegisterProtocolCallback(ProtocolCallback_t callback)
{
    protocol_callback = callback;
}

/**
 * @brief  通过DMA发送协议响应（自动添加\r\n）
 * @param  fmt: 格式化字符串
 */
void UART_Response_DMA(const char *fmt, ...)
{
    static char buf[PRINTF_BUF_SIZE];
    va_list args;
    int len;

    /* 【必须先等待】前一次DMA传输完成，再覆写static缓冲区 */
    UART_WaitDMAReady();

    va_start(args, fmt);
    len = vsnprintf(buf, PRINTF_BUF_SIZE - 2, fmt, args);
    va_end(args);

    if (len > 0)
    {
        /* 追加\r\n */
        buf[len]     = '\r';
        buf[len + 1] = '\n';
        len += 2;
        UART_SendData_DMA((const uint8_t *)buf, (uint16_t)len);
    }
}

/**
 * @brief  解析一行协议数据
 * @param  line: 输入字符串 (格式: $CMD,arg1,arg2,...)
 * @param  cmd:  输出解析结果
 * @retval 0: 成功, -1: 格式错误
 *
 * @note   协议格式: $CMD,param,value
 *         示例:    $GET,speed
 *                  $SET,speed,1500
 *                  $INFO
 *                  $HELP
 */
static int Protocol_ParseLine(const char *line, ProtocolCmd_t *cmd)
{
    char temp[PROTOCOL_RX_BUF_SIZE];
    char *token;
    int i;

    if (line == NULL || cmd == NULL)
        return -1;

    /* 必须以 $ 开头 */
    if (line[0] != '$')
        return -1;

    /* 拷贝到临时缓冲区（strtok会修改原字符串） */
    strncpy(temp, line + 1, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    /* 初始化 */
    memset(cmd, 0, sizeof(ProtocolCmd_t));

    /* 用逗号分割 */
    token = strtok(temp, ",");
    if (token == NULL)
        return -1;

    /* 第一个token是命令 */
    strncpy(cmd->cmd, token, sizeof(cmd->cmd) - 1);
    cmd->cmd[sizeof(cmd->cmd) - 1] = '\0';

    /* 后续是参数 */
    i = 0;
    while ((token = strtok(NULL, ",")) != NULL && i < PROTOCOL_MAX_ARGS)
    {
        strncpy(cmd->args[i], token, sizeof(cmd->args[i]) - 1);
        cmd->args[i][sizeof(cmd->args[i]) - 1] = '\0';
        i++;
    }
    cmd->argc = i;

    return 0;
}

/**
 * @brief  从DMA循环缓冲区读取新数据，提取完整行并处理
 * @note   在任务上下文中调用（非中断）
 */
void UART_ProtocolProcess(void)
{
    uint16_t current_pos;                   // 当前DMA写入位置
    uint16_t new_bytes;                     // 新接收的字节数
    uint8_t  temp_buf[RX_DMA_BUF_SIZE];     // 临时缓冲区
    uint16_t i;                         

    if (!dma_rx_pending)
        return;
    dma_rx_pending = 0;

    /* 获取DMA当前写入位置 */
    current_pos = RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    /* 计算新接收的字节数 */
    if (current_pos >= dma_rx_last_pos)
        new_bytes = current_pos - dma_rx_last_pos;
    else
        new_bytes = (RX_DMA_BUF_SIZE - dma_rx_last_pos) + current_pos;

    if (new_bytes == 0 || new_bytes >= RX_DMA_BUF_SIZE / 2)
    {
        dma_rx_last_pos = current_pos;
        return;
    }

    /* 将新数据从环形缓冲区拷贝到临时缓冲 */
    for (i = 0; i < new_bytes; i++)
    {
        temp_buf[i] = dma_rx_buf[(dma_rx_last_pos + i) % RX_DMA_BUF_SIZE];
    }
    dma_rx_last_pos = current_pos;

    /* 逐字节解析，提取完整行 */
    for (i = 0; i < new_bytes; i++)
    {
        char ch = (char)temp_buf[i];

        if (ch == '\r' || ch == '\n')
        {
            /* 行结束：如果积累了内容则处理 */
            if (rx_line_len > 0)
            {
                rx_line[rx_line_len] = '\0';

                /* 解析并执行命令 */
                ProtocolCmd_t cmd;
                if (Protocol_ParseLine(rx_line, &cmd) == 0)
                {
                    if (protocol_callback != NULL)
                        protocol_callback(&cmd);
                }
                else
                {
                    UART_Response_DMA("$ERR,invalid format");
                }

                rx_line_len = 0;
            }
        }
        else
        {
            /* 普通字符：追加到行缓冲区 */
            if (rx_line_len < PROTOCOL_RX_BUF_SIZE - 1)
            {
                rx_line[rx_line_len++] = ch;
            }
        }
    }
}