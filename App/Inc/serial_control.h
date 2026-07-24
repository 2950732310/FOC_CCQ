#ifndef SERIAL_THREAD_H
#define SERIAL_THREAD_H

#include "main.h"
#include <stdint.h>
#include "usart_printf.h"

/*=================== 参数类型枚举 ===================*/
typedef enum
{
    PARAM_TYPE_FLOAT,       /* float 类型参数 */
    PARAM_TYPE_INT,         /* int 类型参数   */
    PARAM_TYPE_UINT32,      /* uint32_t 类型  */
} ParamType_t;

/*=================== 参数表条目 ===================*/
typedef struct
{
    const char  *name;      /* 参数名（用于协议匹配） */
    ParamType_t  type;      /* 参数类型              */
    void        *var;       /* 参数变量指针          */
    const char  *unit;      /* 单位（显示用）        */
    const char  *desc;      /* 描述信息              */
} ParamDef_t;

/*=================== 命令交互结构体 ===================*/
typedef struct
{
    uint8_t data_received;          //数据接收完成标志位
    uint8_t current_zero;           //电流环零点标定标志位
    uint8_t reset;                  //恢复出厂设置标志位
    uint8_t flash_control;          //Flash控制标志位
    uint8_t state_mode;             //状态模式标志位
} CommandFlag_t;

extern CommandFlag_t command_flag;
void Protocol_CommandHandler(const ProtocolCmd_t *cmd);

#endif
