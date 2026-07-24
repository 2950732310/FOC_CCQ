#ifndef FDCAN_CONTROL_H
#define FDCAN_CONTROL_H

#include "main.h"
#include "motor_control.h"

/* ======================================================================= */
/*                      CAN FD 应用层协议定义 — 多节点支持                   */
/*                                                                         */
/*  每个设备有唯一 node_id (1~15)，0 表示广播。                             */
/*  CAN ID 计算公式 (标准帧 11-bit)：                                       */
/*     Command  (主机→设备):  (0x100 | node_id)   范围 0x100~0x10F         */
/*     Response (设备→主机):  (0x140 | node_id)   范围 0x140~0x14F         */
/*     Report   (设备→主机):  (0x180 | node_id)   范围 0x180~0x18F         */
/*                                                                         */
/*  示例 (node_id=1):  CMD=0x101  RSP=0x141  RPT=0x181                    */
/*  示例 (node_id=3):  CMD=0x103  RSP=0x143  RPT=0x183                    */
/*  广播 (node_id=0):  CMD=0x100（所有节点处理）                           */
/* ======================================================================= */

/* ---------------------------- CAN ID 基址 ------------------------------ */
#define CANID_CMD_BASE      0x100   /* 命令基址：CANID_CMD_BASE | node_id */
#define CANID_RSP_BASE      0x140   /* 应答基址：CANID_RSP_BASE | node_id */
#define CANID_RPT_BASE      0x180   /* 上报基址：CANID_RPT_BASE | node_id */
#define CANID_BROADCAST     0x100   /* 广播 ID（node_id=0 时等价）        */

/* ---------------------------- 节点 ID 相关函数 -------------------------- */
void     FDCAN_SetNodeId(uint8_t nodeId);
uint8_t  FDCAN_GetNodeId(void);
uint32_t FDCAN_GetCmdId(void);      /* 返回本节点的命令接收 ID           */
uint32_t FDCAN_GetRspId(void);      /* 返回本节点的应答发送 ID           */
uint32_t FDCAN_GetRptId(void);      /* 返回本节点的上报发送 ID           */

/* ---------------------------- 命令类型 --------------------------------- */
typedef enum {
    CMD_QUERY       = 0x01,       /* 查询单个参数                       */
    CMD_WRITE       = 0x02,       /* 写入单个参数                       */
    CMD_REPORT      = 0x03,       /* 请求设备立即上报批量数据            */
    CMD_SET_NODE_ID = 0xFE,       /* 设置节点 ID（仅广播下发）           */
} FDCAN_CmdType;

/* ---------------------------- 参数 ID 枚举 ---------------------------- */
typedef enum {
    /* 节点参数 */
    PARAM_NODE_ID           = 0x00,   /* uint8  - 节点 ID              */

    /* 状态参数 */
    PARAM_STATE_MODE        = 0x01,   /* uint8  - 运行模式              */
    PARAM_CTRL_MODE         = 0x02,   /* uint8  - 控制模式              */

    /* 电流参数 */
    PARAM_IQ_FB             = 0x10,   /* float  - Q轴电流反馈           */
    PARAM_ID_FB             = 0x11,   /* float  - D轴电流反馈           */
    PARAM_IQ_SET            = 0x12,   /* float  - Q轴电流设定           */
    PARAM_ID_SET            = 0x13,   /* float  - D轴电流设定           */

    /* 电压参数 */
    PARAM_VQ                = 0x20,   /* float  - Q轴电压               */
    PARAM_VD                = 0x21,   /* float  - D轴电压               */
    PARAM_VBUS              = 0x22,   /* float  - 母线电压              */

    /* 速度 / 位置参数 */
    PARAM_VEL_FB            = 0x30,   /* float  - 速度反馈 (rad/s)      */
    PARAM_VEL_SET           = 0x31,   /* float  - 速度设定 (rad/s)      */
    PARAM_POS_FB            = 0x32,   /* float  - 位置反馈 (°)          */
    PARAM_POS_SET           = 0x33,   /* float  - 位置设定 (°)          */

    /* PID 参数 */
    PARAM_IQ_KP             = 0x40,   /* float  - 电流环 Q 轴 Kp        */
    PARAM_IQ_KI             = 0x41,   /* float  - 电流环 Q 轴 Ki        */
    PARAM_ID_KP             = 0x42,   /* float  - 电流环 D 轴 Kp        */
    PARAM_ID_KI             = 0x43,   /* float  - 电流环 D 轴 Ki        */
    PARAM_VEL_KP            = 0x44,   /* float  - 速度环 Kp             */
    PARAM_VEL_KI            = 0x45,   /* float  - 速度环 Ki             */
    PARAM_POS_KP            = 0x46,   /* float  - 位置环 Kp             */
    PARAM_POS_KI            = 0x47,   /* float  - 位置环 Ki             */

    /* 编码器参数 */
    PARAM_ENCODER_ANGLE     = 0x50,   /* float  - 编码器机械角度 (rad)  */
    PARAM_ENCODER_SPEED     = 0x51,   /* float  - 编码器速度 (rad/s)    */
    PARAM_ELECT_ANGLE       = 0x52,   /* float  - 电角度 (rad)          */

    /* 系统参数 */
    PARAM_THETA             = 0x60,   /* float  - FOC 电角度 theta      */
} FDCAN_ParamID;

/* ---------------------------- 应答状态 --------------------------------- */
typedef enum {
    RSP_OK      = 0x00,     /* 成功                                    */
    RSP_ERR_ID  = 0x01,     /* 无效的参数 ID                            */
    RSP_ERR_BUSY= 0x02,     /* 设备忙                                  */
} FDCAN_RspStatus;

/* ======================================================================= */
/*                            API 函数声明                                 */
/* ======================================================================= */

/* ---------- 初始化 ---------- */
HAL_StatusTypeDef FDCAN_Init(void);

/* ---------- 通用发送（自动选择经典 CAN / CAN FD） ---------- */
HAL_StatusTypeDef FDCAN_Send(uint32_t id, const uint8_t *pData, uint8_t len);

/* ---------- 协议函数 ---------- */

/**
 * @brief  单参数查询应答（经典 CAN 8字节）
 * @note   CAN ID = CANID_RSP_BASE | node_id
 * @param  paramId   参数 ID
 * @param  value     参数字节数据（4字节，float 或 uint32 小端序）
 * @param  status    应答状态
 * @retval HAL_OK    发送成功
 */
HAL_StatusTypeDef FDCAN_SendParamRsp(uint8_t paramId, const uint8_t *value, FDCAN_RspStatus status);

/**
 * @brief  批量数据上报（CAN FD 64字节）
 * @note   CAN ID = CANID_RPT_BASE | node_id
 *         将电机所有关键参数打包到 64 字节中上报
 * @retval HAL_OK    发送成功
 */
HAL_StatusTypeDef FDCAN_SendBulkReport(void);

/**
 * @brief  处理收到的主机命令帧
 * @param  pData    收到的 8 字节数据
 * @param  cmdId    命令 CAN ID（用于判断是否发给本节点）
 * @note   内部解析命令类型和参数 ID，执行相应操作并自动应答
 */
void FDCAN_ProcessCmd(const uint8_t *pData, uint32_t cmdId);

/**
 * @brief  CAN 接收回调（由 HAL 在中断中调用）
 * @param  pData    收到的数据
 * @param  len      数据长度
 * @param  rxId     收到的 CAN ID
 * @note   在 FDCAN 的 RxFifo0Callback 中调用
 */
void FDCAN_OnReceive(const uint8_t *pData, uint8_t len, uint32_t rxId);


HAL_StatusTypeDef FDCAN_UpdateFilter(void);

#endif /* FDCAN_CONTROL_H */
