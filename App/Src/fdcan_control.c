#include "fdcan_control.h"

/* 外部 FDCAN 句柄（由 CubeMX 在 fdcan.c 中定义） */
extern FDCAN_HandleTypeDef hfdcan1;

/* 外部电机数据 */
extern MOTOR_DATA motor;

/* 本设备节点 ID（默认 1，可通过 CAN 命令或代码修改） */
static uint8_t fdcan_node_id = 1;



/* 在 HAL_FDCAN_RxFifo0Callback 中 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[64];
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData);
        FDCAN_OnReceive(rxData, rxHeader.DataLength, rxHeader.Identifier);
    }
}


/* ======================================================================= */
/*                           节点 ID 管理                                    */
/* ======================================================================= */

void FDCAN_SetNodeId(uint8_t nodeId)
{
    if (nodeId > 0x0F) nodeId = 0x0F;  /* 限制 0~15 */
    fdcan_node_id = nodeId;

    /* 立即更新硬件过滤器，只接收新节点 ID 对应的命令 */
    FDCAN_UpdateFilter();
}

uint8_t FDCAN_GetNodeId(void)
{
    return fdcan_node_id;
}

uint32_t FDCAN_GetCmdId(void)
{
    return CANID_CMD_BASE | fdcan_node_id;
}

uint32_t FDCAN_GetRspId(void)
{
    return CANID_RSP_BASE | fdcan_node_id;
}

uint32_t FDCAN_GetRptId(void)
{
    return CANID_RPT_BASE | fdcan_node_id;
}

/* ======================================================================= */
/*                           内部辅助函数                                    */
/* ======================================================================= */

/**
 * @brief  等待 Tx FIFO 有空闲槽位
 * @param  timeoutMs  超时时间（毫秒），0 表示只查一次
 * @retval HAL_OK     FIFO 有空位
 * @retval HAL_TIMEOUT  超时
 */
static HAL_StatusTypeDef FDCAN_WaitTxFifoFree(uint32_t timeoutMs)
{
    uint32_t tickStart = HAL_GetTick();

    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
    {
        if ((timeoutMs > 0) && ((HAL_GetTick() - tickStart) >= timeoutMs))
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

/**
 * @brief  将 float 按小端序写入缓冲区（避免 memcpy 依赖）
 */
static void float_to_bytes(float val, uint8_t *buf)
{
    uint8_t *p = (uint8_t *)&val;
    buf[0] = p[0];
    buf[1] = p[1];
    buf[2] = p[2];
    buf[3] = p[3];
}

/**
 * @brief  从缓冲区按小端序读取 float（避免 memcpy 依赖）
 */
static float bytes_to_float(const uint8_t *buf)
{
    float val;
    uint8_t *p = (uint8_t *)&val;
    p[0] = buf[0];
    p[1] = buf[1];
    p[2] = buf[2];
    p[3] = buf[3];
    return val;
}

/* ======================================================================= */
/*                           过滤器 & 初始化                                 */
/* ======================================================================= */

/**
 * @brief  重新配置硬件过滤器（根据当前 node_id）
 * @note   使用 DUAL 模式，只接收：
 *         - 广播 ID (0x100)
 *         - 本节点命令 ID (0x100 | node_id)
 *         其他 CAN 帧不会触发接收中断，完全由硬件过滤
 */
HAL_StatusTypeDef FDCAN_UpdateFilter(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType         = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex    = 0;
    sFilterConfig.FilterType     = FDCAN_FILTER_DUAL;          /* 双 ID 匹配 */
    sFilterConfig.FilterConfig   = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1      = CANID_BROADCAST;             /* 广播 ID */
    sFilterConfig.FilterID2      = FDCAN_GetCmdId();            /* 本节点命令 ID */

    return HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
}

/**
 * @brief  配置硬件过滤器并启动 FDCAN1
 * @note   必须在 MX_FDCAN1_Init() 之后调用
 *         过滤器使用 DUAL 模式，只接收广播和本节点命令，
 *         其他 CAN 帧硬件直接丢弃，不会进中断
 */
HAL_StatusTypeDef FDCAN_Init(void)
{
    /* 配置硬件过滤器：只收广播 + 本节点命令 */
    if (FDCAN_UpdateFilter() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 启动 FDCAN */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ======================================================================= */
/*                           通用发送（自动选择格式）                          */
/* ======================================================================= */

/**
 * @brief  通用 CAN 帧发送
 * @note   len ≤ 8   → 经典 CAN 格式
 *         len > 8   → CAN FD 格式（不带 BRS）
 *         发送前会等待 FIFO 有空位（超时 100ms）
 */
HAL_StatusTypeDef FDCAN_Send(uint32_t id, const uint8_t *pData, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[64] = {0};
    uint8_t dlc;
    uint8_t copyLen;

    /* 限制最大 64 字节 */
    if (len > 64) len = 64;

    /* 将 len 映射为 DLC 编码 */
    if (len <= 8)
    {
        dlc = len;                              /* 0~8 直接映射 */
    }
    else if (len <= 12)    dlc = 9;             /* FDCAN_DLC_BYTES_12  */
    else if (len <= 16)    dlc = 10;            /* FDCAN_DLC_BYTES_16  */
    else if (len <= 20)    dlc = 11;            /* FDCAN_DLC_BYTES_20  */
    else if (len <= 24)    dlc = 12;            /* FDCAN_DLC_BYTES_24  */
    else if (len <= 32)    dlc = 13;            /* FDCAN_DLC_BYTES_32  */
    else if (len <= 48)    dlc = 14;            /* FDCAN_DLC_BYTES_48  */
    else                   dlc = 15;            /* FDCAN_DLC_BYTES_64  */

    /* 实际需拷贝的字节数（DLC 对应的实际字节数） */
    if (dlc <= 8)
        copyLen = dlc;
    else if (dlc == 9)  copyLen = 12;
    else if (dlc == 10) copyLen = 16;
    else if (dlc == 11) copyLen = 20;
    else if (dlc == 12) copyLen = 24;
    else if (dlc == 13) copyLen = 32;
    else if (dlc == 14) copyLen = 48;
    else                copyLen = 64;

    /* 拷贝待发送数据 */
    for (uint8_t i = 0; i < len; i++)
        txData[i] = pData[i];

    /* 填充发送头 */
    txHeader.Identifier          = id;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = dlc;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    txHeader.FDFormat            = (len <= 8) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    /* 等待 FIFO 有空位 */
    if (FDCAN_WaitTxFifoFree(100) != HAL_OK)
        return HAL_BUSY;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
}

/* ======================================================================= */
/*                  单参数查询应答 (ID=0x101, 经典 CAN 8字节)               */
/* ======================================================================= */

/**
 * @brief  单参数查询应答
 * @note   帧格式 (8字节)：
 *         [0]       = 参数 ID
 *         [1]       = 应答状态 (RspStatus)
 *         [2..5]    = 参数值 (float / uint32, 小端序)
 *         [6..7]    = 节点 ID（供主机识别来源）
 *         CAN ID = CANID_RSP_BASE | node_id
 */
HAL_StatusTypeDef FDCAN_SendParamRsp(uint8_t paramId, const uint8_t *value, FDCAN_RspStatus status)
{
    uint8_t data[8] = {0};

    data[0] = paramId;
    data[1] = (uint8_t)status;

    /* 拷贝 4 字节参数值 */
    if (value != NULL)
    {
        for (uint8_t i = 0; i < 4; i++)
            data[2 + i] = value[i];
    }

    /* 填入节点 ID */
    data[6] = fdcan_node_id;

    return FDCAN_Send(FDCAN_GetRspId(), data, 8);
}

/* ======================================================================= */
/*                  批量数据上报 (ID=0x102, CAN FD 64字节)                  */
/* ======================================================================= */

/**
 * @brief  批量上报所有关键电机参数
 * @note   64 字节布局：
 * 偏移   大小  说明
 * -------------------------------------------
 *   0      1   协议版本 (0x01)
 *   1      1   帧类型 (0x01 = 周期上报)
 *   2      1   节点 ID
 *   3      1   State_Mode
 *   4      1   Control_Mode
 *   5      3   预留
 *   8      4   母线电压 vbus (float)
 *  12      4   i_q 反馈 (float)
 *  16      4   i_d 反馈 (float)
 *  20      4   iq_set (float)
 *  24      4   id_set (float)
 *  28      4   v_q (float)
 *  32      4   v_d (float)
 *  36      4   速度反馈 vel_fb (float)
 *  40      4   速度设定 vel_set (float)
 *  44      4   位置反馈 pos_fb (float)
 *  48      4   位置设定 pos_set (float)
 *  52      4   电角度 theta (float)
 *  56      4   编码器机械角度 (float)
 *  60      4   编码器速度 (float)
 *  CAN ID = CANID_RPT_BASE | node_id
 */
HAL_StatusTypeDef FDCAN_SendBulkReport(void)
{
    uint8_t data[64] = {0};

    /* 帧头 */
    data[0] = 0x01;                              /* 协议版本 */
    data[1] = 0x01;                              /* 帧类型：周期上报 */
    data[2] = fdcan_node_id;                     /* 节点 ID */

    /* 状态 */
    data[3] = (uint8_t)motor.State_Mode;
    data[4] = (uint8_t)motor.Control_Mode;

    /* 电压 */
    float_to_bytes(motor.foc.vbus,     &data[8]);

    /* 电流反馈 */
    float_to_bytes(motor.foc.i_q,      &data[12]);
    float_to_bytes(motor.foc.i_d,      &data[16]);

    /* 电流设定 */
    float_to_bytes(motor.foc.iq_set,   &data[20]);
    float_to_bytes(motor.foc.id_set,   &data[24]);

    /* 电压 */
    float_to_bytes(motor.foc.v_q,      &data[28]);
    float_to_bytes(motor.foc.v_d,      &data[32]);

    /* 速度 */
    float_to_bytes(motor.foc.vel_fb,   &data[36]);
    float_to_bytes(motor.foc.vel_set,  &data[40]);

    /* 位置 */
    float_to_bytes(motor.foc.pos_fb,   &data[44]);
    float_to_bytes(motor.foc.pos_set,  &data[48]);

    /* 角度 */
    float_to_bytes(motor.foc.theta,    &data[52]);

    /* 编码器数据（如果有） */
    if (motor.mt6816 != NULL)
    {
        float_to_bytes(motor.mt6816->mec_angle,   &data[56]);
        float_to_bytes(motor.mt6816->speed,       &data[60]);
    }

    return FDCAN_Send(FDCAN_GetRptId(), data, 64);
}

/* ======================================================================= */
/*                  主机命令处理 (ID=0x100, 经典 CAN 8字节)                 */
/* ======================================================================= */

/**
 * @brief  读取指定参数的值（4字节写入 value 缓冲区）
 * @param  paramId   参数 ID
 * @param  value     输出缓冲区（至少 4 字节）
 * @retval RSP_OK    成功
 * @retval RSP_ERR_ID 无效 ID
 */
static FDCAN_RspStatus param_read_value(uint8_t paramId, uint8_t *value)
{
    float fval;

    switch ((FDCAN_ParamID)paramId)
    {
    case PARAM_NODE_ID:
        value[0] = fdcan_node_id;
        value[1] = value[2] = value[3] = 0;
        break;
    case PARAM_STATE_MODE:
        value[0] = (uint8_t)motor.State_Mode;
        value[1] = value[2] = value[3] = 0;
        break;
    case PARAM_CTRL_MODE:
        value[0] = (uint8_t)motor.Control_Mode;
        value[1] = value[2] = value[3] = 0;
        break;
    case PARAM_IQ_FB:       float_to_bytes(motor.foc.i_q,      value); break;
    case PARAM_ID_FB:       float_to_bytes(motor.foc.i_d,      value); break;
    case PARAM_IQ_SET:      float_to_bytes(motor.foc.iq_set,   value); break;
    case PARAM_ID_SET:      float_to_bytes(motor.foc.id_set,   value); break;
    case PARAM_VQ:          float_to_bytes(motor.foc.v_q,      value); break;
    case PARAM_VD:          float_to_bytes(motor.foc.v_d,      value); break;
    case PARAM_VBUS:        float_to_bytes(motor.foc.vbus,     value); break;
    case PARAM_VEL_FB:      float_to_bytes(motor.foc.vel_fb,   value); break;
    case PARAM_VEL_SET:     float_to_bytes(motor.foc.vel_set,  value); break;
    case PARAM_POS_FB:      float_to_bytes(motor.foc.pos_fb,   value); break;
    case PARAM_POS_SET:     float_to_bytes(motor.foc.pos_set,  value); break;
    case PARAM_THETA:       float_to_bytes(motor.foc.theta,    value); break;

    case PARAM_IQ_KP:       float_to_bytes(motor.IqPID.Kp,    value); break;
    case PARAM_IQ_KI:       float_to_bytes(motor.IqPID.Ki,    value); break;
    case PARAM_ID_KP:       float_to_bytes(motor.IdPID.Kp,    value); break;
    case PARAM_ID_KI:       float_to_bytes(motor.IdPID.Ki,    value); break;
    case PARAM_VEL_KP:      float_to_bytes(motor.VelPID.Kp,   value); break;
    case PARAM_VEL_KI:      float_to_bytes(motor.VelPID.Ki,   value); break;
    case PARAM_POS_KP:      float_to_bytes(motor.PosPID.Kp,   value); break;
    case PARAM_POS_KI:      float_to_bytes(motor.PosPID.Ki,   value); break;

    case PARAM_ENCODER_ANGLE:
        if (motor.mt6816 != NULL)
            float_to_bytes(motor.mt6816->mec_angle, value);
        else
            float_to_bytes(0.0f, value);
        break;
    case PARAM_ENCODER_SPEED:
        if (motor.mt6816 != NULL)
            float_to_bytes(motor.mt6816->speed, value);
        else
            float_to_bytes(0.0f, value);
        break;
    case PARAM_ELECT_ANGLE:
        if (motor.mt6816 != NULL)
            float_to_bytes(motor.mt6816->elec_angle, value);
        else
            float_to_bytes(0.0f, value);
        break;

    default:
        return RSP_ERR_ID;
    }

    return RSP_OK;
}

/**
 * @brief  写入指定参数的值
 * @param  paramId   参数 ID
 * @param  value     4 字节参数值
 * @retval RSP_OK    成功
 * @retval RSP_ERR_ID 无效或不可写 ID
 */
static FDCAN_RspStatus param_write_value(uint8_t paramId, const uint8_t *value)
{
    float fval = bytes_to_float(value);

    switch ((FDCAN_ParamID)paramId)
    {
    case PARAM_NODE_ID:
        FDCAN_SetNodeId((uint8_t)fval);
        break;
    case PARAM_IQ_SET:
        motor.foc.iq_set = fval;
        break;
    case PARAM_ID_SET:
        motor.foc.id_set = fval;
        break;
    case PARAM_VEL_SET:
        motor.foc.vel_set = fval;
        break;
    case PARAM_POS_SET:
        motor.foc.pos_set = fval;
        break;
    case PARAM_IQ_KP:
        motor.IqPID.Kp = fval;
        break;
    case PARAM_IQ_KI:
        motor.IqPID.Ki = fval;
        break;
    case PARAM_ID_KP:
        motor.IdPID.Kp = fval;
        break;
    case PARAM_ID_KI:
        motor.IdPID.Ki = fval;
        break;
    case PARAM_VEL_KP:
        motor.VelPID.Kp = fval;
        break;
    case PARAM_VEL_KI:
        motor.VelPID.Ki = fval;
        break;
    case PARAM_POS_KP:
        motor.PosPID.Kp = fval;
        break;
    case PARAM_POS_KI:
        motor.PosPID.Ki = fval;
        break;
    case PARAM_CTRL_MODE:
        if (fval >= CONTROL_MODE_OPEN && fval <= CONTROL_MODE_GRADIENT_POSITION)
            motor.Control_Mode = (CONTROL_MODE)(uint8_t)fval;
        else
            return RSP_ERR_ID;
        break;
    default:
        return RSP_ERR_ID;   /* 只读参数或无效 ID */
    }

    return RSP_OK;
}

/**
 * @brief  处理收到的主机命令帧
 * @note   主机命令格式 (8字节)：
 *         [0]    = 命令类型 (FDCAN_CmdType)
 *         [1]    = 参数 ID (FDCAN_ParamID)
 *         [2..5] = 数据 (float/uint32, 小端序, 仅写入时有效)
 *         [6..7] = 目标节点 ID (CMD_SET_NODE_ID 时作为新 ID)
 *
 *         处理逻辑：
 *         CMD_QUERY       → 读取参数，应答单参数响应
 *         CMD_WRITE       → 写入参数，应答写入结果
 *         CMD_REPORT      → 直接上报批量数据
 *         CMD_SET_NODE_ID → 设置本设备节点 ID（仅广播 0x100 时生效）
 *
 *         cmdId 参数用于确认命令是否发往本节点或广播
 */
void FDCAN_ProcessCmd(const uint8_t *pData, uint32_t cmdId)
{
    uint8_t  cmdType  = pData[0];
    uint8_t  paramId  = pData[1];
    uint8_t  value[4] = {pData[2], pData[3], pData[4], pData[5]};
    uint8_t  rspData[4] = {0};
    FDCAN_RspStatus status;

    switch ((FDCAN_CmdType)cmdType)
    {
    case CMD_QUERY:
        /* 读取参数并应答 */
        status = param_read_value(paramId, rspData);
        FDCAN_SendParamRsp(paramId, rspData, status);
        break;

    case CMD_WRITE:
        /* 写入参数并应答 */
        status = param_write_value(paramId, value);
        if (status == RSP_OK)
            param_read_value(paramId, rspData);   /* 回读当前值 */
        FDCAN_SendParamRsp(paramId, rspData, status);
        break;

    case CMD_REPORT:
        /* 立即上报批量数据 */
        FDCAN_SendBulkReport();
        break;

    case CMD_SET_NODE_ID:
        /* 设置节点 ID：pData[6] = 新节点 ID */
        /* 仅广播命令 (cmdId == CANID_BROADCAST) 才处理 */
        if (cmdId == CANID_BROADCAST)
        {
            FDCAN_SetNodeId(pData[6]);
        }
        break;

    default:
        /* 未知命令，回复错误 */
        FDCAN_SendParamRsp(paramId, rspData, RSP_ERR_ID);
        break;
    }
}

/* ======================================================================= */
/*                           接收回调入口                                    */
/* ======================================================================= */

/**
 * @brief  CAN 接收回调
 * @note   在 HAL_FDCAN_RxFifo0Callback() 中调用
 *         判断规则：
 *         - rxId == 广播 ID (0x100) → 所有节点处理
 *         - rxId == 本节点命令 ID   → 本节点处理
 *         - 其他 ID → 忽略
 */
void FDCAN_OnReceive(const uint8_t *pData, uint8_t len, uint32_t rxId)
{
    uint32_t myCmdId = FDCAN_GetCmdId();

    /* 广播或发给本节点的命令 */
    if ((rxId == CANID_BROADCAST) || (rxId == myCmdId))
    {
        FDCAN_ProcessCmd(pData, rxId);
    }
}

/* ======================================================================= */
/*                  保留：测试函数（兼容旧代码）                              */
/* ======================================================================= */

/**
 * @brief  经典 CAN 发送（兼容旧的 FDCAN_TestSend，最大 8 字节）
 */
HAL_StatusTypeDef FDCAN_TestSend(uint32_t id, const uint8_t *pData, uint8_t dataLen)
{
    if (dataLen > 8) dataLen = 8;
    return FDCAN_Send(id, pData, dataLen);
}

/**
 * @brief  发送一组预定义的测试帧（用于快速验证总线通信）
 */
uint32_t FDCAN_TestSendSequence(void)
{
    uint32_t sent = 0;
    uint8_t  data[8];
    uint32_t testIds[] = {0x101, 0x102, 0x103, 0x201, 0x202};
    const uint8_t numFrames = sizeof(testIds) / sizeof(testIds[0]);

    for (uint8_t i = 0; i < numFrames; i++)
    {
        data[0] = i;
        data[1] = 0;
        data[2] = (uint8_t)(testIds[i] >> 0);
        data[3] = (uint8_t)(testIds[i] >> 8);
        data[4] = 0xAA;
        data[5] = 0xBB;
        data[6] = 0xCC;
        data[7] = 0xDD;

        if (FDCAN_TestSend(testIds[i], data, 8) == HAL_OK)
            sent++;
    }

    return sent;
}
