#include "motor_control.h"
#include "FOCMotor.h"
#include "adc.h"
#include "stm32g431xx.h"
#include "usart_printf.h"
#include "DWT.h"
#include "algorithm.h"
#include "ws2812.h"

/* 电机结构体初始化 */
MOTOR_DATA motor = {
    .State_Mode = STATE_MODE_RUNNING,
    .Control_Mode = CONTROL_MODE_GRADIENT_POSITION,
    .foc = {
        .flash_data = {
            .ia_zero = 0.0f,
            .ib_zero = 0.0f,
            .ic_zero = 0.0f,
            .iq_kp = 0.0f,
            .iq_ki = 0.0f,
            .iq_kd = 0.0f,
            .id_kp = 0.0f,
            .id_ki = 0.0f,
            .id_kd = 0.0f,
            .vel_kp = 0.0f,
            .vel_ki = 0.0f,
            .vel_kd = 0.0f,
            .pos_kp = 0.0f,
            .pos_ki = 0.0f,
            .pos_kd = 0.0f,
            
        },
        .vbus = BATVEL,
        .inv_vbus = INVBATVEL,
        .theta = 0.0f,
        .vd_set = 0.0f,
        .vq_set = 0.5f,
        .id_set = 0.0f,
        .iq_set = 0.5f,
        .vel_set = 10.0f,
        .pos_set = 0.0f,
        .pos_ref = 90.0f,
    },
    .mt6816 = &encoder_data,
    .IqPID = {
        // 电流环IQ参数
        .mode = PID_POSITION,
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,
        .max_out = MAX_V_LIMIT,
        .max_iout = MAX_V_LIMIT,
    },
    .IdPID = {
        // 电流环ID参数
        .mode = PID_POSITION,
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,
        .max_out = MAX_V_LIMIT,
        .max_iout = MAX_V_LIMIT,
    },
    .VelPID = {
        // 速度环参数
        .mode = PID_POSITION,
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,
        .max_out = MOTOR_IQ_MAX,
        .max_iout = MOTOR_IQ_MAX,
    },
    .PosPID = {
        // 位置环参数
        .mode = PID_POSITION,
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,
        .max_out = MAX_VEL_LIMIT,
        .max_iout = MAX_VEL_LIMIT,
    },
};

int i = 0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static int vel_Hz = 0;

    if (hadc->Instance != ADC1)
    {
        return;
    }

    switch (motor.State_Mode)
    {
    case STATE_MODE_IDLE:
        PID_clear(&motor.IqPID);
        PID_clear(&motor.IdPID);
        PID_clear(&motor.VelPID);
        PID_clear(&motor.PosPID);
        motor.foc.i_d = 0.0f;
        motor.foc.i_q = 0.0f;
        Foc_Pwm_LowSides();
        break;
    case STATE_MODE_DETECTING:
        break;
    case STATE_MODE_RUNNING:
        FOC_UpdateCurrentSampling(&motor);
        GetMotor_Angle(motor.mt6816);
        switch (motor.Control_Mode)
        {
        case CONTROL_MODE_OPEN:
            OpenControlMode(&motor);
            break;
        case CONTROL_MODE_TORQUE:
            CurrentControl(&motor);
            break;
        case CONTROL_MODE_VELOCITY:
            vel_Hz++;
            if (vel_Hz > 4)
            {
                VelocityControl(&motor);
                vel_Hz = 0;
            }
            CurrentControl(&motor);
            break;
        case CONTROL_MODE_POSITION:
            vel_Hz++;
            if (vel_Hz > 4)
            {
                PositionControl(&motor);
                VelocityControl(&motor);
                vel_Hz = 0;
            }
            CurrentControl(&motor);
            break;
        case CONTROL_MODE_GRADIENT_POSITION:
            vel_Hz++;
            if (vel_Hz > 4)
            {
                /* 使用梯形加减速轨迹规划器生成平滑位置参考值 */
                motor.foc.pos_set = Trajectory_Update(
                    motor.foc.pos_ref,                    /* 目标位置 (°) */
                    motor.mt6816->pos_estimate_ * 360.0f, /* 当前反馈位置 (°) */
                    0.0002f                               /* 采样时间 (s) */
                );
                PositionControl(&motor);
                VelocityControl(&motor);
                vel_Hz = 0;
            }
            CurrentControl(&motor);
            break;
        default:
            break;
        }
        i++;
        if (i > 10)
        {
            // UART_Printf_DMA("%f,%f,%f\r\n", motor.foc.theta, motor.foc.v_alpha, motor.foc.v_beta);
            // UART_Printf_DMA("%f,%f,%f,%f\r\n", motor.foc.dtc_a, motor.foc.dtc_b, motor.foc.dtc_c,motor.mt6816->mec_angle_deg);
            // UART_Printf_DMA("%f,%f,%f\r\n", motor.foc.i_a, motor.foc.i_b, motor.foc.i_c);
            // UART_Printf_DMA("%f,%f,%f,%f\r\n", motor.foc.v_q, motor.foc.iq_set, motor.foc.i_q,motor.mt6816->vel_estimate_);
            // UART_Printf_DMA("%f,%f,%f\r\n", motor.foc.v_d, motor.foc.id_set, motor.foc.i_d);
            // UART_Printf_DMA("%f,%f,%f\r\n", motor.foc.iq_set,motor.foc.vel_set,motor.foc.vel_fb);
            UART_Printf_DMA("%f,%f,%f,%f\r\n", motor.foc.pos_fb, motor.foc.pos_set, motor.foc.pos_ref,motor.foc.vel_set);
            i = 0;
        }
        break;
    default:
        break;
    }
}

void Motor_StartControl(void)
{
    Foc_Pwm_Start();
    DWT_Delay_ms(100);
    /*第一次启动时对齐*/
    if (motor.foc.flash_data.first_run != 0)
    {
        RGB_DisplayColorById(PURPLE_F); // 校准时显示紫色
        Foc_Align(&motor);
    }
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    DWT_Delay_ms(100);
    /*第一次启动时电流偏移校准*/
    if (motor.foc.flash_data.first_run != 0)
    {
        FOC_CurrentOffsetCalibration(&motor);
        DWT_Delay_ms(1000);
    }
}
