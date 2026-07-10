#include "motor_control.h"
#include "FOCMotor.h"
#include "adc.h"
#include "stm32g431xx.h"
#include "usart_printf.h"
#include "DWT.h"



/* 电机结构体初始化 */
MOTOR_DATA motor = {
    .State_Mode = STATE_MODE_RUNNING,
    .Control_Mode = CONTROL_MODE_TORQUE,
    .foc ={
            .flash_data = {
              .ia_zero = 0.0f,
              .ib_zero = 0.0f,
              .ic_zero = 0.0f,
              .iq_kp   = 1.0f,
              .iq_ki   = 0.5f,
              .iq_kd   = 0.0f,
              .id_kp   = 0.0f,
              .id_ki   = 0.0f,
              .id_kd   = 0.0f,
              .vel_kp  = 0.0f,
              .vel_ki  = 0.0f,
              .vel_kd  = 0.0f,
              .theta_kp = 0.0f,
              .theta_ki = 0.0f,
              .theta_kd = 0.0f,
            },
            .vbus        = BATVEL,
            .inv_vbus    = INVBATVEL,
            .theta       = 0.0f,
            .vd_set      = 0.0f,
            .vq_set      = -3.0f,
            .id_set      = 0.0f,
            .iq_set      = 1.0f
        },
    .mt6816 = &encoder_data,
    .IqPID                      = {
        // 电流环IQ参数
        .mode                   = PID_POSITION,
        .Kp                     = 0,
        .Ki                     = 0,
        .Kd                     = 0,
        .max_out                = MAX_V_LIMIT,   
        .max_iout               = MAX_V_LIMIT,
    },
    .IdPID                      = {
        // 电流环ID参数
        .mode                   = PID_POSITION,
        .Kp                     = 0,
        .Ki                     = 0,
        .Kd                     = 0,
        .max_out                = MAX_V_LIMIT,
        .max_iout               = MAX_V_LIMIT,
    },
};


int i = 0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }

    switch (motor.State_Mode)
    {
        case STATE_MODE_IDLE:
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
                    break;
                case CONTROL_MODE_POSITION:
                    break;
                default:
                    break;
            }
            i++;
            if(i>10)
            {
                // UART_Printf_DMA("%f,%f,%f\r\n", motor.foc.theta, motor.foc.v_alpha, motor.foc.v_beta);
                // UART_Printf_DMA("%f,%f,%f\r\n", motor.foc.dtc_a, motor.foc.dtc_b, motor.foc.dtc_c);
                // UART_Printf_DMA("%f,%f,%f\r\n", motor.foc.i_a, motor.foc.i_b, motor.foc.i_c);
                UART_Printf_DMA("%f,%f\r\n", motor.foc.v_q, motor.foc.i_q);
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
    /*第一次启动时对齐*/
    if(motor.foc.flash_data.first_run != 0)
    {
        Foc_Align(&motor);
    }
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    DWT_Delay_ms(100);
    /*第一次启动时电流偏移校准*/
    if(motor.foc.flash_data.first_run != 0)
    {
        FOC_CurrentOffsetCalibration(&motor);
    }
    
}




