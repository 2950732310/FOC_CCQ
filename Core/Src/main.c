/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt.h"
#include "stmflash.h"
#include "ws2812.h"
#include "usart_printf.h"
#include "serial_control.h"
#include "motor_control.h"
#include "fdcan_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void flash_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t state_i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  flash_init(); // 初始化Flash
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init(168);
  RGB_SetBrightness(15);                                  // 设置LED亮度
  RGB_DisplayColorById(GREEN_F);                            // 点亮RGB初始状态
  UART_StartRX_DMA();                                                 // 启动串口DMA循环接收
  UART_RegisterProtocolCallback(Protocol_CommandHandler);   // 注册协议命令回调
  Motor_StartControl();                                               // 启动PWM输出并开启注入式ADC采样

  FDCAN_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* 第一次启动时写入FLASH */
  if (motor.foc.flash_data.first_run != 0)
  {
    motor.foc.flash_data.first_run = 0;
    stmflash_write(FLASH_ADDR, (uint32_t *)&motor.foc.flash_data, sizeof(motor.foc.flash_data) / 4);
    motor.State_Mode = STATE_MODE_RUNNING;    // 初始化之后切换到运行模式
    RGB_DisplayColorById(GREEN_F);   // 校准完成后显示绿色
  }

  /* 输出启动信息 */
  UART_Printf_DMA("\r\n=======================================\r\n");
  UART_Printf_DMA("  MY-FOC Serial Protocol Ready\r\n");
  UART_Printf_DMA("  Type $HELP for available commands\r\n");
  UART_Printf_DMA("=======================================\r\n");

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    state_i++;

    UART_ProtocolProcess(); // 处理串口协议（检查是否有完整行到达）
    /* 每 100 ms 执行一次 */
    if ((state_i % 10U) == 0U)
    {

    }

    /* 每 1 s 执行一次 */
    if ((state_i % 100U) == 0U)
    {

      /*========================== 处理数据接收完成标志位 ===========================*/
      if (command_flag.data_received != 0)
      {
        command_flag.data_received = 0;

        /* 恢复出厂设置 */
        if (command_flag.reset != 0)
        {
          motor.foc.flash_data.first_run = 1;
          stmflash_write(FLASH_ADDR, (uint32_t *)&motor.foc.flash_data, sizeof(motor.foc.flash_data) / 4);
        }

        /* 检查是否需要写入FLASH */
        if (command_flag.flash_control != 0)
        {
          stmflash_write(FLASH_ADDR, (uint32_t *)&motor.foc.flash_data, sizeof(motor.foc.flash_data) / 4);
          command_flag.flash_control = 0;
        }
      }


      /*========================== CAN 每秒上报当前位置 ==========================*/
      {
          union { float f; uint8_t b[4]; } pos_val;
          pos_val.f = motor.foc.pos_fb;
          FDCAN_SendParamRsp(PARAM_POS_FB, pos_val.b, RSP_OK);
      }

      /*==========================心跳灯控制==========================*/
      switch (motor.State_Mode)
      {
        case STATE_MODE_IDLE:
          RGB_DisplayColorById(BLUE_F);   // 空闲状态显示蓝色
          break;
        case STATE_MODE_DETECTING:
          RGB_DisplayColorById(RED_F);    // 检测状态显示红色
          break;
        case STATE_MODE_RUNNING:                    // 运行状态绿色闪烁
          RGB_ToggleGreen();
          break;
        default:
          break;
      }
      state_i = 0U;
    }
    DWT_Delay_ms(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void flash_init(void)
{
  stmflash_read(FLASH_ADDR, (uint32_t *)&motor.foc.flash_data, sizeof(motor.foc.flash_data) / 4);
  if (motor.foc.flash_data.first_run != 0)
  {
    // 首次运行，初始化flash数据
    motor.foc.flash_data.version = 260710100;
    motor.foc.flash_data.iq_kp = IQ_KP;
    motor.foc.flash_data.iq_ki = IQ_KI;
    motor.foc.flash_data.iq_kd = ID_KD;
    motor.foc.flash_data.id_kp = ID_KP;
    motor.foc.flash_data.id_ki = ID_KI;
    motor.foc.flash_data.id_kd = ID_KD;
    motor.foc.flash_data.vel_kp = VEL_KP;
    motor.foc.flash_data.vel_ki = VEL_KI;
    motor.foc.flash_data.vel_kd = VEL_KD;
    motor.foc.flash_data.pos_kp = POS_KP;
    motor.foc.flash_data.pos_ki = 0.0f;
    motor.foc.flash_data.pos_kd = 0.0f;
    motor.State_Mode = STATE_MODE_DETECTING;    // 第一次上电在检测模式下校准电流和编码器
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
