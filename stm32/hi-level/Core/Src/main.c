/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body of LEAP-Up.
 *                   Use STM32CubeIDE.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> /* printf(). */
#include <math.h>
#include "kinematics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define L1 (258)
#define L2 (325)

#define FTS_ZERO_POINT_OUTPUT (8192) /* 8192+-655. Refer to WEF-6A200 F/T sensor datasheet*/
#define FTS_RX_BUFFER_LEN (27)       /* 27 bytes. */
#define EFE_RX_BUFFER_LEN (2)
#define SFE_RX_BUFFER_LEN (10)
#define SIE_RX_BUFFER_LEN (10)

/* Reverse joints CW/CCW.  */
// #define INVERT_SIE_DIRECTION
#define INVERT_SFE_DIRECTION
// #define INVERT_EFE_DIRECTION

/* Reverse F/T sensor axis. */
#define INVERT_FTS_FX
// #define INVERT_FTS_FY
// #define INVERT_FTS_FZ
// #define INVERT_FTS_MX
// #define INVERT_FTS_MY
// #define INVERT_FTS_MZ

// #define EXCHANGE_FTS_FX_FY

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t UART_FTS_RxBuffer[FTS_RX_BUFFER_LEN] = {0};
volatile uint8_t UART_FTS_RxDone = 0;
uint8_t UART_EFE_RxBuffer[EFE_RX_BUFFER_LEN] = {0};
volatile uint8_t UART_EFE_RxDone = 0;
uint8_t UART_SFE_RxBuffer[SFE_RX_BUFFER_LEN] = {0};
volatile uint8_t UART_SFE_RxDone = 0;
uint8_t UART_SIE_RxBuffer[SIE_RX_BUFFER_LEN] = {0};
volatile uint8_t UART_SIE_RxDone = 0;

typedef struct
{
  int16_t Fx;
  int16_t Fy;
  int16_t Fz;
  int16_t Mx;
  int16_t My;
  int16_t Mz;
  uint8_t N;
} FTS_Data_t;

FTS_Data_t FTS_Data;
FTS_Data_t FTS_Data_offset;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ASCII_to_int(uint8_t ascii);
uint16_t ASCII_to_int_4(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4);
int FTS_parse_sensor_data(uint8_t *raw_data,
                          int16_t zero_point,
                          int16_t *Fx,
                          int16_t *Fy,
                          int16_t *Fz,
                          int16_t *Mx,
                          int16_t *My,
                          int16_t *Mz,
                          uint8_t *N);
void FTS_Corrdinate_Convert2(float theta, float x, float y, float *out_x, float *out_y);

void SIE_Set_Angle(float rad);
void SFE_Set_Angle(float rad);
void EFE_Set_Angle(float rad);

void SIE_Get_Angle(float *rad);
void SFE_Get_Angle(float *rad);
void EFE_Get_Angle(float *rad);

void SIE_Enable(uint8_t enable);
void SFE_Enable(uint8_t enable);
void EFE_Enable(uint8_t enable);

int __io_putchar(int ch)
{
  //  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
  //  {
  //  }

  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  //  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
  //  {
  //  }

  return ch;
}

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // uint8_t enable = 0xFE;
  // HAL_UART_Transmit(&huart5, (uint8_t *)&enable, 1, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart2, "Init\r\n", 6, HAL_MAX_DELAY); /* Requires FTS output detected value once. */
  HAL_Delay(500);

  SIE_Enable(1);
  SFE_Enable(1);
  EFE_Enable(1);
  HAL_Delay(200);

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)UART_FTS_RxBuffer, FTS_RX_BUFFER_LEN);
  HAL_UART_Transmit(&huart1, "R", 1, HAL_MAX_DELAY); /* Requires FTS output detected value once. */
  HAL_Delay(200);

  // SIE_Set_Angle(0);
  //  SFE_Set_Angle(0);
  //  EFE_Set_Angle(0.35);
  // float tSFE, tEFE = 0;
  //  IK2(L1 / 2.0, 0, L1, L2, &tSFE, &tEFE);
  //  SFE_Set_Angle(tSFE);
  //  EFE_Set_Angle(tEFE);
  // HAL_Delay(5000);

  FTS_Data_offset = FTS_Data;

//    HAL_UART_Receive_IT(&huart1, (uint8_t *)UART_FTS_RxBuffer, FTS_RX_BUFFER_LEN);
//    HAL_UART_Transmit(&huart1, "R", 1, HAL_MAX_DELAY); /* Requires FTS output detected value once. */

  HAL_UART_Transmit(&huart2, "Ready\r\n", 7, HAL_MAX_DELAY); /* Requires FTS output detected value once. */

  // while(1)
  // {
  // 	  SFE_Enable(1);
  // 	  EFE_Enable(1);

  //     float targetSFE, targetEFE = 0;

  //     IK2((L1+L2)/2, 0, L1, L2, &targetSFE, &targetEFE);

  //     SFE_Set_Angle(targetSFE);
  //     EFE_Set_Angle(targetEFE);
  // }

  while (1)
  {
    float oriX, oriY, oriZ = 0;      /* Origin Cartesian coordinate system */
    float targetX, targetY, targetZ = 0; /* Target Cartesian coordinate system*/
    float oriSIE, oriSFE, oriEFE = 0;            /*Origin joint angle*/
    float targetSFE, targetEFE, targetSIE = 0; /* Target joint angle */
    float extFX, extFY, extFZ = 0; /* Force */
    /* Read F/T sensor data */
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)UART_FTS_RxBuffer, FTS_RX_BUFFER_LEN);
    HAL_UART_Transmit(&huart1, "R", 1, HAL_MAX_DELAY); /* Requires FTS output detected value once. */
    while (!UART_FTS_RxDone)
    {
    }
    UART_FTS_RxDone = 0;

    //    SIE_Get_Angle(&oriSIE);
    SFE_Get_Angle(&oriSFE);
    //    SFE_Set_Angle(oriSFE);

    EFE_Get_Angle(&oriEFE);
    //    EFE_Set_Angle(oriEFE);

//    FTS_Corrdinate_Convert2((oriSFE + oriEFE),
//                            (FTS_Data.My - FTS_Data_offset.My),
//                            (FTS_Data.Mx - FTS_Data_offset.Mx),
//                            &extFX,
//                            &extFY);

    /* Offset */
    extFX = (FTS_Data.Fx - FTS_Data_offset.Fx);
    extFY = (FTS_Data.Fy - FTS_Data_offset.Fy);
    extFZ = (FTS_Data.Fz - FTS_Data_offset.Fz);

    /* Admittance Control */
    float offsetX = (0.07 * extFX);
    float offsetY = (0.07 * extFY);
    float offsetZ = (0.001 * extFZ);

    //    if (offsetX > 5)
    //      offsetX = 5;
    //    else if (offsetX < -5)
    //      offsetX = -5;
    //
    //    if (offsetY > 5)
    //      offsetY = 5;
    //    else if (offsetY < -5)
    //      offsetY = -5;
    //
    //    if (offsetZ > 5)
    //      offsetZ = 5;
    //    else if (offsetZ < -5)
    //      offsetZ = -5;

//    FK2(oriSFE, oriEFE, L1, L2, &oriX, &oriY);
//    float r = sqrt(pow(oriX, 2) + pow(oriY, 2));
//    targetX = r + offsetX;
//    IK2(targetX, 0, L1, L2, &targetSFE, &targetEFE);

    /* Limit */
//    if(extFY > extFX)
    {
    	if(offsetY >0.18)
    	{
    		offsetY = 0.18;
    	}
    	else if(offsetY < -0.18)
    	{
    		offsetY = -0.18;
    	}
        targetSFE = oriSFE + offsetY;
//    	targetEFE=oriEFE;
        targetEFE = oriEFE - offsetX;
    }

    targetSIE = oriSIE + offsetZ;

    /* Threshold */
    if (fabs(oriEFE - targetEFE) > 0.15)
    {
      EFE_Set_Angle(targetEFE);
    }

    if (fabs(oriSFE - targetSFE) > 0.16)
    {
      SFE_Set_Angle(targetSFE);
    }

    if (fabs(oriSIE - targetSIE) > 0.18)
    {
      SIE_Set_Angle(targetSIE);
    }

    /* LED blinking */
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    HAL_Delay(100);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{
  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{
  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 3);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

void SIE_Set_Angle(float rad)
{
  if (rad < -0.45)
  {
    rad = -0.45;
  }
  else if (rad > 0.7)
  {
    rad = 0.7;
  }

  // rad += M_PI;
  rad *= 100.0f; /* Gear reduction ratio. */

#ifdef INVERT_SIE_DIRECTION
  rad *= -1;
#endif

  char sign = (rad >= 0) ? '+' : '-';
  rad = fabs(rad);
  /* '0x30' = '0' of ASCII. */
  uint8_t data[10];
  data[0] = 'M';
  data[1] = (uint8_t)sign;
  data[2] = ((int)rad / 100 % 10) + 0x30; /* Handred place. */
  data[3] = ((int)rad / 10 % 10) + 0x30;  /* Ten place. */
  data[4] = ((int)rad / 1 % 10) + 0x30;   /* Unit place. */
  data[5] = '.';
  data[6] = ((int)(rad * 10) / 1 % 10) + 0x30;
  data[7] = ((int)(rad * 100) / 1 % 10) + 0x30;
  data[8] = ((int)(rad * 1000) / 1 % 10) + 0x30;
  data[9] = '\n';

  HAL_UART_Transmit(&huart3, data, 10, HAL_MAX_DELAY);
}

void SFE_Set_Angle(float rad)
{
  rad += M_PI_2; /* Offset. */

//  if (rad < 0)
//  {
//    rad = 0;
//  }
//  else if (rad > M_PI_2)
//  {
//    rad = M_PI_2;
//  }

  rad *= 100; /* Gear reduction ratio. */

#ifdef INVERT_SFE_DIRECTION
  rad *= -1;
#endif

  char sign = (rad >= 0) ? '+' : '-';
  rad = fabs(rad);
  /* '0x30' = '0' of ASCII. */
  uint8_t data[10];
  data[0] = 'M';
  data[1] = (uint8_t)sign;
  data[2] = ((int)rad / 100 % 10) + 0x30; /* Handred place. */
  data[3] = ((int)rad / 10 % 10) + 0x30;  /* Ten place. */
  data[4] = ((int)rad / 1 % 10) + 0x30;   /* Unit place. */
  data[5] = '.';
  data[6] = ((int)(rad * 10) / 1 % 10) + 0x30;
  data[7] = ((int)(rad * 100) / 1 % 10) + 0x30;
  data[8] = ((int)(rad * 1000) / 1 % 10) + 0x30;
  data[9] = '\n';

  HAL_UART_Transmit(&huart4, data, 10, HAL_MAX_DELAY);
}

void EFE_Set_Angle(float rad)
{
  // #ifdef INVERT_EFE_DIRECTION
  //   rad *= -1;
  // #endif

  rad -= 0.349;                    /* Zero offset.  */
  float deg = rad * 180.0f / M_PI; /* To degree. */

  if (deg < 0)
  {
    deg = 0;
  }
  else if (deg > 65)
  {
    deg = 65;
  }

  uint8_t data[] = {((int)deg & 0x7F)};
  EFE_Enable(1);
  HAL_UART_Transmit(&huart5, data, 1, HAL_MAX_DELAY);
}

void SIE_Get_Angle(float *rad)
{
  while (HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)UART_SIE_RxBuffer, SIE_RX_BUFFER_LEN) != HAL_OK)
  {
  }
  HAL_UART_Transmit(&huart3, "A\n", 2, HAL_MAX_DELAY);

  while (!UART_SIE_RxDone)
  {
  }

  /* '\r' = CR, '\n' = LF. */
  if (UART_SIE_RxBuffer[SIE_RX_BUFFER_LEN - 2] != '\r' && UART_SIE_RxBuffer[SIE_RX_BUFFER_LEN - 1] != '\n')
  {
    /* Error occurred. */
    //    Error_Handler();
  }

  float rawRad = (ASCII_to_int(UART_SIE_RxBuffer[1]) * 100 +
                  ASCII_to_int(UART_SIE_RxBuffer[2]) * 10 +
                  ASCII_to_int(UART_SIE_RxBuffer[3]) +
                  ASCII_to_int(UART_SIE_RxBuffer[5]) / 10.0 +
                  ASCII_to_int(UART_SIE_RxBuffer[6]) / 100.0 +
                  ASCII_to_int(UART_SIE_RxBuffer[7]) / 1000.0);

  if (UART_SIE_RxBuffer[0] == '-')
  {
    rawRad *= -1;
  }

#ifdef INVERT_SIE_DIRECTION
  rawRad *= -1;
#endif

  *rad = (rawRad / 9.0f);

  UART_SIE_RxDone = 0; /* Reset flag. */
}

void SFE_Get_Angle(float *rad)
{
  while (HAL_UARTEx_ReceiveToIdle_IT(&huart4, (uint8_t *)UART_SFE_RxBuffer, SFE_RX_BUFFER_LEN) != HAL_OK)
  {
  }
  HAL_UART_Transmit(&huart4, "A\n", 2, HAL_MAX_DELAY);

  while (!UART_SFE_RxDone)
  {
  }

  /* '\r' = CR, '\n' = LF. */
  if (UART_SFE_RxBuffer[SFE_RX_BUFFER_LEN - 2] != '\r' && UART_SFE_RxBuffer[SFE_RX_BUFFER_LEN - 1] != '\n')
  {
    /* Error occurred. */
    //    Error_Handler();
  }

  float rawRad = (ASCII_to_int(UART_SFE_RxBuffer[1]) * 100 +
                  ASCII_to_int(UART_SFE_RxBuffer[2]) * 10 +
                  ASCII_to_int(UART_SFE_RxBuffer[3]) +
                  ASCII_to_int(UART_SFE_RxBuffer[5]) / 10.0 +
                  ASCII_to_int(UART_SFE_RxBuffer[6]) / 100.0 +
                  ASCII_to_int(UART_SFE_RxBuffer[7]) / 1000.0);

  if (UART_SFE_RxBuffer[0] == '-')
  {
    rawRad *= -1;
  }

#ifdef INVERT_SFE_DIRECTION
  rawRad *= -1;
#endif

  *rad = (rawRad / 100.0f) - M_PI_2;

  UART_SFE_RxDone = 0; /* Reset flag. */
}

void EFE_Get_Angle(float *rad)
{
  //  while (HAL_UART_Receive_IT(&huart5, (uint8_t *)UART_EFE_RxBuffer, EFE_RX_BUFFER_LEN) != HAL_OK)
  while (HAL_UARTEx_ReceiveToIdle_IT(&huart5, (uint8_t *)UART_EFE_RxBuffer, EFE_RX_BUFFER_LEN) != HAL_OK)
  {
  }

  uint8_t data[] = {0xFA};
  HAL_UART_Transmit(&huart5, data, 1, HAL_MAX_DELAY); /* Request return angle. */

  while (!UART_EFE_RxDone)
  {
  }

  if (UART_EFE_RxBuffer[0] == 0xFE && UART_EFE_RxBuffer[1] == 0xFE)
  {
    /* Error occurred. */
    Error_Handler();
  }

  uint16_t rawPosition = (UART_EFE_RxBuffer[0] << 8) + UART_EFE_RxBuffer[1];

#ifdef INVERT_EFE_DIRECTION
  rawPosition *= -1;
#endif

  *rad = rawPosition / 16384.0 * 2.0 * M_PI; /* (raw / (2^14) * 360) * Pi / 180. */
  *rad += 0.349;                             /* Zero offset. */

  UART_EFE_RxDone = 0; /* Reset flag. */
}

void SIE_Enable(uint8_t enable)
{
  /* 'ME1' for enable, 'ME0' for disable. */
  uint8_t data[] = "ME0\n";
  if (enable)
  {
    data[2] = '1';
  }

  HAL_UART_Transmit(&huart3, data, 4, HAL_MAX_DELAY);
}

void SFE_Enable(uint8_t enable)
{
  /* 'ME1' for enable, 'ME0' for disable. */
  uint8_t data[] = "ME0\n";
  if (enable)
  {
    data[2] = '1';
  }

  HAL_UART_Transmit(&huart4, data, 4, HAL_MAX_DELAY);
}

void EFE_Enable(uint8_t enable)
{
  /* '0xFE' for enable, '0xFF' for disable. */
  uint8_t data[1];
  if (enable)
  {
    data[0] = 0xFE;
  }
  else
  {
    data[0] = 0xFF;
  }

  HAL_UART_Transmit(&huart5, data, 1, HAL_MAX_DELAY);
}

uint8_t ASCII_to_int(uint8_t ascii)
{
  /*
   * ASCII=HEX:
   * 0=0x30, 1=0x31 ... 9=0x39
   * A=0x41, B=0x42 ... F=0x46
   */

  if (ascii <= 0x39 && ascii >= 0x30)
  {
    return ascii - 0x30;
  }
  else if (ascii <= 0x46 && ascii >= 0x41)
  {
    return (ascii - 0x41) + 10; /* A(hex) = 10. */
  }
  else
  {
    return 0xFF; /* Error. */
  }
}

uint16_t ASCII_to_int_4(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
  uint8_t n1 = ASCII_to_int(a1); /* LSB. */
  uint8_t n2 = ASCII_to_int(a2);
  uint8_t n3 = ASCII_to_int(a3);
  uint8_t n4 = ASCII_to_int(a4);

  return ((n1 << 12) + (n2 << 8) + (n3 << 4) + n4) & 0x3FFF;
}

int FTS_parse_sensor_data(uint8_t *raw_data,
                          int16_t zero_point,
                          int16_t *Fx,
                          int16_t *Fy,
                          int16_t *Fz,
                          int16_t *Mx,
                          int16_t *My,
                          int16_t *Mz,
                          uint8_t *N)
{
  *N = ASCII_to_int(raw_data[0]);

  int16_t rawFx = ASCII_to_int_4(raw_data[1], raw_data[2], raw_data[3], raw_data[4]) - zero_point;
  int16_t rawFy = ASCII_to_int_4(raw_data[5], raw_data[6], raw_data[7], raw_data[8]) - zero_point;
  int16_t rawFz = ASCII_to_int_4(raw_data[9], raw_data[10], raw_data[11], raw_data[12]) - zero_point;
  int16_t rawMx = ASCII_to_int_4(raw_data[13], raw_data[14], raw_data[15], raw_data[16]) - zero_point;
  int16_t rawMy = ASCII_to_int_4(raw_data[17], raw_data[18], raw_data[19], raw_data[20]) - zero_point;
  int16_t rawMz = ASCII_to_int_4(raw_data[21], raw_data[22], raw_data[23], raw_data[24]) - zero_point;

#ifdef INVERT_FTS_FX
  rawFx *= -1;
#endif
#ifdef INVERT_FTS_FY
  rawFy *= -1;
#endif
#ifdef INVERT_FTS_FZ
  rawFz *= -1;
#endif

#ifdef INVERT_FTS_MX
  rawMx *= -1;
#endif
#ifdef INVERT_FTS_MY
  rawMy *= -1;
#endif
#ifdef INVERT_FTS_MZ
  rawMz *= -1;
#endif

#ifdef EXCHANGE_FTS_FX_FY
  uint16_t temp = rawFy;
  rawFy = rawFx;
  rawFx = temp;
#endif

  *Fx = rawFx;
  *Fy = rawFy;
  *Fz = rawFz;
  *Mx = rawMx;
  *My = rawMy;
  *Mz = rawMz;

  return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) /* FT sensor. */
  {
  }
  else if (huart->Instance == UART5) /* EFE. */
  {
    UART_EFE_RxDone = 1;
  }
  else if (huart->Instance == UART4) /* SFE. */
  {
    UART_SFE_RxDone = 1;
  }
  else if (huart->Instance == USART3) /* SIE. */
  {
    UART_SIE_RxDone = 1;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == USART1) /* FT sensor. */
  {
    if (UART_FTS_RxBuffer[FTS_RX_BUFFER_LEN - 2] == 0x0D && UART_FTS_RxBuffer[FTS_RX_BUFFER_LEN - 1] == 0x0A)
    {
      uint8_t N;
      int16_t Fx, Fy, Fz, Mx, My, Mz;
      FTS_parse_sensor_data(UART_FTS_RxBuffer, FTS_ZERO_POINT_OUTPUT, &Fx, &Fy, &Fz, &Mx, &My, &Mz, &N);

      FTS_Data.Fx = Fx;
      FTS_Data.Fy = Fy;
      FTS_Data.Fz = Fz;
      FTS_Data.Mx = Mx;
      FTS_Data.My = My;
      FTS_Data.Mz = Mz;
      FTS_Data.N = N;

      UART_FTS_RxDone = 1;
      // UART_FTS_RxBuffer = {0}; /* Clear. */
    }
  }
  else if (huart->Instance == UART5) /* EFE. */
  {
    UART_EFE_RxDone = 1;
  }
  else if (huart->Instance == UART4) /* SFE. */
  {
    UART_SFE_RxDone = 1;
  }
  else if (huart->Instance == USART3) /* SIE. */
  {
    UART_SIE_RxDone = 1;
  }
}

void FTS_Corrdinate_Convert2(float theta, float x, float y, float *out_x, float *out_y)
{
  int threshold = 10;
  if ((x > 0 && x < threshold) || (x < 0 && x > -threshold))
  {
    x = 0;
  }
  if ((y > 0 && y < threshold) || (y < 0 && y > -threshold))
  {
    y = 0;
  }

  *out_x = x * cos(theta) - y * sin(theta);
  *out_y = x * sin(theta) + y * cos(theta);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  SFE_Enable(0);
  EFE_Enable(0);

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
