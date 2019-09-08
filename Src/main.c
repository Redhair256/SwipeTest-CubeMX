/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "STM32L152xC.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define bool _Bool
#define FALSE 0
#define TRUE !FALSE

#define Button_Delay  20
#define LedON_Delay  3000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern bool UserButton;               /* Set by interrupt handler to indicate that user button is pressed */
int BtDelay = Button_Delay;
int LedOnDelay = LedON_Delay;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

LCD_HandleTypeDef hlcd;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t LCD_Conv_Char_Seg  (uint8_t symbol, uint8_t point);
void LCD_GLASS_WriteChar(uint8_t ch, uint8_t point, uint8_t position);
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
  systickInit(1000);   			// systick frequency 1kHz
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_LCD_Init();
  MX_TS_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//  LCD_GLASS_WriteChar(uint8_t ch, uint8_t point, uint8_t position)
  LCD_GLASS_WriteChar('H', 0, 1);
  LCD_GLASS_WriteChar('E', 0, 2);
  LCD_GLASS_WriteChar('L', 0, 3);
  LCD_GLASS_WriteChar('L', 0, 4);
  LCD_GLASS_WriteChar('O', 0, 5);
  LCD_GLASS_WriteChar(' ', 0, 6);
 /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start(&htim4);				//PWM generator start
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(UserButton)
	  {
	     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	     LedOnDelay = LedON_Delay;
	     UserButton = FALSE;
	  }
	  else HAL_Delay(500);
	  if(LedOnDelay == 0)
	  {

	    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LCD;
  PeriphClkInit.LCDClockSelection = RCC_RTCCLKSOURCE_HSE_DIV8;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  RCC->APB1ENR |= RCC_APB1ENR_PWREN|RCC_APB1ENR_LCDEN;
  RCC->CSR |= RCC_CSR_LSION;
  while (! (RCC->CSR & RCC_CSR_LSIRDY)) ;

  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR  |=  PWR_CR_DBP;       // disable backup write protection !!!

  RCC->CSR |=  RCC_CSR_RTCSEL_1; // LSI select
  RCC->CSR |=  RCC_CSR_RTCEN;    // RTC en   (LCD clocks from RTC mux)
  PWR->CR  &= ~PWR_CR_DBP;       // enable backup write protection

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */
  LCD->CR &= ~0x60; 			// Clear CR
  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_16;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_3;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_3;
  hlcd.Init.DeadTime = LCD_DEADTIME_1;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_ENABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */
  LCD->CR |= 0x1;				// LCD ON

  while(!(LCD->SR&LCD_SR_RDY));

  while(!(LCD->SR&LCD_SR_ENS));
  /* USER CODE END LCD_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 350;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

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

  /* Configure Ports LCD Output*/

    RCC->AHBENR |=(RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN);

   /*  Configure Port B LCD Output pins as alternate function */
    GPIO_InitStruct.Pin = ( GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PB0_GPIO_Port, &GPIO_InitStruct);//GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIOB->AFR[0] = 0x00BBB000;		/* 0xBBB00000 = 0000 0000 1011 1011 1011 0000 0000 0000*/
    GPIOB->AFR[1] = 0xBBBBBBBB; 	/* 0xB0000BBB = 1011 1011 1011 1011 1011 1011 1011 1011*/


    /* Configure Port A LCD Output pins as alternate function */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PA0_GPIO_Port, &GPIO_InitStruct);
    GPIOA->AFR[0] = 0xBBB0;				/* 0xBBB0 = 1011 1011 1011 0000*/
    GPIOA->AFR[1] = 0xB0000BBB; 	/* 0xB0000BBB = 1011 0000 0000 0000 0000 1011 1011 1011*/

    /* Configure Port C LCD Output pins as alternate function */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PA0_GPIO_Port, &GPIO_InitStruct);
    GPIOC->AFR[0] = 0xBB00BBBB;		/* 0xBBB00000 = 1011 1011 0000 0000 1011 1011 1011 1011*/
    GPIOC->AFR[1] = 0x0000BBBB; 	/* 0xB0000BBB = 0000 0000 0000 0000 1011 1011 1011 1011*/

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IDD_CNT_EN_GPIO_Port, IDD_CNT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IDD_CNT_EN_Pin */
/*
  GPIO_InitStruct.Pin = IDD_CNT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IDD_CNT_EN_GPIO_Port, &GPIO_InitStruct);
 */

  /*Configure GPIO pin : PA0_Pin */
  GPIO_InitStruct.Pin = PA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PA0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

static uint16_t LCD_Conv_Char_Seg  (uint8_t symbol, uint8_t point)
{
	uint16_t from_ascii[0x60] = {
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /*															*/
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /*															*/
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /*															*/
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /*         !       "       #       $      %        &       ' 	*/
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /* (       )       *       +       ,       -       .       / 	*/
	  0x0000, 0x0000, 0xA0DD, 0x0000, 0x0000, 0xA000, 0x0000, 0x00C0,
	  /* 0       1       2       3       4       5       6       7 	*/
	  0x5F00, 0x4200, 0xF500, 0x6700, 0xEA00, 0xAF00, 0xBF00, 0x4600,
	  /* 8       9       :       ;       <       =       >       ? 	*/
	  0xFF00, 0xEF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /* @       A       B       C       D       E       F       G 	*/
	  0x0000, 0xFE00, 0x6714, 0x1D00, 0x4714, 0x9D00, 0x9C00, 0x3F00,
	  /* H       I       J       K       L       M       N       O 	*/
	  0xFA00, 0x0014, 0x5300, 0x9841, 0x1900, 0x5A48, 0x5A09, 0x5F00,
	  /* P       Q       R       S       T       U       V       W 	*/
	  0xFC00, 0x5F01, 0xFC01, 0xAF00, 0x0414, 0x5B00, 0x18C0, 0x5A81,
	  /* X       Y       Z       [       \       ]       ^       _ 	*/
	  0x00C9, 0x0058, 0x05C0, 0x1D00, 0x0000, 0x4700, 0x0000, 0x0000
	};

	if (symbol > 0x60) return 0x0000; // masks not defined. Nothing to display

    /* Set the digital point can be displayed if the point is on */
    if(point != 0)
    {
    	return from_ascii[symbol] | 0x0002;
    }
	return from_ascii[symbol];
}


void LCD_GLASS_WriteChar(uint8_t ch, uint8_t point, uint8_t position)
{
	uint16_t digit[0x04];     /* Digit frame buffer */
    uint32_t char_mask;
    uint16_t LCD_char;
    uint8_t i,j;
    /* To convert displayed character in segment in array digit */
    LCD_char = LCD_Conv_Char_Seg(ch, point);
    for(i = 0x0C, j = 0x00 ; j < 0x04; i -= 0x04, j++)
    {
        digit[j] = ((LCD_char >> i) & 0x0F); //To isolate the less signifiant bit
    }

    /* TO wait LCD Ready */
 /*   while(LCD_SRbits.UDR == true);*/
    while(!(LCD->SR&LCD_SR_RDY));

    switch (position)
    {
        /* Position 1 on LCD (Digit1)*/
        case 1:
        {
        	char_mask = 0xCFFFFFFC;

        	HAL_LCD_Write(&hlcd, 0, char_mask, ((((unsigned long)digit[0x00]) & 0x0C) << 0x1A) | (((unsigned long)digit[0x00]) & 0x03)); // 1G 1B 1M 1E
        	HAL_LCD_Write(&hlcd, 2, char_mask, ((((unsigned long)digit[0x01]) & 0x0C) << 0x1A) | (((unsigned long)digit[0x01]) & 0x03)); // 1F 1A 1C 1D
        	HAL_LCD_Write(&hlcd, 4, char_mask, ((((unsigned long)digit[0x02]) & 0x0C) << 0x1A) | (((unsigned long)digit[0x02]) & 0x03)); // 1Q 1K 1Col 1P
        	HAL_LCD_Write(&hlcd, 6, char_mask, ((((unsigned long)digit[0x03]) & 0x0C) << 0x1A) | (((unsigned long)digit[0x03]) & 0x03)); // 1H 1J 1DP 1N

            break;
         }

        /* Position 2 on LCD (Digit2)*/
        case 2:
        {
        	char_mask = 0xF3FFFF03;

        	HAL_LCD_Write(&hlcd, 0, char_mask, ((((unsigned long)digit[0x00]) & 0x0C) << 0x18) | ((((unsigned long)digit[0x00]) & 0x02) << 0x06) | ((((unsigned long)digit[0x00]) & 0x01) << 0x02)); // 2G 2B 2M 2E
        	HAL_LCD_Write(&hlcd, 2, char_mask, ((((unsigned long)digit[0x01]) & 0x0C) << 0x18) | ((((unsigned long)digit[0x01]) & 0x02) << 0x06) | ((((unsigned long)digit[0x01]) & 0x01) << 0x02)); // 2F 2A 2C 2D
        	HAL_LCD_Write(&hlcd, 4, char_mask, ((((unsigned long)digit[0x02]) & 0x0C) << 0x18) | ((((unsigned long)digit[0x02]) & 0x02) << 0x06) | ((((unsigned long)digit[0x02]) & 0x01) << 0x02)); // 2Q 2K 2Col 2P
        	HAL_LCD_Write(&hlcd, 6, char_mask, ((((unsigned long)digit[0x03]) & 0x0C) << 0x18) | ((((unsigned long)digit[0x03]) & 0x02) << 0x06) | ((((unsigned long)digit[0x03]) & 0x01) << 0x02)); // 2H 2J 2DP 2N

            break;
         }

        /* Position 3 on LCD (Digit3)*/
        case 3:
        {
        	char_mask = 0xFCFFFCFF;

        	HAL_LCD_Write(&hlcd, 0, char_mask, ((((unsigned long)digit[0x00]) & 0x0C) << 0x16) | ((((unsigned long)digit[0x00]) & 0x03) << 0x08)); // 3G 3B 3M 3E
        	HAL_LCD_Write(&hlcd, 2, char_mask, ((((unsigned long)digit[0x01]) & 0x0C) << 0x16) | ((((unsigned long)digit[0x01]) & 0x03) << 0x08)); // 3F 3A 3C 3D
        	HAL_LCD_Write(&hlcd, 4, char_mask, ((((unsigned long)digit[0x02]) & 0x0C) << 0x16) | ((((unsigned long)digit[0x02]) & 0x03) << 0x08)); // 3Q 3K 3Col 3P
        	HAL_LCD_Write(&hlcd, 6, char_mask, ((((unsigned long)digit[0x03]) & 0x0C) << 0x16) | ((((unsigned long)digit[0x03]) & 0x03) << 0x08)); // 3H 3J 3DP 3N

            break;
         }

        /* Position 4 on LCD (Digit4)*/
        case 4:
        {
        	char_mask = 0xFFCFF3FF;

        	HAL_LCD_Write(&hlcd, 0, char_mask, ((((unsigned long)digit[0x00]) & 0x0C) << 0x12) | ((((unsigned long)digit[0x00]) & 0x03) << 0x0A)); // 4G 4B 4M 4E
        	HAL_LCD_Write(&hlcd, 2, char_mask, ((((unsigned long)digit[0x01]) & 0x0C) << 0x12) | ((((unsigned long)digit[0x01]) & 0x03) << 0x0A)); // 4F 4A 4C 4D
        	HAL_LCD_Write(&hlcd, 4, char_mask, ((((unsigned long)digit[0x02]) & 0x0C) << 0x12) | ((((unsigned long)digit[0x02]) & 0x03) << 0x0A)); // 4Q 4K 4Col 4P
        	HAL_LCD_Write(&hlcd, 6, char_mask, ((((unsigned long)digit[0x03]) & 0x0C) << 0x12) | ((((unsigned long)digit[0x03]) & 0x03) << 0x0A)); // 4H 4J 4DP 4N

            break;
         }

        /* Position 5 on LCD (Digit5)*/
        case 5:
        {
        	char_mask = 0xFFF3CFFF;

        	HAL_LCD_Write(&hlcd, 0, char_mask, ((((unsigned long)digit[0x00]) & 0x0C) << 0x10) | ((((unsigned long)digit[0x00]) & 0x03) << 0x0C)); // 5G 5B 5M 5E
        	HAL_LCD_Write(&hlcd, 2, char_mask, ((((unsigned long)digit[0x01]) & 0x0C) << 0x10) | ((((unsigned long)digit[0x01]) & 0x03) << 0x0C)); // 5F 5A 5C 5D
        	HAL_LCD_Write(&hlcd, 4, char_mask, ((((unsigned long)digit[0x02]) & 0x0C) << 0x10) | ((((unsigned long)digit[0x02]) & 0x01) << 0x0C)); // 5Q 5K   5P
        	HAL_LCD_Write(&hlcd, 6, char_mask, ((((unsigned long)digit[0x03]) & 0x0C) << 0x10) | ((((unsigned long)digit[0x03]) & 0x01) << 0x0C)); // 5H 5J   5N

            break;
         }

        /* Position 6 on LCD (Digit6)*/
        case 6:
        {
        	char_mask = 0xFFFC3FFF;

        	HAL_LCD_Write(&hlcd, 0, char_mask, ((((unsigned long)digit[0x00]) & 0x04) << 0x0F) | ((((unsigned long)digit[0x00]) & 0x08) << 0x0D) | ((((unsigned long)digit[0x00]) & 0x03) << 0x0E)) ; // 6B 6G 6M 6E
        	HAL_LCD_Write(&hlcd, 2, char_mask, ((((unsigned long)digit[0x01]) & 0x04) << 0x0F) | ((((unsigned long)digit[0x01]) & 0x08) << 0x0D) | ((((unsigned long)digit[0x01]) & 0x03) << 0x0E)) ; // 6A 6F 6C 6D
        	HAL_LCD_Write(&hlcd, 4, char_mask, ((((unsigned long)digit[0x02]) & 0x04) << 0x0F) | ((((unsigned long)digit[0x02]) & 0x08) << 0x0D) | ((((unsigned long)digit[0x02]) & 0x01) << 0x0E)) ; // 6K 6Q    6P
        	HAL_LCD_Write(&hlcd, 6, char_mask, ((((unsigned long)digit[0x03]) & 0x04) << 0x0F) | ((((unsigned long)digit[0x03]) & 0x08) << 0x0D) | ((((unsigned long)digit[0x03]) & 0x01) << 0x0E)) ; // 6J 6H   6N

            break;
         }

         default:
         {
            break;
         }
    }

    HAL_LCD_UpdateDisplayRequest(&hlcd);
}


void systickInit (uint16_t frequency)
{
	uint32_t ticks;
	ticks = HAL_RCC_GetSysClockFreq();
	ticks = ticks / frequency;

    SysTick_Config (ticks);
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
