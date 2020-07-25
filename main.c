/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hex_indicator.h"
#include "RDA5807.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int y;
int z;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
char symbolBuffer[6];
unsigned char ledsData[5]; 
unsigned char volume;
unsigned short frequency;
char sleep;
unsigned char eepromBuffer[12];
unsigned int result;
unsigned char EEPROMadress;
unsigned char pwrdown; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
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
volume = 8;
frequency = 880;
char sleep = 0;
//RDA5807_reciever myReciever;


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
  MX_TIM14_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM14_Init();
  /*************************************************************************/
/*****###  E X A M P L E  initialization  F O R   SI4703 rev.2****************/  
  /***reciever initialization**/
SI4703_Init();
//getting manufacturing ID,firmware and revision - it`s optional
//revision in my chip is 2
z = SI4703_GetMFGID();
z = SI4703_GetFIRMWARE();
z =  SI4703_GetRevision();
//set a band 76-108MHz 
SI4703_SetBAND(0x01);///76-108
//set step 100kHz
SI4307_SetSPACE(0x01);//100kHz
//set volume (in buffer)
SI4703_SetVOLUME(0x03);
//choose a channel and send data to chip
SI4703_channelSelecton(240);
//set volume in buffer and transmitt a buffer to IC
z = SI4703_sendVolume(0x0E);
/****###  E N D    O F   E x A M P L E for SI4703*********************/ 

HAL_Delay(100);

  /***EEPROM***/

 eepromBuffer[0] = 0x01;
 HAL_I2C_Master_Transmit(&hi2c1,0xA0,eepromBuffer,1,1);
 HAL_Delay(10);
   /*read frequency - hight byte*/
    HAL_I2C_Master_Receive(&hi2c1,0xA0,eepromBuffer,1,1);
    HAL_Delay(10);
    frequency = eepromBuffer[0];
    frequency = frequency << 8;
    /*low byte*/
    HAL_I2C_Master_Receive(&hi2c1,0xA0,eepromBuffer,1,1);
    HAL_Delay(10);
     frequency |= eepromBuffer[0];
     /*set adress Volume in EEPROM*/
   eepromBuffer[0] = 0x03;
   HAL_I2C_Master_Transmit(&hi2c1,0xA0,eepromBuffer,1,1);
   /*read volume*/
   HAL_I2C_Master_Receive(&hi2c1,0xA0,eepromBuffer,1,1);
   volume = eepromBuffer[0];
     /*call procedure to convert a frequency to hex led`s buffer*/
    calcIndicatorData(ledsData, frequency);
    ledsData[2] &= 0x7F;
    SI4703_SetVOLUME(volume);
    SI4703_channelSelecton(frequency - 760);

  /*****iiiiiiiiiiiiiiiiii I 2 C  ccccccccc***/
  
 
LL_TIM_EnableIT_UPDATE (TIM14);
 // GPIOA->BSRR = GPIO_BSRR_BS_15;
  GPIOA->BSRR = GPIO_BSRR_BR_11;
  GPIOA->BSRR = GPIO_BSRR_BR_10;
  GPIOA->BSRR = GPIO_BSRR_BR_9;
  GPIOA->BSRR = GPIO_BSRR_BR_8;

 // HAL_I2C_ERROR_NONE 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   
     for(int u = 0; u<1000; u++){
     };
     
  LL_LPM_EnableSleep();
  LL_PWR_SetPowerMode (LL_PWR_MODE_STOP_MAINREGU);
  PWR->CR |= PWR_CR_CWUF; /* Clear Wakeup flag */
 SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* Set SLEEPDEEP bit of Cortex-M0 System Control Register */
 

  __WFI(); /* Request Wait For Interrupt */
      
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_SetSystemCoreClock(24000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();  
  };
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
   /****###  SI4703 EXAMPLE GPIO INITIALIZATION***********/
  /*!!!NRST pin of Si473 must been set LOW before*/
  /* !! SEN pin of si4703 must be connect to HIGH for running in I2C mode */
  GPIOA->BSRR = GPIO_BSRR_BS_12;
  /*set SDATA to 0 - */
GPIOB->MODER |= GPIO_MODER_MODER7_0;
HAL_Delay(9);
/*set NRST to high - rise of this pulse applys the mode '2WIRE' (like I2C) in Si4703*/
GPIOA->BSRR = GPIO_BSRR_BS_15;
HAL_Delay(100);
GPIOA->BSRR = GPIO_BSRR_BR_15;
HAL_Delay(100);
GPIOA->BSRR = GPIO_BSRR_BS_15;
HAL_Delay(500);
/*clear SDATA to default*/
GPIOB->MODER &= ~(GPIO_MODER_MODER7_0);
HAL_Delay(100);

/*****### END OF EXAMPLE**************/
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00101D2D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);

  /* TIM14 interrupt Init */
  NVIC_SetPriority(TIM14_IRQn, 1);
  NVIC_EnableIRQ(TIM14_IRQn);

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  TIM_InitStruct.Prescaler = 0x64;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0x318;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM14, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM14);
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(indicator_A_GPIO_Port, indicator_A_Pin);

  /**/
  LL_GPIO_ResetOutputPin(indicator_B_GPIO_Port, indicator_B_Pin);

  /**/
  LL_GPIO_ResetOutputPin(indicator_C_GPIO_Port, indicator_C_Pin);

  /**/
  LL_GPIO_ResetOutputPin(indicator_D_GPIO_Port, indicator_D_Pin);

  /**/
  LL_GPIO_ResetOutputPin(indicator_E_GPIO_Port, indicator_E_Pin);

  /**/
  LL_GPIO_ResetOutputPin(indicator_F_GPIO_Port, indicator_F_Pin);

  /**/
  LL_GPIO_ResetOutputPin(indicator_G_GPIO_Port, indicator_G_Pin);

  /**/
  LL_GPIO_ResetOutputPin(indicator_DP_GPIO_Port, indicator_DP_Pin);

  /**/
  LL_GPIO_ResetOutputPin(Dig_1_GPIO_Port, Dig_1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(Dig_2_GPIO_Port, Dig_2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(Dig_3_GPIO_Port, Dig_3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(Dig_4_GPIO_Port, Dig_4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(NRST_si_GPIO_Port, NRST_si_Pin);

  /**/
  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  GPIO_InitStruct.Pin = indicator_A_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(indicator_A_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = indicator_B_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(indicator_B_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = indicator_C_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(indicator_C_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = indicator_D_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(indicator_D_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = indicator_E_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(indicator_E_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = indicator_F_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(indicator_F_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = indicator_G_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(indicator_G_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = indicator_DP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(indicator_DP_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Vol__Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Vol__GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Dig_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(Dig_1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Dig_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(Dig_2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Dig_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(Dig_3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Dig_4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(Dig_4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NRST_si_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(NRST_si_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Vol_B3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Vol_B3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Frq__Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Frq__GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Frq_B5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Frq_B5_GPIO_Port, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_1_IRQn, 2);
  NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

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
