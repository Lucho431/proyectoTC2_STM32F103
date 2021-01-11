/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "ssd1306.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MUESTRAS 156
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//flags
uint8_t flag_adcStart = 0;
uint8_t flag_adcDone = 0;
uint8_t flag_print = 0;

//armonicos
uint32_t arm1[MUESTRAS],
			arm3[MUESTRAS],
			arm5[MUESTRAS],
			arm7[MUESTRAS],
			arm11[MUESTRAS],
			arm13[MUESTRAS],
			rms1=0,
			rms3=0,
			rms5=0,
			rms7=0,
			rms11=0,
			rms13=0;

uint8_t i=0;

uint8_t muxABC = 0;

char texto[25];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void RMS (void){

	rms1=0;
	rms3=0;
	rms5=0;
	rms7=0;
	rms11=0;
	rms13=0;

	for (uint8_t j = 0; j < MUESTRAS; j++){
		rms1+=arm1[j];
		rms3+=arm3[j];
		rms5+=arm5[j];
		rms7+=arm7[j];
		rms11+=arm11[j];
		rms13+=arm13[j];
	}

	rms1/=MUESTRAS;
	rms3/=MUESTRAS;
	rms5/=MUESTRAS;
	rms7/=MUESTRAS;
	rms11/=MUESTRAS;
	rms13/=MUESTRAS;

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  SSD1306_Clear();
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (flag_adcStart){
		  GPIOB->ODR = (uint32_t)muxABC;
		  HAL_ADC_Start_IT(&hadc1);
		  flag_adcStart = 0;
	  }//end if flag_adcStart

	  if (flag_adcDone){

		  switch (muxABC){
		  	  case 0:
		  		  arm1[i] = HAL_ADC_GetValue(&hadc1);
		  		  arm1 [i] = pow(arm1[i],2);
		  		  muxABC++;
			  break;
		  	  case 1:
		  		  arm3[i] = HAL_ADC_GetValue(&hadc1);
		  		  arm3 [i] = pow(arm3[i],2);
		  		  muxABC++;
			  break;
		  	  case 2:
		  		  arm5[i] = HAL_ADC_GetValue(&hadc1);
		  		  arm5 [i] = pow(arm5[i],2);
		  		  muxABC++;
			  break;

		  	  case 3:
		  		  arm7[i] = HAL_ADC_GetValue(&hadc1);
		  		  arm7 [i] = pow(arm7[i],2);
		  		  muxABC++;
			  break;
		  	  case 4:
		  		  arm11[i] = HAL_ADC_GetValue(&hadc1);
		  		  arm11 [i] = pow(arm11[i],2);
		  		  muxABC++;
			  break;
		  	  case 5:
		  		  arm13[i] = HAL_ADC_GetValue(&hadc1);
		  		  arm13[i] = pow(arm13[i],2);

		  		  i++;
		  		  if (i > 199){
		  			  RMS();
		  			  flag_print = 1;
		  			  i = 0;
		  		  }
		  		  muxABC = 0;
			  break;
		  }

		  flag_adcDone = 0;
	  }//end if flag_adcDone

	  if (flag_print){
/*
		  SSD1306_Fill(0);
		  SSD1306_GotoXY(0, 18);
		  sprintf (texto, "%d %d", (int)rms1, (int)rms3);
		  SSD1306_Puts(texto, &Font_7x10, 1);
		  SSD1306_GotoXY(0, 30);
		  sprintf (texto, "%d %d", (int)rms5, (int)rms7);
		  SSD1306_Puts(texto, &Font_7x10, 1);
		  SSD1306_GotoXY(0, 42);
		  sprintf (texto, "%d %d", (int)rms11, (int)rms13);
		  SSD1306_Puts(texto, &Font_7x10, 1);
		  SSD1306_UpdateScreen();
*/
		  SSD1306_Fill(0);
		  SSD1306_GotoXY(0, 0);
		  sprintf (texto, "Armonicos:");
		  SSD1306_Puts(texto, &Font_7x10, 1);
		  SSD1306_GotoXY(0, 11);
		  sprintf (texto, "Comp 3ro: %d Comp 5to: %d", (int)(rms3*100/rms1), (int)(rms5*100/rms1));
		  SSD1306_Puts(texto, &Font_7x10, 1);
		  SSD1306_GotoXY(0, 22);
		  sprintf (texto, "Comp 7mo: %d Comp 11vo: %d", (int)(rms7*100/rms1), (int)(rms11*100/rms1));
		  SSD1306_Puts(texto, &Font_7x10, 1);
		  SSD1306_GotoXY(0, 33);
		  sprintf (texto, "Comp 13vo: %d", (int)(rms13*100/rms1));
		  SSD1306_Puts(texto, &Font_7x10, 1);
		  SSD1306_UpdateScreen();


		  flag_print = 0;
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	flag_adcStart = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	flag_adcDone = 1;
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
