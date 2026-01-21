/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include<stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_SIZE 5       // AA F A P CRC
#define MAX_TABLE_SIZE 256
#define FRAME_TIMEOUT_MS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t TABLE_SIZE =32;
uint8_t rx_byte;
uint8_t frame_buf[FRAME_SIZE];
uint8_t frame_index = 0;
uint8_t receiving = 0;
uint32_t frame_timeout =0;
uint16_t sine_table[MAX_TABLE_SIZE];
uint16_t scaled_table[MAX_TABLE_SIZE];
uint16_t dirac_table[MAX_TABLE_SIZE];
uint16_t trigger_table[MAX_TABLE_SIZE];
float amplitude_scale = 1.0f;
uint16_t phase_offset = 0;
uint16_t xx = 2048;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void GenerateSineTable(void)
{
    for (int i = 0; i < TABLE_SIZE; i++)
    {
        float angle = 2.0f * 3.1415926f * i / TABLE_SIZE;
        sine_table[i] = (uint16_t)(xx + (xx-1) * sinf(angle));
    }
}


void GenerateTriggerTable()
{
	for (int i = 0; i < TABLE_SIZE; i++) {
	    trigger_table[i] = 0;
	}
	for (int i = 0; i < (TABLE_SIZE/10); i++) {
		    trigger_table[i] = 4000;
		}
}

void UpdateScaledTable(void)
{
	memset(scaled_table, 0, sizeof(scaled_table));
    for (int i = 0; i < TABLE_SIZE; i++)
    {
        float centered = sine_table[i] - xx;
        centered *= amplitude_scale;
        scaled_table[i] = (uint16_t)(xx + centered);
    }
}

void UpdateTriggerTable()
{
	memset(dirac_table, 0, sizeof(dirac_table));
	for(int i = 0;i < TABLE_SIZE;i++)
	{
		int index = (i + phase_offset)% TABLE_SIZE;
		dirac_table[index] = trigger_table[i];
	}
}


void SetAmplitude(uint8_t amp)
{
    amplitude_scale = (float)amp / 255.0f;;
    UpdateScaledTable();
}

void Setphase(uint8_t phase)
{
	phase_offset = phase;
	UpdateTriggerTable();
}



void SetFrequency(uint8_t freq,uint8_t amp,uint8_t phase)
{
    if (freq == 0) return;

    uint32_t real_freq = freq*1000;
    uint32_t timer_clk = 84000000;   // APB1 Timer Clock = 84MHz
    HAL_TIM_Base_Stop(&htim6);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);

    if (real_freq > 80000)      TABLE_SIZE = 16;    // 80k~100kHz 16
        else if (real_freq > 32000) TABLE_SIZE = 32;    // 32k~80kHz 32
        else if (real_freq > 16000) TABLE_SIZE = 64;    // 16k~32kHz 64
        else                        TABLE_SIZE = 256;   // <16kHz 256
    GenerateSineTable();
    GenerateTriggerTable();
    SetAmplitude(amp);
    Setphase(phase);

    uint32_t psc =0;
    uint32_t arr = (timer_clk / (real_freq * TABLE_SIZE)) - 1;
    htim6.Instance->PSC = psc;
    htim6.Instance->ARR = arr;
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)scaled_table, TABLE_SIZE, DAC_ALIGN_12B_R);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)dirac_table, TABLE_SIZE, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(&htim6);

}



uint8_t CalcCRC(uint8_t *buf)
{
    uint8_t sum = 0;
    sum += buf[1];  // F
    sum += buf[2];  // A
    sum += buf[3];  // P
    return sum & 0xFF;
}

void ProcessFrame(uint8_t *buf)
{
    if (buf[0] != 0xAA)
        return;

    uint8_t freq  = buf[1];
    uint8_t amp   = buf[2];
    uint8_t phase = buf[3];
    uint8_t crc   = buf[4];

    uint8_t calc = CalcCRC(buf);
    if (calc != crc)
        return;

    SetFrequency(freq,amp,phase);


}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	GenerateSineTable();
	UpdateScaledTable();
	GenerateTriggerTable();
	UpdateTriggerTable();
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)scaled_table, TABLE_SIZE, DAC_ALIGN_12B_R);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)dirac_table, TABLE_SIZE, DAC_ALIGN_12B_R);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	    if (huart->Instance == USART2)
	    {
	        uint8_t b = rx_byte;

	        if (!receiving)
	        {
	            if (b == 0xAA)
	            {
	                receiving = 1;
	                frame_index = 0;
	                frame_buf[frame_index++] = b;
	            }
	        }
	        else
	        {
	            frame_buf[frame_index++] = b;
	            if (frame_index >= 5)
	            {
	                receiving = 0;

	                ProcessFrame(frame_buf);
	            }
	        }

	        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
